#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
deploy_myquad_policy.py

用途：工控机/笔记本运行 Isaac Lab 导出的 TorchScript policy.pt，
通过串口接收 STM32 的机器人状态，推理得到 12 个目标关节角 q_des，
再通过串口发回 STM32。

协议说明：
- STM32 -> PC: RobotStatePacket，小端，CRC16-CCITT-FALSE
- PC -> STM32: JointCommandPacket，小端，CRC16-CCITT-FALSE
- 所有关节角都是 URDF/模型坐标系下的输出轴关节角，单位 rad。
"""

from __future__ import annotations

import argparse
import struct
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import torch

try:
    import serial
    import serial.tools.list_ports
except ModuleNotFoundError:
    serial = None

# =========================
# 1. 机器人/策略常量
# =========================

# 训练环境：sim.dt=0.005, decimation=4，所以 policy 频率约 50Hz。
POLICY_HZ = 50.0
POLICY_DT = 1.0 / POLICY_HZ

# 你的当前训练/Sim2Sim配置是 JointPositionActionCfg(scale=0.3, use_default_offset=True)
# 仿真公式：q_des = q_default + 0.3 * action。
TRAIN_ACTION_SCALE = 0.3

# 串口协议中的关节顺序。需要和 STM32 状态包/命令包完全一致。
WIRE_JOINT_NAMES = [
    "LF_HAA", "LF_HFE", "LF_KFE",
    "RF_HAA", "RF_HFE", "RF_KFE",
    "LH_HAA", "LH_HFE", "LH_KFE",
    "RH_HAA", "RH_HFE", "RH_KFE",
]

# IsaacLab policy 的 action/joint 顺序。这个顺序已在 Sim2Sim 中确认。
POLICY_JOINT_NAMES = [
    "LF_HAA", "LH_HAA", "RF_HAA", "RH_HAA",
    "LF_HFE", "LH_HFE", "RF_HFE", "RH_HFE",
    "LF_KFE", "LH_KFE", "RF_KFE", "RH_KFE",
]

WIRE_TO_POLICY = np.array([WIRE_JOINT_NAMES.index(name) for name in POLICY_JOINT_NAMES], dtype=np.int64)
POLICY_TO_WIRE = np.array([POLICY_JOINT_NAMES.index(name) for name in WIRE_JOINT_NAMES], dtype=np.int64)

# 表 5 / my_quad.py 默认站姿角：URDF/模型关节角，单位 rad。
Q_DEFAULT_WIRE = np.array([
    -0.027960174616949163,  1.0191152035320088, -1.5190473144732646,
     0.027960174616949163,  1.0191152035320088, -1.5190473144732646,
    -0.027960174616949163,  1.0191152035320088, -1.5190473144732646,
     0.027960174616949163,  1.0191152035320088, -1.5190473144732646,
], dtype=np.float32)
Q_DEFAULT_POLICY = Q_DEFAULT_WIRE[WIRE_TO_POLICY]
Q_DEFAULT = Q_DEFAULT_WIRE

# 第一版保守关节限位。请后续替换为你 URDF 里的精确 lower/upper。
Q_LOW_WIRE = np.array([
    -0.785398163, -0.785398163, -2.333505210,
    -1.570796327, -0.785398163, -2.333505210,
    -0.785398163, -0.785398163, -2.333505210,
    -1.570796327, -0.785398163, -2.333505210,
], dtype=np.float32)

Q_HIGH_WIRE = np.array([
     1.570796327,  1.570796327, -0.483456203,
     0.785398163,  1.570796327, -0.483456203,
     1.570796327,  1.570796327, -0.483456203,
     0.785398163,  1.570796327, -0.483456203,
], dtype=np.float32)
Q_LOW_POLICY = Q_LOW_WIRE[WIRE_TO_POLICY]
Q_HIGH_POLICY = Q_HIGH_WIRE[WIRE_TO_POLICY]

# 进一步缩小到关节范围的中间安全区。
SOFT_LIMIT_RATIO = 0.90
Q_CENTER = 0.5 * (Q_LOW_POLICY + Q_HIGH_POLICY)
Q_HALF_RANGE = 0.5 * (Q_HIGH_POLICY - Q_LOW_POLICY) * SOFT_LIMIT_RATIO
Q_LOW_SOFT = Q_CENTER - Q_HALF_RANGE
Q_HIGH_SOFT = Q_CENTER + Q_HALF_RANGE

# 每个 policy 周期 q_des 最大变化量。
DEFAULT_MAX_DQ_PER_STEP = 0.03

CMD_LOW = np.array([-0.3, -0.15, -0.5], dtype=np.float32)
CMD_HIGH = np.array([0.6, 0.15, 0.5], dtype=np.float32)
MAX_ABS_JOINT_VEL = 40.0

# 观测维度：
# base_lin_vel(3), base_ang_vel(3), projected_gravity(3), command(3),
# joint_pos_rel(12), joint_vel_rel(12), last_action(12), height_scan(187)
HEIGHT_SCAN_DIM = 187
HEIGHT_SCAN_VALUE = -0.13
OBS_DIM = 3 + 3 + 3 + 3 + 12 + 12 + 12 + HEIGHT_SCAN_DIM
ACTION_DIM = 12
MAX_ABS_ACTION = 8.0

# =========================
# 2. 串口协议
# =========================

# STM32 -> PC 状态包，小端，无 padding：
# uint16  head = 0xFEFE
# uint32  tick_ms
# uint8   mode
# uint8   fault
# float32 base_lin_vel[3]
# float32 base_ang_vel[3]
# float32 projected_gravity[3]
# float32 cmd[3]
# float32 joint_pos[12]
# float32 joint_vel[12]
# float32 battery_v
# uint16  crc16_ccitt_false over bytes before crc
STATE_HEAD = 0xFEFE
STATE_FMT_NOCRC = "<H I B B 3f 3f 3f 3f 12f 12f f"
STATE_STRUCT_NOCRC = struct.Struct(STATE_FMT_NOCRC)
STATE_STRUCT = struct.Struct(STATE_FMT_NOCRC + " H")
STATE_SIZE = STATE_STRUCT.size

# PC -> STM32 命令包，小端，无 padding：
# uint16  head = 0xA5A5
# uint32  tick_ms
# uint8   enable
# uint8   mode
# float32 q_des[12]
# uint16  crc16_ccitt_false over bytes before crc
CMD_HEAD = 0xA5A5
CMD_FMT_NOCRC = "<H I B B 12f"
CMD_STRUCT_NOCRC = struct.Struct(CMD_FMT_NOCRC)
CMD_STRUCT = struct.Struct(CMD_FMT_NOCRC + " H")
CMD_SIZE = CMD_STRUCT.size

MODE_STOP = 0
MODE_STAND = 1
MODE_WALK = 2


@dataclass
class RobotState:
    tick_ms: int
    mode: int
    fault: int
    base_lin_vel: np.ndarray
    base_ang_vel: np.ndarray
    projected_gravity: np.ndarray
    cmd: np.ndarray
    joint_pos: np.ndarray
    joint_vel: np.ndarray
    battery_v: float


# =========================
# 3. CRC16-CCITT-FALSE
# =========================

def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF."""
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


# =========================
# 4. 串口工具
# =========================

def list_serial_ports() -> None:
    if serial is None:
        print("缺少 pyserial：请先安装 `pip install pyserial`。")
        return
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("没有发现串口。检查 USB 转串口/驱动/线缆。")
        return

    print("发现以下串口：")
    for p in ports:
        print(f"  {p.device:>8} | {p.description} | HWID={p.hwid}")


def open_serial(port: str, baud: int, timeout: float = 0.02) -> serial.Serial:
    if serial is None:
        raise RuntimeError("缺少 pyserial，live 模式无法打开串口。请先安装 `pip install pyserial`。")
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
        write_timeout=timeout,
    )
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def read_state_packet(ser: serial.Serial) -> Optional[RobotState]:
    """从串口流中同步并读取一个状态包。读不到完整包则返回 None。"""
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b == b"\xFE":
            b2 = ser.read(1)
            if b2 == b"\xFE":
                rest = ser.read(STATE_SIZE - 2)
                if len(rest) != STATE_SIZE - 2:
                    return None
                packet = b + b2 + rest
                break

    payload = packet[:-2]
    recv_crc = struct.unpack_from("<H", packet, STATE_SIZE - 2)[0]
    calc_crc = crc16_ccitt_false(payload)
    if recv_crc != calc_crc:
        print(f"[WARN] 状态包 CRC 错误 recv=0x{recv_crc:04X}, calc=0x{calc_crc:04X}")
        return None

    values = STATE_STRUCT.unpack(packet)
    head = values[0]
    if head != STATE_HEAD:
        return None

    tick_ms = values[1]
    mode = values[2]
    fault = values[3]

    idx = 4
    base_lin_vel = np.array(values[idx:idx+3], dtype=np.float32); idx += 3
    base_ang_vel = np.array(values[idx:idx+3], dtype=np.float32); idx += 3
    projected_gravity = np.array(values[idx:idx+3], dtype=np.float32); idx += 3
    cmd = np.array(values[idx:idx+3], dtype=np.float32); idx += 3
    joint_pos = np.array(values[idx:idx+12], dtype=np.float32); idx += 12
    joint_vel = np.array(values[idx:idx+12], dtype=np.float32); idx += 12
    battery_v = float(values[idx])

    return RobotState(
        tick_ms=tick_ms,
        mode=mode,
        fault=fault,
        base_lin_vel=base_lin_vel,
        base_ang_vel=base_ang_vel,
        projected_gravity=projected_gravity,
        cmd=cmd,
        joint_pos=joint_pos,
        joint_vel=joint_vel,
        battery_v=battery_v,
    )


def pack_command_packet(q_des: np.ndarray, enable: int, mode: int, tick_ms: int) -> bytes:
    q_des = np.asarray(q_des, dtype=np.float32).reshape(12)
    payload = CMD_STRUCT_NOCRC.pack(
        CMD_HEAD,
        int(tick_ms) & 0xFFFFFFFF,
        int(enable) & 0xFF,
        int(mode) & 0xFF,
        *[float(x) for x in q_des],
    )
    crc = crc16_ccitt_false(payload)
    return payload + struct.pack("<H", crc)


# =========================
# 5. 策略推理与安全层
# =========================

def build_observation(state: RobotState, last_action: np.ndarray) -> np.ndarray:
    """复现训练时 observation 拼接顺序。"""
    joint_pos_policy = state.joint_pos.astype(np.float32)[WIRE_TO_POLICY]
    joint_vel_policy = state.joint_vel.astype(np.float32)[WIRE_TO_POLICY]
    joint_pos_rel = joint_pos_policy - Q_DEFAULT_POLICY
    joint_vel_rel = joint_vel_policy
    cmd = np.clip(state.cmd.astype(np.float32), CMD_LOW, CMD_HIGH)
    height_scan = np.full(HEIGHT_SCAN_DIM, HEIGHT_SCAN_VALUE, dtype=np.float32)

    obs = np.concatenate([
        state.base_lin_vel.astype(np.float32),
        state.base_ang_vel.astype(np.float32),
        state.projected_gravity.astype(np.float32),
        cmd,
        joint_pos_rel,
        joint_vel_rel,
        last_action.astype(np.float32),
        height_scan,
    ]).astype(np.float32)

    if obs.shape[0] != OBS_DIM:
        raise RuntimeError(f"obs dim mismatch: got {obs.shape[0]}, expected {OBS_DIM}")
    return obs


def sanitize_state(state: RobotState) -> bool:
    """基础状态检查。返回 False 表示不应继续使能。"""
    arrays = [
        state.base_lin_vel, state.base_ang_vel, state.projected_gravity,
        state.cmd, state.joint_pos, state.joint_vel,
    ]
    for arr in arrays:
        if not np.all(np.isfinite(arr)):
            print("[SAFE] 状态中出现 NaN/Inf")
            return False

    if state.fault != 0:
        print(f"[SAFE] STM32 fault != 0: {state.fault}")
        return False

    g = state.projected_gravity
    g_norm = np.linalg.norm(g)
    if g_norm < 0.7 or g_norm > 1.3:
        print("[SAFE] projected_gravity 异常")
        return False

    if abs(g[0]) > 0.65 or abs(g[1]) > 0.65:
        print("[SAFE] 机身倾角过大")
        return False

    joint_pos_policy = state.joint_pos.astype(np.float32)[WIRE_TO_POLICY]
    if np.any(joint_pos_policy < Q_LOW_POLICY - 0.05) or np.any(joint_pos_policy > Q_HIGH_POLICY + 0.05):
        print("[SAFE] 关节角超出硬限位附近")
        return False

    if np.max(np.abs(state.joint_vel.astype(np.float32))) > MAX_ABS_JOINT_VEL:
        print("[SAFE] 关节速度异常过大")
        return False

    return True


def sanitize_action(action: np.ndarray) -> bool:
    if not np.all(np.isfinite(action)):
        print("[SAFE] policy action 中出现 NaN/Inf")
        return False
    if np.max(np.abs(action)) > MAX_ABS_ACTION:
        print("[SAFE] policy action 绝对值异常过大")
        return False
    return True


def limit_joint_position(q: np.ndarray) -> np.ndarray:
    return np.clip(q, Q_LOW_SOFT, Q_HIGH_SOFT)


def limit_joint_rate(q_des: np.ndarray, q_prev: np.ndarray, max_dq: float) -> np.ndarray:
    dq = np.clip(q_des - q_prev, -max_dq, max_dq)
    return q_prev + dq


def action_to_q_des(action: np.ndarray, action_scale_real: float) -> np.ndarray:
    action = np.asarray(action, dtype=np.float32).reshape(12)
    q = Q_DEFAULT_POLICY + float(action_scale_real) * action
    return limit_joint_position(q)


def policy_to_wire_joints(q_policy: np.ndarray) -> np.ndarray:
    return np.asarray(q_policy, dtype=np.float32).reshape(12)[POLICY_TO_WIRE]


# =========================
# 6. dry-run / 主循环
# =========================

def make_fake_state(cmd: Tuple[float, float, float]) -> RobotState:
    return RobotState(
        tick_ms=0,
        mode=MODE_STAND,
        fault=0,
        base_lin_vel=np.zeros(3, dtype=np.float32),
        base_ang_vel=np.zeros(3, dtype=np.float32),
        projected_gravity=np.array([0.0, 0.0, -1.0], dtype=np.float32),
        cmd=np.array(cmd, dtype=np.float32),
        joint_pos=Q_DEFAULT_WIRE.copy(),
        joint_vel=np.zeros(12, dtype=np.float32),
        battery_v=24.0,
    )


def load_policy(policy_path: str, device: str):
    policy = torch.jit.load(policy_path, map_location=device)
    policy.eval()
    return policy


def infer_action(policy, obs: np.ndarray, device: str) -> np.ndarray:
    obs_t = torch.from_numpy(obs).to(device).unsqueeze(0)
    with torch.inference_mode():
        act_t = policy(obs_t)
    action = act_t.squeeze(0).detach().cpu().numpy().astype(np.float32)
    if action.shape[0] != ACTION_DIM:
        raise RuntimeError(f"action dim mismatch: got {action.shape[0]}, expected {ACTION_DIM}")
    return action


def run_dry(policy_path: str, device: str, action_scale_real: float, cmd: Tuple[float, float, float]) -> None:
    print("[DRY] 加载 policy:", policy_path)
    policy = load_policy(policy_path, device)
    last_action = np.zeros(12, dtype=np.float32)
    q_prev = Q_DEFAULT_POLICY.copy()

    print(f"[DRY] 输入是假状态：水平站立、关节在默认站姿、height_scan={HEIGHT_SCAN_VALUE}")
    for i in range(20):
        state = make_fake_state(cmd)
        obs = build_observation(state, last_action)
        action = infer_action(policy, obs, device)
        q_des = action_to_q_des(action, action_scale_real)
        q_des = limit_joint_rate(q_des, q_prev, DEFAULT_MAX_DQ_PER_STEP)
        q_prev = q_des
        last_action = action.copy()

        print(f"\n[DRY {i:02d}] action min/max=({action.min():+.3f}, {action.max():+.3f})")
        for name, q in zip(POLICY_JOINT_NAMES, q_des):
            print(f"  {name:6s}: {q:+.4f} rad")
        time.sleep(POLICY_DT)


def run_live(args) -> None:
    policy = load_policy(args.policy, args.device)
    ser = open_serial(args.port, args.baud)

    print(f"[LIVE] port={args.port}, baud={args.baud}")
    print(f"[LIVE] policy={args.policy}, device={args.device}")
    print(f"[LIVE] no_send={args.no_send}, enable={args.enable}, mode={args.mode}")
    print(f"[LIVE] action_scale_real={args.action_scale}, max_dq_per_step={args.max_dq}")
    print(f"[LIVE] height_scan_value={HEIGHT_SCAN_VALUE}")
    print("[LIVE] Ctrl+C 退出。")

    last_action = np.zeros(12, dtype=np.float32)
    q_prev = Q_DEFAULT_POLICY.copy()
    q_prev_initialized = False

    last_print = time.monotonic()
    last_state_time = time.monotonic()
    loop_count = 0

    try:
        while True:
            t0 = time.monotonic()
            state = read_state_packet(ser)
            if state is None:
                if time.monotonic() - last_state_time > args.state_timeout:
                    print("[SAFE] 状态包超时，发送 disable/stand")
                    pkt = pack_command_packet(
                        Q_DEFAULT_WIRE, enable=0, mode=MODE_STAND, tick_ms=int(time.time() * 1000)
                    )
                    if not args.no_send:
                        ser.write(pkt)
                    last_state_time = time.monotonic()
                    q_prev_initialized = False
                continue

            last_state_time = time.monotonic()

            safe = sanitize_state(state)
            if not safe:
                q_policy = Q_DEFAULT_POLICY.copy()
                enable = 0
                mode = MODE_STAND
                action = np.zeros(12, dtype=np.float32)
                q_prev_initialized = False
            else:
                if not q_prev_initialized:
                    q_prev = np.clip(state.joint_pos.astype(np.float32)[WIRE_TO_POLICY], Q_LOW_POLICY, Q_HIGH_POLICY)
                    q_prev_initialized = True
                obs = build_observation(state, last_action)
                action = infer_action(policy, obs, args.device)
                if sanitize_action(action):
                    q_des = action_to_q_des(action, args.action_scale)
                    q_policy = limit_joint_rate(q_des, q_prev, args.max_dq)
                    enable = 1 if args.enable else 0
                    mode = args.mode
                else:
                    q_policy = Q_DEFAULT_POLICY.copy()
                    enable = 0
                    mode = MODE_STAND
                    action = np.zeros(12, dtype=np.float32)
                    q_prev_initialized = False

            q_prev = q_policy.copy()
            last_action = action.copy()
            q_send = policy_to_wire_joints(q_policy)

            pkt = pack_command_packet(q_send, enable=enable, mode=mode, tick_ms=int(time.time() * 1000))
            if not args.no_send:
                ser.write(pkt)

            loop_count += 1
            now = time.monotonic()
            if now - last_print > 1.0:
                freq = loop_count / (now - last_print)
                last_print = now
                loop_count = 0
                print(
                    f"[LIVE] freq={freq:5.1f} Hz | "
                    f"tick={state.tick_ms} | mode={state.mode} fault={state.fault} | "
                    f"cmd=({state.cmd[0]:+.2f},{state.cmd[1]:+.2f},{state.cmd[2]:+.2f}) | "
                    f"act=({last_action.min():+.2f},{last_action.max():+.2f}) | "
                    f"q=({q_send.min():+.2f},{q_send.max():+.2f}) | "
                    f"bat={state.battery_v:.1f}V"
                )
                print("action =", np.array2string(last_action, precision=3, suppress_small=True))
                print("q_policy =", np.array2string(q_policy, precision=3, suppress_small=True))
                print("q_send_wire =", np.array2string(q_send, precision=3, suppress_small=True))
                print("state.cmd =", state.cmd)
                print("base_lin_vel =", state.base_lin_vel)
                print("projected_gravity =", state.projected_gravity)

            dt = time.monotonic() - t0
            sleep_t = max(0.0, POLICY_DT - dt)
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        if args.no_send:
            print("\n[LIVE] Ctrl+C，no_send=True，不向 STM32 发送 disable 包。")
        else:
            print("\n[LIVE] Ctrl+C，发送 disable 默认站姿。")
            pkt = pack_command_packet(
                Q_DEFAULT_WIRE,
                enable=0,
                mode=MODE_STAND,
                tick_ms=int(time.time() * 1000)
            )
            try:
                ser.write(pkt)
            except Exception:
                pass


def parse_args():
    parser = argparse.ArgumentParser(description="Deploy MyQuad Isaac Lab policy to STM32 over serial.")

    parser.add_argument("--list-ports", action="store_true", help="列出当前可用串口后退出")
    parser.add_argument("--policy", type=str, default="policy.pt", help="Isaac Lab 导出的 exported/policy.pt 路径")
    parser.add_argument("--device", type=str, default="cpu", choices=["cpu", "cuda"], help="PyTorch 推理设备")

    parser.add_argument("--port", type=str, default=None, help="串口名，例如 COM6 或 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=921600, help="串口波特率，需和 STM32 一致")

    parser.add_argument("--dry-run", action="store_true", help="不连接串口，用假状态测试 policy 推理")
    parser.add_argument("--no-send", action="store_true", help="连接串口并读取状态，但不向 STM32 下发命令")
    parser.add_argument("--enable", action="store_true", help="真的使能下发。首次测试不要加这个参数")
    parser.add_argument("--mode", type=int, default=MODE_WALK, help="下发给 STM32 的 mode，默认 WALK=2")

    parser.add_argument("--action-scale", type=float, default=0.15, help="真机 action scale，首次建议 0.10~0.15；确认安全后再升到 0.3")
    parser.add_argument("--max-dq", type=float, default=DEFAULT_MAX_DQ_PER_STEP, help="每个 20ms 周期 q_des 最大变化，rad")
    parser.add_argument("--state-timeout", type=float, default=0.10, help="多久没收到状态包就发送 disable，秒")

    parser.add_argument("--cmd-vx", type=float, default=0.0, help="dry-run 假命令 vx")
    parser.add_argument("--cmd-vy", type=float, default=0.0, help="dry-run 假命令 vy")
    parser.add_argument("--cmd-yaw", type=float, default=0.0, help="dry-run 假命令 yaw_rate")

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.list_ports:
        list_serial_ports()
        return 0

    if args.dry_run:
        run_dry(
            policy_path=args.policy,
            device=args.device,
            action_scale_real=args.action_scale,
            cmd=(args.cmd_vx, args.cmd_vy, args.cmd_yaw),
        )
        return 0

    if args.port is None:
        print("错误：live 模式必须指定 --port。先运行：python deploy_myquad_policy.py --list-ports")
        return 2

    run_live(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
