#ifndef PC_COMM_UART10_H
#define PC_COMM_UART10_H

#include <stdint.h>
#include "robot_map.h"

/*
 * UART10 上位机通信模块（PC policy 接管接口）。
 * 作用：
 * 1) 下行接收关节指令（JointCommandPacket）；
 * 2) 上行发送机器人状态（RobotStatePacket）。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* 帧头与周期/超时配置（单位：ms）。 */
#define PC_COMM_STATE_HEAD 0xFEFEU
#define PC_COMM_COMMAND_HEAD 0xA5A5U
#define PC_COMM_STATE_PERIOD_MS 20U
#define PC_COMM_COMMAND_TIMEOUT_MS 100U

#pragma pack(push, 1)
/*
 * 机器人状态包（发送给 PC）。
 * head: 帧头固定值 PC_COMM_STATE_HEAD，用于帧同步。
 * tick_ms: 本地系统时间戳，来自 HAL_GetTick()，单位 ms。
 * mode: 当前控制模式（与 App_GetControlMode() 对应）。
 * fault: 当前故障码（急停/姿态超限等），供 PC 侧提示。
 * base_lin_vel[3]: 机体坐标系线速度 x/y/z（m/s），当前为占位。
 * base_ang_vel[3]: 机体坐标系角速度 roll/pitch/yaw（rad/s），由 IMU 陀螺换算。
 * projected_gravity[3]: 重力向量投影到机体坐标系的结果，供姿态估计/控制使用。
 * cmd[3]: 当前速度指令（vx/vy/偏航角速度），暂为占位。
 * joint_pos[J_NUM]: URDF 顺序的关节角（rad）。
 * joint_vel[J_NUM]: URDF 顺序的关节角速度（rad/s）。
 * battery_v: 电池电压（V）。
 * crc: CRC16-CCITT(FALSE)，计算范围为该字段之前的所有字节。
 */
typedef struct
{
    uint16_t head;
    uint32_t tick_ms;
    uint8_t mode;
    uint8_t fault;

    float base_lin_vel[3];
    float base_ang_vel[3];
    float projected_gravity[3];
    float cmd[3];

    float joint_pos[J_NUM];
    float joint_vel[J_NUM];

    float battery_v;
    uint16_t crc;
} RobotStatePacket;

/*
 * 关节指令包（PC -> MCU）。
 * head: 帧头固定值 PC_COMM_COMMAND_HEAD，用于帧同步。
 * tick_ms: PC 侧时间戳（ms），用于诊断与延迟评估。
 * enable: 1 表示允许 policy 接管，0 表示仅监视不下发。
 * mode: policy 模式选择（由上位机定义，目前只接受 0~2）。
 * q_des[J_NUM]: URDF 顺序的目标关节角（rad）。
 * crc: CRC16-CCITT(FALSE)，计算范围为该字段之前的所有字节。
 */
typedef struct
{
    uint16_t head;
    uint32_t tick_ms;
    uint8_t enable;
    uint8_t mode;

    float q_des[J_NUM];

    uint16_t crc;
} JointCommandPacket;
#pragma pack(pop)

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
#define PC_COMM_STATIC_ASSERT(expr, msg) _Static_assert((expr), msg)
#else
#define PC_COMM_STATIC_ASSERT_JOIN_(a, b) a##b
#define PC_COMM_STATIC_ASSERT_JOIN(a, b) PC_COMM_STATIC_ASSERT_JOIN_(a, b)
#define PC_COMM_STATIC_ASSERT(expr, msg) \
    typedef char PC_COMM_STATIC_ASSERT_JOIN(pc_comm_static_assert_, __LINE__)[(expr) ? 1 : -1]
#endif

/* 固定线协议长度，避免编译器填充导致尺寸变化。 */
PC_COMM_STATIC_ASSERT(sizeof(RobotStatePacket) == 158U, "RobotStatePacket size mismatch");
PC_COMM_STATIC_ASSERT(sizeof(JointCommandPacket) == 58U, "JointCommandPacket size mismatch");

/* 计算 CRC16-CCITT(FALSE)，用于串口包校验。 */
uint16_t PCComm_CRC16_CCITT_FALSE(const uint8_t *data, uint16_t len);

/* 初始化通信模块内部状态、滤波值与安全标志位。 */
void PCComm_Init(void);
/* 启动 UART10 的 1 字节中断接收（后续在回调里持续重启）。 */
void PCComm_StartReceive(void);
/* 1ms 周期任务：更新待处理命令、姿态安全与目标滤波。 */
void PCComm_Task1ms(void);
/* 每 20ms 发送一次状态包（若 UART 空闲）。 */
void PCComm_SendState20ms(void);

/* 判断是否在超时窗口内收到有效指令。 */
uint8_t PCComm_IsCommandFresh(void);
/* 返回最近一次指令的“年龄”(ms)，无指令时返回 0xFFFFFFFF。 */
uint32_t PCComm_GetLastCommandAgeMs(void);
/* 读取最新指令的 enable 标志。 */
uint8_t PCComm_GetEnable(void);
/* 读取最新指令的 mode 标志。 */
uint8_t PCComm_GetMode(void);
/* 读取当前故障码。 */
uint8_t PCComm_GetFault(void);
/* 判断当前是否允许 PC policy 接管控制。 */
uint8_t PCComm_IsPolicyControlAllowed(void);
/* 复制滤波后的目标关节角（URDF 顺序）到输出数组。 */
void PCComm_GetQDesUrdf(float q_des_out[J_NUM]);
/* 复制最新 PC 原始目标关节角（URDF 顺序）到输出数组。 */
void PCComm_GetLatestQDesUrdf(float q_des_out[J_NUM]);

/* 串口逐字节解析入口：喂入 1 个接收字节。 */
void PCComm_OnUart10RxByte(uint8_t byte);
/* UART10 接收完成回调：解析该字节并重新挂起接收。 */
void PCComm_OnUart10RxCplt(void);
/* UART10 发送完成回调：清除 busy 标志。 */
void PCComm_OnUart10TxCplt(void);
/* 急停接口：禁止 policy 接管并锁存故障。 */
void PCComm_EStop(void);

/* 调试用速度命令缓存，由 Keil Watch/上位机状态包共用。 */
extern float volecity_cmd[3];

#ifdef __cplusplus
}
#endif

#endif
