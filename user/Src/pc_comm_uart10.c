#include "pc_comm_uart10.h"

#include "app_robot.h"
#include "imu.h"
#include "usart.h"
#include <math.h>
#include <string.h>

/*
 * UART10 上位机通信模块（PC policy 接管）：
 * 1) RX：接收 JointCommandPacket，校验帧头与 CRC；
 * 2) TX：每 20ms 发送 RobotStatePacket；
 * 3) 安全门限：指令超时、急停、姿态角限制、关节角限幅。
 */

/* 串口接收状态机：用于识别 A5A5 帧头并拼接完整指令帧。 */
#define PC_COMM_RX_WAIT_HEAD1 0U
#define PC_COMM_RX_WAIT_HEAD2 1U
#define PC_COMM_RX_READ_FRAME 2U

/* 关节目标滤波与安全阈值参数。 */
#define PC_COMM_MAX_DQ_PER_1MS 0.0015f
#define PC_COMM_ATTITUDE_LIMIT_DEG 30.0f
#define PC_COMM_DEG_TO_RAD 0.01745329251994329577f
#define PC_COMM_FAULT_NONE 0U
#define PC_COMM_FAULT_ESTOP 1U
#define PC_COMM_FAULT_ATTITUDE 2U
/* 调试开关：ASCII hello 测试 / 假状态数据测试。 */
#define PC_COMM_ASCII_HELLO_TEST 0U
#define PC_COMM_FAKE_STATE_TEST 0U

/* 关节角安全下限（URDF 顺序，单位 rad）。 */
static const float g_joint_limit_low[J_NUM] = {
    -1.2f, -1.2f, -2.3f,
    -1.2f, -1.2f, -2.3f,
    -1.2f, -1.2f, -2.3f,
    -1.2f, -1.2f, -2.3f,
};

/* 关节角安全上限（URDF 顺序，单位 rad）。 */
static const float g_joint_limit_high[J_NUM] = {
    1.2f, 1.5f, -0.5f,
    1.2f, 1.5f, -0.5f,
    1.2f, 1.5f, -0.5f,
    1.2f, 1.5f, -0.5f,
};

/*
 * UART10 逐字节解析缓存：
 * g_rx_byte: HAL_UART_Receive_IT 的单字节缓冲区；
 * g_rx_state: 当前解析状态（等待帧头/读取帧体）；
 * g_rx_frame: 临时帧缓存，满帧后再做 CRC 校验；
 * g_rx_index: 当前写入位置。
 */
static uint8_t g_rx_byte;
static uint8_t g_rx_state = PC_COMM_RX_WAIT_HEAD1;
static uint8_t g_rx_frame[sizeof(JointCommandPacket)];
static uint16_t g_rx_index = 0U;

/*
 * 指令缓存：
 * g_pending_command: ISR 中接收并通过校验的最新指令（待转交）；
 * g_latest_command: 任务态真正生效的最新指令；
 * g_pending_ready: ISR -> 任务态的“新指令就绪”标志；
 * g_pending_tick_ms: 指令到达时间戳；
 * g_last_command_tick_ms: 最后一次生效指令的时间戳。
 */
static JointCommandPacket g_latest_command;
static JointCommandPacket g_pending_command;
static volatile uint8_t g_pending_ready = 0U;
static volatile uint32_t g_pending_tick_ms = 0U;
static uint32_t g_last_command_tick_ms = 0U;

/*
 * 目标关节与安全状态：
 * g_q_des_filtered: 经过限速滤波后的目标关节角；
 * g_estop: 急停锁存标志；
 * g_attitude_safe: 姿态是否安全（roll/pitch 未超限）；
 * g_fault_code: 当前故障码（急停/姿态超限等）。
 */
static float g_q_des_filtered[J_NUM];
static uint8_t g_estop = 0U;
static uint8_t g_attitude_safe = 1U;
static uint8_t g_fault_code = PC_COMM_FAULT_NONE;

/*
 * 状态包发送控制：
 * g_tx_packet: 待发送的状态包；
 * g_tx_busy: DMA 发送中标志，防止重入；
 * g_last_state_tx_tick_ms: 上次发送的时间戳。
 */
static RobotStatePacket g_tx_packet;
static volatile uint8_t g_tx_busy = 0U;
static uint32_t g_last_state_tx_tick_ms = 0U;

/* 快速绝对值：仅处理 float，用于姿态安全判断。 */
static float pccomm_absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

/* 浮点限幅：把输入限制在 [low, high] 区间。 */
static float pccomm_clampf(float x, float low, float high)
{
    if (x < low)
    {
        return low;
    }
    if (x > high)
    {
        return high;
    }
    return x;
}

/*
 * 简易有限性检查：
 * - 拒绝 NaN/Inf；
 * - 拒绝过大的异常值，避免污染控制链路。
 */
static uint8_t pccomm_isfinite_float(float x)
{
    return ((x == x) && (x <= 1000000.0f) && (x >= -1000000.0f)) ? 1U : 0U;
}

/* 解析出错时重置状态机，回到“等待帧头”。 */
static void pccomm_reset_parser(void)
{
    g_rx_state = PC_COMM_RX_WAIT_HEAD1;
    g_rx_index = 0U;
}

/* 校验 policy 模式范围（当前仅允许 0~2）。 */
static uint8_t pccomm_is_mode_valid(uint8_t mode)
{
    return (mode <= 2U) ? 1U : 0U;
}

/*
 * 指令包校验流程：
 * 1) 指针非空；
 * 2) 帧头匹配；
 * 3) CRC 校验通过；
 * 4) enable/mode 合法；
 * 5) 关节角有限且在安全范围内。
 */
static uint8_t pccomm_validate_command(const JointCommandPacket *pkt)
{
    uint16_t crc_calc;
    uint8_t i;

    if (pkt == 0)
    {
        return 0U;
    }

    if (pkt->head != PC_COMM_COMMAND_HEAD)
    {
        return 0U;
    }

    crc_calc = PCComm_CRC16_CCITT_FALSE((const uint8_t *)pkt,
                                        (uint16_t)(sizeof(JointCommandPacket) - sizeof(uint16_t)));
    if (crc_calc != pkt->crc)
    {
        return 0U;
    }

    if (pkt->enable > 1U)
    {
        return 0U;
    }
    if (pccomm_is_mode_valid(pkt->mode) == 0U)
    {
        return 0U;
    }

    for (i = 0U; i < (uint8_t)J_NUM; i++)
    {
        float q = pkt->q_des[i];
        if (pccomm_isfinite_float(q) == 0U)
        {
            return 0U;
        }
        if ((pkt->enable != 0U) &&
            ((q < g_joint_limit_low[i]) || (q > g_joint_limit_high[i])))
        {
            return 0U;
        }
    }

    return 1U;
}

/*
 * ISR 中接收到合法指令后调用：
 * 写入 pending 缓冲，并记录时间戳。
 * 若已急停则直接忽略。
 */
static void pccomm_accept_command_from_isr(const JointCommandPacket *pkt)
{
    if (g_estop != 0U)
    {
        return;
    }

    g_pending_command = *pkt;
    g_pending_tick_ms = HAL_GetTick();
    g_pending_ready = 1U;
}

/*
 * 将 pending 指令转移为 latest 指令。
 * 运行在任务态，使用关中断保护与 ISR 的并发写入。
 */
static void pccomm_take_pending_command(void)
{
    JointCommandPacket cmd_local;
    uint32_t tick_local;
    uint32_t primask;

    if (g_pending_ready == 0U)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (g_pending_ready == 0U)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return;
    }
    cmd_local = g_pending_command;
    tick_local = g_pending_tick_ms;
    g_pending_ready = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    g_latest_command = cmd_local;
    g_last_command_tick_ms = tick_local;
}

/*
 * 更新姿态安全状态：
 * - 读取 IMU 的 roll/pitch；
 * - 超过阈值则置为不安全，并更新故障码。
 */
static void pccomm_update_attitude_safety(void)
{
    IMU_Body *imu = imu_get_body_data();
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;

    if (imu != 0)
    {
        roll_deg = imu->angle[0];
        pitch_deg = imu->angle[1];
    }

    if ((pccomm_absf(roll_deg) > PC_COMM_ATTITUDE_LIMIT_DEG) ||
        (pccomm_absf(pitch_deg) > PC_COMM_ATTITUDE_LIMIT_DEG))
    {
        g_attitude_safe = 0U;
        if (g_estop == 0U)
        {
            g_fault_code = PC_COMM_FAULT_ATTITUDE;
        }
        return;
    }

    g_attitude_safe = 1U;
    if (g_estop == 0U)
    {
        g_fault_code = PC_COMM_FAULT_NONE;
    }
}

/*
 * 关节目标限速：
 * 先做关节角限幅，再限制每 1ms 的最大变化量，
 * 以此平滑 PC 下发的目标轨迹。
 */
static void pccomm_rate_limit_qdes(const float target[J_NUM])
{
    uint8_t i;

    for (i = 0U; i < (uint8_t)J_NUM; i++)
    {
        float clipped = pccomm_clampf(target[i], g_joint_limit_low[i], g_joint_limit_high[i]);
        float delta = clipped - g_q_des_filtered[i];

        if (delta > PC_COMM_MAX_DQ_PER_1MS)
        {
            delta = PC_COMM_MAX_DQ_PER_1MS;
        }
        else if (delta < -PC_COMM_MAX_DQ_PER_1MS)
        {
            delta = -PC_COMM_MAX_DQ_PER_1MS;
        }

        g_q_des_filtered[i] += delta;
    }
}

/* 根据 roll/pitch 计算机体坐标系下的重力投影向量。 */
static void pccomm_fill_projected_gravity(float projected_gravity[3])
{
    IMU_Body *imu = imu_get_body_data();
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float sr;
    float cr;
    float sp;
    float cp;

    if (imu != 0)
    {
        roll_rad = imu->angle[0] * PC_COMM_DEG_TO_RAD;
        pitch_rad = imu->angle[1] * PC_COMM_DEG_TO_RAD;
    }

    sr = sinf(roll_rad);
    cr = cosf(roll_rad);
    sp = sinf(pitch_rad);
    cp = cosf(pitch_rad);

    projected_gravity[0] = sp;
    projected_gravity[1] = -sr * cp;
    projected_gravity[2] = -cr * cp;
}

/* 填充状态包内容并计算 CRC。 */
static void pccomm_fill_state_packet(RobotStatePacket *pkt)
{
    IMU_Body *imu = imu_get_body_data();
    uint8_t i;

    memset(pkt, 0, sizeof(*pkt));

    pkt->head = PC_COMM_STATE_HEAD;
    pkt->tick_ms = HAL_GetTick();
    pkt->mode = (uint8_t)App_GetControlMode();
    pkt->fault = g_fault_code;

    /* TODO: 后续替换为机体坐标系线速度估计值。 */
    pkt->base_lin_vel[0] = 0.0f;
    pkt->base_lin_vel[1] = 0.0f;
    pkt->base_lin_vel[2] = 0.0f;

    if (imu != 0)
    {
        pkt->base_ang_vel[0] = imu->gyro[0] * PC_COMM_DEG_TO_RAD;
        pkt->base_ang_vel[1] = imu->gyro[1] * PC_COMM_DEG_TO_RAD;
        pkt->base_ang_vel[2] = imu->gyro[2] * PC_COMM_DEG_TO_RAD;
    }

    pccomm_fill_projected_gravity(pkt->projected_gravity);

    /* TODO: 后续补充当前有效的 vx/vy/偏航角速度指令。 */
    pkt->cmd[0] = 0.0f;
    pkt->cmd[1] = 0.0f;
    pkt->cmd[2] = 0.0f;

#if PC_COMM_FAKE_STATE_TEST
    for (i = 0U; i < (uint8_t)J_NUM; i++)
    {
        pkt->joint_pos[i] = g_joint_default_stand_rad[i];
        pkt->joint_vel[i] = 0.0f;
    }
    pkt->battery_v = 24.0f;
    pkt->crc = PCComm_CRC16_CCITT_FALSE((const uint8_t *)pkt,
                                        (uint16_t)(sizeof(RobotStatePacket) - sizeof(uint16_t)));
    return;
#endif

    App_Get_Model_Joint_Angles(pkt->joint_pos);
    App_Get_Model_Joint_Velocities(pkt->joint_vel);

    for (i = 0U; i < (uint8_t)J_NUM; i++)
    {
        if (pccomm_isfinite_float(pkt->joint_pos[i]) == 0U)
        {
            pkt->joint_pos[i] = g_joint_default_stand_rad[i];
        }
        if (pccomm_isfinite_float(pkt->joint_vel[i]) == 0U)
        {
            pkt->joint_vel[i] = 0.0f;
        }
    }

    /* TODO: 后续替换为 ADC/BMS 的真实电池电压。 */
    pkt->battery_v = 24.0f;
    pkt->crc = PCComm_CRC16_CCITT_FALSE((const uint8_t *)pkt,
                                        (uint16_t)(sizeof(RobotStatePacket) - sizeof(uint16_t)));
}

/* CRC16-CCITT(FALSE) 标准实现，用于串口帧校验。 */
uint16_t PCComm_CRC16_CCITT_FALSE(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;

    if (data == 0)
    {
        return crc;
    }

    while (len-- != 0U)
    {
        crc ^= (uint16_t)(*data++) << 8;
        for (i = 0U; i < 8U; i++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    return crc;
}

/* 初始化通信模块状态：清零指令、滤波、时间戳与安全标志。 */
void PCComm_Init(void)
{
    uint8_t i;

    memset(&g_latest_command, 0, sizeof(g_latest_command));
    memset(&g_pending_command, 0, sizeof(g_pending_command));
    pccomm_reset_parser();

    g_latest_command.head = PC_COMM_COMMAND_HEAD;
    g_latest_command.enable = 0U;
    g_latest_command.mode = 0U;

    for (i = 0U; i < (uint8_t)J_NUM; i++)
    {
        g_latest_command.q_des[i] = g_joint_default_stand_rad[i];
        g_q_des_filtered[i] = g_joint_default_stand_rad[i];
    }

    g_pending_ready = 0U;
    g_pending_tick_ms = 0U;
    g_last_command_tick_ms = 0U;
    g_last_state_tx_tick_ms = 0U;
    g_tx_busy = 0U;
    g_estop = 0U;
    g_attitude_safe = 1U;
    g_fault_code = PC_COMM_FAULT_NONE;
}

/* 启动 UART10 1 字节中断接收（回调里会持续重启）。 */
void PCComm_StartReceive(void)
{
    (void)HAL_UART_Receive_IT(&huart10, &g_rx_byte, 1U);
}

/* 1ms 周期入口：更新 pending 指令、姿态安全与目标滤波。 */
void PCComm_Task1ms(void)
{
    pccomm_take_pending_command();
    pccomm_update_attitude_safety();

    if (PCComm_IsPolicyControlAllowed() != 0U)
    {
        pccomm_rate_limit_qdes(g_latest_command.q_des);
    }
    else
    {
        pccomm_rate_limit_qdes(g_joint_default_stand_rad);
    }
}

/* 若到达发送周期且 UART 空闲，则发送状态包。 */
void PCComm_SendState20ms(void)
{
    uint32_t now = HAL_GetTick();

#if PC_COMM_ASCII_HELLO_TEST
    static const uint8_t hello[] = "hello stm32\r\n";

    if ((now - g_last_state_tx_tick_ms) < 100U)
    {
        return;
    }
    if (g_tx_busy != 0U)
    {
        return;
    }

    g_tx_busy = 1U;
    if (HAL_UART_Transmit_DMA(&huart10, (uint8_t *)hello, (uint16_t)(sizeof(hello) - 1U)) == HAL_OK)
    {
        g_last_state_tx_tick_ms = now;
    }
    else
    {
        g_tx_busy = 0U;
    }
    return;
#endif

    if ((now - g_last_state_tx_tick_ms) < PC_COMM_STATE_PERIOD_MS)
    {
        return;
    }
    if (g_tx_busy != 0U)
    {
        return;
    }

    pccomm_fill_state_packet(&g_tx_packet);
    g_tx_busy = 1U;
    if (HAL_UART_Transmit_DMA(&huart10, (uint8_t *)&g_tx_packet, sizeof(g_tx_packet)) == HAL_OK)
    {
        g_last_state_tx_tick_ms = now;
    }
    else
    {
        g_tx_busy = 0U;
    }
}

/* 判断指令是否在超时窗口内到达（无指令则返回 0）。 */
uint8_t PCComm_IsCommandFresh(void)
{
    if (g_last_command_tick_ms == 0U)
    {
        return 0U;
    }

    return ((HAL_GetTick() - g_last_command_tick_ms) <= PC_COMM_COMMAND_TIMEOUT_MS) ? 1U : 0U;
}

/* 返回最后一次指令的时间差（ms），无指令返回 0xFFFFFFFF。 */
uint32_t PCComm_GetLastCommandAgeMs(void)
{
    if (g_last_command_tick_ms == 0U)
    {
        return 0xFFFFFFFFU;
    }

    return HAL_GetTick() - g_last_command_tick_ms;
}

/* 读取最新指令的 enable 标志。 */
uint8_t PCComm_GetEnable(void)
{
    return g_latest_command.enable;
}

/* 读取最新指令的 mode 标志。 */
uint8_t PCComm_GetMode(void)
{
    return g_latest_command.mode;
}

/* 读取当前故障码。 */
uint8_t PCComm_GetFault(void)
{
    return g_fault_code;
}

/* 判断是否允许 PC policy 接管控制（多重安全门限）。 */
uint8_t PCComm_IsPolicyControlAllowed(void)
{
    if (g_estop != 0U)
    {
        return 0U;
    }
    if (g_fault_code != PC_COMM_FAULT_NONE)
    {
        return 0U;
    }
    if (g_attitude_safe == 0U)
    {
        return 0U;
    }
    if (PCComm_IsCommandFresh() == 0U)
    {
        return 0U;
    }
    if (g_latest_command.enable != 1U)
    {
        return 0U;
    }
    if ((g_latest_command.mode != 1U) && (g_latest_command.mode != 2U))
    {
        return 0U;
    }

    return 1U;
}

/* 复制滤波后的目标关节角到输出数组（URDF 顺序）。 */
void PCComm_GetQDesUrdf(float q_des_out[J_NUM])
{
    uint8_t i;

    if (q_des_out == 0)
    {
        return;
    }

    for (i = 0U; i < (uint8_t)J_NUM; i++)
    {
        q_des_out[i] = g_q_des_filtered[i];
    }
}

/* UART10 逐字节指令解析（A5A5 帧头 + 固定长度帧体）。 */
void PCComm_OnUart10RxByte(uint8_t byte)
{
    JointCommandPacket pkt;

    switch (g_rx_state)
    {
    case PC_COMM_RX_WAIT_HEAD1:
        if (byte == 0xA5U)
        {
            g_rx_frame[0] = byte;
            g_rx_index = 1U;
            g_rx_state = PC_COMM_RX_WAIT_HEAD2;
        }
        break;

    case PC_COMM_RX_WAIT_HEAD2:
        if (byte == 0xA5U)
        {
            g_rx_frame[1] = byte;
            g_rx_index = 2U;
            g_rx_state = PC_COMM_RX_READ_FRAME;
        }
        else
        {
            pccomm_reset_parser();
        }
        break;

    case PC_COMM_RX_READ_FRAME:
        g_rx_frame[g_rx_index++] = byte;
        if (g_rx_index >= sizeof(JointCommandPacket))
        {
            memcpy(&pkt, g_rx_frame, sizeof(pkt));
            if (pccomm_validate_command(&pkt) != 0U)
            {
                pccomm_accept_command_from_isr(&pkt);
            }
            pccomm_reset_parser();
        }
        break;

    default:
        pccomm_reset_parser();
        break;
    }
}

/* UART10 接收完成回调：解析该字节并重新挂起接收。 */
void PCComm_OnUart10RxCplt(void)
{
    PCComm_OnUart10RxByte(g_rx_byte);
    PCComm_StartReceive();
}

/* UART10 发送完成回调：释放 busy 标志。 */
void PCComm_OnUart10TxCplt(void)
{
    g_tx_busy = 0U;
}

/* 急停：锁存故障并禁止 policy 接管。 */
void PCComm_EStop(void)
{
    g_estop = 1U;
    g_fault_code = PC_COMM_FAULT_ESTOP;
    g_latest_command.enable = 0U;
}
