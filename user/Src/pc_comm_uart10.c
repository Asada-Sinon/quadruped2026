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
#define PC_COMM_LPF_ALPHA 0.03f  /* 一阶低通 α，fc≈4.8Hz @1ms，对 ±0.05rad 噪声衰减约 10x */
#define PC_COMM_ATTITUDE_LIMIT_DEG 30.0f
#define PC_COMM_DEG_TO_RAD 0.01745329251994329577f
#define PC_COMM_FAULT_NONE 0U
#define PC_COMM_FAULT_ESTOP 1U
#define PC_COMM_FAULT_ATTITUDE 2U
/* 调试开关：ASCII hello 测试 / 假状态数据测试。 */
#define PC_COMM_ASCII_HELLO_TEST 0U
#define PC_COMM_FAKE_STATE_TEST 0U
float volecity_cmd[3] = {0.0f, 0.0f, 0.0f};
/*
 * PC 侧命令的运行时缓存。
 *
 * 注意：
 * JointCommandPacket 是 UART 线协议结构体，受 pc_comm_uart10.h 中
 * #pragma pack(1) 约束，只能用于收包、帧头检查、CRC 校验和逐字段解析。
 *
 * 运行时控制逻辑不能直接保存 JointCommandPacket，也不能把
 * JointCommandPacket.q_des 作为普通 float* 传入控制函数。packed 结构体
 * 内部的 float 数组可能不是 4 字节自然对齐，在 STM32H7 / Cortex-M 上
 * 可能因为非对齐 float 访问触发 UsageFault/HardFault，表现为 enable=1
 * 后状态包停止发送。
 *
 * 因此这里定义自然对齐的运行时结构体。ISR 收到合法线协议包后，只把
 * tick_ms、enable、mode 和 q_des[] 逐字段复制进来；后续 1ms 控制任务
 * 只能读取这个自然对齐缓存中的 q_des。
 */
typedef struct
{
    uint32_t tick_ms;
    uint8_t enable;
    uint8_t mode;
    float q_des[J_NUM];
} PCCommRuntimeCommand;

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
 * g_last_command_tick_ms: 最后一次生效指令的时间戳；
 * g_command_ever_received: 是否曾收到过合法 PC 指令（ISR 中置 1，初始化清零）。
 */
PCCommRuntimeCommand g_latest_command;//这个是电脑发过来解析后的原始数据
static PCCommRuntimeCommand g_pending_command;
static volatile uint8_t g_pending_ready = 0U;
static volatile uint32_t g_pending_tick_ms = 0U;
static uint32_t g_last_command_tick_ms = 0U;
static uint8_t g_command_ever_received = 0U;

/*
 * PC 命令接收调试计数：ISR 中通过帧头、CRC、字段范围和安全校验后，
 * 已经被接受并复制到 pending 运行时缓存的命令数量。
 */
volatile uint32_t g_debug_pc_cmd_accept_count = 0U;

/*
 * PC 命令转交调试计数：任务态已经把 pending 命令短临界区复制出来，
 * 并更新为 latest 运行时命令的次数。
 */
volatile uint32_t g_debug_pc_cmd_take_count = 0U;

/*
 * PC q_des 滤波调试计数：PCComm_Task1ms() 当前确实处于 policy allowed
 * 路径，并使用 PC 下发的自然对齐 q_des[] 做限速滤波的次数。
 */
volatile uint32_t g_debug_pc_allowed_rate_qdes_count = 0U;

/*
 * 默认站姿滤波调试计数：PCComm_Task1ms() 当前未允许 policy 接管，
 * 因而使用 g_joint_default_stand_rad[] 做限速滤波的次数。
 */
volatile uint32_t g_debug_pc_stand_rate_qdes_count = 0U;

/*
 * 目标关节与安全状态：
 * g_q_des_filtered: 经过限速滤波后的目标关节角；
 * g_estop: 急停锁存标志；
 * g_attitude_safe: 姿态是否安全（roll/pitch 未超限）；
 * g_fault_code: 当前故障码（急停/姿态超限等）。
 */
float g_q_des_filtered[J_NUM];//过了滤波的电脑发送关节角
/*
 * 一阶低通滤波器状态（PC policy 下发目标的内层平滑）。
 * g_q_des_lpf_state[j]: 对 g_latest_command.q_des[j] 做 IIR 滤波后的记忆值；
 * g_q_des_lpf_inited: 首次收到指令时直接装载，避免从零值长斜坡。
 */
static float g_q_des_lpf_state[J_NUM];
static uint8_t g_q_des_lpf_inited = 0U;
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
    uint8_t i;

    if (g_estop != 0U)
    {
        return;
    }

    if (pkt == 0)
    {
        return;
    }

    /*
     * 这里的 pkt 是 packed 线协议包，只允许在本函数中逐字段读取。
     * 读取后立即复制到自然对齐的 g_pending_command。
     *
     * 绝对不要把 packed JointCommandPacket 整体保存为运行时状态，
     * 也不要把 pkt->q_des 的地址传给普通 float* 函数。这样可以避免
     * STM32H7 / Cortex-M 在 enable=1 后访问非 4 字节对齐 float 数组时
     * 触发 UsageFault/HardFault，导致状态包停止发送。
     */
    g_pending_command.tick_ms = pkt->tick_ms;
    g_pending_command.enable = pkt->enable;
    g_pending_command.mode = pkt->mode;

    for (i = 0U; i < (uint8_t)J_NUM; i++)
    {
        g_pending_command.q_des[i] = pkt->q_des[i];
    }

    g_pending_tick_ms = HAL_GetTick();
    g_pending_ready = 1U;
    g_command_ever_received = 1U;
    g_debug_pc_cmd_accept_count++;
}

/*
 * 将 pending 指令转移为 latest 指令。
 * 运行在任务态，使用关中断保护与 ISR 的并发写入。
 */
static void pccomm_take_pending_command(void)
{
    PCCommRuntimeCommand cmd_local;
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
    /*
     * g_pending_command 已经是自然对齐的运行时结构体，这里可以整体复制。
     * 临界区只保护 pending -> local 的短复制，不做限速滤波、CRC 或其它
     * 耗时操作，避免扩大关中断时间。
     */
    cmd_local = g_pending_command;
    tick_local = g_pending_tick_ms;
    g_pending_ready = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    g_latest_command = cmd_local;
    g_last_command_tick_ms = tick_local;
    g_debug_pc_cmd_take_count++;
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
    pkt->cmd[0] = volecity_cmd[0];
    pkt->cmd[1] = volecity_cmd[1];
    pkt->cmd[2] = volecity_cmd[2];

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

    /*
     * g_latest_command / g_pending_command 是自然对齐的运行时缓存，
     * 不再是 JointCommandPacket，因此没有 head/crc 字段。帧头和 CRC
     * 只属于 UART 线协议解析阶段，不能进入长期控制状态。
     *
     * 关键安全设计：
     * - g_latest_command.q_des / g_pending_command.q_des 保持 memset 的零值，
     *   不预填默认站姿。这样在 Keil Watch 窗口可以直观确认"尚未收到 PC 指令"。
     * - g_q_des_filtered 仍初始化为默认站姿，保证未接管时关节滤波目标安全。
     * - g_command_ever_received 清零：只有 ISR 收到 CRC 校验通过的帧后才置 1，
     *   杜绝上电/断连后误入 policy 接管。
     */
    g_latest_command.tick_ms = 0U;
    g_latest_command.enable = 0U;
    g_latest_command.mode = 0U;
    g_pending_command.tick_ms = 0U;
    g_pending_command.enable = 0U;
    g_pending_command.mode = 0U;

    for (i = 0U; i < (uint8_t)J_NUM; i++)
    {
        g_latest_command.q_des[i] = g_joint_default_stand_rad[i];
        g_pending_command.q_des[i] = g_joint_default_stand_rad[i];
        g_q_des_filtered[i] = g_joint_default_stand_rad[i];
        g_q_des_lpf_state[i] = g_joint_default_stand_rad[i];
    }

    g_pending_ready = 0U;
    g_pending_tick_ms = 0U;
    g_last_command_tick_ms = 0U;
    g_last_state_tx_tick_ms = 0U;
    g_tx_busy = 0U;
    g_estop = 0U;
    g_attitude_safe = 1U;
    g_fault_code = PC_COMM_FAULT_NONE;
    g_command_ever_received = 0U;
    g_q_des_lpf_inited = 0U;
    g_debug_pc_cmd_accept_count = 0U;
    g_debug_pc_cmd_take_count = 0U;
    g_debug_pc_allowed_rate_qdes_count = 0U;
    g_debug_pc_stand_rate_qdes_count = 0U;
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

    /*
     * 指令超时后显式清零 enable，避免 g_latest_command.enable 残留在 1
     * 导致下一次莫名进入 policy 接管（belt-and-suspenders）。
     */
    if (PCComm_IsCommandFresh() == 0U)
    {
        g_latest_command.enable = 0U;
    }

    if (PCComm_IsPolicyControlAllowed() != 0U)
    {
        /*
         * 一阶低通滤波：对 PC 下发的 q_des 做 IIR 平滑，
         * 衰减 ±0.05rad 高频噪声（约 10x），避免电机高频抖动。
         * enable=1 时会进入这里。传入的是自然对齐的
         * PCCommRuntimeCommand.q_des，不再是 packed JointCommandPacket.q_des。
         */
        uint8_t k;
        if (g_q_des_lpf_inited == 0U)
        {
            for (k = 0U; k < (uint8_t)J_NUM; k++)
            {
                g_q_des_lpf_state[k] = g_latest_command.q_des[k];
            }
            g_q_des_lpf_inited = 1U;
        }
        else
        {
            const float alpha = PC_COMM_LPF_ALPHA;
            const float one_minus_alpha = 1.0f - alpha;
            for (k = 0U; k < (uint8_t)J_NUM; k++)
            {
                g_q_des_lpf_state[k] = alpha * g_latest_command.q_des[k]
                                       + one_minus_alpha * g_q_des_lpf_state[k];
            }
        }
        g_debug_pc_allowed_rate_qdes_count++;
        pccomm_rate_limit_qdes(g_q_des_lpf_state);
    }
    else
    {
        g_debug_pc_stand_rate_qdes_count++;
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
    /* 从未收到过合法 PC 指令：直接禁止，杜绝上电/断连后误入接管。 */
    if (g_command_ever_received == 0U)
    {
        return 0U;
    }
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

/* 复制最新 PC 原始目标关节角到输出数组（URDF 顺序）。 */
void PCComm_GetLatestQDesUrdf(float q_des_out[J_NUM])
{
    uint8_t i;
    uint32_t primask;

    if (q_des_out == 0)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    for (i = 0U; i < (uint8_t)J_NUM; i++)
    {
        q_des_out[i] = g_latest_command.q_des[i];
    }
    if (primask == 0U)
    {
        __enable_irq();
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
