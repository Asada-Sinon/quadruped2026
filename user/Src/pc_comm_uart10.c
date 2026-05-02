#include "pc_comm_uart10.h"

#include "app_robot.h"
#include "imu.h"
#include "usart.h"
#include <math.h>
#include <string.h>

#define PC_COMM_RX_WAIT_HEAD1 0U
#define PC_COMM_RX_WAIT_HEAD2 1U
#define PC_COMM_RX_READ_FRAME 2U

#define PC_COMM_MAX_DQ_PER_1MS 0.0015f
#define PC_COMM_ATTITUDE_LIMIT_DEG 30.0f
#define PC_COMM_DEG_TO_RAD 0.01745329251994329577f
#define PC_COMM_FAULT_NONE 0U
#define PC_COMM_FAULT_ESTOP 1U
#define PC_COMM_FAULT_ATTITUDE 2U
#define PC_COMM_ASCII_HELLO_TEST 0U
#define PC_COMM_FAKE_STATE_TEST 0U

static const float g_joint_limit_low[J_NUM] = {
    -1.2f, -1.2f, -2.3f,
    -1.2f, -1.2f, -2.3f,
    -1.2f, -1.2f, -2.3f,
    -1.2f, -1.2f, -2.3f,
};

static const float g_joint_limit_high[J_NUM] = {
    1.2f, 1.5f, -0.5f,
    1.2f, 1.5f, -0.5f,
    1.2f, 1.5f, -0.5f,
    1.2f, 1.5f, -0.5f,
};

static uint8_t g_rx_byte;
static uint8_t g_rx_state = PC_COMM_RX_WAIT_HEAD1;
static uint8_t g_rx_frame[sizeof(JointCommandPacket)];
static uint16_t g_rx_index = 0U;

static JointCommandPacket g_latest_command;
static JointCommandPacket g_pending_command;
static volatile uint8_t g_pending_ready = 0U;
static volatile uint32_t g_pending_tick_ms = 0U;
static uint32_t g_last_command_tick_ms = 0U;

static float g_q_des_filtered[J_NUM];
static uint8_t g_estop = 0U;
static uint8_t g_attitude_safe = 1U;
static uint8_t g_fault_code = PC_COMM_FAULT_NONE;

static RobotStatePacket g_tx_packet;
static volatile uint8_t g_tx_busy = 0U;
static uint32_t g_last_state_tx_tick_ms = 0U;

static float pccomm_absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

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

static uint8_t pccomm_isfinite_float(float x)
{
    return ((x == x) && (x <= 1000000.0f) && (x >= -1000000.0f)) ? 1U : 0U;
}

static void pccomm_reset_parser(void)
{
    g_rx_state = PC_COMM_RX_WAIT_HEAD1;
    g_rx_index = 0U;
}

static uint8_t pccomm_is_mode_valid(uint8_t mode)
{
    return (mode <= 2U) ? 1U : 0U;
}

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

static void pccomm_fill_state_packet(RobotStatePacket *pkt)
{
    IMU_Body *imu = imu_get_body_data();
    uint8_t i;

    memset(pkt, 0, sizeof(*pkt));

    pkt->head = PC_COMM_STATE_HEAD;
    pkt->tick_ms = HAL_GetTick();
    pkt->mode = (uint8_t)App_GetControlMode();
    pkt->fault = g_fault_code;

    /* TODO: replace with body-frame linear velocity estimator. */
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

    /* TODO: publish the active vx/vy/yaw-rate command when available. */
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

    /* TODO: replace with ADC/BMS battery voltage. */
    pkt->battery_v = 24.0f;
    pkt->crc = PCComm_CRC16_CCITT_FALSE((const uint8_t *)pkt,
                                        (uint16_t)(sizeof(RobotStatePacket) - sizeof(uint16_t)));
}

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

void PCComm_StartReceive(void)
{
    (void)HAL_UART_Receive_IT(&huart10, &g_rx_byte, 1U);
}

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

uint8_t PCComm_IsCommandFresh(void)
{
    if (g_last_command_tick_ms == 0U)
    {
        return 0U;
    }

    return ((HAL_GetTick() - g_last_command_tick_ms) <= PC_COMM_COMMAND_TIMEOUT_MS) ? 1U : 0U;
}

uint32_t PCComm_GetLastCommandAgeMs(void)
{
    if (g_last_command_tick_ms == 0U)
    {
        return 0xFFFFFFFFU;
    }

    return HAL_GetTick() - g_last_command_tick_ms;
}

uint8_t PCComm_GetEnable(void)
{
    return g_latest_command.enable;
}

uint8_t PCComm_GetMode(void)
{
    return g_latest_command.mode;
}

uint8_t PCComm_GetFault(void)
{
    return g_fault_code;
}

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

void PCComm_OnUart10RxCplt(void)
{
    PCComm_OnUart10RxByte(g_rx_byte);
    PCComm_StartReceive();
}

void PCComm_OnUart10TxCplt(void)
{
    g_tx_busy = 0U;
}

void PCComm_EStop(void)
{
    g_estop = 1U;
    g_fault_code = PC_COMM_FAULT_ESTOP;
    g_latest_command.enable = 0U;
}
