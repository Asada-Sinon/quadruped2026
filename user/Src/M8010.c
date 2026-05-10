#include "M8010.h"
#include "main.h"
#include <stdint.h>
#include <string.h>
#include "usart.h"
#include "robot_map.h"
#define M8010_TWO_PI 6.28318530718f
#define M8010_DMA_WAIT_TIMEOUT_MS 5U
#define M8010_MAIN_MOTOR_COUNT (ROBOT_LEG_NUM * MOTORS_PER_LEG)
#define M8010_EXTRA_MOTOR13_LEG_IDX (ROBOT_LEG_NUM - 1U)
#define M8010_EXTRA_MOTOR13_ID 13U
#define M8010_EXTRA_MOTOR13_SIGN (-1)
/*
 * M8010 协议编解码实现。
 *
 * 这里完成两件事：
 * 1) modify_data(): 将上层浮点控制量裁剪并量化到协议定点格式；
 * 2) extract_data(): 对回包做包头/CRC校验并反量化成浮点反馈。
 */

/* CRC-CCITT 查表，加速逐字节校验。 */
const uint16_t crc_ccitt_table[256] = {
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c)
{
	/* 使用查表法更新 1 字节 CRC。 */
	return (crc >> 8) ^ crc_ccitt_table[(crc ^ c) & 0xff];
}

/**
 *	crc_ccitt - recompute the CRC (CRC-CCITT variant) for the data
 *	buffer
 *	@crc: previous CRC value
 *	@buffer: data pointer
 *	@len: number of bytes in the buffer
 */
uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, size_t len)
{
	/* 对整个缓存逐字节累积 CRC。 */
	while (len--)
		crc = crc_ccitt_byte(crc, *buffer++);
	return crc;
}

/*
 * 饱和宏：把输入限制到 [_MIN, _MAX]。
 * 用于把上层命令约束到协议允许范围，避免溢出。
 */
#define SATURATE(_IN, _MIN, _MAX) \
	{                             \
		if ((_IN) <= (_MIN))      \
			(_IN) = (_MIN);       \
		else if ((_IN) >= (_MAX)) \
			(_IN) = (_MAX);       \
	}

/// @brief 将发送给电机的浮点参数转换为定点类型参数
/// @param motor_s 要转换的电机指令结构体
void modify_data(MotorCmd_t *motor_s)
{
	/* 1) 写入协议包头。 */
	motor_s->motor_send_data.head[0] = 0xFE;
	motor_s->motor_send_data.head[1] = 0xEE;

	/* 2) 对所有输入做范围裁剪，防止定点量化溢出。 */
	SATURATE(motor_s->id, 0, 15);
	SATURATE(motor_s->mode, 0, 7);
	SATURATE(motor_s->K_P, 0.0f, 25.599f);
	SATURATE(motor_s->K_W, 0.0f, 25.599f);
	SATURATE(motor_s->T, -127.99f, 127.99f);
	SATURATE(motor_s->W, -804.00f, 804.00f);
	SATURATE(motor_s->Pos, -411774.0f, 411774.0f);

	/*
	 * 3) 浮点 -> 定点（协议约定比例）：
	 *    - k_pos/k_spd: 0~25.6 映射到 q15
	 *    - pos/spd/tor: 按协议比例映射到对应整数位宽
	 */
	motor_s->motor_send_data.mode.id = motor_s->id;
	motor_s->motor_send_data.mode.status = motor_s->mode;
	motor_s->motor_send_data.comd.k_pos = motor_s->K_P / 25.6f * 32768.0f;
	motor_s->motor_send_data.comd.k_spd = motor_s->K_W / 25.6f * 32768.0f;
	motor_s->motor_send_data.comd.pos_des = motor_s->Pos / M8010_TWO_PI * 32768.0f;
	motor_s->motor_send_data.comd.spd_des = motor_s->W / M8010_TWO_PI * 256.0f;
	motor_s->motor_send_data.comd.tor_des = motor_s->T * 256.0f;

	/* 4) 计算并填入帧 CRC（不包含 CRC 字段本身）。 */
	motor_s->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, sizeof(RIS_ControlData_t) - sizeof(motor_s->motor_send_data.CRC16));
}

/// @brief 将接收到的定点类型原始数据转换为浮点参数类型
/// @param motor_r 要转换的电机反馈结构体
void extract_data(MotorData_t *motor_r)
{
	/* 1) 包头校验：快速过滤无效帧。 */
	if (motor_r->motor_recv_data.head[0] != 0xFD || motor_r->motor_recv_data.head[1] != 0xEE)
	{
		motor_r->correct = 0;
		return;
	}

	/* 2) CRC 校验：校验失败直接计数并丢弃。 */
	motor_r->calc_crc = crc_ccitt(0, (uint8_t *)&motor_r->motor_recv_data, sizeof(RIS_MotorData_t) - sizeof(motor_r->motor_recv_data.CRC16));
	if (motor_r->motor_recv_data.CRC16 != motor_r->calc_crc)
	{
		memset(&motor_r->motor_recv_data, 0, sizeof(RIS_MotorData_t));
		motor_r->correct = 0;
		motor_r->bad_msg++;
		return;
	}
	else
	{
		/* 3) 定点反馈 -> 浮点反馈，供控制层直接使用。 */
		motor_r->motor_id = motor_r->motor_recv_data.mode.id;
		motor_r->mode = motor_r->motor_recv_data.mode.status;
		motor_r->Temp = motor_r->motor_recv_data.fbk.temp;
		motor_r->MError = motor_r->motor_recv_data.fbk.MError;
		motor_r->W = ((float)motor_r->motor_recv_data.fbk.speed / 256.0f) * M8010_TWO_PI;
		motor_r->T = ((float)motor_r->motor_recv_data.fbk.torque) / 256.0f;
		motor_r->Pos = M8010_TWO_PI * ((float)motor_r->motor_recv_data.fbk.pos) / 32768.0f;
		if (motor_r->PosZeroInited == 0U)
		{
			motor_r->PosZero = motor_r->Pos;
			motor_r->PosZeroInited = 1U;
		}
		motor_r->PosRel = (motor_r->Pos - motor_r->PosZero) / ROBOT_MOTOR_GEAR_RATIO;
		// motor_r->PosRel = motor_r->Pos - motor_r->PosZero;
		motor_r->footForce = motor_r->motor_recv_data.fbk.force;
		motor_r->correct = 1;
		return;
	}
}

MotorCmd_t cmd[M8010_MAIN_MOTOR_COUNT];
MotorData_t recv;

/*
 * 外部可写：
 * - g_motor13_target_angle: ID13 本周期位置目标(rad)
 * - g_motor13_target_kp:    ID13 本周期 Kp
 */
float g_motor13_target_angle = 0.0f;
float g_motor13_target_kp = 0.0f;

/* 4 号腿新增 ID13 的本地发送/接收槽位。 */
static MotorCmd_t g_motor13_cmd;
M8010 g_motor13;

/* 4 路 485 口对应的 DMA 接收缓冲（每路 1 帧）。 */
static uint8_t g_motor_dma_rx_buf[ROBOT_LEG_NUM][sizeof(RIS_MotorData_t)] = {0};
/* 当前发送等待的目标电机 ID（用于匹配回包）。 */
static volatile uint8_t g_motor_dma_expected_id[ROBOT_LEG_NUM] = {0};
/* 本轮是否收到回包。 */
static volatile uint8_t g_motor_dma_rx_done[ROBOT_LEG_NUM] = {0};
/* 本轮回包是否匹配目标电机且校验通过。 */
static volatile uint8_t g_motor_dma_rx_match[ROBOT_LEG_NUM] = {0};

static void MotorCmd_SetDefaults(MotorCmd_t *motor_s,
								 unsigned short id,
								 float kp,
								 float kw)
{
	if (motor_s == NULL)
	{
		return;
	}

	motor_s->id = id;
	motor_s->mode = 1U;
	motor_s->T = 0.0f;
	motor_s->W = 0.0f;
	motor_s->Pos = 0.0f;
	motor_s->K_P = kp;
	motor_s->K_W = kw;
	modify_data(motor_s);
}

static void MotorBus_AbortStaleReceive(uint8_t leg_idx)
{
	UART_HandleTypeDef *huart;

	if (leg_idx >= ROBOT_LEG_NUM)
	{
		return;
	}

	huart = legs[leg_idx].huart;
	if (huart == NULL)
	{
		return;
	}

	/*
	 * The M8010 bus is half duplex: one command produces one 16-byte reply.
	 * Do not keep an old RX DMA alive across transactions. If the TxCplt
	 * callback tries to start a fresh receive while the old one is active,
	 * HAL_UART_Receive_DMA() returns HAL_BUSY and the real reply is skipped.
	 */
	(void)HAL_UART_AbortReceive(huart);
}

/* 初始化新增 ID13 电机的默认发送参数。 */
static void motor13_init_cmd(void)
{
	MotorCmd_SetDefaults(&g_motor13_cmd, M8010_EXTRA_MOTOR13_ID, 0.0f, 0.02f);

	g_motor13.motor_s.id = M8010_EXTRA_MOTOR13_ID;
	g_motor13.sign = M8010_EXTRA_MOTOR13_SIGN;
}

void MotorBus_Restart(uint8_t leg_idx)
{
	UART_HandleTypeDef *huart;

	if (leg_idx >= ROBOT_LEG_NUM)
	{
		return;
	}

	huart = legs[leg_idx].huart;
	if (huart == NULL)
	{
		return;
	}

	/* 发送完成后切回接收方向。 */
	HAL_GPIO_WritePin(legs[leg_idx].dir_port, legs[leg_idx].dir_pin, GPIO_PIN_RESET);

	/* 启动 DMA 接收 1 帧电机反馈。 */
	if (HAL_UART_Receive_DMA(huart,
						 g_motor_dma_rx_buf[leg_idx],
						 sizeof(RIS_MotorData_t)) != HAL_OK)
	{
		/* 启动失败时直接标记本轮失败，避免发送线程一直等待。 */
		g_motor_dma_rx_done[leg_idx] = 1U;
		g_motor_dma_rx_match[leg_idx] = 0U;
		return;
	}

	/* 关闭半传输中断，避免 16B 帧中途打断。 */
	if (huart->hdmarx != NULL)
	{
		__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
	}
}

void MotorBus_Process(uint8_t leg_idx, uint16_t size)
{
	RIS_MotorData_t *rx_frame;
	M8010 *motor;
	uint8_t motor_id;

	if (leg_idx >= ROBOT_LEG_NUM)
	{
		return;
	}

	g_motor_dma_rx_done[leg_idx] = 1U;
	g_motor_dma_rx_match[leg_idx] = 0U;

	if (size < sizeof(RIS_MotorData_t))
	{
		return;
	}

	rx_frame = (RIS_MotorData_t *)g_motor_dma_rx_buf[leg_idx];
	motor_id = (uint8_t)rx_frame->mode.id;
	motor = NULL;

	/* 按电机 ID 分发到这一路 485 对应的 3 个电机槽位。 */
	if (motor_id == legs[leg_idx].motors_peer_leg[0].motor_s.id)
	{
		motor = &legs[leg_idx].motors_peer_leg[0];
	}
	else if (motor_id == legs[leg_idx].motors_peer_leg[1].motor_s.id)
	{
		motor = &legs[leg_idx].motors_peer_leg[1];
	}
	else if (motor_id == legs[leg_idx].motors_peer_leg[2].motor_s.id)
	{
		motor = &legs[leg_idx].motors_peer_leg[2];
	}
	/* 4 号腿额外电机：ID13。 */
	else if ((leg_idx == M8010_EXTRA_MOTOR13_LEG_IDX) &&
			 (motor_id == M8010_EXTRA_MOTOR13_ID))
	{
		motor = &g_motor13;
	}

	if (motor != NULL)
	{
		memcpy(&motor->motor_r.motor_recv_data, rx_frame, sizeof(RIS_MotorData_t));
		extract_data(&motor->motor_r);
		if (motor->motor_r.correct != 0)
		{
			motor->motor_r.PosRel *= (float)motor->sign;
			if (motor_id == g_motor_dma_expected_id[leg_idx])
			{
				g_motor_dma_rx_match[leg_idx] = 1U;
			}
		}
	}
}

void cmd_init(void)
{
	for (uint8_t i = 0U; i < M8010_MAIN_MOTOR_COUNT; i++)
	{
		MotorCmd_SetDefaults(&cmd[i], (unsigned short)(i + 1U), 0.0f, 0.0f);
	}
	motor13_init_cmd();
}
void cmd_init_2(void)
{
	for (uint8_t i = 0U; i < M8010_MAIN_MOTOR_COUNT; i++)
	{
		MotorCmd_SetDefaults(&cmd[i], (unsigned short)(i + 1U), 5.0f, 0.03f);
	}
	motor13_init_cmd();
}
void cmd_single_test_init(void)
{
	static const uint8_t hip_cmd_indices[ROBOT_LEG_NUM] = {0U, 3U, 6U, 9U};
	uint8_t i;

	for (i = 0U; i < M8010_MAIN_MOTOR_COUNT; i++)
	{
		MotorCmd_SetDefaults(&cmd[i], (unsigned short)(i + 1U), 2.0f, 0.02f);
	}
	for (i = 6U; i < M8010_MAIN_MOTOR_COUNT; i++)
	{
		MotorCmd_SetDefaults(&cmd[i], (unsigned short)(i + 1U), 3.5f, 0.02f);
	}
	for (i = 0U; i < ROBOT_LEG_NUM; i++)
	{
		uint8_t cmd_idx = hip_cmd_indices[i];
		MotorCmd_SetDefaults(&cmd[cmd_idx], (unsigned short)(cmd_idx + 1U), 2.2f, 0.02f);
	}
	motor13_init_cmd();
}

void set_cmd_pos_by_index(uint8_t cmd_idx, float pos)
{
	if (cmd_idx >= M8010_MAIN_MOTOR_COUNT)
	{
		return;
	}

	/* 写入该电机本次要发送的位置目标。 */
	cmd[cmd_idx].Pos = pos;
	/* 位置更新后立刻重打包，确保发送字节流同步更新。 */
	modify_data(&cmd[cmd_idx]);
}

void set_cmd_by_index(uint8_t cmd_idx, const MotorCmd_t *motor_s)
{
	if ((cmd_idx >= M8010_MAIN_MOTOR_COUNT) ||
		(motor_s == NULL))
	{
		return;
	}

	/*
	 * 只复制发送命令字段，不接触反馈、串口句柄、方向引脚或安装方向。
	 * motorsend 任务在调用 send_data_all() 之前用它刷新 cmd[]，这样控制任务
	 * 只需要发布快照，不再直接改写底层发送缓冲。
	 */
	cmd[cmd_idx].id = motor_s->id;
	cmd[cmd_idx].mode = motor_s->mode;
	cmd[cmd_idx].T = motor_s->T;
	cmd[cmd_idx].W = motor_s->W;
	cmd[cmd_idx].Pos = motor_s->Pos;
	cmd[cmd_idx].K_P = motor_s->K_P;
	cmd[cmd_idx].K_W = motor_s->K_W;
	modify_data(&cmd[cmd_idx]);
}

static void MotorBus_SendCommandAndWait(uint8_t leg_idx,
										const MotorCmd_t *motor_s,
										M8010 *feedback_motor,
										Leg *leg)
{
	uint32_t start_tick;

	if ((leg == NULL) || (motor_s == NULL) || (feedback_motor == NULL) ||
		(leg_idx >= ROBOT_LEG_NUM))
	{
		return;
	}

	MotorBus_AbortStaleReceive(leg_idx);

	g_motor_dma_expected_id[leg_idx] = (uint8_t)motor_s->id;
	g_motor_dma_rx_done[leg_idx] = 0U;
	g_motor_dma_rx_match[leg_idx] = 0U;

	HAL_GPIO_WritePin(leg[leg_idx].dir_port, leg[leg_idx].dir_pin, GPIO_PIN_SET);
	if (HAL_UART_Transmit_DMA(leg[leg_idx].huart,
							  (uint8_t *)&motor_s->motor_send_data,
							  sizeof(RIS_ControlData_t)) != HAL_OK)
	{
		HAL_GPIO_WritePin(leg[leg_idx].dir_port, leg[leg_idx].dir_pin, GPIO_PIN_RESET);
		feedback_motor->motor_r.timeout++;
		return;
	}

	start_tick = HAL_GetTick();
	while (g_motor_dma_rx_done[leg_idx] == 0U)
	{
		if ((HAL_GetTick() - start_tick) >= M8010_DMA_WAIT_TIMEOUT_MS)
		{
			(void)HAL_UART_AbortTransmit(leg[leg_idx].huart);
			(void)HAL_UART_AbortReceive(leg[leg_idx].huart);
			HAL_GPIO_WritePin(leg[leg_idx].dir_port, leg[leg_idx].dir_pin, GPIO_PIN_RESET);
			feedback_motor->motor_r.timeout++;
			break;
		}
	}

	if ((g_motor_dma_rx_done[leg_idx] != 0U) &&
		(g_motor_dma_rx_match[leg_idx] == 0U))
	{
		feedback_motor->motor_r.timeout++;
	}
}

void send_data_all(Leg *leg)
{
	if (leg == NULL)
	{
		return;
	}

	for (uint8_t leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++)
	{
		for (uint8_t motor_idx = 0U; motor_idx < MOTORS_PER_LEG; motor_idx++)
		{
			uint8_t cmd_idx = (uint8_t)(leg_idx * MOTORS_PER_LEG + motor_idx);
			M8010 *motor = &leg[leg_idx].motors_peer_leg[motor_idx];

			MotorBus_SendCommandAndWait(leg_idx, &cmd[cmd_idx], motor, leg);
		}

		if (leg_idx == M8010_EXTRA_MOTOR13_LEG_IDX)
		{
			g_motor13_cmd.Pos = g_motor13_target_angle;
			g_motor13_cmd.K_P = g_motor13_target_kp;
			modify_data(&g_motor13_cmd);

			MotorBus_SendCommandAndWait(leg_idx, &g_motor13_cmd, &g_motor13, leg);
		}
	}
}
