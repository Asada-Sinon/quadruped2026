#include "robot_map.h"
#include <math.h>

/* 四条腿的硬件映射表（串口、方向引脚、电机对象）。 */
Leg legs[ROBOT_LEG_NUM] = {0};
/* 单腿真实几何尺寸缓存（单位 m）。 */
LegRealSize leg_real_size = {0};

/*
 * 电机安装方向（URDF 一维顺序）。
 * 说明：底层 MotorBus_Process 会先用该符号修正 PosRel，
 *       使“正方向”统一到机械安装约定。
 * 取值：+1 表示正向不变，-1 表示反向。
 */
int sign[J_NUM] =
    {
        1, 1, 1,
        1, -1, -1,
        -1, 1, 1,
        -1, -1, -1};

/*
 * 上电机械止动位对应的 URDF/模型关节角（PosRel = 0）。
 * 注意：该值表示“机械零位偏置”，并不是站立姿态。
 */
const float g_joint_offset_rad[J_NUM] =
    {
        0.78742018f, 0.97477274f, -2.28535870f,
        -0.78742018f, 0.97477274f, -2.28535870f,
        0.78742018f, 0.97477274f, -2.28535870f,
        -0.78742018f, 0.97477274f, -2.28535870f};

/* 默认站立姿态关节角（URDF 顺序，单位 rad）。 */
const float g_joint_default_stand_rad[J_NUM] =
    {
        -0.0279601746f, 1.0191152035f, -1.5190473145f,
        0.0279601746f, 1.0191152035f, -1.5190473145f,
        -0.0279601746f, 1.0191152035f, -1.5190473145f,
        0.0279601746f, 1.0191152035f, -1.5190473145f};

/*
 * 模型关节方向映射（URDF 顺序）。
 * 说明：PosRel 已在电机总线层通过 sign 修正；
 *       这里用于把 PosRel 的方向统一到模型关节角方向。
 */
const float g_joint_transmission_sign[J_NUM] =
    {
        1.0f, 1.0f, 1.0f,
        1.0f, 1.0f, 1.0f,
        1.0f, 1.0f, 1.0f,
        1.0f, 1.0f, 1.0f};

/* 写入单腿真实尺寸参数（单位 m）。 */
void RobotMap_InitLegRealSize(void)
{
    leg_real_size.hip_length = 0.087f;
    leg_real_size.thigh_length = 0.210f;
    leg_real_size.calf_length = 0.240f;
}

/*
 * 初始化腿到硬件的映射关系：
 * 1) 每条腿绑定对应的 RS485 串口与方向引脚；
 * 2) 填充每个电机的 ID 与安装方向 sign。
 */
void RobotMap_Init(void)
{
    /* 每条腿对应的串口通道。 */
    UART_HandleTypeDef *const huart_map[ROBOT_LEG_NUM] = {&huart9, &huart3, &huart2, &huart7};
    /* 每条腿对应的 485 方向控制端口。 */
    GPIO_TypeDef *const dir_port_map[ROBOT_LEG_NUM] = {
        RS485_1_DIR_GPIO_Port,
        RS485_2_DIR_GPIO_Port,
        RS485_3_DIR_GPIO_Port,
        RS485_4_DIR_GPIO_Port};
    /* 每条腿对应的 485 方向控制引脚。 */
    const uint16_t dir_pin_map[ROBOT_LEG_NUM] = {
        RS485_1_DIR_Pin,
        RS485_2_DIR_Pin,
        RS485_3_DIR_Pin,
        RS485_4_DIR_Pin};
    /* 每条腿 3 个电机的协议 ID 映射。 */
    const uint8_t motor_id_map[ROBOT_LEG_NUM][MOTORS_PER_LEG] = {
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 9},
        {10, 11, 12}};

    for (uint8_t leg_idx = 0; leg_idx < ROBOT_LEG_NUM; leg_idx++)
    {
        legs[leg_idx].huart = huart_map[leg_idx];
        legs[leg_idx].dir_port = dir_port_map[leg_idx];
        legs[leg_idx].dir_pin = dir_pin_map[leg_idx];

        for (uint8_t motor_idx = 0; motor_idx < MOTORS_PER_LEG; motor_idx++)
        {
            /* 将 (leg_idx, motor_idx) 转成 URDF 一维索引，便于共享关节数组。 */
            uint8_t motor_linear_idx = ROBOT_JOINT_INDEX(leg_idx, motor_idx);
            legs[leg_idx].motors_peer_leg[motor_idx].motor_s.id = motor_id_map[leg_idx][motor_idx];
            /* 安装方向符号映射到电机对象，供底层读取 PosRel 时使用。 */
            legs[leg_idx].motors_peer_leg[motor_idx].sign = sign[motor_linear_idx];
        }
    }

    /* 映射完成后，写入单腿真实几何尺寸。 */
    RobotMap_InitLegRealSize();
}
