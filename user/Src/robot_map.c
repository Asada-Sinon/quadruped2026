#include "robot_map.h"
#include <math.h>

Leg legs[ROBOT_LEG_NUM] = {0};
LegRealSize leg_real_size = {0};

/* Low-level motor installation direction. MotorBus_Process applies this to PosRel. */
int sign[J_NUM] =
    {
        1, 1, 1,
        1, -1, -1,
        -1, 1, 1,
        -1, -1, -1};

/*
 * URDF/model joint angle at the mechanical power-on stop where PosRel = 0.
 * This is not the default standing pose.
 */
const float g_joint_offset_rad[J_NUM] =
    {
        0.78742018f, 0.97477274f, -2.28535870f,
        -0.78742018f, 0.97477274f, -2.28535870f,
        0.78742018f, 0.97477274f, -2.28535870f,
        -0.78742018f, 0.97477274f, -2.28535870f};

const float g_joint_default_stand_rad[J_NUM] =
    {
        -0.0279601746f, 1.0191152035f, -1.5190473145f,
        0.0279601746f, 1.0191152035f, -1.5190473145f,
        -0.0279601746f, 1.0191152035f, -1.5190473145f,
        0.0279601746f, 1.0191152035f, -1.5190473145f};

/*
 * Upper-level transmission direction between PosRel and URDF/model joints.
 * PosRel has already been corrected by motor->sign in the motor bus layer.
 */
const float g_joint_transmission_sign[J_NUM] =
    {
        1.0f, 1.0f, 1.0f,
        1.0f, 1.0f, 1.0f,
        1.0f, 1.0f, 1.0f,
        1.0f, 1.0f, 1.0f};

void RobotMap_InitLegRealSize(void)
{
    leg_real_size.hip_length = 0.087f;
    leg_real_size.thigh_length = 0.210f;
    leg_real_size.calf_length = 0.240f;
}

void RobotMap_Init(void)
{
    UART_HandleTypeDef *const huart_map[ROBOT_LEG_NUM] = {&huart9, &huart3, &huart2, &huart7};
    GPIO_TypeDef *const dir_port_map[ROBOT_LEG_NUM] = {
        RS485_1_DIR_GPIO_Port,
        RS485_2_DIR_GPIO_Port,
        RS485_3_DIR_GPIO_Port,
        RS485_4_DIR_GPIO_Port};
    const uint16_t dir_pin_map[ROBOT_LEG_NUM] = {
        RS485_1_DIR_Pin,
        RS485_2_DIR_Pin,
        RS485_3_DIR_Pin,
        RS485_4_DIR_Pin};
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
            uint8_t motor_linear_idx = ROBOT_JOINT_INDEX(leg_idx, motor_idx);
            legs[leg_idx].motors_peer_leg[motor_idx].motor_s.id = motor_id_map[leg_idx][motor_idx];
            legs[leg_idx].motors_peer_leg[motor_idx].sign = sign[motor_linear_idx];
        }
    }

    RobotMap_InitLegRealSize();
}
