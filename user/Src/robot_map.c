#include "robot_map.h"
#include <math.h>

/*
 * 串口映射也只在这里改：
 * 端口1 -> 左前 -> ID 1 2 3
 * 端口2 -> 右前 -> ID 4 5 6
 * 端口3 -> 左后 -> ID 7 8 9
 * 端口4 -> 右后 -> ID 10 11 12
 */
Leg legs[ROBOT_LEG_NUM] = {0};
LegRealSize leg_real_size = {0}; // 真实单腿几何尺寸参数缓存，启动后由 RobotMap_InitLegRealSize() 填充
//从四足前面看，逆时针为正，从四足左边看，逆时针为正
int sign[12] = {1,1,1,-1,-1,-1,-1,1,1,1,-1,-1};
//这个注意是输出侧的角度，就是要有减速比，这个是电机初始偏置角度，并非足端
float g_joint_offset_rad[ROBOT_LEG_NUM][MOTORS_PER_LEG] =
{
    { 0.78742018f,  0.97477274f, -2.28535870f }, // FL
    { -0.78742018f,  0.97477274f, -2.28535870f }, // FR
    { 0.78742018f,  0.97477274f, -2.28535870f }, // HL
    { -0.78742018f,  0.97477274f, -2.28535870f }  // HR
};
//电机和关节的传动关系，小腿因为是通过连杆，电机转动方向和小腿摆动方向不同
int g_joint_transmission_sign[ROBOT_LEG_NUM][MOTORS_PER_LEG] =
{
    { 1,  1, 1 }, // FL: 髋、腿直连；小腿通过平行四边形连杆，方向反
    { 1,  1, 1 }, // FR
    { 1,  1, 1 }, // HL
    { 1,  1, 1 }  // HR
};
/*
 * 将实测的单腿关键几何尺寸写入 leg_real_size。
 * 下面每个数值都与机械定义中的点位关系一一对应，单位统一为 m。
 */
void RobotMap_InitLegRealSize(void)
{
    leg_real_size.hip_length = 0.087f; //髋关节中心到大腿电机轴心的距离
    leg_real_size.thigh_length = 0.210f; //大腿长度：大腿电机轴心到膝关节轴心的距离
    leg_real_size.calf_length = 0.240f; //小腿长度：膝关节轴心到足端的距离
}

void RobotMap_Init(void)
{
    UART_HandleTypeDef *const huart_map[ROBOT_LEG_NUM] = {&huart9, &huart3, &huart2, &huart7};
    GPIO_TypeDef *const dir_port_map[ROBOT_LEG_NUM] = {
        RS485_1_DIR_GPIO_Port,
        RS485_2_DIR_GPIO_Port,
        RS485_3_DIR_GPIO_Port,
        RS485_4_DIR_GPIO_Port
    };
    const uint16_t dir_pin_map[ROBOT_LEG_NUM] = {
        RS485_1_DIR_Pin,
        RS485_2_DIR_Pin,
        RS485_3_DIR_Pin,
        RS485_4_DIR_Pin
    };
    const uint8_t motor_id_map[ROBOT_LEG_NUM][MOTORS_PER_LEG] = {
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 9},
        {10, 11, 12}
    };

    for (uint8_t leg_idx = 0; leg_idx < ROBOT_LEG_NUM; leg_idx++)
    {
        legs[leg_idx].huart = huart_map[leg_idx];
        legs[leg_idx].dir_port = dir_port_map[leg_idx];
        legs[leg_idx].dir_pin = dir_pin_map[leg_idx];

        for (uint8_t motor_idx = 0; motor_idx < MOTORS_PER_LEG; motor_idx++)
        {
            uint8_t motor_linear_idx = (uint8_t)(leg_idx * MOTORS_PER_LEG + motor_idx);
            legs[leg_idx].motors_peer_leg[motor_idx].motor_s.id = motor_id_map[leg_idx][motor_idx];
            legs[leg_idx].motors_peer_leg[motor_idx].sign = sign[motor_linear_idx];
        }
    }

    RobotMap_InitLegRealSize(); // 在端口与电机映射初始化完成后，写入单腿真实几何参数
}

