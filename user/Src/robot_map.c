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

/*
 * 将实测的单腿关键几何尺寸写入 leg_real_size。
 * 下面每个数值都与机械定义中的点位关系一一对应，单位统一为 mm。
 */
void RobotMap_InitLegRealSize(void)
{
    /*
     * 这几个局部常量只用于初始化阶段做长度投影计算，
     * 保证“向量参数”和“简化 3R 标量参数”来自同一组实测数据。
     */
    const float bc_x = 174.48f;
    const float bc_z = 116.95f;
    const float cd_x = 125.22f;
    const float cd_z = 205.53f;

    leg_real_size.imu_center_to_motor1_axis.x_mm = 166.3f; // O -> A：IMU 中心到单腿电机1轴心的 X 分量
    leg_real_size.imu_center_to_motor1_axis.y_mm = 90.0f;  // O -> A：IMU 中心到单腿电机1轴心的 Y 分量
    leg_real_size.imu_center_to_motor1_axis.z_mm = 77.5f;  // O -> A：IMU 中心到单腿电机1轴心的 Z 分量

    leg_real_size.motor1_axis_to_motor2_axis.x_mm = 66.05f; // A -> B：单腿电机1轴心到单腿电机2轴心的 X 分量
    leg_real_size.motor1_axis_to_motor2_axis.y_mm = 20.03f; // A -> B：单腿电机1轴心到单腿电机2轴心的 Y 分量
    leg_real_size.motor1_axis_to_motor2_axis.z_mm = 0.0f;   // A -> B：单腿电机1轴心到单腿电机2轴心的 Z 分量

    leg_real_size.motor2_axis_to_knee_joint_axis.x_mm = 174.48f; // B -> C：单腿电机2轴心到膝关节轴心的 X 分量
    leg_real_size.motor2_axis_to_knee_joint_axis.y_mm = 67.5f;   // B -> C：单腿电机2轴心到膝关节轴心的 Y 分量
    leg_real_size.motor2_axis_to_knee_joint_axis.z_mm = 116.95f; // B -> C：单腿电机2轴心到膝关节轴心的 Z 分量

    leg_real_size.knee_joint_axis_to_foot_tip.x_mm = 125.22f; // C -> D：膝关节轴心到足端点的 X 分量
    leg_real_size.knee_joint_axis_to_foot_tip.y_mm = 0.0f;    // C -> D：膝关节轴心到足端点的 Y 分量
    leg_real_size.knee_joint_axis_to_foot_tip.z_mm = 205.53f; // C -> D：膝关节轴心到足端点的 Z 分量

    leg_real_size.motor3_axis_to_knee_joint_axis.x_mm = 174.48f; // E -> C：单腿电机3轴心到膝关节轴心的 X 分量
    leg_real_size.motor3_axis_to_knee_joint_axis.y_mm = 15.2f;   // E -> C：单腿电机3轴心到膝关节轴心的 Y 分量
    leg_real_size.motor3_axis_to_knee_joint_axis.z_mm = 116.95f; // E -> C：单腿电机3轴心到膝关节轴心的 Z 分量

    leg_real_size.motor3_axis_to_motor3_side_link_pin.x_mm = 16.54f; // E -> F：单腿电机3轴心到电机3侧连杆销轴的 X 分量
    leg_real_size.motor3_axis_to_motor3_side_link_pin.y_mm = 15.2f;  // E -> F：单腿电机3轴心到电机3侧连杆销轴的 Y 分量
    leg_real_size.motor3_axis_to_motor3_side_link_pin.z_mm = 15.25f; // E -> F：单腿电机3轴心到电机3侧连杆销轴的 Z 分量

    leg_real_size.motor3_side_link_pin_to_shank_side_link_pin.x_mm = 174.48f; // F -> G：电机3侧销轴到小腿侧销轴位移的 X 分量
    leg_real_size.motor3_side_link_pin_to_shank_side_link_pin.y_mm = 0.0f;    // F -> G：电机3侧销轴到小腿侧销轴位移的 Y 分量
    leg_real_size.motor3_side_link_pin_to_shank_side_link_pin.z_mm = 116.95f; // F -> G：电机3侧销轴到小腿侧销轴位移的 Z 分量

    leg_real_size.link_pin_center_distance_mm = 210.05f; // F -> G：连杆两端销轴中心距（标量长度）

    leg_real_size.knee_joint_axis_to_shank_side_link_pin.x_mm = 16.54f; // C -> G：膝关节轴心到小腿侧连杆销轴的 X 分量
    leg_real_size.knee_joint_axis_to_shank_side_link_pin.y_mm = 0.0f;   // C -> G：膝关节轴心到小腿侧连杆销轴的 Y 分量
    leg_real_size.knee_joint_axis_to_shank_side_link_pin.z_mm = 15.25f; // C -> G：膝关节轴心到小腿侧连杆销轴的 Z 分量

    /*
     * 简化 3R 模型参数写入（单位 mm）：
     * hip_offset_x/y 直接使用 A->B 的 X/Y 偏置；
     * thigh/shank 使用 X-Z 平面投影长度，便于 2R 平面解算。
     */
    leg_real_size.hip_offset_x_mm = leg_real_size.motor1_axis_to_motor2_axis.x_mm;
    leg_real_size.hip_offset_y_mm = leg_real_size.motor1_axis_to_motor2_axis.y_mm;
    leg_real_size.thigh_length_mm = sqrtf((bc_x * bc_x) + (bc_z * bc_z));
    leg_real_size.shank_length_mm = sqrtf((cd_x * cd_x) + (cd_z * cd_z));
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

