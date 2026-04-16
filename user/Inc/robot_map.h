#ifndef ROBOT_MAP_H
#define ROBOT_MAP_H

#include "main.h"
#include "usart.h"
#include <stdint.h>
#include "M8010.h"
#define ROBOT_LEG_NUM 4
#define MOTORS_PER_LEG 3

typedef struct Leg
{
    UART_HandleTypeDef *huart;                // 该腿对应的 UART 句柄
    GPIO_TypeDef *dir_port;                   // RS485 方向控制引脚端口（RE/DE）
    uint16_t dir_pin;                         // RS485 方向控制引脚
    M8010 motors_peer_leg[MOTORS_PER_LEG];        // 该腿 3 个电机的协议 ID
} Leg;

typedef struct
{
    float x_mm; // X 方向长度分量，单位 mm
    float y_mm; // Y 方向长度分量，单位 mm
    float z_mm; // Z 方向长度分量，单位 mm
} LegSizeVectorMm;

typedef struct
{
    LegSizeVectorMm imu_center_to_motor1_axis;                    // O -> A：IMU 中心到单腿电机1轴心
    LegSizeVectorMm motor1_axis_to_motor2_axis;                   // A -> B：单腿电机1轴心到单腿电机2轴心
    LegSizeVectorMm motor2_axis_to_knee_joint_axis;               // B -> C：单腿电机2轴心到膝关节轴心
    LegSizeVectorMm knee_joint_axis_to_foot_tip;                  // C -> D：膝关节轴心到足端点
    LegSizeVectorMm motor3_axis_to_knee_joint_axis;               // E -> C：单腿电机3轴心到膝关节轴心
    LegSizeVectorMm motor3_axis_to_motor3_side_link_pin;          // E -> F：单腿电机3轴心到电机3侧连杆销轴
    LegSizeVectorMm motor3_side_link_pin_to_shank_side_link_pin;  // F -> G：电机3侧销轴到小腿侧销轴的三维分量
    float link_pin_center_distance_mm;                            // F -> G：连杆两端销轴中心距（标量）
    LegSizeVectorMm knee_joint_axis_to_shank_side_link_pin;       // C -> G：膝关节轴心到小腿侧连杆销轴
} LegRealSize;


extern Leg legs[ROBOT_LEG_NUM];
extern LegRealSize leg_real_size; // 单腿真实几何尺寸参数（按实测值保存）

// 用实测值初始化 leg_real_size。
// 单位全部使用 mm，便于和机械图纸直接对照。
void RobotMap_InitLegRealSize(void);
void RobotMap_Init(void);
#endif
