#ifndef ROBOT_MAP_H
#define ROBOT_MAP_H

#include "main.h"
#include "usart.h"
#include <stdint.h>
#include "M8010.h"
#define ROBOT_LEG_NUM 4
#define MOTORS_PER_LEG 3
/* GO-M8010-6 减速比：输出端 1 rad <-> 转子侧 6.33 rad */
#define ROBOT_MOTOR_GEAR_RATIO 6.33f

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
    float hip_length;   // 髋关节偏置长度，单位 m
    float thigh_length; // 大腿长度，单位 m
    float calf_length;  // 小腿长度，单位 m
} LegRealSize;


extern Leg legs[ROBOT_LEG_NUM];
extern LegRealSize leg_real_size; // 单腿真实几何尺寸参数（按实测值保存，单位 m）
extern float g_joint_offset_rad[ROBOT_LEG_NUM][MOTORS_PER_LEG];//初始位置偏置，单位 rad
extern int g_joint_transmission_sign[ROBOT_LEG_NUM][MOTORS_PER_LEG];
// 用实测值初始化 leg_real_size。
// 单位全部使用 m。
void RobotMap_InitLegRealSize(void);
void RobotMap_Init(void);
#endif
