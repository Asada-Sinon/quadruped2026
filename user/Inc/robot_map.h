#ifndef ROBOT_MAP_H
#define ROBOT_MAP_H

#include "main.h"
#include "usart.h"
#include <stdint.h>
#include "M8010.h"

#define ROBOT_LEG_NUM 4
#define MOTORS_PER_LEG 3
/* GO-M8010-6 减速比：输出轴 1 rad 对应转子 6.33 rad。 */
#define ROBOT_MOTOR_GEAR_RATIO 6.33f

/*
 * URDF/模型关节的一维索引顺序。
 * 顺序约定：左前(FL) -> 右前(FR) -> 左后(HL) -> 右后(HR)，每条腿依次 HAA/HFE/KFE。
 * J_NUM 表示关节总数（4 条腿 * 3 关节 = 12）。
 */
typedef enum
{
    J_LF_HAA = 0,
    J_LF_HFE,
    J_LF_KFE,

    J_RF_HAA,
    J_RF_HFE,
    J_RF_KFE,

    J_LH_HAA,
    J_LH_HFE,
    J_LH_KFE,

    J_RH_HAA,
    J_RH_HFE,
    J_RH_KFE,

    J_NUM
} JointIndex_e;

/*
 * 把 (leg_idx, motor_idx) 映射为 URDF 一维关节索引。
 * leg_idx: 0~3（FL/FR/HL/HR），motor_idx: 0~2（HAA/HFE/KFE）。
 */
#define ROBOT_JOINT_INDEX(leg_idx, motor_idx) \
    ((uint8_t)(((leg_idx) * MOTORS_PER_LEG) + (motor_idx)))

/*
 * 单条腿的硬件映射信息：
 * huart: 该腿对应的 RS485 串口句柄；
 * dir_port/dir_pin: 485 方向控制引脚（RE/DE）；
 * motors_peer_leg: 该腿 3 个电机的对象数组。
 */
typedef struct Leg
{
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    M8010 motors_peer_leg[MOTORS_PER_LEG];
} Leg;

/* 三维向量（单位 mm），用于几何/运动学计算。 */
typedef struct
{
    float x_mm;
    float y_mm;
    float z_mm;
} LegSizeVectorMm;

/* 单腿真实几何参数（单位 m）。 */
typedef struct
{
    float hip_length;
    float thigh_length;
    float calf_length;
} LegRealSize;

extern Leg legs[ROBOT_LEG_NUM];
extern LegRealSize leg_real_size;
/* 上电机械止动位对应的 URDF 关节角（PosRel = 0 时的模型角）。 */
extern const float g_joint_offset_rad[J_NUM];
/* 默认站立姿态的 URDF 关节角（rad）。 */
extern const float g_joint_default_stand_rad[J_NUM];
/* 电机 PosRel 与 URDF 关节角之间的方向映射（±1）。 */
extern const float g_joint_transmission_sign[J_NUM];

/* 写入单腿真实尺寸参数（单位 m）。 */
void RobotMap_InitLegRealSize(void);
/* 初始化腿的串口/方向引脚/电机 ID 映射关系。 */
void RobotMap_Init(void);

#endif
