#ifndef ROBOT_MAP_H
#define ROBOT_MAP_H

#include "main.h"
#include "usart.h"
#include <stdint.h>
#include "M8010.h"

#define ROBOT_LEG_NUM 4
#define MOTORS_PER_LEG 3
/* GO-M8010-6: output shaft 1 rad maps to rotor 6.33 rad. */
#define ROBOT_MOTOR_GEAR_RATIO 6.33f

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

#define ROBOT_JOINT_INDEX(leg_idx, motor_idx) \
    ((uint8_t)(((leg_idx) * MOTORS_PER_LEG) + (motor_idx)))

typedef struct Leg
{
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    M8010 motors_peer_leg[MOTORS_PER_LEG];
} Leg;

typedef struct
{
    float x_mm;
    float y_mm;
    float z_mm;
} LegSizeVectorMm;

typedef struct
{
    float hip_length;
    float thigh_length;
    float calf_length;
} LegRealSize;

extern Leg legs[ROBOT_LEG_NUM];
extern LegRealSize leg_real_size;
extern const float g_joint_offset_rad[J_NUM];
extern const float g_joint_default_stand_rad[J_NUM];
extern const float g_joint_transmission_sign[J_NUM];

void RobotMap_InitLegRealSize(void);
void RobotMap_Init(void);

#endif
