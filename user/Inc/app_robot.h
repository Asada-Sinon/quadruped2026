#ifndef APP_ROBOT_H
#define APP_ROBOT_H

#include <stdint.h>
#include "robot_map.h"

typedef enum
{
    ROBOT_MODE_STAND = 0U,
    ROBOT_MODE_WALK = 1U,
    ROBOT_MODE_CRAWL = 2U,
    ROBOT_MODE_FREE_MOVE = 3U
} RobotControlMode;

void App_Robot_Init(void);
void App_Robot_Loop1ms(void);
void App_Robot_MotorSendLoop(void);
void App_vofa_Send(void);

void App_SetControlMode(RobotControlMode mode);
RobotControlMode App_GetControlMode(void);

float App_Get_Model_Joint_Angle(uint8_t leg_idx,
                                uint8_t motor_idx,
                                const M8010 *motor);
void App_Get_Model_Joint_Angles(float q_urdf_out[J_NUM]);
void App_Get_Model_Joint_Velocities(float qd_urdf_out[J_NUM]);
void App_Set_Model_Joint_Target_Angle(const float q_des_urdf[J_NUM]);

void App_SetWalkParams(float freq_hz,
                       float step_length_m,
                       float swing_height_m);
void App_SetStandPose(const float stand_x_m_by_leg[ROBOT_LEG_NUM],
                      const float stand_y_m_by_leg[ROBOT_LEG_NUM],
                      const float stand_z_m_by_leg[ROBOT_LEG_NUM]);

extern float Target_Angle[ROBOT_LEG_NUM][MOTORS_PER_LEG];
extern volatile uint32_t g_debug_motor_send_cost_ms;
extern volatile uint32_t g_debug_motor_send_loop_count;
extern volatile uint32_t g_debug_motor_snapshot_publish_count;
extern volatile float g_policy_motor_kp;
extern volatile float g_policy_motor_kw;

float App_motor_angle_calculate(float target_angle, float pos_rel);
float App_target_relative_to_absolute(float pos_rel,
                                      float target_angle_rel,
                                      float pos_abs,
                                      int sign);
void App_all_motor_claculate(float target_angle[ROBOT_LEG_NUM][MOTORS_PER_LEG],
                             Leg leg[ROBOT_LEG_NUM]);
void App_UpdateCurrentFootPosFromMotor(Leg leg[ROBOT_LEG_NUM]);
void App_Robot_Send_Loop(void);

#endif
