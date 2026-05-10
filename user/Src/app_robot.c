#include "app_robot.h"

#include "HT10A.h"
#include "M8010.h"
#include "app_diagnostics.h"
#include "app_motion_control.h"
#include "app_motor_control.h"
#include "gait.h"
#include "imu.h"
#include "pc_comm_uart10.h"
#include "robot_math.h"
#include "usart.h"
#include "vofa.h"

#define APP_LOOP_DEFAULT_DT_MS 1.0f
#define APP_LOOP_MAX_DT_MS     20.0f

static float App_GetRealDtMs(void)
{
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    uint32_t diff;

    if (last_tick == 0U) {
        last_tick = now;
        return APP_LOOP_DEFAULT_DT_MS;
    }

    diff = now - last_tick;
    last_tick = now;

    if (diff == 0U) {
        return APP_LOOP_DEFAULT_DT_MS;
    }

    return Robot_ClampF((float)diff, APP_LOOP_DEFAULT_DT_MS, APP_LOOP_MAX_DT_MS);
}

void App_Robot_Init(void)
{
    uint8_t leg_idx;

    RobotMap_Init();

    for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++) {
        MotorBus_Restart(leg_idx);
    }

    cmd_init();
    AppMotor_SetMainMotorCommandDefaults(0.0f, 0.0f, 0.0f, 0.0f, 1, 1U);
    AppMotor_PublishCommandSnapshot();

    AppMotion_Init();
    AppMotor_ResetInterpolationState();

    VOFA_JF_DMA_Init(&hvofa, &huart6);
    Teaching_Pendant_Restart();
    IMU_Restart();
    PCComm_Init();
    PCComm_StartReceive();
}

void App_Robot_Loop1ms(void)
{
    static uint8_t pc_policy_was_active = 0;
    float dt_ms;
    float dt_s;
    uint8_t pc_policy_active;
    RobotControlMode mode;

    dt_ms = App_GetRealDtMs();
    dt_s = dt_ms * 0.001f;
    AppMotor_SetInterpolationDtMs(dt_ms);

    PCComm_Task1ms();
    PCComm_SendState20ms();

    if ((AppMotor_IsKpKwArmed() == 0U) && (AppMotor_AllFeedbackValid() != 0U)) {
        AppMotor_SetMainMotorCommandDefaults(0.0f, 0.0f, 3.15f, 0.06f, 1, 0U);
        AppMotor_ResetInterpolationState();
        AppMotor_SetKpKwArmed(1U);
    }

    pc_policy_active = PCComm_IsPolicyControlAllowed();
    mode = App_GetControlMode();

    if ((pc_policy_active != 0U) && ((mode == ROBOT_MODE_STAND) || (mode == ROBOT_MODE_WALK))) {
        float q_des[J_NUM];
        PCComm_GetQDesUrdf(q_des);
        if (AppMotor_IsKpKwArmed() != 0U) {
            AppMotor_SetMainMotorCommandDefaults(0.0f, 0.0f, g_policy_motor_kp, g_policy_motor_kw, 1, 0U);
        }
        App_Set_Model_Joint_Target_Angle(q_des);
        AppMotor_PublishCommandSnapshot();
        pc_policy_was_active = 1U;
        return;
    }

    if (pc_policy_was_active != 0U) {
        if (AppMotor_IsKpKwArmed() != 0U) {
            AppMotor_SetMainMotorCommandDefaults(0.0f, 0.0f, 3.15f, 0.06f, 1, 0U);
        } else {
            AppMotor_SetMainMotorCommandDefaults(0.0f, 0.0f, 0.0f, 0.0f, 1, 0U);
        }
        App_SetControlMode(ROBOT_MODE_STAND);
        pc_policy_was_active = 0U;
    }

    AppMotion_UpdateFootTargets(dt_s);
    Gait_UpdateTargetAngleFromFootTarget(Target_Angle);
    App_all_motor_claculate(Target_Angle, legs);
    AppMotor_PublishCommandSnapshot();
}

void App_Robot_Send_Loop(void)
{
}
