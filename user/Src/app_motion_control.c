#include "app_motion_control.h"

#include "FreeRTOS.h"
#include "HT10A.h"
#include "app_motor_control.h"
#include "gait.h"
#include "robot_math.h"
#include "task.h"
#include "trajectory.h"

#define APP_DEFAULT_STAND_X_M          (-0.0638f)
#define APP_DEFAULT_STAND_Y_M           (0.0780f)
#define APP_DEFAULT_STAND_Z_M          (-0.3230f)
#define APP_DEFAULT_WALK_FREQ_HZ        (1.2f)
#define APP_DEFAULT_WALK_SWING_HEIGHT_M (0.07f)
#define APP_DEFAULT_WALK_STEP_LENGTH_M  (0.10f)
#define APP_CRAWL_Z_OFFSET_M            (0.10f)
#define APP_FREE_RC_AXIS_RAW_LIMIT      (10000.0f)
#define APP_FREE_RC_DEADZONE_RAW        (800.0f)
#define APP_FREE_VX_MAX_M_S             (0.20f)
#define APP_FREE_VY_MAX_M_S             (0.15f)
#define APP_FREE_W_MAX_RAD_S            (0.80f)
#define APP_FREE_VX_SIGN                (1.0f)
#define APP_FREE_VY_SIGN                (-1.0f)
#define APP_FREE_W_SIGN                 (-1.0f)

typedef struct
{
    RobotControlMode mode;
    RobotControlMode last_mode;
    float stand_x_m_by_leg[ROBOT_LEG_NUM];
    float stand_y_m_by_leg[ROBOT_LEG_NUM];
    float stand_z_m_by_leg[ROBOT_LEG_NUM];
    float walk_freq_hz;
    float walk_swing_height_m;
    float walk_step_length_m;
    uint8_t walk_cmd_inited;
} AppControlContext;

static AppControlContext g_app_ctrl = {
    ROBOT_MODE_STAND,
    ROBOT_MODE_STAND,
    {
        APP_DEFAULT_STAND_X_M,
        APP_DEFAULT_STAND_X_M,
        APP_DEFAULT_STAND_X_M,
        APP_DEFAULT_STAND_X_M,
    },
    {
        APP_DEFAULT_STAND_Y_M,
        -APP_DEFAULT_STAND_Y_M,
        APP_DEFAULT_STAND_Y_M,
        -APP_DEFAULT_STAND_Y_M,
    },
    {
        APP_DEFAULT_STAND_Z_M,
        APP_DEFAULT_STAND_Z_M,
        APP_DEFAULT_STAND_Z_M + 0.005f,
        APP_DEFAULT_STAND_Z_M + 0.005f,
    },
    APP_DEFAULT_WALK_FREQ_HZ,
    APP_DEFAULT_WALK_SWING_HEIGHT_M,
    APP_DEFAULT_WALK_STEP_LENGTH_M,
    0U,
};

static DiagonalCycloidGait g_gait;
float bios = 0.015f;

static uint8_t App_IsValidMode(RobotControlMode mode)
{
    return ((mode == ROBOT_MODE_STAND) ||
            (mode == ROBOT_MODE_WALK) ||
            (mode == ROBOT_MODE_CRAWL) ||
            (mode == ROBOT_MODE_FREE_MOVE)) ? 1U : 0U;
}

static void App_SyncWalkNominalFromStandPose(void)
{
    uint8_t leg_idx;

    for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++) {
        Trajectory_SetNominalFoot(&g_gait,
                                  leg_idx,
                                  g_app_ctrl.stand_x_m_by_leg[leg_idx],
                                  g_app_ctrl.stand_y_m_by_leg[leg_idx],
                                  g_app_ctrl.stand_z_m_by_leg[leg_idx]);
    }
}

static void App_UpdateStandFootTarget(void)
{
    uint8_t leg_idx;

    for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++) {
        (void)Gait_SetLegFootTargetM(leg_idx,
                                     g_app_ctrl.stand_x_m_by_leg[leg_idx],
                                     g_app_ctrl.stand_y_m_by_leg[leg_idx],
                                     g_app_ctrl.stand_z_m_by_leg[leg_idx]);
    }
}

static float App_MapRcAxisToUnit(float raw_axis)
{
    float abs_axis;
    float span;

    raw_axis = Robot_ClampF(raw_axis,
                            -APP_FREE_RC_AXIS_RAW_LIMIT,
                            APP_FREE_RC_AXIS_RAW_LIMIT);
    abs_axis = Robot_AbsF(raw_axis);
    if (abs_axis <= APP_FREE_RC_DEADZONE_RAW) {
        return 0.0f;
    }

    span = APP_FREE_RC_AXIS_RAW_LIMIT - APP_FREE_RC_DEADZONE_RAW;
    if (span <= 0.0f) {
        return 0.0f;
    }

    if (raw_axis > 0.0f) {
        return (raw_axis - APP_FREE_RC_DEADZONE_RAW) / span;
    }

    return (raw_axis + APP_FREE_RC_DEADZONE_RAW) / span;
}

static void App_GetFreeMoveVelocityCmd(float *vx_m_s,
                                       float *vy_m_s,
                                       float *w_rad_s)
{
    float rc_vx_raw;
    float rc_vy_raw;
    float rc_w_raw;

    if ((vx_m_s == 0) || (vy_m_s == 0) || (w_rad_s == 0)) {
        return;
    }

    taskENTER_CRITICAL();
    rc_vx_raw = Teaching_Pendant.Vx;
    rc_vy_raw = Teaching_Pendant.Vy;
    rc_w_raw = Teaching_Pendant.Vw;
    taskEXIT_CRITICAL();

    *vx_m_s = APP_FREE_VX_SIGN * App_MapRcAxisToUnit(rc_vx_raw) * APP_FREE_VX_MAX_M_S;
    *vy_m_s = APP_FREE_VY_SIGN * App_MapRcAxisToUnit(rc_vy_raw) * APP_FREE_VY_MAX_M_S;
    *w_rad_s = APP_FREE_W_SIGN * App_MapRcAxisToUnit(rc_w_raw) * APP_FREE_W_MAX_RAD_S;
}

static void App_HandleModeEntry(RobotControlMode mode)
{
    if ((mode == ROBOT_MODE_WALK) ||
        (mode == ROBOT_MODE_CRAWL) ||
        (mode == ROBOT_MODE_FREE_MOVE)) {
        App_SyncWalkNominalFromStandPose();
        if ((g_app_ctrl.walk_cmd_inited == 0U) &&
            (AppMotor_IsKpKwArmed() != 0U)) {
            AppMotor_SetMainMotorCommandDefaults(0.0f, 0.0f, 0.2f, 0.0f, 1U, 1U);
            g_app_ctrl.walk_cmd_inited = 1U;
        }
        return;
    }

    if (mode == ROBOT_MODE_STAND) {
        Trajectory_Reset(&g_gait);
        AppMotor_ResetInterpolationState();
        g_app_ctrl.walk_cmd_inited = 0U;
    }
}

static void App_UpdateWalkFootTarget(float dt_s)
{
    Trajectory_SetStepLength(&g_gait, g_app_ctrl.walk_step_length_m);
    Trajectory_SetFrequency(&g_gait, g_app_ctrl.walk_freq_hz);
    Trajectory_SetSwingHeight(&g_gait, g_app_ctrl.walk_swing_height_m);
    Trajectory_SetBodyVelocity(&g_gait, 0.0f, 0.0f, 0.0f);
    Trajectory_Update(&g_gait, dt_s);
}

static void App_AddFootTargetZOffset(float offset_m)
{
    uint8_t leg_idx;

    for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++) {
        (void)Gait_SetLegFootTargetM(leg_idx,
                                     g_foot_target_m[leg_idx].x_m,
                                     g_foot_target_m[leg_idx].y_m,
                                     g_foot_target_m[leg_idx].z_m + offset_m);
    }
}

static void App_UpdateCrawlFootTarget(float dt_s)
{
    App_UpdateWalkFootTarget(dt_s);
    App_AddFootTargetZOffset(APP_CRAWL_Z_OFFSET_M);
}

static void App_UpdateFreeMoveFootTarget(float dt_s)
{
    float vx_cmd_m_s;
    float vy_cmd_m_s;
    float w_cmd_rad_s;

    App_GetFreeMoveVelocityCmd(&vx_cmd_m_s, &vy_cmd_m_s, &w_cmd_rad_s);

    Trajectory_SetFrequency(&g_gait, g_app_ctrl.walk_freq_hz);
    Trajectory_SetSwingHeight(&g_gait, g_app_ctrl.walk_swing_height_m);
    Trajectory_SetStepLength(&g_gait, 0.0f);
    Trajectory_SetBodyVelocity(&g_gait, vx_cmd_m_s, vy_cmd_m_s, w_cmd_rad_s);
    Trajectory_Update(&g_gait, dt_s);

    App_AddFootTargetZOffset(APP_CRAWL_Z_OFFSET_M);
}

void AppMotion_Init(void)
{
    Trajectory_InitDefault(&g_gait);
    App_SyncWalkNominalFromStandPose();
    g_app_ctrl.last_mode = g_app_ctrl.mode;
    g_app_ctrl.walk_cmd_inited = 0U;
}

void AppMotion_UpdateFootTargets(float dt_s)
{
    Trajectory_SetFrontSwingHeightBias(&g_gait, bios);

    if (g_app_ctrl.mode != g_app_ctrl.last_mode) {
        App_HandleModeEntry(g_app_ctrl.mode);
        g_app_ctrl.last_mode = g_app_ctrl.mode;
    }

    switch (g_app_ctrl.mode) {
    case ROBOT_MODE_STAND:
        App_UpdateStandFootTarget();
        break;
    case ROBOT_MODE_WALK:
        App_UpdateWalkFootTarget(dt_s);
        break;
    case ROBOT_MODE_CRAWL:
        App_UpdateCrawlFootTarget(dt_s);
        break;
    case ROBOT_MODE_FREE_MOVE:
        App_UpdateFreeMoveFootTarget(dt_s);
        break;
    default:
        g_app_ctrl.mode = ROBOT_MODE_STAND;
        App_HandleModeEntry(g_app_ctrl.mode);
        g_app_ctrl.last_mode = g_app_ctrl.mode;
        App_UpdateStandFootTarget();
        break;
    }
}

void App_SetControlMode(RobotControlMode mode)
{
    if (App_IsValidMode(mode) == 0U) {
        return;
    }
    g_app_ctrl.mode = mode;
}

RobotControlMode App_GetControlMode(void)
{
    return g_app_ctrl.mode;
}

void App_SetWalkParams(float freq_hz,
                       float step_length_m,
                       float swing_height_m)
{
    if (freq_hz < 0.0f) {
        freq_hz = 0.0f;
    }
    if (swing_height_m < 0.0f) {
        swing_height_m = 0.0f;
    }

    g_app_ctrl.walk_freq_hz = freq_hz;
    g_app_ctrl.walk_step_length_m = step_length_m;
    g_app_ctrl.walk_swing_height_m = swing_height_m;
}

void App_SetStandPose(const float stand_x_m_by_leg[ROBOT_LEG_NUM],
                      const float stand_y_m_by_leg[ROBOT_LEG_NUM],
                      const float stand_z_m_by_leg[ROBOT_LEG_NUM])
{
    uint8_t leg_idx;

    if (stand_x_m_by_leg != 0) {
        for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++) {
            g_app_ctrl.stand_x_m_by_leg[leg_idx] = stand_x_m_by_leg[leg_idx];
        }
    }
    if (stand_y_m_by_leg != 0) {
        for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++) {
            g_app_ctrl.stand_y_m_by_leg[leg_idx] = stand_y_m_by_leg[leg_idx];
        }
    }
    if (stand_z_m_by_leg != 0) {
        for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++) {
            g_app_ctrl.stand_z_m_by_leg[leg_idx] = stand_z_m_by_leg[leg_idx];
        }
    }

    if (g_app_ctrl.mode == ROBOT_MODE_STAND) {
        App_SyncWalkNominalFromStandPose();
    }
}
