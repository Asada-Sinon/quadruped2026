#include "app_motor_control.h"

#include "FreeRTOS.h"
#include "M8010.h"
#include "app_robot.h"
#include "gait.h"
#include "robot_math.h"
#include "task.h"

#define APP_INTERP_DT_MS          1.0f
#define APP_INTERP_DT_MAX_MS      20.0f
#define APP_INTERP_MIN_DURATION_MS 300.0f
#define APP_INTERP_MAX_DURATION_MS 2000.0f
#define APP_INTERP_SPEED_RAD_PER_S 0.8f
#define APP_INTERP_TARGET_EPS     1.0e-4f
#define APP_MOTOR_COUNT           (ROBOT_LEG_NUM * MOTORS_PER_LEG)

typedef struct
{
    float Pos;
    float W;
    float T;
    float K_P;
    float K_W;
    unsigned short mode;
    unsigned short id;
} AppMotorCmdSnapshot;

static AppMotorCmdSnapshot g_motor_cmd_snapshot[ROBOT_LEG_NUM][MOTORS_PER_LEG] = {0};
static volatile uint32_t g_motor_cmd_snapshot_seq = 0U;
static volatile uint32_t g_motor_cmd_snapshot_take_seq = 0U;

static uint8_t g_interp_ctx_idx = 0U;
static float g_interp_dt_ms = APP_INTERP_DT_MS;
static float g_interp_start_angle[APP_MOTOR_COUNT] = {0};
static float g_interp_end_angle[APP_MOTOR_COUNT] = {0};
static float g_interp_elapsed_ms[APP_MOTOR_COUNT] = {0};
static float g_interp_duration_ms[APP_MOTOR_COUNT] = {0};
static uint8_t g_interp_inited[APP_MOTOR_COUNT] = {0};
static uint8_t g_kpkw_armed = 0U;

float Target_Angle[ROBOT_LEG_NUM][MOTORS_PER_LEG] = {0};
volatile float g_policy_motor_kp = 0.50f;
volatile float g_policy_motor_kw = 0.01f;
volatile uint8_t g_debug_motor_feedback_ok_count = 0U;
volatile uint32_t g_debug_motor_snapshot_publish_count = 0U;
volatile uint32_t g_debug_motor_send_loop_count = 0U;
volatile uint32_t g_debug_motor_send_cost_ms = 0U;

static uint8_t AppMotor_IsFeedbackValid(const M8010 *motor)
{
    if (motor == 0) {
        return 0U;
    }
    return ((motor->motor_r.PosZeroInited != 0U) && (motor->motor_r.correct != 0)) ? 1U : 0U;
}

static float App_Model_Joint_Angle_To_Motor_Rel(uint8_t leg_idx,
                                                uint8_t motor_idx,
                                                float joint_angle)
{
    uint8_t joint_idx = ROBOT_JOINT_INDEX(leg_idx, motor_idx);
    float trans_dir = g_joint_transmission_sign[joint_idx];

    if (trans_dir == 0.0f) {
        return 0.0f;
    }

    return (joint_angle - g_joint_offset_rad[joint_idx]) / trans_dir;
}

void AppMotor_SetMainMotorCommandDefaults(float T,
                                          float W,
                                          float K_P,
                                          float K_W,
                                          unsigned short mode,
                                          uint8_t reset_pos)
{
    uint8_t leg_idx;
    uint8_t motor_idx;

    for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++) {
        for (motor_idx = 0U; motor_idx < MOTORS_PER_LEG; motor_idx++) {
            M8010 *motor = &legs[leg_idx].motors_peer_leg[motor_idx];

            motor->motor_s.mode = mode;
            motor->motor_s.T = T;
            motor->motor_s.W = W;
            motor->motor_s.K_P = K_P;
            motor->motor_s.K_W = K_W;
            if (reset_pos != 0U) {
                motor->motor_s.Pos = 0.0f;
            }
        }
    }
}

void AppMotor_PublishCommandSnapshot(void)
{
    uint8_t leg_idx;
    uint8_t motor_idx;

    taskENTER_CRITICAL();
    for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++) {
        for (motor_idx = 0U; motor_idx < MOTORS_PER_LEG; motor_idx++) {
            const MotorCmd_t *motor_s = &legs[leg_idx].motors_peer_leg[motor_idx].motor_s;

            g_motor_cmd_snapshot[leg_idx][motor_idx].Pos = motor_s->Pos;
            g_motor_cmd_snapshot[leg_idx][motor_idx].W = motor_s->W;
            g_motor_cmd_snapshot[leg_idx][motor_idx].T = motor_s->T;
            g_motor_cmd_snapshot[leg_idx][motor_idx].K_P = motor_s->K_P;
            g_motor_cmd_snapshot[leg_idx][motor_idx].K_W = motor_s->K_W;
            g_motor_cmd_snapshot[leg_idx][motor_idx].mode = motor_s->mode;
            g_motor_cmd_snapshot[leg_idx][motor_idx].id = motor_s->id;
        }
    }
    g_motor_cmd_snapshot_seq++;
    g_debug_motor_snapshot_publish_count++;
    taskEXIT_CRITICAL();
}

static void AppMotor_CopyCommandSnapshotToLocal(
    AppMotorCmdSnapshot local[ROBOT_LEG_NUM][MOTORS_PER_LEG])
{
    uint8_t leg_idx;
    uint8_t motor_idx;

    if (local == 0) {
        return;
    }

    taskENTER_CRITICAL();
    for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++) {
        for (motor_idx = 0U; motor_idx < MOTORS_PER_LEG; motor_idx++) {
            local[leg_idx][motor_idx] = g_motor_cmd_snapshot[leg_idx][motor_idx];
        }
    }
    g_motor_cmd_snapshot_take_seq = g_motor_cmd_snapshot_seq;
    taskEXIT_CRITICAL();
}

static void AppMotor_ApplyCommandSnapshotToSendBuffer(
    AppMotorCmdSnapshot local[ROBOT_LEG_NUM][MOTORS_PER_LEG])
{
    uint8_t leg_idx;
    uint8_t motor_idx;

    if (local == 0) {
        return;
    }

    for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++) {
        for (motor_idx = 0U; motor_idx < MOTORS_PER_LEG; motor_idx++) {
            uint8_t cmd_idx = (uint8_t)(leg_idx * MOTORS_PER_LEG + motor_idx);
            MotorCmd_t motor_s;

            motor_s.Pos = local[leg_idx][motor_idx].Pos;
            motor_s.W = local[leg_idx][motor_idx].W;
            motor_s.T = local[leg_idx][motor_idx].T;
            motor_s.K_P = local[leg_idx][motor_idx].K_P;
            motor_s.K_W = local[leg_idx][motor_idx].K_W;
            motor_s.mode = local[leg_idx][motor_idx].mode;
            motor_s.id = local[leg_idx][motor_idx].id;
            set_cmd_by_index(cmd_idx, &motor_s);
        }
    }
}

void App_Robot_MotorSendLoop(void)
{
    AppMotorCmdSnapshot local[ROBOT_LEG_NUM][MOTORS_PER_LEG];
    uint32_t t0;

    AppMotor_CopyCommandSnapshotToLocal(local);
    AppMotor_ApplyCommandSnapshotToSendBuffer(local);

    t0 = HAL_GetTick();
    send_data_all(legs);
    g_debug_motor_send_cost_ms = HAL_GetTick() - t0;
    g_debug_motor_send_loop_count++;
}

void AppMotor_ResetInterpolationState(void)
{
    uint8_t idx;

    for (idx = 0U; idx < APP_MOTOR_COUNT; idx++) {
        g_interp_start_angle[idx] = 0.0f;
        g_interp_end_angle[idx] = 0.0f;
        g_interp_elapsed_ms[idx] = 0.0f;
        g_interp_duration_ms[idx] = APP_INTERP_MIN_DURATION_MS;
        g_interp_inited[idx] = 0U;
    }
}

void AppMotor_SetInterpolationDtMs(float dt_ms)
{
    g_interp_dt_ms = Robot_ClampF(dt_ms, APP_INTERP_DT_MS, APP_INTERP_DT_MAX_MS);
}

uint8_t AppMotor_AllFeedbackValid(void)
{
    uint8_t leg_idx;
    uint8_t motor_idx;
    uint8_t ok_count = 0U;

    for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++) {
        for (motor_idx = 0U; motor_idx < MOTORS_PER_LEG; motor_idx++) {
            if (AppMotor_IsFeedbackValid(&legs[leg_idx].motors_peer_leg[motor_idx]) != 0U) {
                ok_count++;
            }
        }
    }

    g_debug_motor_feedback_ok_count = ok_count;
    return (ok_count >= APP_MOTOR_COUNT) ? 1U : 0U;
}

uint8_t AppMotor_IsKpKwArmed(void)
{
    return g_kpkw_armed;
}

void AppMotor_SetKpKwArmed(uint8_t armed)
{
    g_kpkw_armed = (armed != 0U) ? 1U : 0U;
}

uint8_t AppMotor_GetFeedbackOkCount(void)
{
    return g_debug_motor_feedback_ok_count;
}

float App_Get_Model_Joint_Angle(uint8_t leg_idx,
                                uint8_t motor_idx,
                                const M8010 *motor)
{
    uint8_t joint_idx;
    float trans_dir;

    if ((motor == 0) || (leg_idx >= ROBOT_LEG_NUM) || (motor_idx >= MOTORS_PER_LEG)) {
        return 0.0f;
    }

    joint_idx = ROBOT_JOINT_INDEX(leg_idx, motor_idx);
    trans_dir = g_joint_transmission_sign[joint_idx];
    return trans_dir * motor->motor_r.PosRel + g_joint_offset_rad[joint_idx];
}

void App_Get_Model_Joint_Angles(float q_urdf_out[J_NUM])
{
    uint8_t leg_idx;
    uint8_t motor_idx;

    if (q_urdf_out == 0) {
        return;
    }

    for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++) {
        for (motor_idx = 0U; motor_idx < MOTORS_PER_LEG; motor_idx++) {
            uint8_t joint_idx = ROBOT_JOINT_INDEX(leg_idx, motor_idx);
            M8010 *motor = &legs[leg_idx].motors_peer_leg[motor_idx];
            q_urdf_out[joint_idx] = App_Get_Model_Joint_Angle(leg_idx, motor_idx, motor);
        }
    }
}

void App_Get_Model_Joint_Velocities(float qd_urdf_out[J_NUM])
{
    uint8_t leg_idx;
    uint8_t motor_idx;

    if (qd_urdf_out == 0) {
        return;
    }

    for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++) {
        for (motor_idx = 0U; motor_idx < MOTORS_PER_LEG; motor_idx++) {
            uint8_t joint_idx = ROBOT_JOINT_INDEX(leg_idx, motor_idx);
            M8010 *motor = &legs[leg_idx].motors_peer_leg[motor_idx];
            float pos_rel_vel = (motor->motor_r.W / ROBOT_MOTOR_GEAR_RATIO) * (float)motor->sign;
            qd_urdf_out[joint_idx] = g_joint_transmission_sign[joint_idx] * pos_rel_vel;
        }
    }
}

float App_motor_angle_calculate(float target_angle, float pos_rel)
{
    uint8_t idx = g_interp_ctx_idx;
    float delta;
    float s;
    float s2;
    float s3;
    float s4;
    float s5;
    float blend;

    if (idx >= APP_MOTOR_COUNT) {
        return pos_rel;
    }

    if (g_interp_inited[idx] == 0U) {
        g_interp_start_angle[idx] = pos_rel;
        g_interp_end_angle[idx] = target_angle;
        delta = g_interp_end_angle[idx] - g_interp_start_angle[idx];
        g_interp_duration_ms[idx] =
            Robot_ClampF(Robot_AbsF(delta) / APP_INTERP_SPEED_RAD_PER_S * 1000.0f,
                         APP_INTERP_MIN_DURATION_MS,
                         APP_INTERP_MAX_DURATION_MS);
        g_interp_elapsed_ms[idx] = 0.0f;
        g_interp_inited[idx] = 1U;
    }

    if (Robot_AbsF(target_angle - g_interp_end_angle[idx]) > APP_INTERP_TARGET_EPS) {
        g_interp_start_angle[idx] = pos_rel;
        g_interp_end_angle[idx] = target_angle;
        delta = g_interp_end_angle[idx] - g_interp_start_angle[idx];
        g_interp_duration_ms[idx] =
            Robot_ClampF(Robot_AbsF(delta) / APP_INTERP_SPEED_RAD_PER_S * 1000.0f,
                         APP_INTERP_MIN_DURATION_MS,
                         APP_INTERP_MAX_DURATION_MS);
        g_interp_elapsed_ms[idx] = 0.0f;
    }

    if (g_interp_elapsed_ms[idx] < g_interp_duration_ms[idx]) {
        g_interp_elapsed_ms[idx] += g_interp_dt_ms;
        if (g_interp_elapsed_ms[idx] > g_interp_duration_ms[idx]) {
            g_interp_elapsed_ms[idx] = g_interp_duration_ms[idx];
        }
    }

    s = Robot_ClampF(g_interp_elapsed_ms[idx] / g_interp_duration_ms[idx], 0.0f, 1.0f);
    s2 = s * s;
    s3 = s2 * s;
    s4 = s3 * s;
    s5 = s4 * s;

    blend = 10.0f * s3 - 15.0f * s4 + 6.0f * s5;
    return g_interp_start_angle[idx] + (g_interp_end_angle[idx] - g_interp_start_angle[idx]) * blend;
}

float App_target_relative_to_absolute(float pos_rel,
                                      float target_angle_rel,
                                      float pos_abs,
                                      int sign)
{
    float delta_rel = target_angle_rel - pos_rel;
    float dir = (sign >= 0) ? 1.0f : -1.0f;

    return pos_abs + dir * delta_rel * ROBOT_MOTOR_GEAR_RATIO;
}

static void App_all_motor_calculate_internal(float target_angle[ROBOT_LEG_NUM][MOTORS_PER_LEG],
                                             Leg leg[ROBOT_LEG_NUM],
                                             uint8_t use_stand_interpolation)
{
    uint8_t leg_idx;
    uint8_t motor_idx;

    if ((target_angle == 0) || (leg == 0)) {
        return;
    }

    for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++) {
        for (motor_idx = 0U; motor_idx < MOTORS_PER_LEG; motor_idx++) {
            uint8_t cmd_idx = (uint8_t)(leg_idx * MOTORS_PER_LEG + motor_idx);
            M8010 *motor = &leg[leg_idx].motors_peer_leg[motor_idx];
            uint8_t feedback_valid = AppMotor_IsFeedbackValid(motor);
            float current_model_angle = App_Get_Model_Joint_Angle(leg_idx, motor_idx, motor);
            float control_model_angle;
            float target_motor_rel;
            float target_abs;

            if (feedback_valid == 0U) {
                control_model_angle = current_model_angle;
                g_interp_inited[cmd_idx] = 0U;
            } else if ((use_stand_interpolation != 0U) &&
                       (App_GetControlMode() == ROBOT_MODE_STAND)) {
                if (g_kpkw_armed != 0U) {
                    g_interp_ctx_idx = cmd_idx;
                    control_model_angle = App_motor_angle_calculate(target_angle[leg_idx][motor_idx],
                                                                    current_model_angle);
                } else {
                    control_model_angle = current_model_angle;
                    g_interp_inited[cmd_idx] = 0U;
                }
            } else {
                control_model_angle = target_angle[leg_idx][motor_idx];
                g_interp_inited[cmd_idx] = 0U;
            }

            target_motor_rel = App_Model_Joint_Angle_To_Motor_Rel(leg_idx,
                                                                  motor_idx,
                                                                  control_model_angle);
            target_abs = App_target_relative_to_absolute(motor->motor_r.PosRel,
                                                         target_motor_rel,
                                                         motor->motor_r.Pos,
                                                         motor->sign);
            motor->motor_s.Pos = target_abs;
        }
    }
}

void App_all_motor_claculate(float target_angle[ROBOT_LEG_NUM][MOTORS_PER_LEG],
                             Leg leg[ROBOT_LEG_NUM])
{
    App_all_motor_calculate_internal(target_angle, leg, 1U);
}

void App_Set_Model_Joint_Target_Angle(const float q_des_urdf[J_NUM])
{
    float target_angle[ROBOT_LEG_NUM][MOTORS_PER_LEG];
    uint8_t leg_idx;
    uint8_t motor_idx;

    if (q_des_urdf == 0) {
        return;
    }

    for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++) {
        for (motor_idx = 0U; motor_idx < MOTORS_PER_LEG; motor_idx++) {
            uint8_t joint_idx = ROBOT_JOINT_INDEX(leg_idx, motor_idx);
            target_angle[leg_idx][motor_idx] = q_des_urdf[joint_idx];
        }
    }

    App_all_motor_calculate_internal(target_angle, legs, 0U);
}

void App_UpdateCurrentFootPosFromMotor(Leg leg[ROBOT_LEG_NUM])
{
    uint8_t leg_idx;
    uint8_t motor_idx;

    if (leg == 0) {
        return;
    }

    for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++) {
        float joint_pos[MOTORS_PER_LEG];
        float foot_pos[3];
        uint8_t valid = 1U;

        for (motor_idx = 0U; motor_idx < MOTORS_PER_LEG; motor_idx++) {
            M8010 *motor = &leg[leg_idx].motors_peer_leg[motor_idx];

            if (AppMotor_IsFeedbackValid(motor) == 0U) {
                valid = 0U;
                break;
            }

            joint_pos[motor_idx] = App_Get_Model_Joint_Angle(leg_idx, motor_idx, motor);
        }

        if (valid == 0U) {
            continue;
        }

        leg_forward_kinematics(leg_idx, joint_pos, foot_pos);
        g_foot_current_m[leg_idx].x_m = foot_pos[X_IDX];
        g_foot_current_m[leg_idx].y_m = foot_pos[Y_IDX];
        g_foot_current_m[leg_idx].z_m = foot_pos[Z_IDX];
    }
}
