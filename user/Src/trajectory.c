#include "trajectory.h"
#include <math.h>

#define TRAJ_MAX_STEP_LENGTH_M 0.100f
#define TRAJ_MAX_SWING_HEIGHT_M 0.200f
/* FREE_MOVE 机体速度限幅（与遥控保守标定一致）。 */
#define TRAJ_MAX_BODY_VX_M_S 0.20f
#define TRAJ_MAX_BODY_VY_M_S 0.15f
#define TRAJ_MAX_BODY_W_RAD_S 0.80f
#define TRAJ_MIN_EFFECTIVE_FREQ_HZ 1.0e-4f
/* 前腿摆动高度偏置的上限保护，避免叠加后抬脚过大。 */
#define TRAJ_MAX_FRONT_SWING_BIAS_M 0.100f
/* 默认前腿在摆动相比后腿额外多抬 1.5cm。 */
#define TRAJ_DEFAULT_FRONT_SWING_BIAS_M 0.015f
#define TRAJ_SWING_PHASE_RATIO 0.50f
#define TRAJ_STANCE_PHASE_RATIO (1.0f - TRAJ_SWING_PHASE_RATIO)
#define TRAJ_DIAGONAL_PHASE_OFFSET 0.50f
#define TRAJ_TWO_PI (2.0f * GAIT_PI)

static float traj_clampf(float x, float min_val, float max_val)
{
    if (x < min_val)
    {
        return min_val;
    }
    if (x > max_val)
    {
        return max_val;
    }
    return x;
}

/* 对二维步幅做限幅：
 * 1) x/y 分量先各自限幅；
 * 2) 再限制平面向量模长不超过 TRAJ_MAX_STEP_LENGTH_M，
 *    防止组合运动（平移+自转）导致单腿步幅过大。
 */
static void Trajectory_ClampPlanarStep(float *step_x_m, float *step_y_m)
{
    float mag;
    float scale;

    if ((step_x_m == 0) || (step_y_m == 0))
    {
        return;
    }

    *step_x_m = traj_clampf(*step_x_m, -TRAJ_MAX_STEP_LENGTH_M, TRAJ_MAX_STEP_LENGTH_M);
    *step_y_m = traj_clampf(*step_y_m, -TRAJ_MAX_STEP_LENGTH_M, TRAJ_MAX_STEP_LENGTH_M);

    mag = sqrtf((*step_x_m) * (*step_x_m) + (*step_y_m) * (*step_y_m));
    if (mag > TRAJ_MAX_STEP_LENGTH_M)
    {
        scale = TRAJ_MAX_STEP_LENGTH_M / mag;
        *step_x_m *= scale;
        *step_y_m *= scale;
    }
}

/* 初始化运行时参考点 */
static void Trajectory_RuntimeInit(FootTrajRuntime *rt)
{
    uint8_t i;

    if (rt == 0)
    {
        return;
    }

    for (i = 0U; i < ROBOT_LEG_NUM; i++)
    {
        rt->phase_anchor[i].x_m = 0.0f;
        rt->phase_anchor[i].y_m = 0.0f;
        rt->phase_anchor[i].z_m = 0.0f;
        rt->last_state[i] = STEP_LEG_STANCE;
        rt->valid[i] = 0U;
    }
}

/* 摆线进度函数：
 * p(s) = s - sin(2*pi*s)/(2*pi), s in [0,1]
 * p(0)=0, p(1)=1，且端点速度为 0。
 * 用它做 x 方向的相位映射，可避免端点速度突变。
 */
static float Trajectory_CycloidProgress(float s)
{
    return s - sinf(TRAJ_TWO_PI * s) / TRAJ_TWO_PI;
}

/* 摆线抬脚包络：
 * l(s) = 0.5 * (1 - cos(2*pi*s)), s in [0,1]
 * l(0)=0, l(1)=0, l(0.5)=1，形成中间抬高、两端贴地。
 */
static float Trajectory_CycloidLift(float s)
{
    return 0.5f * (1.0f - cosf(TRAJ_TWO_PI * s));
}

/* 步态调度层：
 * 1) 每条腿单独看：摆动相 0.4，支撑相 0.6
 * 2) FL+HR 与 FR+HL 相差 0.5 周期（对角交替）
 *
 * 这样仍然保持“先左前+右后，再右前+左后”的顺序，
 * 同时把每条腿的支撑/摆动时间占比改成 0.6/0.4。
 */
static void Trajectory_GetLegPhase(uint8_t leg_idx,
                                   float gait_phase,
                                   StepLegState *leg_state,
                                   float *local_phase)
{
    float phase_offset;
    float leg_phase;

    if ((leg_state == 0) || (local_phase == 0))
    {
        return;
    }

    /* 对角两组腿保持半周期错相。 */
    if ((leg_idx == LEG_FL) || (leg_idx == LEG_HR))
    {
        phase_offset = 0.0f;
    }
    else
    {
        phase_offset = TRAJ_DIAGONAL_PHASE_OFFSET;
    }

    leg_phase = gait_phase + phase_offset;
    while (leg_phase >= 1.0f)
    {
        leg_phase -= 1.0f;
    }
    while (leg_phase < 0.0f)
    {
        leg_phase += 1.0f;
    }

    if (leg_phase < TRAJ_SWING_PHASE_RATIO)
    {
        *leg_state = STEP_LEG_SWING;
        *local_phase = leg_phase / TRAJ_SWING_PHASE_RATIO;
    }
    else
    {
        *leg_state = STEP_LEG_STANCE;
        *local_phase = (leg_phase - TRAJ_SWING_PHASE_RATIO) / TRAJ_STANCE_PHASE_RATIO;
    }

    *local_phase = traj_clampf(*local_phase, 0.0f, 1.0f);
}
static GaitFootPosM Trajectory_GenerateNominalCenteredFoot(const GaitFootPosM *nominal,
                                                           StepLegState leg_state,
                                                           float local_phase,
                                                           float step_x_m,
                                                           float step_y_m,
                                                           float swing_height_m)
{
    GaitFootPosM out;
    float s;
    float prog;
    float lift;
    float half_step_x;
    float half_step_y;
    float x_rear;
    float x_front;
    float y_rear;
    float y_front;

    if (nominal == 0)
    {
        out.x_m = 0.0f;
        out.y_m = 0.0f;
        out.z_m = 0.0f;
        return out;
    }

    /* 轨迹参数限幅，防止上层给过大值导致足端冲到极限位。 */
    Trajectory_ClampPlanarStep(&step_x_m, &step_y_m);
    swing_height_m = traj_clampf(swing_height_m, 0.0f, TRAJ_MAX_SWING_HEIGHT_M);

    s = traj_clampf(local_phase, 0.0f, 1.0f);
    prog = Trajectory_CycloidProgress(s);
    lift = Trajectory_CycloidLift(s);
    half_step_x = 0.5f * step_x_m;
    half_step_y = 0.5f * step_y_m;
    x_rear = nominal->x_m - half_step_x;
    x_front = nominal->x_m + half_step_x;
    y_rear = nominal->y_m - half_step_y;
    y_front = nominal->y_m + half_step_y;

    if (leg_state == STEP_LEG_SWING)
    {
        /* 摆动相：x/y 同步用摆线从后到前，z 用摆线包络抬脚。 */
        out.x_m = x_rear + step_x_m * prog;
        out.y_m = y_rear + step_y_m * prog;
        out.z_m = nominal->z_m + swing_height_m * lift;
    }
    else
    {
        /* 支撑相：x/y 同步用摆线从前回到后，z 贴地。 */
        out.x_m = x_front - step_x_m * prog;
        out.y_m = y_front - step_y_m * prog;
        out.z_m = nominal->z_m;
    }

    return out;
}
void Trajectory_Update(DiagonalCycloidGait *gait,
                       float dt_s)
{
    uint8_t leg_idx;
    if (dt_s <= 0.0f)
    {
        return;
    }
    gait->gait_phase += gait->freq_hz * dt_s;

    while (gait->gait_phase >= 1.0f)
    {
        gait->gait_phase -= 1.0f;
    }
    while (gait->gait_phase < 0.0f)
    {
        gait->gait_phase += 1.0f;
    }
    for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++)
    {
        StepLegState leg_state;
        float local_phase;
        float leg_swing_height_m;
        float leg_step_x_m;
        float leg_step_y_m;
        float leg_rx_m;
        float leg_ry_m;
        float inv_freq_hz;
        GaitFootPosM foot_target;

        Trajectory_GetLegPhase(leg_idx,
                               gait->gait_phase,
                               &leg_state,
                               &local_phase);

        /* 仅在摆动相给前腿加额外抬脚偏置，后腿维持基础抬脚高度。 */
        leg_swing_height_m = gait->swing_height_m;
        if ((leg_state == STEP_LEG_SWING) && ((leg_idx == LEG_FL) || (leg_idx == LEG_FR)))
        {
            leg_swing_height_m += gait->front_swing_height_bias_m;
        }

        /* 组合步幅：
         * - 基础前后步幅来自 step_length_m（兼容 WALK/CRAWL）；
         * - FREE_MOVE 可通过 body_vx/body_vy/body_w 叠加二维平移与自转。
         */
        leg_step_x_m = gait->step_length_m;
        leg_step_y_m = 0.0f;

        if ((gait->freq_hz > TRAJ_MIN_EFFECTIVE_FREQ_HZ) || (gait->freq_hz < -TRAJ_MIN_EFFECTIVE_FREQ_HZ))
        {
            inv_freq_hz = 1.0f / gait->freq_hz;
            leg_rx_m = gait->nominal[leg_idx].x_m;
            leg_ry_m = gait->nominal[leg_idx].y_m;

            /* step = body velocity at leg point / freq */
            leg_step_x_m += (gait->body_vx_m_s - gait->body_w_rad_s * leg_ry_m) * inv_freq_hz;
            leg_step_y_m += (gait->body_vy_m_s + gait->body_w_rad_s * leg_rx_m) * inv_freq_hz;
        }

        Trajectory_ClampPlanarStep(&leg_step_x_m, &leg_step_y_m);

        foot_target = Trajectory_GenerateNominalCenteredFoot(&gait->nominal[leg_idx],
                                                             leg_state,
                                                             local_phase,
                                                             leg_step_x_m,
                                                             leg_step_y_m,
                                                             leg_swing_height_m);
        Gait_SetLegFootTargetM(leg_idx,
                               foot_target.x_m,
                               foot_target.y_m,
                               foot_target.z_m);
    }
}
void Trajectory_InitDefault(DiagonalCycloidGait *gait)
{
    gait->gait_phase = 0.0f;
    gait->freq_hz = 1.2f;
    gait->step_length_m = 0.10f; /* 默认原地踏步 */
    gait->swing_height_m = 0.07f;
    gait->body_vx_m_s = 0.0f;
    gait->body_vy_m_s = 0.0f;
    gait->body_w_rad_s = 0.0f;
    /* 让前腿在摆动相默认抬得比后腿更高一些。 */
    gait->front_swing_height_bias_m = TRAJ_DEFAULT_FRONT_SWING_BIAS_M;

    gait->nominal[LEG_FL].x_m = -0.0638f;
    gait->nominal[LEG_FL].y_m = 0.0780f;
    gait->nominal[LEG_FL].z_m = -0.3230f;

    gait->nominal[LEG_FR].x_m = -0.0638f;
    gait->nominal[LEG_FR].y_m = -0.0780f;
    gait->nominal[LEG_FR].z_m = -0.3230f;

    gait->nominal[LEG_HL].x_m = -0.0638f;
    gait->nominal[LEG_HL].y_m = 0.0780f;
    gait->nominal[LEG_HL].z_m = -0.3230f;

    gait->nominal[LEG_HR].x_m = -0.0638f;
    gait->nominal[LEG_HR].y_m = -0.0780f;
    gait->nominal[LEG_HR].z_m = -0.3230f;

    Trajectory_RuntimeInit(&gait->runtime);
}

void Trajectory_Reset(DiagonalCycloidGait *gait)
{
    if (gait == 0)
    {
        return;
    }

    gait->gait_phase = 0.0f;
    Trajectory_RuntimeInit(&gait->runtime);
}

void Trajectory_SetNominalFoot(DiagonalCycloidGait *gait,
                               uint8_t leg_idx,
                               float x_m,
                               float y_m,
                               float z_m)
{
    if (gait == 0)
    {
        return;
    }

    if (leg_idx >= ROBOT_LEG_NUM)
    {
        return;
    }

    gait->nominal[leg_idx].x_m = x_m;
    gait->nominal[leg_idx].y_m = y_m;
    gait->nominal[leg_idx].z_m = z_m;
}

void Trajectory_SetFrequency(DiagonalCycloidGait *gait, float freq_hz)
{
    if (gait == 0)
    {
        return;
    }

    if (freq_hz < 0.0f)
    {
        freq_hz = 0.0f;
    }

    gait->freq_hz = freq_hz;
}

void Trajectory_SetStepLength(DiagonalCycloidGait *gait, float step_length_m)
{
    if (gait == 0)
    {
        return;
    }

    gait->step_length_m = traj_clampf(step_length_m,
                                      -TRAJ_MAX_STEP_LENGTH_M,
                                      TRAJ_MAX_STEP_LENGTH_M);
}

void Trajectory_SetSwingHeight(DiagonalCycloidGait *gait, float swing_height_m)
{
    if (gait == 0)
    {
        return;
    }

    if (swing_height_m < 0.0f)
    {
        swing_height_m = 0.0f;
    }

    gait->swing_height_m = traj_clampf(swing_height_m,
                                       0.0f,
                                       TRAJ_MAX_SWING_HEIGHT_M);
}

void Trajectory_SetBodyVelocity(DiagonalCycloidGait *gait,
                                float body_vx_m_s,
                                float body_vy_m_s,
                                float body_w_rad_s)
{
    if (gait == 0)
    {
        return;
    }

    gait->body_vx_m_s = traj_clampf(body_vx_m_s,
                                    -TRAJ_MAX_BODY_VX_M_S,
                                    TRAJ_MAX_BODY_VX_M_S);
    gait->body_vy_m_s = traj_clampf(body_vy_m_s,
                                    -TRAJ_MAX_BODY_VY_M_S,
                                    TRAJ_MAX_BODY_VY_M_S);
    gait->body_w_rad_s = traj_clampf(body_w_rad_s,
                                     -TRAJ_MAX_BODY_W_RAD_S,
                                     TRAJ_MAX_BODY_W_RAD_S);
}

void Trajectory_SetFrontSwingHeightBias(DiagonalCycloidGait *gait, float front_bias_m)
{
    if (gait == 0)
    {
        return;
    }

    if (front_bias_m < 0.0f)
    {
        front_bias_m = 0.0f;
    }

    /* 该偏置只会在前腿(FL/FR)摆动相被叠加，支撑相与后腿不受影响。 */
    gait->front_swing_height_bias_m = traj_clampf(front_bias_m,
                                                   0.0f,
                                                   TRAJ_MAX_FRONT_SWING_BIAS_M);
}
