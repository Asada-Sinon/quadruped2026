#include "trajectory.h"
#include <math.h>

#define TRAJ_MAX_STEP_LENGTH_M 0.100f
#define TRAJ_MAX_SWING_HEIGHT_M 0.200f
#define TRAJ_SWING_PHASE_RATIO 0.40f
#define TRAJ_STANCE_PHASE_RATIO (1.0f - TRAJ_SWING_PHASE_RATIO)
#define TRAJ_DIAGONAL_PHASE_OFFSET 0.50f

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

/* 五次多项式混合函数（minimum-jerk）：
 * s=0 -> 0, s=1 -> 1，且两端速度/加速度都为 0。
 * 用它做 x 方向插值，轨迹会比线性过渡更平滑。
 */
static float Trajectory_Poly5Blend(float s)
{
    float s2 = s * s;
    float s3 = s2 * s;
    float s4 = s3 * s;
    float s5 = s4 * s;
    return 10.0f * s3 - 15.0f * s4 + 6.0f * s5;
}

/* 四次多项式抬脚包络：
 * 16*s^2*(1-s)^2 在 s=0 和 s=1 时为 0，在 s=0.5 时为 1。
 * 这个函数专门用来生成“中间高、两端低”的 z 抬脚形状。
 */
static float Trajectory_Poly4Lift(float s)
{
    float one_minus_s = 1.0f - s;
    return 16.0f * s * s * one_minus_s * one_minus_s;
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
GaitFootPosM out;
float s;
float blend;
float lift;
float half_step;
float x_rear;
float x_front;
static GaitFootPosM Trajectory_GenerateNominalCenteredFoot(const GaitFootPosM *nominal,
                                                           StepLegState leg_state,
                                                           float local_phase,
                                                           float step_length_m,
                                                           float swing_height_m)
{
    // GaitFootPosM out;
    // float s;
    // float blend;
    // float lift;
    // float half_step;
    // float x_rear;
    // float x_front;

    if (nominal == 0)
    {
        out.x_m = 0.0f;
        out.y_m = 0.0f;
        out.z_m = 0.0f;
        return out;
    }

    /* 轨迹参数限幅，防止上层给过大值导致足端冲到极限位。 */
    step_length_m = traj_clampf(step_length_m, -TRAJ_MAX_STEP_LENGTH_M, TRAJ_MAX_STEP_LENGTH_M);
    swing_height_m = traj_clampf(swing_height_m, 0.0f, TRAJ_MAX_SWING_HEIGHT_M);

    s = traj_clampf(local_phase, 0.0f, 1.0f);
    blend = Trajectory_Poly5Blend(s);
    lift = Trajectory_Poly4Lift(s);
    half_step = 0.5f * step_length_m;
    x_rear = nominal->x_m - half_step;
    x_front = nominal->x_m + half_step;

    /* 固定围绕 nominal 做周期运动 */
    out.y_m = nominal->y_m;

    if (leg_state == STEP_LEG_SWING)
    {
        /* 摆动相：x 从后向前走五次多项式，z 用四次多项式抬脚。 */
        // out.x_m = x_rear + step_length_m * blend;
        // out.z_m = nominal->z_m + swing_height_m * lift;
        out.x_m = nominal->x_m + 0.4f*step_length_m/2.0f;
        out.z_m = nominal->z_m + swing_height_m;
    }
    else
    {
        /* 支撑相：x 从前向后做五次多项式回扫，z 贴地。 */
        //out.x_m = x_front - step_length_m * blend;
        out.z_m = nominal->z_m;
        out.x_m = nominal->x_m - 0.6f*step_length_m/2.0f;
        // out.z_m = nominal->z_m ;
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
        GaitFootPosM foot_target;

        Trajectory_GetLegPhase(leg_idx,
                               gait->gait_phase,
                               &leg_state,
                               &local_phase);
        foot_target = Trajectory_GenerateNominalCenteredFoot(&gait->nominal[leg_idx],
                                                             leg_state,
                                                             local_phase,
                                                             gait->step_length_m,
                                                             gait->swing_height_m);
        Gait_SetLegFootTargetM(leg_idx,
                               foot_target.x_m,
                               foot_target.y_m,
                               foot_target.z_m);
    }
}
void Trajectory_InitDefault(DiagonalCycloidGait *gait)
{
    gait->gait_phase = 0.0f;
    gait->freq_hz = 0.8f;
    gait->step_length_m = 0.0f; /* 默认原地踏步 */
    gait->swing_height_m = 0.030f;

    gait->nominal[LEG_FL].x_m = -0.0638f;
    gait->nominal[LEG_FL].y_m = 0.0780f;
    gait->nominal[LEG_FL].z_m = -0.3230f;

    gait->nominal[LEG_FR].x_m = -0.0638f;
    gait->nominal[LEG_FR].y_m = -0.0780f;
    gait->nominal[LEG_FR].z_m = -0.3230f;

    gait->nominal[LEG_HL].x_m = -0.0638f;
    gait->nominal[LEG_HL].y_m = 0.0780f;
    gait->nominal[LEG_HL].z_m = -0.3330f;

    gait->nominal[LEG_HR].x_m = -0.0638f;
    gait->nominal[LEG_HR].y_m = -0.0780f;
    gait->nominal[LEG_HR].z_m = -0.3330f;

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
