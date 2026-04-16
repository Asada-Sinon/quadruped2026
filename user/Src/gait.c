#include "gait.h"

#include <math.h>
#include <string.h>

/*
 * 内部数学常量
 * GAIT_EPS 用于“是否接近 0”的判断，例如奇异判断、参数有效性判断等。
 */
#define GAIT_PI 3.14159265358979323846f
#define GAIT_EPS 1.0e-6f

/* 计算三维向量模长 */
static float gait_vec3_norm(const GaitVec3 *v)
{
    if (v == NULL)
    {
        return 0.0f;
    }
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

/*
 * 五次平滑 S 曲线，定义在归一化相位 s ∈ [0,1]
 *
 * u(s)    = 10*s^3 - 15*s^4 + 6*s^5
 * du/ds   = 30*s^2 - 60*s^3 + 30*s^4
 * d2u/ds2 = 60*s - 180*s^2 + 120*s^3
 *
 * 该曲线在两端速度、加速度都为 0，适合用作平滑支撑相插值。
 */
static void gait_quintic_scurve(float s, float *u, float *du, float *ddu)
{
    float s2;
    float s3;
    float s4;
    float s5;

    s = gait_clampf(s, 0.0f, 1.0f);
    s2 = s * s;
    s3 = s2 * s;
    s4 = s3 * s;
    s5 = s4 * s;

    if (u != NULL)
    {
        *u = 10.0f * s3 - 15.0f * s4 + 6.0f * s5;
    }
    if (du != NULL)
    {
        *du = 30.0f * s2 - 60.0f * s3 + 30.0f * s4;
    }
    if (ddu != NULL)
    {
        *ddu = 60.0f * s - 180.0f * s2 + 120.0f * s3;
    }
}

/*
 * 通用贝塞尔曲线求值（de Casteljau 算法）
 *
 * 采用 de Casteljau 的原因：
 * 1) 数值稳定
 * 2) 容易扩展到一阶、二阶导控制点
 * 3) 不需要显式写组合数
 */
static GaitVec3 gait_bezier_eval(const GaitVec3 *ctrl_pts, uint32_t count, float t)
{
    GaitVec3 work[6];
    GaitVec3 out = {0.0f, 0.0f, 0.0f};
    uint32_t r;
    uint32_t i;

    if ((ctrl_pts == NULL) || (count == 0U))
    {
        return out;
    }

    if (count > 6U)
    {
        count = 6U;
    }

    t = gait_clampf(t, 0.0f, 1.0f);
    for (i = 0U; i < count; i++)
    {
        work[i] = ctrl_pts[i];
    }

    for (r = 1U; r < count; r++)
    {
        for (i = 0U; i < count - r; i++)
        {
            work[i].x = (1.0f - t) * work[i].x + t * work[i + 1U].x;
            work[i].y = (1.0f - t) * work[i].y + t * work[i + 1U].y;
            work[i].z = (1.0f - t) * work[i].z + t * work[i + 1U].z;
        }
    }

    return work[0];
}

/*
 * 3x3 矩阵求逆（伴随矩阵 / 行列式公式）
 *
 * 当矩阵接近奇异（|det| < GAIT_EPS）时返回 0。
 */
static uint8_t gait_inv3x3(float A[3][3], float invA[3][3])
{
    float a = A[0][0];
    float b = A[0][1];
    float c = A[0][2];
    float d = A[1][0];
    float e = A[1][1];
    float f = A[1][2];
    float g = A[2][0];
    float h = A[2][1];
    float i = A[2][2];
    float det;
    float inv_det;

    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    if (fabsf(det) < GAIT_EPS)
    {
        return 0U;
    }

    inv_det = 1.0f / det;
    invA[0][0] = (e * i - f * h) * inv_det;
    invA[0][1] = (c * h - b * i) * inv_det;
    invA[0][2] = (b * f - c * e) * inv_det;

    invA[1][0] = (f * g - d * i) * inv_det;
    invA[1][1] = (a * i - c * g) * inv_det;
    invA[1][2] = (c * d - a * f) * inv_det;

    invA[2][0] = (d * h - e * g) * inv_det;
    invA[2][1] = (b * g - a * h) * inv_det;
    invA[2][2] = (a * e - b * d) * inv_det;
    return 1U;
}

/*
 * 参数合法性检查
 *
 * 当前简化模型真正必须的只有：
 * - thigh_len > 0
 * - shank_len > 0
 * - hip_len >= 0
 */
static uint8_t gait_param_valid(const GaitLegParam *param)
{
    if (param == NULL)
    {
        return 0U;
    }
    if ((param->thigh_len <= GAIT_EPS) || (param->shank_len <= GAIT_EPS))
    {
        return 0U;
    }
    if (param->hip_len < 0.0f)
    {
        return 0U;
    }
    return 1U;
}

/*
 * 构造当前“简化 3 自由度单腿模型”的动力学项
 *
 * 模型划分：
 * - 关节 0（髋外展）被简化为一个解耦标量惯量 M00
 * - 关节 1/2（髋俯仰 + 膝俯仰）采用标准 2 连杆平面耦合动力学
 *
 * 输出满足：
 * tau = M*ddq + C + G + B*dq
 *
 * 重要说明：
 * 这里的动力学仍然是“近似简化模型”，并不是基于你真实四杆和真实三维装配位置
 * 完整推导出来的精确动力学。当前的目标是先让单腿控制和轨迹模块可跑。
 */
static void gait_dynamics_terms(const GaitLegParam *param,
                                const float q[3],
                                const float dq[3],
                                float *M00,
                                float M2[2][2],
                                float C[3],
                                float G[3])
{
    float q0 = q[0];
    float q1 = q[1];
    float q2 = q[2];
    float dq1 = dq[1];
    float dq2 = dq[2];

    float l1 = param->hip_len;
    float l2 = param->thigh_len;
    float l3 = param->shank_len;

    float m0 = param->mass[0];
    float m1 = param->mass[1];
    float m2 = param->mass[2];
    float I0 = param->inertia[0];
    float I1 = param->inertia[1];
    float I2 = param->inertia[2];
    float g = param->gravity;

    float lc1 = 0.5f * l2;
    float lc2 = 0.5f * l3;
    float b = m2 * l2 * lc2;
    float a = I1 + I2 + m1 * lc1 * lc1 + m2 * (l2 * l2 + lc2 * lc2);
    float d = I2 + m2 * lc2 * lc2;
    float c2 = cosf(q2);
    float s2 = sinf(q2);
    float m_total = m0 + m1 + m2;

    /* 关节 0 的解耦等效惯量 */
    *M00 = I0 + m_total * l1 * l1;

    /* 关节 1/2 的 2x2 平面惯性矩阵 */
    M2[0][0] = a + 2.0f * b * c2;
    M2[0][1] = d + b * c2;
    M2[1][0] = M2[0][1];
    M2[1][1] = d;

    /* 科里奥利/离心项 */
    C[0] = 0.0f;
    C[1] = -b * s2 * (2.0f * dq1 * dq2 + dq2 * dq2);
    C[2] = b * s2 * dq1 * dq1;

    /* 重力项 */
    G[0] = m_total * g * l1 * sinf(q0);
    G[1] = (m1 * lc1 + m2 * l2) * g * sinf(q1) + m2 * lc2 * g * sinf(q1 + q2);
    G[2] = m2 * lc2 * g * sinf(q1 + q2);
}

/*
 * 使用你给的最新实测尺寸填充参数
 *
 * 原始输入（单位 m，先不管正负号，直接使用量值）：
 * O -> A = (0.16630, 0.09000, 0.07750)
 * A -> B = (0.06605, 0.02003, 0.00000)
 * B -> C = (0.17448, 0.06750, 0.11695)
 * C -> D = (0.12522, 0.00000, 0.20553)
 * E -> C = (0.17448, 0.01520, 0.11695)
 * E -> F = (0.01654, 0.01520, 0.01525)
 * C -> G = (0.01654, 0.00000, 0.01525)
 * FG_LEN = 0.21005
 *
 * 当前等效化方式：
 * - hip_len   = |A->B|
 * - thigh_len = |B->C|
 * - shank_len = |C->D|
 *
 * 注意：
 * 1) 这是为了适配你当前这个 gait 模块的简化 3DOF 数学模型。
 * 2) O->A、E->C、E->F、C->G、FG_LEN 已保存在 raw_* 中，便于后续扩展。
 * 3) 若后续要做更真实的 3D 空间运动学或连杆映射，这些 raw_* 数据可以继续使用。
 */
void gait_leg_param_default(GaitLegParam *param)
{
    if (param == NULL)
    {
        return;
    }

    memset(param, 0, sizeof(GaitLegParam));

    /* ===== 写入原始实测尺寸 ===== */
    param->raw_OA.x = 0.16630f;
    param->raw_OA.y = 0.09000f;
    param->raw_OA.z = 0.07750f;

    param->raw_AB.x = 0.06605f;
    param->raw_AB.y = 0.02003f;
    param->raw_AB.z = 0.00000f;

    param->raw_BC.x = 0.17448f;
    param->raw_BC.y = 0.06750f;
    param->raw_BC.z = 0.11695f;

    param->raw_CD.x = 0.12522f;
    param->raw_CD.y = 0.00000f;
    param->raw_CD.z = 0.20553f;

    param->raw_EC.x = 0.17448f;
    param->raw_EC.y = 0.01520f;
    param->raw_EC.z = 0.11695f;

    param->raw_EF.x = 0.01654f;
    param->raw_EF.y = 0.01520f;
    param->raw_EF.z = 0.01525f;

    param->raw_CG.x = 0.01654f;
    param->raw_CG.y = 0.00000f;
    param->raw_CG.z = 0.01525f;

    param->raw_FG_len = 0.21005f;

    /* ===== 由原始实测数据提取当前简化模型需要的等效长度 ===== */
    param->hip_len = gait_vec3_norm(&param->raw_AB);     /* |A->B| ≈ 0.06902 m */
    param->thigh_len = gait_vec3_norm(&param->raw_BC);   /* |B->C| ≈ 0.22063 m */
    param->shank_len = gait_vec3_norm(&param->raw_CD);   /* |C->D| ≈ 0.24067 m */

    /* ===== 动力学参数：当前仍为保守占位值，需要后续辨识/实测 ===== */
    param->mass[0] = 0.20f;
    param->mass[1] = 0.45f;
    param->mass[2] = 0.35f;

    param->inertia[0] = 0.0012f;
    param->inertia[1] = 0.0025f;
    param->inertia[2] = 0.0018f;

    param->damping[0] = 0.02f;
    param->damping[1] = 0.02f;
    param->damping[2] = 0.02f;

    param->gravity = 9.81f;
}

/* 浮点数限幅 */
float gait_clampf(float v, float lo, float hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

/*
 * 相位包裹到 [0,1)
 */
float gait_wrap_phase(float phase01)
{
    float wrapped = fmodf(phase01, 1.0f);
    if (wrapped < 0.0f)
    {
        wrapped += 1.0f;
    }
    return wrapped;
}

/*
 * 单腿正运动学
 *
 * 几何理解：
 * - q0：控制髋外展带来的横向偏置
 * - q1、q2：在 x-z 平面上构成一个二连杆链
 *
 * 重要说明：
 * 这是“简化等效模型”的正运动学，不直接使用 O->A / E->C / E->F / C->G 等完整实测结构。
 */
uint8_t gait_forward_kinematics(const GaitLegParam *param, const float q[3], GaitVec3 *foot_pos)
{
    float q0;
    float q1;
    float q2;
    float s0;
    float c0;
    float s1;
    float c1;
    float s12;
    float c12;

    if ((!gait_param_valid(param)) || (q == NULL) || (foot_pos == NULL))
    {
        return 0U;
    }

    q0 = q[0];
    q1 = q[1];
    q2 = q[2];
    s0 = sinf(q0);
    c0 = cosf(q0);
    s1 = sinf(q1);
    c1 = cosf(q1);
    s12 = sinf(q1 + q2);
    c12 = cosf(q1 + q2);

    foot_pos->x = param->thigh_len * s1 + param->shank_len * s12;
    foot_pos->y = param->hip_len * s0;
    foot_pos->z = param->hip_len * c0 + param->thigh_len * c1 + param->shank_len * c12;
    return 1U;
}

/*
 * 单腿逆运动学
 *
 * 步骤：
 * 1) 由 y = hip_len * sin(q0) 求外展角 q0
 * 2) 去掉外展关节对 z 的贡献后，得到平面二连杆目标
 * 3) 由余弦定理解 q2
 * 4) 再求 q1
 *
 * 注意：
 * 当前 knee_cfg 只是简单选择膝关节的两支解。
 */
uint8_t gait_inverse_kinematics(const GaitLegParam *param,
                                const GaitVec3 *foot_pos,
                                GaitKneeConfig knee_cfg,
                                float q_out[3])
{
    float q0;
    float z_planar;
    float x;
    float d2;
    float l2;
    float l3;
    float cos_q2;
    float sin_q2;
    float q2;
    float k1;
    float k2;
    float q1;

    if ((!gait_param_valid(param)) || (foot_pos == NULL) || (q_out == NULL))
    {
        return 0U;
    }

    if (fabsf(param->hip_len) > GAIT_EPS)
    {
        float ratio = foot_pos->y / param->hip_len;
        if ((ratio < -1.0f - 1.0e-4f) || (ratio > 1.0f + 1.0e-4f))
        {
            return 0U;
        }
        ratio = gait_clampf(ratio, -1.0f, 1.0f);
        q0 = asinf(ratio);
        z_planar = foot_pos->z - param->hip_len * cosf(q0);
    }
    else
    {
        if (fabsf(foot_pos->y) > 1.0e-4f)
        {
            return 0U;
        }
        q0 = 0.0f;
        z_planar = foot_pos->z;
    }

    l2 = param->thigh_len;
    l3 = param->shank_len;
    x = foot_pos->x;
    d2 = x * x + z_planar * z_planar;

    cos_q2 = (d2 - l2 * l2 - l3 * l3) / (2.0f * l2 * l3);
    if ((cos_q2 < -1.0f - 1.0e-4f) || (cos_q2 > 1.0f + 1.0e-4f))
    {
        return 0U;
    }

    cos_q2 = gait_clampf(cos_q2, -1.0f, 1.0f);
    sin_q2 = sqrtf(gait_clampf(1.0f - cos_q2 * cos_q2, 0.0f, 1.0f));
    if (knee_cfg == GAIT_KNEE_BEND_BACKWARD)
    {
        sin_q2 = -sin_q2;
    }

    q2 = atan2f(sin_q2, cos_q2);
    k1 = l2 + l3 * cos_q2;
    k2 = l3 * sin_q2;
    q1 = atan2f(x, z_planar) - atan2f(k2, k1);

    q_out[0] = q0;
    q_out[1] = q1;
    q_out[2] = q2;
    return 1U;
}

/*
 * 解析雅可比矩阵 J = d(foot_pos)/d(q)
 * v_foot = J * dq
 */
uint8_t gait_calc_jacobian(const GaitLegParam *param, const float q[3], float J[3][3])
{
    float q0;
    float q1;
    float q2;
    float s0;
    float c0;
    float s1;
    float c1;
    float s12;
    float c12;

    if ((!gait_param_valid(param)) || (q == NULL) || (J == NULL))
    {
        return 0U;
    }

    q0 = q[0];
    q1 = q[1];
    q2 = q[2];
    s0 = sinf(q0);
    c0 = cosf(q0);
    s1 = sinf(q1);
    c1 = cosf(q1);
    s12 = sinf(q1 + q2);
    c12 = cosf(q1 + q2);

    /* dx/dq */
    J[0][0] = 0.0f;
    J[0][1] = param->thigh_len * c1 + param->shank_len * c12;
    J[0][2] = param->shank_len * c12;

    /* dy/dq */
    J[1][0] = param->hip_len * c0;
    J[1][1] = 0.0f;
    J[1][2] = 0.0f;

    /* dz/dq */
    J[2][0] = -param->hip_len * s0;
    J[2][1] = -param->thigh_len * s1 - param->shank_len * s12;
    J[2][2] = -param->shank_len * s12;
    return 1U;
}

/*
 * 逆动力学：
 * tau = M*ddq + C + G + B*dq
 */
uint8_t gait_inverse_dynamics(const GaitLegParam *param,
                              const float q[3],
                              const float dq[3],
                              const float ddq[3],
                              float tau_out[3])
{
    float M00;
    float M2[2][2];
    float C[3];
    float G[3];

    if ((!gait_param_valid(param)) || (q == NULL) || (dq == NULL) || (ddq == NULL) || (tau_out == NULL))
    {
        return 0U;
    }

    gait_dynamics_terms(param, q, dq, &M00, M2, C, G);

    tau_out[0] = M00 * ddq[0] + C[0] + G[0] + param->damping[0] * dq[0];
    tau_out[1] = M2[0][0] * ddq[1] + M2[0][1] * ddq[2] + C[1] + G[1] + param->damping[1] * dq[1];
    tau_out[2] = M2[1][0] * ddq[1] + M2[1][1] * ddq[2] + C[2] + G[2] + param->damping[2] * dq[2];
    return 1U;
}

/*
 * 正动力学：
 * ddq = M^-1 * (tau - C - G - B*dq)
 */
uint8_t gait_forward_dynamics(const GaitLegParam *param,
                              const float q[3],
                              const float dq[3],
                              const float tau[3],
                              float ddq_out[3])
{
    float M00;
    float M2[2][2];
    float C[3];
    float G[3];
    float rhs1;
    float rhs2;
    float det;

    if ((!gait_param_valid(param)) || (q == NULL) || (dq == NULL) || (tau == NULL) || (ddq_out == NULL))
    {
        return 0U;
    }

    gait_dynamics_terms(param, q, dq, &M00, M2, C, G);
    if (fabsf(M00) < GAIT_EPS)
    {
        return 0U;
    }

    ddq_out[0] = (tau[0] - C[0] - G[0] - param->damping[0] * dq[0]) / M00;

    rhs1 = tau[1] - C[1] - G[1] - param->damping[1] * dq[1];
    rhs2 = tau[2] - C[2] - G[2] - param->damping[2] * dq[2];
    det = M2[0][0] * M2[1][1] - M2[0][1] * M2[1][0];

    if (fabsf(det) < GAIT_EPS)
    {
        return 0U;
    }

    ddq_out[1] = (M2[1][1] * rhs1 - M2[0][1] * rhs2) / det;
    ddq_out[2] = (-M2[1][0] * rhs1 + M2[0][0] * rhs2) / det;
    return 1U;
}

/*
 * 足端力 -> 关节力矩
 * tau = J^T * F
 */
uint8_t gait_foot_force_to_joint_torque(const GaitLegParam *param,
                                        const float q[3],
                                        const GaitVec3 *foot_force,
                                        float tau_out[3])
{
    float J[3][3];

    if ((foot_force == NULL) || (tau_out == NULL))
    {
        return 0U;
    }
    if (!gait_calc_jacobian(param, q, J))
    {
        return 0U;
    }

    tau_out[0] = J[0][0] * foot_force->x + J[1][0] * foot_force->y + J[2][0] * foot_force->z;
    tau_out[1] = J[0][1] * foot_force->x + J[1][1] * foot_force->y + J[2][1] * foot_force->z;
    tau_out[2] = J[0][2] * foot_force->x + J[1][2] * foot_force->y + J[2][2] * foot_force->z;
    return 1U;
}

/*
 * 关节力矩 -> 足端等效力
 * F = (J^T)^(-1) * tau
 *
 * 在奇异位形附近会失败。
 */
uint8_t gait_joint_torque_to_foot_force(const GaitLegParam *param,
                                        const float q[3],
                                        const float tau[3],
                                        GaitVec3 *foot_force_out)
{
    float J[3][3];
    float JT[3][3];
    float JT_inv[3][3];

    if ((tau == NULL) || (foot_force_out == NULL))
    {
        return 0U;
    }
    if (!gait_calc_jacobian(param, q, J))
    {
        return 0U;
    }

    JT[0][0] = J[0][0];
    JT[0][1] = J[1][0];
    JT[0][2] = J[2][0];
    JT[1][0] = J[0][1];
    JT[1][1] = J[1][1];
    JT[1][2] = J[2][1];
    JT[2][0] = J[0][2];
    JT[2][1] = J[1][2];
    JT[2][2] = J[2][2];

    if (!gait_inv3x3(JT, JT_inv))
    {
        return 0U;
    }

    foot_force_out->x = JT_inv[0][0] * tau[0] + JT_inv[0][1] * tau[1] + JT_inv[0][2] * tau[2];
    foot_force_out->y = JT_inv[1][0] * tau[0] + JT_inv[1][1] * tau[1] + JT_inv[1][2] * tau[2];
    foot_force_out->z = JT_inv[2][0] * tau[0] + JT_inv[2][1] * tau[1] + JT_inv[2][2] * tau[2];
    return 1U;
}

/*
 * 摆线摆动相轨迹
 *
 * 水平方向采用摆线相位函数，天然满足首尾速度为 0。
 * 竖直方向叠加一个中间抬脚凸起。
 *
 * 返回的导数是对归一化相位 s 的导数，不是对时间 t 的导数。
 */
uint8_t gait_cycloid_curve(const GaitVec3 *start,
                           const GaitVec3 *end,
                           float step_height,
                           float s,
                           GaitVec3 *pos,
                           GaitVec3 *vel_norm,
                           GaitVec3 *acc_norm)
{
    float dx;
    float dy;
    float dz;
    float two_pi;
    float u;
    float du;
    float ddu;
    float bump;
    float dbump;
    float ddbump;

    if ((start == NULL) || (end == NULL) || (pos == NULL))
    {
        return 0U;
    }

    s = gait_clampf(s, 0.0f, 1.0f);
    two_pi = 2.0f * GAIT_PI;
    u = s - sinf(two_pi * s) / two_pi;
    du = 1.0f - cosf(two_pi * s);
    ddu = two_pi * sinf(two_pi * s);

    bump = 0.5f * (1.0f - cosf(two_pi * s));
    dbump = GAIT_PI * sinf(two_pi * s);
    ddbump = 2.0f * GAIT_PI * GAIT_PI * cosf(two_pi * s);

    dx = end->x - start->x;
    dy = end->y - start->y;
    dz = end->z - start->z;

    pos->x = start->x + dx * u;
    pos->y = start->y + dy * u;
    pos->z = start->z + dz * u + step_height * bump;

    if (vel_norm != NULL)
    {
        vel_norm->x = dx * du;
        vel_norm->y = dy * du;
        vel_norm->z = dz * du + step_height * dbump;
    }

    if (acc_norm != NULL)
    {
        acc_norm->x = dx * ddu;
        acc_norm->y = dy * ddu;
        acc_norm->z = dz * ddu + step_height * ddbump;
    }

    return 1U;
}

/*
 * 五次贝塞尔轨迹
 *
 * 若需要导数：
 * - 一阶导仍是贝塞尔，可由差分控制点构造
 * - 二阶导同理
 */
uint8_t gait_bezier_curve_5th(const GaitVec3 ctrl_pts[6],
                              float s,
                              GaitVec3 *pos,
                              GaitVec3 *vel_norm,
                              GaitVec3 *acc_norm)
{
    if ((ctrl_pts == NULL) || (pos == NULL))
    {
        return 0U;
    }

    s = gait_clampf(s, 0.0f, 1.0f);
    *pos = gait_bezier_eval(ctrl_pts, 6U, s);

    if (vel_norm != NULL)
    {
        GaitVec3 dcp[5];
        uint32_t i;
        for (i = 0U; i < 5U; i++)
        {
            dcp[i].x = 5.0f * (ctrl_pts[i + 1U].x - ctrl_pts[i].x);
            dcp[i].y = 5.0f * (ctrl_pts[i + 1U].y - ctrl_pts[i].y);
            dcp[i].z = 5.0f * (ctrl_pts[i + 1U].z - ctrl_pts[i].z);
        }
        *vel_norm = gait_bezier_eval(dcp, 5U, s);
    }

    if (acc_norm != NULL)
    {
        GaitVec3 ddcp[4];
        uint32_t i;
        for (i = 0U; i < 4U; i++)
        {
            ddcp[i].x = 20.0f * (ctrl_pts[i + 2U].x - 2.0f * ctrl_pts[i + 1U].x + ctrl_pts[i].x);
            ddcp[i].y = 20.0f * (ctrl_pts[i + 2U].y - 2.0f * ctrl_pts[i + 1U].y + ctrl_pts[i].y);
            ddcp[i].z = 20.0f * (ctrl_pts[i + 2U].z - 2.0f * ctrl_pts[i + 1U].z + ctrl_pts[i].z);
        }
        *acc_norm = gait_bezier_eval(ddcp, 4U, s);
    }

    return 1U;
}

/*
 * 完整步态周期的足端轨迹规划
 *
 * 相位划分：
 * - 摆动相：[0, swing_ratio)
 * - 支撑相：[swing_ratio, 1)
 *
 * 输出的 vel / acc 为对物理时间 t 的导数。
 */
uint8_t gait_plan_foot_trajectory(const GaitTrajectoryParam *param,
                                  float phase01,
                                  GaitVec3 *pos,
                                  GaitVec3 *vel,
                                  GaitVec3 *acc)
{
    float phase;
    float swing_ratio;

    if ((param == NULL) || (pos == NULL) || (param->cycle_time <= GAIT_EPS))
    {
        return 0U;
    }

    phase = gait_wrap_phase(phase01);
    swing_ratio = gait_clampf(param->swing_ratio, 0.05f, 0.95f);

    if (phase < swing_ratio)
    {
        float s = phase / swing_ratio;
        float swing_time = param->cycle_time * swing_ratio;
        GaitVec3 vel_norm = {0.0f, 0.0f, 0.0f};
        GaitVec3 acc_norm = {0.0f, 0.0f, 0.0f};
        GaitVec3 *vel_norm_ptr = (vel != NULL) ? &vel_norm : NULL;
        GaitVec3 *acc_norm_ptr = (acc != NULL) ? &acc_norm : NULL;

        if (param->swing_curve == GAIT_SWING_BEZIER)
        {
            GaitVec3 ctrl[6];
            float dx = param->touch_down_pos.x - param->lift_off_pos.x;
            float dy = param->touch_down_pos.y - param->lift_off_pos.y;

            ctrl[0] = param->lift_off_pos;

            ctrl[1].x = param->lift_off_pos.x + 0.20f * dx;
            ctrl[1].y = param->lift_off_pos.y + 0.20f * dy;
            ctrl[1].z = param->lift_off_pos.z;

            ctrl[2].x = param->lift_off_pos.x + 0.40f * dx;
            ctrl[2].y = param->lift_off_pos.y + 0.40f * dy;
            ctrl[2].z = param->lift_off_pos.z + param->step_height;

            ctrl[3].x = param->lift_off_pos.x + 0.60f * dx;
            ctrl[3].y = param->lift_off_pos.y + 0.60f * dy;
            ctrl[3].z = param->touch_down_pos.z + param->step_height;

            ctrl[4].x = param->lift_off_pos.x + 0.80f * dx;
            ctrl[4].y = param->lift_off_pos.y + 0.80f * dy;
            ctrl[4].z = param->touch_down_pos.z;

            ctrl[5] = param->touch_down_pos;

            if (!gait_bezier_curve_5th(ctrl, s, pos, vel_norm_ptr, acc_norm_ptr))
            {
                return 0U;
            }
        }
        else
        {
            if (!gait_cycloid_curve(&param->lift_off_pos,
                                    &param->touch_down_pos,
                                    param->step_height,
                                    s,
                                    pos,
                                    vel_norm_ptr,
                                    acc_norm_ptr))
            {
                return 0U;
            }
        }

        if (vel != NULL)
        {
            vel->x = vel_norm.x / swing_time;
            vel->y = vel_norm.y / swing_time;
            vel->z = vel_norm.z / swing_time;
        }

        if (acc != NULL)
        {
            float inv_t2 = 1.0f / (swing_time * swing_time);
            acc->x = acc_norm.x * inv_t2;
            acc->y = acc_norm.y * inv_t2;
            acc->z = acc_norm.z * inv_t2;
        }

        return 1U;
    }
    else
    {
        float stance_ratio = 1.0f - swing_ratio;
        float s = (phase - swing_ratio) / stance_ratio;
        float stance_time = param->cycle_time * stance_ratio;
        float u;
        float du;
        float ddu;
        float dx;
        float dy;
        float dz;

        gait_quintic_scurve(s, &u, &du, &ddu);

        dx = param->lift_off_pos.x - param->touch_down_pos.x;
        dy = param->lift_off_pos.y - param->touch_down_pos.y;
        dz = param->lift_off_pos.z - param->touch_down_pos.z;

        pos->x = param->touch_down_pos.x + dx * u;
        pos->y = param->touch_down_pos.y + dy * u;
        pos->z = param->touch_down_pos.z + dz * u;

        if (vel != NULL)
        {
            vel->x = dx * du / stance_time;
            vel->y = dy * du / stance_time;
            vel->z = dz * du / stance_time;
        }

        if (acc != NULL)
        {
            float inv_t2 = 1.0f / (stance_time * stance_time);
            acc->x = dx * ddu * inv_t2;
            acc->y = dy * ddu * inv_t2;
            acc->z = dz * ddu * inv_t2;
        }

        return 1U;
    }
}
