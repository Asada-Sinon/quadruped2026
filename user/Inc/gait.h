#ifndef GAIT_H
#define GAIT_H

#include <stdint.h>

/*
 * gait 模块说明
 *
 * 本模块当前提供：
 * 1) 单腿正/逆运动学与雅可比矩阵
 * 2) 单腿简化动力学（逆动力学 / 正动力学）
 * 3) 足端轨迹生成（摆线 / 五次贝塞尔）
 * 4) 单步相位下的足端轨迹规划
 *
 * 当前模块采用的坐标约定：
 * - x：前方
 * - y：横向（左正右负，具体符号由上层统一）
 * - z：竖直方向
 *
 * 关节角约定：
 * - q[0]：髋外展/内收关节
 * - q[1]：髋俯仰（大腿前后摆）关节
 * - q[2]：膝俯仰关节
 *
 * 重要说明：
 * 1) 这个 gait 模块仍然是“单腿简化模型”，不是完整的四足整机模型。
 * 2) 你提供的 O->A、A->B、B->C、C->D、E->C、E->F、C->G、FG_LEN
 *    已经被写入参数结构体中，便于后续继续扩展。
 * 3) 但当前的正逆运动学仍然只直接使用 3 个等效长度：
 *    - hip_len   ：由 A->B 的长度近似得到
 *    - thigh_len ：由 B->C 的长度近似得到
 *    - shank_len ：由 C->D 的长度近似得到
 * 4) 也就是说，当前模型已经把实测三维尺寸“收缩”为一个 3 自由度等效腿模型。
 *    这适合先跑通位置控制、正逆运动学和基础轨迹规划，
 *    但还不能完整反映真实连杆和完整空间安装几何。
 *
 * 所有 uint8_t 接口的返回值约定：
 * - 1U：成功
 * - 0U：失败（空指针、目标不可达、矩阵奇异等）
 */

#ifdef __cplusplus
extern "C"
{
#endif

/* 三维向量，既可用于位置，也可用于速度、加速度、力 */
typedef struct
{
    float x;
    float y;
    float z;
} GaitVec3;

/* 单腿关节状态 */
typedef struct
{
    /* 关节角，单位 rad */
    float q[3];
    /* 关节角速度，单位 rad/s */
    float dq[3];
    /* 关节角加速度，单位 rad/s^2 */
    float ddq[3];
} GaitJointState;

/*
 * 单腿参数
 *
 * 单位约定：
 * - 长度：m
 * - 质量：kg
 * - 转动惯量：kg*m^2
 * - 阻尼：N*m*s/rad
 */
typedef struct
{
    /* ===== 当前简化模型直接使用的等效参数 ===== */

    /* A->B 的等效长度：
     * 从外展关节中心到俯仰链根部的等效偏置长度 */
    float hip_len;

    /* B->C 的等效长度：
     * 大腿等效长度 */
    float thigh_len;

    /* C->D 的等效长度：
     * 小腿到足端接触点的等效长度 */
    float shank_len;

    /* 各刚体质量：[髋侧、 大腿、 小腿] */
    float mass[3];

    /* 围绕建模关节轴的转动惯量 */
    float inertia[3];

    /* 各关节粘性阻尼 */
    float damping[3];

    /* 重力加速度大小，通常 9.81 */
    float gravity;

    /* ===== 原始实测参数（保留，便于后续继续扩展） ===== */

    /* O->A：机身原点(如 IMU 中心) 到外展关节中心 A */
    GaitVec3 raw_OA;

    /* A->B：外展关节中心 A 到大腿前后摆关节中心 B */
    GaitVec3 raw_AB;

    /* B->C：大腿前后摆关节中心 B 到膝关节中心 C */
    GaitVec3 raw_BC;

    /* C->D：膝关节中心 C 到足端接触点 D */
    GaitVec3 raw_CD;

    /* E->C：膝驱动电机轴中心 E 到膝关节中心 C */
    GaitVec3 raw_EC;

    /* E->F：电机侧摇臂轴中心 E 到电机侧连杆销轴 F */
    GaitVec3 raw_EF;

    /* C->G：膝关节中心 C 到小腿侧连杆销轴 G */
    GaitVec3 raw_CG;

    /* F->G 连杆中心距长度 */
    float raw_FG_len;
} GaitLegParam;

/* 膝关节分支选择：
 * 对同一个足端点，通常可能存在两支膝构型 */
typedef enum
{
    /* 膝关节正向弯折支 */
    GAIT_KNEE_BEND_FORWARD = 1,
    /* 膝关节反向弯折支 */
    GAIT_KNEE_BEND_BACKWARD = -1
} GaitKneeConfig;

/* 摆动相轨迹曲线类型 */
typedef enum
{
    /* 摆线轨迹 */
    GAIT_SWING_CYCLOID = 0,
    /* 五次贝塞尔轨迹 */
    GAIT_SWING_BEZIER = 1
} GaitSwingCurve;

/*
 * 单个完整步态周期的足端轨迹参数
 * phase in [0,1)：
 * - [0, swing_ratio) 为摆动相
 * - [swing_ratio, 1) 为支撑相
 */
typedef struct
{
    /* 抬脚点（摆动起点） */
    GaitVec3 lift_off_pos;
    /* 落脚点（摆动终点） */
    GaitVec3 touch_down_pos;
    /* 摆动相额外抬脚高度 */
    float step_height;
    /* 整个周期中摆动相占比，例如 0.4 */
    float swing_ratio;
    /* 一个完整周期的总时间，单位 s */
    float cycle_time;
    /* 摆动相使用哪种曲线 */
    GaitSwingCurve swing_curve;
} GaitTrajectoryParam;

/*
 * 使用你提供的最新实测数据填充单腿参数。
 *
 * 注意：
 * 1) 当前先不管正负号，直接按“量值大小”写入。
 * 2) 该接口会同时填 raw_* 原始实测参数，
 *    并自动计算 hip_len / thigh_len / shank_len 三个等效长度。
 */
void gait_leg_param_default(GaitLegParam *param);

/* 浮点数限幅 */
float gait_clampf(float v, float lo, float hi);

/*
 * 将任意相位包裹到 [0,1) 区间。
 * 例如：
 * -0.2 -> 0.8
 *  1.35 -> 0.35
 */
float gait_wrap_phase(float phase01);

/*
 * 单腿正运动学：q -> 足端位置
 *
 * 输入：
 * - param：单腿参数
 * - q：关节角 [q0, q1, q2]
 *
 * 输出：
 * - foot_pos：计算得到的足端位置（腿局部坐标系/本模块局部坐标系）
 */
uint8_t gait_forward_kinematics(const GaitLegParam *param, const float q[3], GaitVec3 *foot_pos);

/*
 * 单腿逆运动学：足端位置 -> 关节角
 *
 * 输入：
 * - foot_pos：目标足端位置
 * - knee_cfg：膝关节解分支
 *
 * 输出：
 * - q_out：求解得到的关节角
 */
uint8_t gait_inverse_kinematics(const GaitLegParam *param, const GaitVec3 *foot_pos, GaitKneeConfig knee_cfg, float q_out[3]);

/*
 * 雅可比矩阵：
 * v_foot = J * dq
 */
uint8_t gait_calc_jacobian(const GaitLegParam *param, const float q[3], float J[3][3]);

/*
 * 逆动力学：
 * tau = M(q)*ddq + C(q,dq) + G(q) + B*dq
 */
uint8_t gait_inverse_dynamics(const GaitLegParam *param, const float q[3], const float dq[3], const float ddq[3], float tau_out[3]);

/*
 * 正动力学：
 * ddq = M(q)^(-1) * (tau - C - G - B*dq)
 */
uint8_t gait_forward_dynamics(const GaitLegParam *param, const float q[3], const float dq[3], const float tau[3], float ddq_out[3]);

/*
 * 通过虚功原理将足端力映射到关节力矩：
 * tau = J^T * F_foot
 */
uint8_t gait_foot_force_to_joint_torque(const GaitLegParam *param, const float q[3], const GaitVec3 *foot_force, float tau_out[3]);

/*
 * 通过雅可比逆映射将关节力矩转换为足端等效力：
 * F_foot = (J^T)^(-1) * tau
 *
 * 注意：在奇异位形附近会失败。
 */
uint8_t gait_joint_torque_to_foot_force(const GaitLegParam *param, const float q[3], const float tau[3], GaitVec3 *foot_force_out);

/*
 * 摆线轨迹
 *
 * 输入 s 为归一化相位 [0,1]
 * 输出：
 * - pos      ：位置
 * - vel_norm ：对 s 的一阶导
 * - acc_norm ：对 s 的二阶导
 *
 * 若要转换为对时间 t 的导数，需要再乘相位变化率。
 */
uint8_t gait_cycloid_curve(const GaitVec3 *start, const GaitVec3 *end, float step_height, float s,
                           GaitVec3 *pos, GaitVec3 *vel_norm, GaitVec3 *acc_norm);

/*
 * 五次贝塞尔轨迹
 *
 * ctrl_pts 固定为 6 个控制点，对应 5 次多项式。
 */
uint8_t gait_bezier_curve_5th(const GaitVec3 ctrl_pts[6], float s,
                              GaitVec3 *pos, GaitVec3 *vel_norm, GaitVec3 *acc_norm);

/*
 * 单个完整步态周期的足端轨迹规划
 *
 * 输入 phase01 可以是任意实数，函数内部会包裹到 [0,1)
 * - 摆动相：从 lift_off_pos -> touch_down_pos
 * - 支撑相：从 touch_down_pos 平滑回到 lift_off_pos
 *
 * 输出 vel / acc 为物理时间下的导数（SI 单位）
 */
uint8_t gait_plan_foot_trajectory(const GaitTrajectoryParam *param, float phase01,
                                  GaitVec3 *pos, GaitVec3 *vel, GaitVec3 *acc);

#ifdef __cplusplus
}
#endif

#endif
