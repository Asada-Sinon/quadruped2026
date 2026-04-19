#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "gait.h"
#include <stdint.h>

/* 单腿当前所处相位：支撑 / 摆动 */
typedef enum
{
    STEP_LEG_STANCE = 0U,
    STEP_LEG_SWING  = 1U
} StepLegState;

/* 单腿轨迹规划器在运行时需要记住的一些状态
 * 1) phase_anchor：这条腿当前这一段轨迹的参考起点
 *    - 摆动相：本轮摆动的离地点
 *    - 支撑相：本轮支撑的落地点
 * 2) last_state：上个周期这条腿是支撑还是摆动，用来判断相位切换
 * 3) valid：初始化标志位
 */
typedef struct
{
    GaitFootPosM phase_anchor[ROBOT_LEG_NUM];
    StepLegState last_state[ROBOT_LEG_NUM];
    uint8_t valid[ROBOT_LEG_NUM];
} FootTrajRuntime;

/* 对角步态 + 多项式轨迹 的统一模式结构体
 * 这个结构体里同时放：
 * 1) 步态调度参数（频率、全局相位）
 * 2) 足端轨迹参数（步长、抬脚高度）
 * 3) 四条腿各自的名义落脚点
 * 4) 运行时状态（phase anchor / 上一拍状态）
 *
 * 这样做的目的：
 * - 减少全局变量
 * - 减少函数到处传很多零散参数
 * - 以后状态机切模式时，只需要切这一整个 mode 结构体
 */
typedef struct
{
    /* 全局步态相位，范围 [0,1)
     * 当前默认占比：摆动相 0.4，支撑相 0.6。
     * 对角两组腿相差 0.5 周期，因此：
     * 0.0 ~ 0.4：左前 + 右后 摆动
     * 0.5 ~ 0.9：右前 + 左后 摆动
     */
    float gait_phase;

    /* 踏步频率 Hz。以后你想通过改频率加速，就直接改这个。 */
    float freq_hz;

    /* 步长：
     * =0  -> 原地踏步
     * >0  -> 向前走
     * <0  -> 向后退
     */
    float step_length_m;

    /* 摆动相基础抬脚高度，单位 m */
    float swing_height_m;

    /* 仅前腿(FL/FR)在摆动相额外增加的抬脚偏置，单位 m。
     * 实际前腿摆动高度 = swing_height_m + front_swing_height_bias_m。
     * 后腿(HL/HR)仍使用 swing_height_m。
     */
    float front_swing_height_bias_m;

    /* 四条腿各自的名义落脚点（髋坐标系） */
    GaitFootPosM nominal[ROBOT_LEG_NUM];

    /* 轨迹运行时状态 */
    FootTrajRuntime runtime;
} DiagonalCycloidGait;

/* 初始化默认参数：
 * - gait_phase = 0
 * - freq_hz = 0.8
 * - step_length_m = 0 （默认原地踏步）
 * - swing_height_m = 0.07
 * - front_swing_height_bias_m = 0.015
 * - 默认名义点按你现在代码中的值写入
 */
void Trajectory_InitDefault(DiagonalCycloidGait *gait);

/* 相位清零 + 运行时参考点清空 */
void Trajectory_Reset(DiagonalCycloidGait *gait);

/* 单独设置某条腿的名义落脚点 */
void Trajectory_SetNominalFoot(DiagonalCycloidGait *gait,
                               uint8_t leg_idx,
                               float x_m,
                               float y_m,
                               float z_m);

/* 设置频率 */
void Trajectory_SetFrequency(DiagonalCycloidGait *gait, float freq_hz);

/* 设置步长：
 * step_length_m = 0   -> 原地踏步
 * step_length_m > 0   -> 向前走
 */
void Trajectory_SetStepLength(DiagonalCycloidGait *gait, float step_length_m);

/* 设置摆动相抬脚高度 */
void Trajectory_SetSwingHeight(DiagonalCycloidGait *gait, float swing_height_m);

/* 设置“前腿相对后腿”的摆动抬脚偏置（只在前腿摆动相生效） */
void Trajectory_SetFrontSwingHeightBias(DiagonalCycloidGait *gait, float front_bias_m);

/* 每周期调用一次：
 * dt_s: 本周期时间，单位秒
 *
 * 当前实现采用“围绕名义足点”的对角步态：
 * 1) 推进 gait_phase
 * 2) 判断每条腿是支撑还是摆动
 * 3) 生成四条腿的足端目标
 * 4) 调用 Gait_SetLegFootTargetM() 写给 gait 层
 */
void Trajectory_Update(DiagonalCycloidGait *gait,
                       float dt_s);

#endif /* TRAJECTORY_H */
