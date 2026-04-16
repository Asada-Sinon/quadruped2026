#ifndef APP_ROBOT_H
#define APP_ROBOT_H

/*
 * 这里引入 robot_map.h 的目的是拿到：
 * 1) ROBOT_LEG_NUM / MOTORS_PER_LEG 宏；
 * 2) Leg 结构体类型定义。
 */
#include "robot_map.h"

/*
 * 应用层入口模块（业务调度层）。
 *
 * 职责：
 * 1) 初始化机器人业务模块；
 * 2) 在固定 1ms 周期里推进一次控制流程。
 *
 * 建议调用顺序：
 * - 上电后先调用 App_Robot_Init()
 * - 然后在 1ms 定时任务/中断中循环调用 App_Robot_Loop1ms()
 */

/* 初始化业务模块（电机总线、目标生成器等）。 */
void App_Robot_Init(void);

/*
 * 1ms 周期任务：
 * - 生成/刷新关节目标
 * - 执行一次总线轮询收发
 */
void App_Robot_Loop1ms(void);
/* 调试波形发送接口（用于 VOFA 观察运行状态）。 */
void App_vofa_Send(void);

/* 上层算法可直接写入该目标角数组（4腿x每腿3电机）。 */
/*
 * 上层算法直接写这个数组就行：
 * - 第 1 维是腿编号（0~3）
 * - 第 2 维是该腿的关节编号（0~2）
 */
extern float Target_Angle[ROBOT_LEG_NUM][MOTORS_PER_LEG];

/*
 * 关节角平滑插值：
 * - target_angle: 上层解算出的目标角(rad)
 * - pos_rel:      当前电机反馈相对角(rad)
 * 返回值：本周期插值后的平滑目标角(rad)
 *
 * 该函数设计为 1ms 周期调用。
 */
float App_motor_angle_calculate(float target_angle, float pos_rel);

/*
 * 将“相对角目标”转换为“电机发送绝对角”。
 * - pos_rel:          当前反馈相对角（已按 sign 统一方向）
 * - target_angle_rel: 目标相对角（和 pos_rel 同坐标系）
 * - pos_abs:          当前反馈原始绝对角（motor_r.Pos）
 * - sign:             安装方向（+1 或 -1）
 * 返回值：可直接发送给电机的绝对角(rad)
 */
float App_target_relative_to_absolute(float pos_rel,
                                      float target_angle_rel,
                                      float pos_abs,
                                      int sign);

/*
 * 批量计算 12 个电机的平滑目标角，并写入发送缓冲。
 * - target_angle: 上层给出的 12 个目标角(rad)
 * - leg:          电机反馈容器（用于读取各电机 PosRel）
 */
void App_all_motor_claculate(float target_angle[ROBOT_LEG_NUM][MOTORS_PER_LEG],
                             Leg leg[ROBOT_LEG_NUM]);

/*
 * 返回值是一个二维数组指针：
 * - 指向 [ROBOT_LEG_NUM][MOTORS_PER_LEG] 的平滑角缓存
 * - 读取后可直接用于日志/可视化，不建议外部改写
 */
float (*App_Get_Smoothed_Angle(void))[MOTORS_PER_LEG];
#endif
