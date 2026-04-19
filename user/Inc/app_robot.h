#ifndef APP_ROBOT_H
#define APP_ROBOT_H

/*
 * 这里引入 robot_map.h 的目的是拿到：
 * 1) ROBOT_LEG_NUM / MOTORS_PER_LEG 宏；
 * 2) Leg 结构体类型定义。
 */
#include "robot_map.h"

/*
 * 机器人控制模式（最小状态机）：
 * - STAND：站立/定点姿态
 * - WALK：行走/步态轨迹
 * - CRAWL：匍匐/低姿态步态（与 WALK 相同，但四足 z 统一上移）
 */
typedef enum
{
    ROBOT_MODE_STAND = 0U,//正常站立
    ROBOT_MODE_WALK = 1U,//正常行走
    ROBOT_MODE_CRAWL = 2U//匍匐行走
} RobotControlMode;

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

/*
 * 设置控制模式：
 * - 仅通过该接口切换 STAND/WALK/CRAWL，避免外置标志位导致逻辑分散。
 */
void App_SetControlMode(RobotControlMode mode);

/* 获取当前控制模式，便于上层做状态显示或调试。 */
RobotControlMode App_GetControlMode(void);

/*
 * 设置行走参数：
 * - freq_hz: 步态频率(Hz)
 * - step_length_m: 步长(m)
 * - swing_height_m: 抬脚高度(m)
 */
void App_SetWalkParams(float freq_hz,
                       float step_length_m,
                       float swing_height_m);

/*
 * 设置站立目标足端位置：
 * - stand_x_m_by_leg: 四条腿各自 x 目标(m)
 * - stand_y_m_by_leg: 四条腿各自 y 目标(m)
 * - stand_z_m_by_leg: 四条腿各自 z 目标(m)
 *   下标顺序与 gait.h 的腿编号一致：
 *   [0]=LEG_FL, [1]=LEG_FR, [2]=LEG_HL, [3]=LEG_HR
 */
void App_SetStandPose(const float stand_x_m_by_leg[ROBOT_LEG_NUM],
                      const float stand_y_m_by_leg[ROBOT_LEG_NUM],
                      const float stand_z_m_by_leg[ROBOT_LEG_NUM]);

/* 上层算法可直接写入该目标角数组（4腿x每腿3电机）。 */
/*
 * 上层算法直接写这个数组就行：
 * - 第 1 维是腿编号（0~3）
 * - 第 2 维是该腿的关节编号（0~2）
 */
extern float Target_Angle[ROBOT_LEG_NUM][MOTORS_PER_LEG];

/* 当前相对角请直接读取 leg[x].motors_peer_leg[y].motor_r.PosRel（单位 rad）。 */

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
 * 批量计算 12 个电机控制目标，并写入发送缓冲。
 * - target_angle: 上层给出的 12 个目标角(rad)
 * - leg:          电机反馈容器（用于读取各电机 PosRel）
 *
 * 模式策略：
 * - STAND：启用电机插值平滑（适合“直接给终点角”的站立动作）
 * - WALK：关闭电机插值，直接跟踪 IK 输出（避免行走链路双层平滑）
 */
void App_all_motor_claculate(float target_angle[ROBOT_LEG_NUM][MOTORS_PER_LEG],
                             Leg leg[ROBOT_LEG_NUM]);
//根据电机当前角度来计算当前足端位置
void App_UpdateCurrentFootPosFromMotor(Leg leg[ROBOT_LEG_NUM]);
#endif
