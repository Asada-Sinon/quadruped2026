#include "app_robot.h"
#include "cmsis_os.h"
#include "M8010.h"
#include "robot_map.h"
#include "vofa.h"
#include <math.h>

#define APP_INTERP_DT_MS 1.0f /* 每次调用插值函数时，默认推进 1ms 的相位。 */
#define APP_INTERP_DT_MAX_MS 20.0f /* 实测 dt 的上限保护，防止调试停顿后一步跳太大。 */
#define APP_INTERP_MIN_DURATION_MS 300.0f /* 单次插值最短持续时间，防止动作太猛。 */
#define APP_INTERP_MAX_DURATION_MS 2000.0f /* 单次插值最长持续时间，防止动作过慢。 */
#define APP_INTERP_SPEED_RAD_PER_S 0.8f /* 依据角度差推算时长时使用的期望角速度(rad/s)。 */
#define APP_INTERP_TARGET_EPS 1.0e-4f /* 目标变化阈值，小于该值视为同一目标。 */
#define APP_MOTOR_COUNT (ROBOT_LEG_NUM * MOTORS_PER_LEG) /* 4 条腿 x 每腿 3 个电机 = 12。 */

/* 上层算法写入的 12 路目标角（单位 rad），按 [腿][关节] 索引。 */
float Target_Angle[ROBOT_LEG_NUM][MOTORS_PER_LEG] = {0};
/* 保存每个电机本周期算出的平滑目标角，便于调试和可视化。 */
static float g_smoothed_angle[ROBOT_LEG_NUM][MOTORS_PER_LEG] = {0};
/* 当前正在计算第几个电机（0~11），供单电机插值函数选择对应状态槽。 */
static uint8_t g_interp_ctx_idx = 0U;
/* 本次控制循环测得的真实 dt(ms)，供 12 个电机共享同一时间步长。 */
static float g_interp_dt_ms = APP_INTERP_DT_MS;

static void app_interp_set_ctx(uint8_t idx)
{
    /* 正常范围内就直接使用该电机编号。 */
    if (idx < APP_MOTOR_COUNT)
    {
        g_interp_ctx_idx = idx;
    }
    else
    {
        /* 异常索引时回落到 0，避免数组越界。 */
        g_interp_ctx_idx = 0U;
    }
}

static float app_clampf(float in, float min_val, float max_val)
{
    if (in < min_val)
    {
        return min_val;
    }
    if (in > max_val)
    {
        return max_val;
    }
    return in;
}

static float app_get_real_dt_ms(void)
{
    /* 记录上一周期的系统 tick（单位 ms）。 */
    static uint32_t last_tick = 0U;
    /* 当前系统 tick（HAL 默认 1ms 递增）。 */
    uint32_t now_tick = HAL_GetTick();
    /* 本周期 tick 差值。 */
    uint32_t diff_tick;

    /* 首次调用时无法做差，直接返回默认 1ms。 */
    if (last_tick == 0U)
    {
        last_tick = now_tick;
        return APP_INTERP_DT_MS;
    }

    /* 无符号减法天然支持 tick 回卷。 */
    diff_tick = now_tick - last_tick;
    /* 刷新 last_tick，供下次循环使用。 */
    last_tick = now_tick;

    /* 若同一毫秒内重复进入，至少按 1ms 推进，避免 dt=0。 */
    if (diff_tick == 0U)
    {
        diff_tick = 1U;
    }

    /* 对异常大 dt 做上限保护，避免一次性插值跳跃过大。 */
    return app_clampf((float)diff_tick, 1.0f, APP_INTERP_DT_MAX_MS);
}
/*
 * 业务初始化：
 * 当前只需要初始化电机总线层。
 * 后续若新增状态估计/任务管理，也建议从这里统一初始化。
 */
void App_Robot_Init(void)
{ // 电机参数初始化，485端口和电机绑定
    cmd_init();
    RobotMap_Init();
    send_data_all(legs);
    cmd_single_test_init();
    VOFA_JF_DMA_Init(&hvofa, &huart6);
}

/*
 * 业务 1ms 主循环：
 * 1) 读取当前系统 tick；
 * 2) 刷新控制目标（目前是固定站立角）；
 * 3) 推进一次通信状态机。
 *
 * 注意：这里不做阻塞延时，实时性由外层 1ms 调度保障。
 */
void App_Robot_Loop1ms(void)
{
    /* 计算本周期真实 dt，后续 12 路插值都使用这个时间步长。 */
    g_interp_dt_ms = app_get_real_dt_ms();
    /* 先基于最新目标角 + 反馈 PosRel 计算 12 路平滑位置。 */
    App_all_motor_claculate(Target_Angle, legs);
    /* 再统一下发本周期已经平滑后的 12 路位置命令。 */
    send_data_all(legs);
}
float ch[VOFA_JF_MAX_CH] = {0};

void App_vofa_Send(void)
{

    ch[0] += 0.1f;
    ch[1] += 0.2f;
    ch[2] += 0.3f;
    ch[3] += 0.4f;

    VOFA_JF_DMA_Send(&hvofa, ch, 16);
}

float App_motor_angle_calculate(float target_angle, float pos_rel)
{
    /* 每个电机独立的插值起点，避免 12 路电机串状态。 */
    static float start_angle[APP_MOTOR_COUNT] = {0};
    /* 每个电机独立的插值终点（上一次确认过的目标）。 */
    static float end_angle[APP_MOTOR_COUNT] = {0};
    /* 每个电机当前已经走了多少毫秒。 */
    static float elapsed_ms[APP_MOTOR_COUNT] = {0};
    /* 每个电机本段插值总时长（毫秒）。 */
    static float duration_ms[APP_MOTOR_COUNT] = {APP_INTERP_MIN_DURATION_MS};
    /* 每个电机是否做过首次初始化。 */
    static uint8_t inited[APP_MOTOR_COUNT] = {0};

    /* 读取当前电机上下文编号（由批量函数在外层提前设置）。 */
    uint8_t idx = g_interp_ctx_idx;

    /* 当前段角度差。 */
    float delta;
    /* 归一化相位 s，范围 [0,1]。 */
    float s;
    /* 以下是 s 的幂次，减少重复计算。 */
    float s2;
    float s3;
    float s4;
    float s5;
    /* 五次多项式插值系数 u(s)。 */
    float blend;

    /* 首次进入时：以当前反馈角作为起点，目标角作为终点。 */
    if (inited[idx] == 0U)
    {
        /* 把当前反馈角锁为起点，避免首次跳变。 */
        start_angle[idx] = pos_rel;
        /* 记录当前插值段终点。 */
        end_angle[idx] = target_angle;
        /* 计算本段角度差。 */
        delta = end_angle[idx] - start_angle[idx];
        /* 按“角度差/期望速度”推导该段时长，并转成 ms。 */
        duration_ms[idx] = fabsf(delta) / APP_INTERP_SPEED_RAD_PER_S * 1000.0f;
        /* 给时长上/下限，保证动作可控。 */
        duration_ms[idx] = app_clampf(duration_ms[idx], APP_INTERP_MIN_DURATION_MS, APP_INTERP_MAX_DURATION_MS);
        /* 新段开始，已用时清零。 */
        elapsed_ms[idx] = 0.0f;
        /* 标记该电机已初始化。 */
        inited[idx] = 1U;
    }

    /* 如果目标角明显变化，则从“当前反馈角”重新起一段新插值。 */
    if (fabsf(target_angle - end_angle[idx]) > APP_INTERP_TARGET_EPS)
    {
        /* 新目标来时，用实时反馈作为新起点最稳妥。 */
        start_angle[idx] = pos_rel;
        /* 更新插值段终点为新目标。 */
        end_angle[idx] = target_angle;
        /* 重新计算该段角度差。 */
        delta = end_angle[idx] - start_angle[idx];
        /* 重新估算时长。 */
        duration_ms[idx] = fabsf(delta) / APP_INTERP_SPEED_RAD_PER_S * 1000.0f;
        /* 仍然做时长裁剪，防止过快过慢。 */
        duration_ms[idx] = app_clampf(duration_ms[idx], APP_INTERP_MIN_DURATION_MS, APP_INTERP_MAX_DURATION_MS);
        /* 新段从 0ms 开始计时。 */
        elapsed_ms[idx] = 0.0f;
    }

    /* 仅在还没走到终点时推进时间。 */
    if (elapsed_ms[idx] < duration_ms[idx])
    {
        /* 按本周期实测 dt 推进插值时间。 */
        elapsed_ms[idx] += g_interp_dt_ms;
        /* 防止最后一步超过总时长。 */
        if (elapsed_ms[idx] > duration_ms[idx])
        {
            /* 超了就钳位到终点时间。 */
            elapsed_ms[idx] = duration_ms[idx];
        }
    }

    /* 把时间进度映射成 0~1 的归一化相位。 */
    s = elapsed_ms[idx] / duration_ms[idx];
    /* 再做一次保险钳位。 */
    s = app_clampf(s, 0.0f, 1.0f);

    /* 预计算幂次，供五次曲线计算。 */
    s2 = s * s;
    s3 = s2 * s;
    s4 = s3 * s;
    s5 = s4 * s;

    /* 五次多项式 S 曲线：端点速度/加速度均为 0。 */
    blend = 10.0f * s3 - 15.0f * s4 + 6.0f * s5;
    return start_angle[idx] + (end_angle[idx] - start_angle[idx]) * blend;
}
//相对角和绝对角转换
float App_target_relative_to_absolute(float pos_rel,
                                      float target_angle_rel,
                                      float pos_abs,
                                      int sign)
{
    /* 相对角误差：目标相对角 - 当前相对角。 */
    float delta_rel = target_angle_rel - pos_rel;
    /* 安装方向归一化为 ±1，避免异常 sign 值影响结果。 */
    float dir = (sign >= 0) ? 1.0f : -1.0f;

    /*
     * 绝对角换算：
     * abs_cmd = 当前绝对角 + dir * (目标相对角 - 当前相对角)
     * 这样可把“相对目标”准确映射回电机协议要的“绝对角”。
     */
    return pos_abs + dir * delta_rel;
}

void App_all_motor_claculate(float target_angle[ROBOT_LEG_NUM][MOTORS_PER_LEG],
                             Leg leg[ROBOT_LEG_NUM])
{
    /* 外层腿编号 0~3。 */
    uint8_t leg_idx;
    /* 内层关节编号 0~2。 */
    uint8_t motor_idx;

    /* 空指针保护，防止异常调用导致崩溃。 */
    if ((target_angle == NULL) || (leg == NULL))
    {
        return;
    }

    /* 双层循环覆盖全部 12 个电机。 */
    for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++)
    {
        for (motor_idx = 0U; motor_idx < MOTORS_PER_LEG; motor_idx++)
        {
            /* [腿][关节] 映射成线性下标 0~11。 */
            uint8_t cmd_idx = (uint8_t)(leg_idx * MOTORS_PER_LEG + motor_idx);
            /* 拿到当前电机对象，后续读取 PosRel 和写入 motor_s。 */
            M8010 *motor = &leg[leg_idx].motors_peer_leg[motor_idx];
            /* 当前电机最终要发送的绝对角。 */
            float target_abs;

            /* 告诉单电机插值函数：当前正在算哪一个电机。 */
            app_interp_set_ctx(cmd_idx);
            /* 用当前电机目标角 + 当前电机反馈角，求本周期平滑角。 */
            g_smoothed_angle[leg_idx][motor_idx] = App_motor_angle_calculate(target_angle[leg_idx][motor_idx],
                                                                              motor->motor_r.PosRel);

            /* 将平滑后的相对角目标转换成电机协议所需绝对角。 */
            target_abs = App_target_relative_to_absolute(motor->motor_r.PosRel,
                                                         g_smoothed_angle[leg_idx][motor_idx],
                                                         motor->motor_r.Pos,
                                                         motor->sign);

            /* 同步保存绝对角到电机对象，便于本地状态查看。 */
            motor->motor_s.Pos = target_abs;
            /* 直接写入发送缓冲 cmd[]，保证 send_data_all 可直接发送。 */
            set_cmd_pos_by_index(cmd_idx, target_abs);
        }
    }
}

// float (*App_Get_Smoothed_Angle(void))[MOTORS_PER_LEG]
// {
//     /* 返回二维数组首地址，外部可直接读取 12 路平滑结果。 */
//     return g_smoothed_angle;
// }

