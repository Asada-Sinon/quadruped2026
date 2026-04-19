#include "app_robot.h"
#include "cmsis_os.h"
#include "M8010.h"
#include "robot_map.h"
#include "gait.h"
#include "vofa.h"
#include <math.h>
#include "trajectory.h"
#include "HT10A.h"
#include "FreeRTOS.h"
#include "task.h"

#define APP_INTERP_DT_MS 1.0f                            /* 每次调用插值函数时，默认推进 1ms 的相位。 */
#define APP_INTERP_DT_MAX_MS 20.0f                       /* 实测 dt 的上限保护，防止调试停顿后一步跳太大。 */
#define APP_INTERP_MIN_DURATION_MS 300.0f                /* 单次插值最短持续时间，防止动作太猛。 */
#define APP_INTERP_MAX_DURATION_MS 2000.0f               /* 单次插值最长持续时间，防止动作过慢。 */
#define APP_INTERP_SPEED_RAD_PER_S 0.8f                  /* 依据角度差推算时长时使用的期望角速度(rad/s)。 */
#define APP_INTERP_TARGET_EPS 1.0e-4f                    /* 目标变化阈值，小于该值视为同一目标。 */
#define APP_MOTOR_COUNT (ROBOT_LEG_NUM * MOTORS_PER_LEG) /* 4 条腿 x 每腿 3 个电机 = 12。 */
#define APP_DEFAULT_STAND_X_M (-0.0638f)
#define APP_DEFAULT_STAND_Y_M (0.0780f)
#define APP_DEFAULT_STAND_Z_M (-0.3230f)
#define APP_DEFAULT_WALK_FREQ_HZ (1.2f)
#define APP_DEFAULT_WALK_SWING_HEIGHT_M (0.07f)
#define APP_DEFAULT_WALK_STEP_LENGTH_M (0.10f)
/* 匍匐模式相对 WALK 的固定 z 偏移：四足统一上移 0.1m。 */
#define APP_CRAWL_Z_OFFSET_M (0.10f)
/* FREE_MOVE 摇杆原始范围与死区。 */
#define APP_FREE_RC_AXIS_RAW_LIMIT (10000.0f)
#define APP_FREE_RC_DEADZONE_RAW (800.0f)
/* FREE_MOVE 保守速度标定（摇杆打满对应值）。 */
#define APP_FREE_VX_MAX_M_S (0.20f)
#define APP_FREE_VY_MAX_M_S (0.15f)
#define APP_FREE_W_MAX_RAD_S (0.80f)
/* 若后续发现方向相反，只需改这 3 个符号即可。 */
#define APP_FREE_VX_SIGN (1.0f)
#define APP_FREE_VY_SIGN (-1.0f)
#define APP_FREE_W_SIGN (-1.0f)

typedef struct
{
    /* 当前控制模式，只允许通过 App_SetControlMode 切换。 */
    RobotControlMode mode;
    /* 上一周期模式，用于检测状态切换并执行一次性 entry 动作。 */
    RobotControlMode last_mode;
    /* 四条腿各自的站立 x 目标，下标顺序: FL, FR, HL, HR。 */
    float stand_x_m_by_leg[ROBOT_LEG_NUM];
    /* 四条腿各自的站立 y 目标，下标顺序: FL, FR, HL, HR。 */
    float stand_y_m_by_leg[ROBOT_LEG_NUM];
    /* 四条腿各自的站立 z 目标，下标顺序: FL, FR, HL, HR。 */
    float stand_z_m_by_leg[ROBOT_LEG_NUM];
    /* 行走模式轨迹参数。 */
    float walk_freq_hz;
    float walk_swing_height_m;
    float walk_step_length_m;

    /* WALK 模式的电机参数初始化只做一次，避免每周期重复发初始化命令。 */
    uint8_t walk_cmd_inited;
} AppControlContext;

static DiagonalCycloidGait g_gait;
/* 应用层统一控制上下文，替代分散的全局标志位。 */
AppControlContext g_app_ctrl =
    {
        ROBOT_MODE_STAND,
        ROBOT_MODE_STAND,
        {APP_DEFAULT_STAND_X_M,
         APP_DEFAULT_STAND_X_M,
         APP_DEFAULT_STAND_X_M,
         APP_DEFAULT_STAND_X_M},
        {APP_DEFAULT_STAND_Y_M,
         -APP_DEFAULT_STAND_Y_M,
         APP_DEFAULT_STAND_Y_M,
         -APP_DEFAULT_STAND_Y_M},
        {APP_DEFAULT_STAND_Z_M,
         APP_DEFAULT_STAND_Z_M,
         APP_DEFAULT_STAND_Z_M+0.005f, // 后腿相对前腿的 z 偏置，默认多降低 1cm，避免后腿抬得过高不稳
         APP_DEFAULT_STAND_Z_M+0.005f},
        APP_DEFAULT_WALK_FREQ_HZ,
        APP_DEFAULT_WALK_SWING_HEIGHT_M,
        APP_DEFAULT_WALK_STEP_LENGTH_M,
        0U};
// 这个是电机输出轴的目标角度
float Target_Angle[ROBOT_LEG_NUM][MOTORS_PER_LEG] = {0};
/* 当前正在计算第几个电机（0~11），供单电机插值函数选择对应状态槽。 */
static uint8_t g_interp_ctx_idx = 0U;
/* 本次控制循环测得的真实 dt(ms)，供 12 个电机共享同一时间步长。 */
static float g_interp_dt_ms = APP_INTERP_DT_MS;
/* 每个电机独立的插值起点。 */
static float g_interp_start_angle[APP_MOTOR_COUNT] = {0};
/* 每个电机独立的插值终点（上次确认过的目标）。 */
static float g_interp_end_angle[APP_MOTOR_COUNT] = {0};
/* 每个电机当前插值已推进时间。 */
static float g_interp_elapsed_ms[APP_MOTOR_COUNT] = {0};
/* 每个电机本段插值总时长。 */
static float g_interp_duration_ms[APP_MOTOR_COUNT] = {0};
/* 每个电机插值状态是否已初始化。 */
static uint8_t g_interp_inited[APP_MOTOR_COUNT] = {0};
// 物理世界中固定的上电位置并非0位，而是有一个偏置的，这个函数就是把这个偏置加上去，得到一个更符合机械定义的角度值，方便上层算法使用。
static float App_Get_Model_Joint_Angle(uint8_t leg_idx,
                                       uint8_t motor_idx,
                                       const M8010 *motor)
{
    float trans_dir = (float)g_joint_transmission_sign[leg_idx][motor_idx];
    return trans_dir * motor->motor_r.PosRel + g_joint_offset_rad[leg_idx][motor_idx];
}
// 上面函数的反算版本，模型关节角转电机相对角，输入是模型关节角，输出是电机相对角（已经考虑安装方向和零位偏置）。这个函数在 gait.c 的 Gait_UpdateTargetAngleFromFootTarget() 里被调用，用于把足端目标经过 IK 转成关节角后，再转成电机相对角发给电机。
static float App_Model_Joint_Angle_To_Motor_Rel(uint8_t leg_idx,
                                                uint8_t motor_idx,
                                                float joint_angle)
{
    float trans_dir = (float)g_joint_transmission_sign[leg_idx][motor_idx];
    return (joint_angle - g_joint_offset_rad[leg_idx][motor_idx]) / trans_dir;
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

/* 把遥控原始轴值映射到 [-1,1]：
 * 1) 先按原始量限幅；
 * 2) 应用死区；
 * 3) 去死区后归一化。
 */
static float app_map_rc_axis_to_unit(float raw_axis)
{
    float abs_axis;
    float span;

    raw_axis = app_clampf(raw_axis,
                          -APP_FREE_RC_AXIS_RAW_LIMIT,
                          APP_FREE_RC_AXIS_RAW_LIMIT);
    abs_axis = fabsf(raw_axis);
    if (abs_axis <= APP_FREE_RC_DEADZONE_RAW)
    {
        return 0.0f;
    }

    span = APP_FREE_RC_AXIS_RAW_LIMIT - APP_FREE_RC_DEADZONE_RAW;
    if (span <= 0.0f)
    {
        return 0.0f;
    }

    if (raw_axis > 0.0f)
    {
        return (raw_axis - APP_FREE_RC_DEADZONE_RAW) / span;
    }

    return (raw_axis + APP_FREE_RC_DEADZONE_RAW) / span;
}

/* 从遥控全局数据抓取 FREE_MOVE 速度命令（m/s, rad/s）。 */
static void App_GetFreeMoveVelocityCmd(float *vx_m_s,
                                       float *vy_m_s,
                                       float *w_rad_s)
{
    float rc_vx_raw;
    float rc_vy_raw;
    float rc_w_raw;
    float vx_unit;
    float vy_unit;
    float w_unit;

    if ((vx_m_s == 0) || (vy_m_s == 0) || (w_rad_s == 0))
    {
        return;
    }

    /* ISR 在更新 Teaching_Pendant，任务侧读取时做一次短临界区快照。 */
    taskENTER_CRITICAL();
    rc_vx_raw = Teaching_Pendant.Vx;
    rc_vy_raw = Teaching_Pendant.Vy;
    rc_w_raw = Teaching_Pendant.Vw;
    taskEXIT_CRITICAL();

    vx_unit = app_map_rc_axis_to_unit(rc_vx_raw);
    vy_unit = app_map_rc_axis_to_unit(rc_vy_raw);
    w_unit = app_map_rc_axis_to_unit(rc_w_raw);

    *vx_m_s = APP_FREE_VX_SIGN * vx_unit * APP_FREE_VX_MAX_M_S;
    *vy_m_s = APP_FREE_VY_SIGN * vy_unit * APP_FREE_VY_MAX_M_S;
    *w_rad_s = APP_FREE_W_SIGN * w_unit * APP_FREE_W_MAX_RAD_S;
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

static uint8_t app_is_valid_mode(RobotControlMode mode)
{
    if ((mode == ROBOT_MODE_STAND) ||
        (mode == ROBOT_MODE_WALK) ||
        (mode == ROBOT_MODE_CRAWL) ||
        (mode == ROBOT_MODE_FREE_MOVE))
    {
        return 1U;
    }
    return 0U;
}

/* 统一重置 12 路插值内部状态，避免模式切换后沿用旧插值段造成突变。 */
static void App_ResetAllInterpolationState(void)
{
    uint8_t idx;
    for (idx = 0U; idx < APP_MOTOR_COUNT; idx++)
    {
        g_interp_start_angle[idx] = 0.0f;
        g_interp_end_angle[idx] = 0.0f;
        g_interp_elapsed_ms[idx] = 0.0f;
        g_interp_duration_ms[idx] = APP_INTERP_MIN_DURATION_MS;
        g_interp_inited[idx] = 0U;
    }
}

/*
 * 把步态名义点同步到当前站立位姿：
 * 这样从 STAND 切到 WALK 时，轨迹中心不再使用 trajectory.c 里的固定默认值，
 * 而是直接以各腿 stand_x/y/z 数组为基准，
 * 避免切换瞬间出现位置/高度突变。
 */
static void App_SyncWalkNominalFromStandPose(void)
{
    Trajectory_SetNominalFoot(&g_gait,
                              LEG_FL,
                              g_app_ctrl.stand_x_m_by_leg[LEG_FL],
                              g_app_ctrl.stand_y_m_by_leg[LEG_FL],
                              g_app_ctrl.stand_z_m_by_leg[LEG_FL]);
    Trajectory_SetNominalFoot(&g_gait,
                              LEG_FR,
                              g_app_ctrl.stand_x_m_by_leg[LEG_FR],
                              g_app_ctrl.stand_y_m_by_leg[LEG_FR],
                              g_app_ctrl.stand_z_m_by_leg[LEG_FR]);
    Trajectory_SetNominalFoot(&g_gait,
                              LEG_HL,
                              g_app_ctrl.stand_x_m_by_leg[LEG_HL],
                              g_app_ctrl.stand_y_m_by_leg[LEG_HL],
                              g_app_ctrl.stand_z_m_by_leg[LEG_HL]); // 后腿相对前腿的 z 偏置，默认多降低 1cm，避免后腿抬得过高不稳
    Trajectory_SetNominalFoot(&g_gait,
                              LEG_HR,
                              g_app_ctrl.stand_x_m_by_leg[LEG_HR],
                              g_app_ctrl.stand_y_m_by_leg[LEG_HR],
                              g_app_ctrl.stand_z_m_by_leg[LEG_HR]); // 后腿相对前腿的 z 偏置，默认多降低 1cm，避免后腿抬得过高不稳
}

/* 状态切换入口动作：只做一次的初始化/收尾放在这里，避免散落到主循环。 */
static void App_HandleModeEntry(RobotControlMode mode)
{
    if ((mode == ROBOT_MODE_WALK) ||
        (mode == ROBOT_MODE_CRAWL) ||
        (mode == ROBOT_MODE_FREE_MOVE))
    {
        /* 进入行走相关模式前先把轨迹中心对齐到当前 STAND 位姿。 */
        App_SyncWalkNominalFromStandPose();
        /* 行走相关模式首次进入时设置电机参数，后续不重复发。 */
        if (g_app_ctrl.walk_cmd_inited == 0U)
        {
            cmd_init_2();
            g_app_ctrl.walk_cmd_inited = 1U;
        }
        return;
    }

    if (mode == ROBOT_MODE_STAND)
    {
        /* 切回站立时重置步态相位，保证下次进入 WALK 从干净相位开始。 */
        Trajectory_Reset(&g_gait);
        /* 站立模式会再次启用电机插值，先清空插值状态，下一拍从当前反馈角重起。 */
        App_ResetAllInterpolationState();
    }
}

/* 站立模式：x/y/z 全部使用按腿独立数组。 */
static void App_UpdateStandFootTarget(void)
{
    Gait_SetLegFootTargetM(LEG_FL,
                           g_app_ctrl.stand_x_m_by_leg[LEG_FL],
                           g_app_ctrl.stand_y_m_by_leg[LEG_FL],
                           g_app_ctrl.stand_z_m_by_leg[LEG_FL]);
    Gait_SetLegFootTargetM(LEG_HL,
                           g_app_ctrl.stand_x_m_by_leg[LEG_HL],
                           g_app_ctrl.stand_y_m_by_leg[LEG_HL],
                           g_app_ctrl.stand_z_m_by_leg[LEG_HL]);
    Gait_SetLegFootTargetM(LEG_FR,
                           g_app_ctrl.stand_x_m_by_leg[LEG_FR],
                           g_app_ctrl.stand_y_m_by_leg[LEG_FR],
                           g_app_ctrl.stand_z_m_by_leg[LEG_FR]);
    Gait_SetLegFootTargetM(LEG_HR,
                           g_app_ctrl.stand_x_m_by_leg[LEG_HR],
                           g_app_ctrl.stand_y_m_by_leg[LEG_HR],
                           g_app_ctrl.stand_z_m_by_leg[LEG_HR]);
}

/* 行走模式：参数写入轨迹器后推进一步，生成四腿足端目标。 */
static void App_UpdateWalkFootTarget(float dt_s)
{
    Trajectory_SetStepLength(&g_gait, g_app_ctrl.walk_step_length_m);
    Trajectory_SetFrequency(&g_gait, g_app_ctrl.walk_freq_hz);
    Trajectory_SetSwingHeight(&g_gait, g_app_ctrl.walk_swing_height_m);
    Trajectory_SetBodyVelocity(&g_gait, 0.0f, 0.0f, 0.0f);
    Trajectory_Update(&g_gait, dt_s);
}

/* 匍匐模式：
 * 1) 先完整复用 WALK 轨迹生成（保证时序/步长/抬脚等完全一致）；
 * 2) 再把四条腿的 z 目标统一加固定偏置 0.1m。
 */
static void App_UpdateCrawlFootTarget(float dt_s)
{
    uint8_t leg_idx;

    App_UpdateWalkFootTarget(dt_s);

    for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++)
    {
        Gait_SetLegFootTargetM(leg_idx,
                               g_foot_target_m[leg_idx].x_m,
                               g_foot_target_m[leg_idx].y_m,
                               g_foot_target_m[leg_idx].z_m + APP_CRAWL_Z_OFFSET_M);
    }
}

/* 自由运动模式：
 * 1) 频率/抬脚高度仍沿用当前 WALK 参数（按你的要求不被摇杆覆盖）；
 * 2) 摇杆实时控制 body-frame Vx/Vy/Vw，实现全向平移 + 自转；
 * 3) 该模式不使用固定前后步长，故 step_length 置零。
 */
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
}
// 修改模式
void App_SetControlMode(RobotControlMode mode)
{
    if (app_is_valid_mode(mode) == 0U)
    {
        return;
    }
    g_app_ctrl.mode = mode;
}

RobotControlMode App_GetControlMode(void)
{
    return g_app_ctrl.mode;
}
// 行走参数
void App_SetWalkParams(float freq_hz,
                       float step_length_m,
                       float swing_height_m)
{
    if (freq_hz < 0.0f)
    {
        freq_hz = 0.0f;
    }
    if (swing_height_m < 0.0f)
    {
        swing_height_m = 0.0f;
    }

    g_app_ctrl.walk_freq_hz = freq_hz;
    g_app_ctrl.walk_step_length_m = step_length_m;
    g_app_ctrl.walk_swing_height_m = swing_height_m;
}
// 站立足端位置
void App_SetStandPose(const float stand_x_m_by_leg[ROBOT_LEG_NUM],
                      const float stand_y_m_by_leg[ROBOT_LEG_NUM],
                      const float stand_z_m_by_leg[ROBOT_LEG_NUM])
{
    uint8_t leg_idx;

    if (stand_x_m_by_leg != 0)
    {
        for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++)
        {
            g_app_ctrl.stand_x_m_by_leg[leg_idx] = stand_x_m_by_leg[leg_idx];
        }
    }
    if (stand_y_m_by_leg != 0)
    {
        for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++)
        {
            g_app_ctrl.stand_y_m_by_leg[leg_idx] = stand_y_m_by_leg[leg_idx];
        }
    }
    if (stand_z_m_by_leg != 0)
    {
        for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++)
        {
            g_app_ctrl.stand_z_m_by_leg[leg_idx] = stand_z_m_by_leg[leg_idx];
        }
    }

    /*
     * 仅在 STAND 模式下立即同步 nominal；
     * 若当前正在 WALK/CRAWL/FREE_MOVE，则把新参数缓存起来，
     * 等下次进入该模式前再同步，避免运行中跳变。
     */
    if (g_app_ctrl.mode == ROBOT_MODE_STAND)
    {
        App_SyncWalkNominalFromStandPose();
    }
}

/*
 * 业务初始化：
 * 当前只需要初始化电机总线层。
 * 后续若新增状态估计/任务管理，也建议从这里统一初始化。
 */
void App_Robot_Init(void)
{ // 电机参数初始化，485端口和电机绑定，初始化第一次，发送全空命令，先get到一次回传，拿到零位
    MotorBus_Restart(LEG_FL);
    MotorBus_Restart(LEG_FR);
    MotorBus_Restart(LEG_HL);
    MotorBus_Restart(LEG_HR);
    cmd_init();
    // 直接把腿和485端口绑定
    RobotMap_Init();
    send_data_all(legs);
    cmd_single_test_init();
    Trajectory_InitDefault(&g_gait);
    /* 启动时也统一让 walk nominal 以 stand 位姿为基准。 */
    App_SyncWalkNominalFromStandPose();
    App_ResetAllInterpolationState();
    /* 默认从 STAND 启动，last_mode 同步后可避免启动瞬间误判“模式切换”。 */
    g_app_ctrl.last_mode = g_app_ctrl.mode;
    VOFA_JF_DMA_Init(&hvofa, &huart6);
    Teaching_Pendant_Restart();
}
float bios = 0.015f; // 前腿相对后腿的额外抬高，默认多抬高 1.5cm，避免前腿摆动时过低不稳
/*
 * 业务 1ms 主循环：
 * 1) 读取当前系统 tick；
 * 2) 按 STAND/WALK/CRAWL/FREE_MOVE 状态机刷新足端目标；
 * 3) 推进一次通信状态机。
 *
 * 注意：这里不做阻塞延时，实时性由外层 1ms 调度保障。
 */
void App_Robot_Loop1ms(void)
{
    Trajectory_SetFrontSwingHeightBias(&g_gait, bios); // 前腿相对后腿的额外抬高，默认多抬高 1.5cm，避免前腿摆动时过低不稳
    /* 计算本周期真实 dt，后续 12 路插值都使用这个时间步长。 */
    g_interp_dt_ms = app_get_real_dt_ms();
    float dt_s = g_interp_dt_ms / 1000.0f;

    /* 模式切换检测：只在切换边沿执行 entry 动作，避免每拍重复做初始化。 */
    if (g_app_ctrl.mode != g_app_ctrl.last_mode)
    {
        App_HandleModeEntry(g_app_ctrl.mode);
        g_app_ctrl.last_mode = g_app_ctrl.mode;
    }

    switch (g_app_ctrl.mode)
    {
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
        /* 非法模式回退到 STAND，保证控制链路可预期。 */
        g_app_ctrl.mode = ROBOT_MODE_STAND;
        App_HandleModeEntry(g_app_ctrl.mode);
        g_app_ctrl.last_mode = g_app_ctrl.mode;
        App_UpdateStandFootTarget();
        break;
    }

    // 四个腿根据足端位置，逆运动学设置关节位置
    Gait_UpdateTargetAngleFromFootTarget(Target_Angle);
    // 站立：电机插值平滑；行走/匍匐：直接跟踪 IK 输出，避免“轨迹 + 电机”双层平滑带来的滞后
    App_all_motor_claculate(Target_Angle, legs);
    /* 再统一下发本周期已经平滑后的 12 路位置命令。 */
    send_data_all(legs);
}
void App_Robot_Send_Loop(void)
{
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
// 单电机平滑计算
float App_motor_angle_calculate(float target_angle, float pos_rel)
{
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
    if (g_interp_inited[idx] == 0U)
    {
        /* 把当前反馈角锁为起点，避免首次跳变。 */
        g_interp_start_angle[idx] = pos_rel;
        /* 记录当前插值段终点。 */
        g_interp_end_angle[idx] = target_angle;
        /* 计算本段角度差。 */
        delta = g_interp_end_angle[idx] - g_interp_start_angle[idx];
        /* 按“角度差/期望速度”推导该段时长，并转成 ms。 */
        g_interp_duration_ms[idx] = fabsf(delta) / APP_INTERP_SPEED_RAD_PER_S * 1000.0f;
        /* 给时长上/下限，保证动作可控。 */
        g_interp_duration_ms[idx] = app_clampf(g_interp_duration_ms[idx], APP_INTERP_MIN_DURATION_MS, APP_INTERP_MAX_DURATION_MS);
        /* 新段开始，已用时清零。 */
        g_interp_elapsed_ms[idx] = 0.0f;
        /* 标记该电机已初始化。 */
        g_interp_inited[idx] = 1U;
    }

    /* 如果目标角明显变化，则从“当前反馈角”重新起一段新插值。 */
    if (fabsf(target_angle - g_interp_end_angle[idx]) > APP_INTERP_TARGET_EPS)
    {
        /* 新目标来时，用实时反馈作为新起点最稳妥。 */
        g_interp_start_angle[idx] = pos_rel;
        /* 更新插值段终点为新目标。 */
        g_interp_end_angle[idx] = target_angle;
        /* 重新计算该段角度差。 */
        delta = g_interp_end_angle[idx] - g_interp_start_angle[idx];
        /* 重新估算时长。 */
        g_interp_duration_ms[idx] = fabsf(delta) / APP_INTERP_SPEED_RAD_PER_S * 1000.0f;
        /* 仍然做时长裁剪，防止过快过慢。 */
        g_interp_duration_ms[idx] = app_clampf(g_interp_duration_ms[idx], APP_INTERP_MIN_DURATION_MS, APP_INTERP_MAX_DURATION_MS);
        /* 新段从 0ms 开始计时。 */
        g_interp_elapsed_ms[idx] = 0.0f;
    }

    /* 仅在还没走到终点时推进时间。 */
    if (g_interp_elapsed_ms[idx] < g_interp_duration_ms[idx])
    {
        /* 按本周期实测 dt 推进插值时间。 */
        g_interp_elapsed_ms[idx] += g_interp_dt_ms;
        /* 防止最后一步超过总时长。 */
        if (g_interp_elapsed_ms[idx] > g_interp_duration_ms[idx])
        {
            /* 超了就钳位到终点时间。 */
            g_interp_elapsed_ms[idx] = g_interp_duration_ms[idx];
        }
    }

    /* 把时间进度映射成 0~1 的归一化相位。 */
    s = g_interp_elapsed_ms[idx] / g_interp_duration_ms[idx];
    /* 再做一次保险钳位。 */
    s = app_clampf(s, 0.0f, 1.0f);

    /* 预计算幂次，供五次曲线计算。 */
    s2 = s * s;
    s3 = s2 * s;
    s4 = s3 * s;
    s5 = s4 * s;

    /* 五次多项式 S 曲线：端点速度/加速度均为 0。 */
    blend = 10.0f * s3 - 15.0f * s4 + 6.0f * s5;
    return g_interp_start_angle[idx] + (g_interp_end_angle[idx] - g_interp_start_angle[idx]) * blend;
}
// 相对角和绝对角转换
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
    // return pos_abs + dir * delta_rel;
    return pos_abs + dir * delta_rel * ROBOT_MOTOR_GEAR_RATIO;
}
// 所有电机平滑计算，算完直接给到cmd里面
void App_all_motor_claculate(float target_angle[ROBOT_LEG_NUM][MOTORS_PER_LEG],
                             Leg leg[ROBOT_LEG_NUM])
{
    /* 外层腿编号 0~3。 */
    uint8_t leg_idx;
    /* 内层关节编号 0~2。 */
    uint8_t motor_idx;
    /* 双层循环覆盖全部 12 个电机。 */
    for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++)
    {
        for (motor_idx = 0U; motor_idx < MOTORS_PER_LEG; motor_idx++)
        {
            /* [腿][关节] 映射成线性下标 0~11。 */
            uint8_t cmd_idx = (uint8_t)(leg_idx * MOTORS_PER_LEG + motor_idx);
            /* 拿到当前电机对象，后续读取 PosRel 和写入 motor_s。 */
            M8010 *motor = &leg[leg_idx].motors_peer_leg[motor_idx];
            float current_model_angle;
            float control_model_angle;
            float target_motor_rel;
            /* 当前电机最终要发送的绝对角。 */
            float target_abs;

            current_model_angle = App_Get_Model_Joint_Angle(leg_idx, motor_idx, motor);
            if (g_app_ctrl.mode == ROBOT_MODE_STAND)
            {
                /* 站立模式保留关节插值：直接给终点角时，用五次曲线减少瞬时冲击。 */
                g_interp_ctx_idx = cmd_idx;
                control_model_angle = App_motor_angle_calculate(target_angle[leg_idx][motor_idx],
                                                                current_model_angle);
            }
            else
            {
                /* 行走/匍匐/自由运动模式关闭电机插值：轨迹层已保证轨迹连续，这里直接跟踪 IK 输出。 */
                control_model_angle = target_angle[leg_idx][motor_idx];
            }

            /* 模型关节目标角 -> 电机相对角 */
            target_motor_rel =
                App_Model_Joint_Angle_To_Motor_Rel(leg_idx,
                                                   motor_idx,
                                                   control_model_angle);

            /* 电机相对角 -> 电机绝对角命令 */
            target_abs = App_target_relative_to_absolute(motor->motor_r.PosRel,
                                                         target_motor_rel,
                                                         motor->motor_r.Pos,
                                                         motor->sign);
            /* 同步保存绝对角到电机对象，便于本地状态查看。 */
            motor->motor_s.Pos = target_abs;
            /* 直接写入发送缓冲 cmd[]，保证 send_data_all 可直接发送。 */
            set_cmd_pos_by_index(cmd_idx, target_abs);
        }
    }
}
// 根据电机当前角度来计算当前足端位置
float joint_pos[3];
void App_UpdateCurrentFootPosFromMotor(Leg leg[ROBOT_LEG_NUM])
{
    uint8_t leg_idx;
    uint8_t motor_idx;

    for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++)
    {
        // float joint_pos[3];
        float foot_pos[3];
        uint8_t valid = 1U;

        for (motor_idx = 0U; motor_idx < MOTORS_PER_LEG; motor_idx++)
        {
            M8010 *motor = &leg[leg_idx].motors_peer_leg[motor_idx];

            /* 如果这一关节还没有有效回传，就先不更新这一条腿 */
            if ((motor->motor_r.PosZeroInited == 0U) || (motor->motor_r.correct == 0))
            {
                valid = 0U;
                break;
            }

            /* 当前模型角 = 当前相对角 + 固定偏置 */
            joint_pos[motor_idx] = App_Get_Model_Joint_Angle(leg_idx, motor_idx, motor);
        }

        if (valid == 0U)
        {
            continue;
        }

        /* 用模型角做正运动学，得到当前足端位置 */
        leg_forward_kinematics(leg_idx, joint_pos, foot_pos);

        g_foot_current_m[leg_idx].x_m = foot_pos[X_IDX];
        g_foot_current_m[leg_idx].y_m = foot_pos[Y_IDX];
        g_foot_current_m[leg_idx].z_m = foot_pos[Z_IDX];
    }
}
