#include "gait.h" /* 引入 gait 模块头文件，拿到对外声明和数据类型定义。 */

#include <math.h>   /* 引入数学函数：sinf/cosf/atan2f/acosf/sqrtf/fabsf。 */
#include <string.h> /* 引入内存操作函数：memset。 */

#define GAIT_PI 3.14159265358979323846f      /* 圆周率常量：用于角度包裹到 [-pi, pi]。 */
#define GAIT_IK_EPS 1.0e-4f                   /* IK 数值容差：用于边界浮点误差保护（适当放大死区，减小边界抖动）。 */

#define GAIT_IK_FAIL_NONE 0U
#define GAIT_IK_FAIL_INPUT 1U
#define GAIT_IK_FAIL_YZ_UNREACHABLE 2U
#define GAIT_IK_FAIL_COS_UNREACHABLE 3U
#define GAIT_IK_FAIL_NONFINITE 4U

/*
 * gait 全局缓存：4 条腿的足端目标点（单位 mm）。
 * 数组下标与腿编号一致：0 左前，1 右前，2 左后，3 右后。
 * 这个坐标是髋关节局部坐标系的，原点在该腿 motor1 轴心，+X 向前，+Y 向左，+Z 向上。
 */
GaitFootPosMm g_foot_target_mm[ROBOT_LEG_NUM];

/*
 * IK 调试全局变量：保存最近一次 IK 解算的全部中间量。
 * 便于在调试器 Watch 窗口中直接观察 NaN 从哪一步开始出现。
 */
GaitIkDebug g_ik_debug;

/*
 * gait 内部缓存：上一帧 IK 解出的关节角（单位 rad）。
 * 用途：本帧 IK 选分支时，优先选“更接近上一帧”的那支，避免跳解。
 */
static float g_last_ik_angle_rad[ROBOT_LEG_NUM][MOTORS_PER_LEG];

/*
 * gait 内部标记：上一帧 IK 结果是否有效。
 * - 0：该腿还没有有效 IK 历史；
 * - 1：该腿已有有效 IK 历史。
 */
static uint8_t g_last_ik_inited[ROBOT_LEG_NUM];

/*
 * IK 功能总开关：
 * - 0：关闭（保持外部直接写 target_angle 的模式）；
 * - 1：开启（每周期根据足端目标反解角度）。
 */
static uint8_t g_ik_enable = 0U;

/*
 * 首次同步标记：
 * - 0：尚未把“当前关节角”通过 FK 同步为“足端目标”；
 * - 1：已同步。
 *
 * 作用：IK 刚开启时先做一次同步，避免第一帧发生姿态跳变。
 */
static uint8_t g_foot_target_synced_from_angle = 0U;

/*
 * 函数：gait_wrap_to_pi
 * 作用：把任意角度包裹到 [-pi, pi]。
 * 用途：
 * 1) 比较两个角度“距离”时避免 2pi 周期歧义；
 * 2) 输出角度保持统一范围。
 */
static float gait_wrap_to_pi(float angle_rad)
{
	while (angle_rad > GAIT_PI) /* 角度大于 +pi 时，减去 2pi。 */
	{
		angle_rad -= 2.0f * GAIT_PI;
	}
	while (angle_rad < -GAIT_PI) /* 角度小于 -pi 时，加上 2pi。 */
	{
		angle_rad += 2.0f * GAIT_PI;
	}
	return angle_rad; /* 返回包裹后的角度。 */
}

/*
 * 函数：gait_angle_distance
 * 作用：计算两个角度的“最小周期差”的绝对值。
 * 数学意义：|wrap_to_pi(a - b)|。
 */
static float gait_angle_distance(float a_rad, float b_rad)
{
	return fabsf(gait_wrap_to_pi(a_rad - b_rad));
}

/*
 * 函数：gait_leg_side_sign
 * 作用：给出左右腿几何镜像符号。
 * 返回：
 * - 左腿（0,2）返回 +1；
 * - 右腿（1,3）返回 -1。
 *
 * 注意：这里是“几何镜像符号”，不是电机安装方向 sign。
 */
static float gait_leg_side_sign(uint8_t leg_idx)
{
	if ((leg_idx == 0U) || (leg_idx == 2U)) /* 左前或左后腿。 */
	{
		return 1.0f; /* 左腿记为 +1。 */
	}
	return -1.0f; /* 其余（右腿）记为 -1。 */
}

/*
 * 函数：Gait_Init
 * 作用：初始化 gait 模块内部状态。
 * 可以在这里写初始化站姿
 */
void Gait_Init(void)
{
	uint8_t leg_idx;      /* 循环变量：腿编号。 */
	float side;           /* 当前腿的左右镜像符号（+1/-1）。 */
	float default_reach;  /* 默认伸展量（mm）：用于初始化足端目标。 */

	memset(g_foot_target_mm, 0, sizeof(g_foot_target_mm));          /* 清空足端目标缓存。 */
	memset(g_last_ik_angle_rad, 0, sizeof(g_last_ik_angle_rad));    /* 清空上一帧 IK 角度缓存。 */
	memset(g_last_ik_inited, 0, sizeof(g_last_ik_inited));          /* 清空 IK 历史有效标记。 */
	memset(&g_ik_debug, 0, sizeof(g_ik_debug));                     /* 清空 IK 调试缓存。 */

	g_ik_enable = 0U;                          /* 默认关闭 IK，避免影响现有角度控制链路。 */
	g_foot_target_synced_from_angle = 0U;      /* 默认尚未同步。 */

	/*
	 * 计算一组安全默认目标：
	 * 0.70 * (大腿长度 + 小腿长度) 代表一个适中的伸展比例，
	 * 既不会太蜷，也不会接近伸直极限。
	 */
	default_reach = 0.70f * (leg_real_size.thigh_length_mm + leg_real_size.shank_length_mm);

	for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++) /* 遍历 4 条腿逐条初始化目标。 */
	{
		side = gait_leg_side_sign(leg_idx); /* 取当前腿左右符号。 */

		/*
		 * x 方向：以髋偏置为基准，再向前给 40mm。
		 * 含义：让默认点不在关节正下方，避免某些奇异姿态。
		 */
		g_foot_target_mm[leg_idx].x_mm = leg_real_size.hip_offset_x_mm + 40.0f;

		/*
		 * y 方向：按左右腿镜像给出外侧偏置。
		 * side=+1（左腿）时 y 为正；side=-1（右腿）时 y 为负。
		 */
		g_foot_target_mm[leg_idx].y_mm = side * (leg_real_size.hip_offset_y_mm + 10.0f);

		/*
		 * z 方向：向下（负值）给默认伸展量。
		 * 约定里 +Z 向上，因此站立足端通常是负 z。
		 */
		g_foot_target_mm[leg_idx].z_mm = -default_reach;
	}
}

/*
 * 函数：Gait_EnableIkControl
 * 作用：设置 IK 功能开关。
 */
void Gait_EnableIkControl(uint8_t enable)
{
	g_ik_enable = (enable != 0U) ? 1U : 0U; /* 非 0 统一当作开启，0 为关闭。 */

	if (g_ik_enable != 0U) /* 如果本次是开启 IK... */
	{
		g_foot_target_synced_from_angle = 0U; /* ...要求下一次先同步，防止突跳。 */
	}
}

/*
 * 函数：Gait_IsIkControlEnabled
 * 作用：读取 IK 开关。
 */
uint8_t Gait_IsIkControlEnabled(void)
{
	return g_ik_enable; /* 直接返回内部开关值。 */
}

/*
 * 函数：Gait_SetLegFootTargetMm
 * 作用：设置单腿足端目标。
 */
uint8_t Gait_SetLegFootTargetMm(uint8_t leg_idx, float x_mm, float y_mm, float z_mm)
{
	if (leg_idx >= ROBOT_LEG_NUM) /* 腿编号越界检查。 */
	{
		return 1U; /* 失败：腿编号非法。 */
	}

	g_foot_target_mm[leg_idx].x_mm = x_mm; /* 写入目标 X。 */
	g_foot_target_mm[leg_idx].y_mm = y_mm; /* 写入目标 Y。 */
	g_foot_target_mm[leg_idx].z_mm = z_mm; /* 写入目标 Z。 */
	return 0U; /* 成功。 */
}


/*
 * 函数：Gait_LegFK
 * 作用：单腿正运动学（关节角 -> 足端位置）。
 * 模型：简化 3R（hip_roll + hip_pitch + knee_pitch）。
 */
void Gait_LegFK(uint8_t leg_idx,
				const float angle_rad[3],
				GaitFootPosMm *out_foot_pos_mm)
{
	float side;                /* 左右腿镜像符号。 */
	float hip_offset_x;        /* 髋在 X 方向的固定偏置（mm）。 */
	float hip_offset_y_signed; /* 带左右符号的髋 Y 偏置（mm）。 */
	float thigh_len;           /* 大腿等效长度（mm）。 */
	float shank_len;           /* 小腿等效长度（mm）。 */
	float q1;                  /* 关节角1：hip_roll（rad）。 */
	float q2;                  /* 关节角2：hip_pitch（rad）。 */
	float q3;                  /* 关节角3：knee_pitch（rad）。 */
	float x_plane;             /* 去掉 roll 后，2R 平面中的 X 坐标（mm）。 */
	float z_plane;             /* 去掉 roll 后，2R 平面中的 Z 坐标（mm）。 */
	float cos_q1;              /* cos(q1)，提前算减少重复调用。 */
	float sin_q1;              /* sin(q1)，提前算减少重复调用。 */

	if ((angle_rad == NULL) || (out_foot_pos_mm == NULL)) /* 空指针保护。 */
	{
		return; /* 输入无效，直接退出。 */
	}

	if (leg_idx >= ROBOT_LEG_NUM) /* 腿编号越界保护。 */
	{
		return; /* 越界直接退出。 */
	}

	side = gait_leg_side_sign(leg_idx);                    /* 当前腿左右符号。 */
	hip_offset_x = leg_real_size.hip_offset_x_mm;          /* 读取 X 偏置。 */
	hip_offset_y_signed = side * leg_real_size.hip_offset_y_mm; /* 读取带符号 Y 偏置。 */
	thigh_len = leg_real_size.thigh_length_mm;             /* 读取大腿长度。 */
	shank_len = leg_real_size.shank_length_mm;             /* 读取小腿长度。 */

	q1 = angle_rad[0]; /* 提取 hip_roll。 */
	q2 = angle_rad[1]; /* 提取 hip_pitch。 */
	q3 = angle_rad[2]; /* 提取 knee_pitch。 */

	/*
	 * 平面 2R 正运动学：
	 * x_plane = hip_offset_x + L1*cos(q2) + L2*cos(q2+q3)
	 * z_plane =               L1*sin(q2) + L2*sin(q2+q3)
	 *
	 * 含义：先在“去掉 roll 的二维平面”里算出足端点。
	 */
	x_plane = hip_offset_x + (thigh_len * cosf(q2)) + (shank_len * cosf(q2 + q3));
	z_plane = (thigh_len * sinf(q2)) + (shank_len * sinf(q2 + q3));

	/* 预计算 roll 的正余弦。 */
	cos_q1 = cosf(q1);
	sin_q1 = sinf(q1);

	/*
	 * 把平面坐标 [hip_offset_y_signed, z_plane] 绕 X 轴旋转 q1 回到 3D：
	 * y =  cos(q1)*hy - sin(q1)*z_plane
	 * z =  sin(q1)*hy + cos(q1)*z_plane
	 */
	out_foot_pos_mm->x_mm = x_plane;                                                /* 输出 X。 */
	out_foot_pos_mm->y_mm = (cos_q1 * hip_offset_y_signed) - (sin_q1 * z_plane);   /* 输出 Y。 */
	out_foot_pos_mm->z_mm = (sin_q1 * hip_offset_y_signed) + (cos_q1 * z_plane);   /* 输出 Z。 */
}

/*
 * 函数：Gait_LegIK
 * 作用：单腿逆运动学（足端位置 -> 关节角）。
 * 纯解算步骤：
 * 1) 先解 hip_roll（q1），有两支解；
 * 2) 再解平面 2R（q2,q3），也有两支解；
 * 3) 用 seed/历史角选择更接近当前姿态的一支。
 */
GaitIkStatus Gait_LegIK(uint8_t leg_idx,
						const GaitFootPosMm *foot_pos_mm,
						const float seed_angle_rad[3],
						float out_angle_rad[3])
{
	GaitIkDebug *dbg = &g_ik_debug;

	/* 记录本次调用基础信息。 */
	dbg->call_count++;
	dbg->leg_idx = leg_idx;
	dbg->seed_valid = (seed_angle_rad != NULL) ? 1U : 0U;
	dbg->reachable_flag = 1U;
	dbg->ik_fail_reason = GAIT_IK_FAIL_NONE;
	dbg->out_angle_rad[0] = NAN;
	dbg->out_angle_rad[1] = NAN;
	dbg->out_angle_rad[2] = NAN;

	if (foot_pos_mm != NULL)
	{
		dbg->foot_input_mm = *foot_pos_mm;
	}
	else
	{
		dbg->foot_input_mm.x_mm = 0.0f;
		dbg->foot_input_mm.y_mm = 0.0f;
		dbg->foot_input_mm.z_mm = 0.0f;
	}

	if (seed_angle_rad != NULL)
	{
		dbg->seed_angle_rad[0] = seed_angle_rad[0];
		dbg->seed_angle_rad[1] = seed_angle_rad[1];
		dbg->seed_angle_rad[2] = seed_angle_rad[2];
	}
	else
	{
		dbg->seed_angle_rad[0] = 0.0f;
		dbg->seed_angle_rad[1] = 0.0f;
		dbg->seed_angle_rad[2] = 0.0f;
	}

	/* 入参保护：空指针或腿索引越界直接失败。 */
	if ((foot_pos_mm == NULL) || (out_angle_rad == NULL) || (leg_idx >= ROBOT_LEG_NUM))
	{
		dbg->reachable_flag = 0U;
		dbg->ik_fail_reason = GAIT_IK_FAIL_INPUT;
		return GAIT_IK_ERR_NULL;
	}

	/* 读取几何参数。 */
	dbg->side = gait_leg_side_sign(leg_idx);
	dbg->hip_offset_x = leg_real_size.hip_offset_x_mm;
	dbg->hip_offset_y_signed = dbg->side * leg_real_size.hip_offset_y_mm;
	dbg->thigh_len = leg_real_size.thigh_length_mm;
	dbg->shank_len = leg_real_size.shank_length_mm;

	/* 读取目标足端坐标。 */
	dbg->x = foot_pos_mm->x_mm;
	dbg->y = foot_pos_mm->y_mm;
	dbg->z = foot_pos_mm->z_mm;

	/* 计算平面目标。 */
	dbg->x_plane = dbg->x - dbg->hip_offset_x;

	/* 计算 q1 两分支。 */
	dbg->yz_sq = (dbg->y * dbg->y) + (dbg->z * dbg->z);
	dbg->yz_delta_sq = dbg->yz_sq - (dbg->hip_offset_y_signed * dbg->hip_offset_y_signed);

	/*
	 * 数值保护1（sqrt 输入）：
	 * - yz_delta_sq < -eps：目标不可达，直接返回；
	 * - -eps <= yz_delta_sq < 0：按浮点误差处理，钳到 0 再开方。
	 */
	dbg->yz_delta_sq_clamped = dbg->yz_delta_sq;
	if (dbg->yz_delta_sq < -GAIT_IK_EPS)
	{
		dbg->reachable_flag = 0U;
		dbg->ik_fail_reason = GAIT_IK_FAIL_YZ_UNREACHABLE;
		return GAIT_IK_ERR_UNREACHABLE;
	}
	if (dbg->yz_delta_sq_clamped < 0.0f)
	{
		dbg->yz_delta_sq_clamped = 0.0f;
	}
	dbg->yz_rot_abs = sqrtf(dbg->yz_delta_sq_clamped);
	dbg->z_plane_candidate[0] = dbg->yz_rot_abs;
	dbg->z_plane_candidate[1] = -dbg->yz_rot_abs;
	dbg->q1_candidate[0] = atan2f(dbg->z, dbg->y) - atan2f(dbg->z_plane_candidate[0], dbg->hip_offset_y_signed);
	dbg->q1_candidate[1] = atan2f(dbg->z, dbg->y) - atan2f(dbg->z_plane_candidate[1], dbg->hip_offset_y_signed);

	if (seed_angle_rad != NULL)
	{
		dbg->q1_ref = seed_angle_rad[0];
	}
	else if (g_last_ik_inited[leg_idx] != 0U)
	{
		dbg->q1_ref = g_last_ik_angle_rad[leg_idx][0];
	}
	else
	{
		dbg->q1_ref = 0.0f;
	}

	if (gait_angle_distance(dbg->q1_candidate[0], dbg->q1_ref) <= gait_angle_distance(dbg->q1_candidate[1], dbg->q1_ref))
	{
		dbg->q1_pick_idx = 0U;
	}
	else
	{
		dbg->q1_pick_idx = 1U;
	}

	dbg->z_plane = dbg->z_plane_candidate[dbg->q1_pick_idx];

	/* 计算 q2/q3 两分支。 */
	dbg->r2 = (dbg->x_plane * dbg->x_plane) + (dbg->z_plane * dbg->z_plane);
	dbg->cos_q3_raw = (dbg->r2 - (dbg->thigh_len * dbg->thigh_len) - (dbg->shank_len * dbg->shank_len)) /
							 (2.0f * dbg->thigh_len * dbg->shank_len);
	dbg->cos_q3 = dbg->cos_q3_raw;

	/*
	 * 数值保护2（acos 输入）：
	 * - 超过 [-1-eps, 1+eps]：认为不可达，直接失败；
	 * - 轻微越界：钳到 [-1,1] 再 acosf。
	 */
	if ((dbg->cos_q3_raw > (1.0f + GAIT_IK_EPS)) || (dbg->cos_q3_raw < (-1.0f - GAIT_IK_EPS)))
	{
		dbg->reachable_flag = 0U;
		dbg->ik_fail_reason = GAIT_IK_FAIL_COS_UNREACHABLE;
		return GAIT_IK_ERR_UNREACHABLE;
	}

	dbg->cos_q3_clamped = dbg->cos_q3_raw;
	if (dbg->cos_q3_clamped > 1.0f)
	{
		dbg->cos_q3_clamped = 1.0f;
	}
	else if (dbg->cos_q3_clamped < -1.0f)
	{
		dbg->cos_q3_clamped = -1.0f;
	}

	dbg->q3_candidate[0] = acosf(dbg->cos_q3_clamped);
	dbg->q3_candidate[1] = -dbg->q3_candidate[0];

	dbg->q2_candidate[0] = atan2f(dbg->z_plane, dbg->x_plane) - atan2f(dbg->shank_len * sinf(dbg->q3_candidate[0]),
																  dbg->thigh_len + (dbg->shank_len * cosf(dbg->q3_candidate[0])));
	dbg->q2_candidate[1] = atan2f(dbg->z_plane, dbg->x_plane) - atan2f(dbg->shank_len * sinf(dbg->q3_candidate[1]),
																  dbg->thigh_len + (dbg->shank_len * cosf(dbg->q3_candidate[1])));

	if (seed_angle_rad != NULL)
	{
		dbg->q2_ref = seed_angle_rad[1];
		dbg->q3_ref = seed_angle_rad[2];
	}
	else if (g_last_ik_inited[leg_idx] != 0U)
	{
		dbg->q2_ref = g_last_ik_angle_rad[leg_idx][1];
		dbg->q3_ref = g_last_ik_angle_rad[leg_idx][2];
	}
	else
	{
		dbg->q2_ref = 0.0f;
		dbg->q3_ref = -0.5f;
	}

	if ((gait_angle_distance(dbg->q2_candidate[0], dbg->q2_ref) + gait_angle_distance(dbg->q3_candidate[0], dbg->q3_ref)) <=
		(gait_angle_distance(dbg->q2_candidate[1], dbg->q2_ref) + gait_angle_distance(dbg->q3_candidate[1], dbg->q3_ref)))
	{
		dbg->q23_pick_idx = 0U;
	}
	else
	{
		dbg->q23_pick_idx = 1U;
	}

	/* 输出结果角。 */
	out_angle_rad[0] = gait_wrap_to_pi(dbg->q1_candidate[dbg->q1_pick_idx]);
	out_angle_rad[1] = gait_wrap_to_pi(dbg->q2_candidate[dbg->q23_pick_idx]);
	out_angle_rad[2] = gait_wrap_to_pi(dbg->q3_candidate[dbg->q23_pick_idx]);

	dbg->out_angle_rad[0] = out_angle_rad[0];
	dbg->out_angle_rad[1] = out_angle_rad[1];
	dbg->out_angle_rad[2] = out_angle_rad[2];

	/* 数值保护3（最终输出）：任一非有限值则失败，不污染历史角。 */
	if ((!isfinite(out_angle_rad[0])) || (!isfinite(out_angle_rad[1])) || (!isfinite(out_angle_rad[2])))
	{
		dbg->reachable_flag = 0U;
		dbg->ik_fail_reason = GAIT_IK_FAIL_NONFINITE;
		return GAIT_IK_ERR_NUMERIC;
	}

	/* 仅在成功时缓存历史角，供下次分支选择。 */
	g_last_ik_angle_rad[leg_idx][0] = out_angle_rad[0];
	g_last_ik_angle_rad[leg_idx][1] = out_angle_rad[1];
	g_last_ik_angle_rad[leg_idx][2] = out_angle_rad[2];
	g_last_ik_inited[leg_idx] = 1U;

	return GAIT_IK_OK; /* 仅成功路径会走到这里。 */
}

/*
 * 函数：Gait_UpdateTargetAngleFromFootTarget
 * 作用：批量把足端目标反解到 target_angle（4 条腿一起处理）。
 * 读取4个腿的g_foot_target_mm也就是足端目标点，输入当前电机角度，最后要的是target电机角度
 */
uint8_t Gait_UpdateTargetAngleFromFootTarget(float target_angle[ROBOT_LEG_NUM][MOTORS_PER_LEG],
											 float current_angle[ROBOT_LEG_NUM][MOTORS_PER_LEG])
{
	uint8_t leg_idx;         /* 循环腿编号。 */
	uint8_t fail_count = 0U; /* IK 失败腿计数器。 */
	float ik_out[3];         /* 单腿 IK 输出临时缓存。 */
	GaitIkStatus ik_status;  /* 单腿 IK 返回状态。 */

	if (target_angle == NULL) /* 输出数组为空时无法工作。 */
	{
		return ROBOT_LEG_NUM; /* 视为全部失败。 */
	}

	if (g_ik_enable == 0U) /* IK 关闭时... */
	{
		return 0U; /* ...不改任何角度，按“0失败”返回。 */
	}

	/*
	 * IK 首次开启时的防跳变处理：
	 * 把当前角度先 FK 到足端目标缓存，
	 * 这样“当前姿态 == 目标姿态”，第一帧不会突跳。
	 */
	if ((g_foot_target_synced_from_angle == 0U) && (current_angle != NULL))
	{
		for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++)
		{
			Gait_LegFK(leg_idx, current_angle[leg_idx], &g_foot_target_mm[leg_idx]); /* 关节角 -> 足端目标同步。 */

			g_last_ik_angle_rad[leg_idx][0] = current_angle[leg_idx][0]; /* 同步历史 q1。 */
			g_last_ik_angle_rad[leg_idx][1] = current_angle[leg_idx][1]; /* 同步历史 q2。 */
			g_last_ik_angle_rad[leg_idx][2] = current_angle[leg_idx][2]; /* 同步历史 q3。 */
			g_last_ik_inited[leg_idx] = 1U; /* 标记该腿历史有效。 */
		}
		g_foot_target_synced_from_angle = 1U; /* 标记“首次同步已完成”。 */
	}

	/* 逐腿执行 IK，并把成功结果写回 target_angle。 */
	for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++)
	{
		const float *seed = NULL; /* 默认没有 seed。 */

		if (current_angle != NULL) /* 若外部提供当前角度... */
		{
			seed = current_angle[leg_idx]; /* ...该腿就用当前角度当 seed。 */
		}

		ik_status = Gait_LegIK(leg_idx, &g_foot_target_mm[leg_idx], seed, ik_out); /* 调单腿 IK。 */

		if (ik_status == GAIT_IK_OK) /* 该腿 IK 成功。 */
		{
			target_angle[leg_idx][0] = ik_out[0]; /* 写回 q1 目标。 */
			target_angle[leg_idx][1] = ik_out[1]; /* 写回 q2 目标。 */
			target_angle[leg_idx][2] = ik_out[2]; /* 写回 q3 目标。 */
		}
		else
		{
			/* 失败时保持该腿原目标角不变，只累计失败数。 */
			fail_count++;
		}
	}

	return fail_count; /* 返回 4 条腿中失败的数量。 */
}
