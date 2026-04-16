#if !defined(__GAIT_H) /* 头文件防重复包含：如果 __GAIT_H 未定义，则继续编译本文件。 */
#define __GAIT_H        /* 定义宏 __GAIT_H，防止同一个头文件被重复包含。 */

#include "robot_map.h" /* 引入机器人基础映射定义（腿数量、关节数量、几何参数等）。 */

/*
 * 结构体：GaitFootPosMm
 * 用途：表示“单条腿足端点”的三维位置。
 * 单位：全部使用 mm（毫米）。
 * 坐标系：
 * - 原点：该腿 motor1 轴心（髋关节局部坐标）。
 * - +X：机器人前方。
 * - +Y：机器人左侧。
 * - +Z：机器人上方。
 */
typedef struct
{
	float x_mm; /* 足端在 X 方向的位置（毫米）。 */
	float y_mm; /* 足端在 Y 方向的位置（毫米）。 */
	float z_mm; /* 足端在 Z 方向的位置（毫米）。 */
} GaitFootPosMm;

/*
 * 结构体：GaitIkDebug
 * 用途：保存一次 IK 解算过程的全部中间变量，便于在线调试 NaN 来源。
 */
typedef struct
{
	uint32_t call_count;    /* IK 被调用次数（每次调用 +1）。 */
	uint8_t leg_idx;        /* 本次解算腿编号。 */
	uint8_t seed_valid;     /* 本次是否传入 seed。 */

	GaitFootPosMm foot_input_mm; /* 输入足端目标。 */
	float seed_angle_rad[3];     /* 输入 seed（无 seed 时为 0）。 */

	float side;                /* 左右腿镜像符号。 */
	float hip_offset_x;        /* 髋 X 偏置。 */
	float hip_offset_y_signed; /* 带符号髋 Y 偏置。 */
	float thigh_len;           /* 大腿长度。 */
	float shank_len;           /* 小腿长度。 */

	float x;       /* 输入 x。 */
	float y;       /* 输入 y。 */
	float z;       /* 输入 z。 */
	float x_plane; /* 去髋偏置后平面 x。 */

	float yz_sq;       /* y^2 + z^2。 */
	float yz_delta_sq; /* y^2 + z^2 - hy^2。 */
	float yz_delta_sq_clamped; /* yz_delta_sq 经过数值保护后的值。 */
	float yz_rot_abs;  /* z_plane 的绝对值。 */

	float z_plane_candidate[2]; /* z_plane 两个分支。 */
	float q1_candidate[2];      /* q1 两个分支。 */
	float q1_ref;               /* q1 参考值。 */
	uint8_t q1_pick_idx;        /* q1 选中分支。 */

	float z_plane;  /* 选中后的 z_plane。 */
	float r2;       /* 平面半径平方。 */
	float cos_q3_raw; /* q3 余弦原始公式结果。 */
	float cos_q3;   /* q3 余弦（兼容字段，保持与 raw 同步）。 */
	float cos_q3_clamped; /* q3 余弦（钳位到 [-1,1]，实际用于 acosf）。 */
	float q3_candidate[2]; /* q3 两个分支。 */
	float q2_candidate[2]; /* q2 两个分支。 */
	float q2_ref;          /* q2 参考值。 */
	float q3_ref;          /* q3 参考值。 */
	uint8_t q23_pick_idx;  /* q2/q3 选中分支。 */
	uint8_t reachable_flag; /* 1=当前目标可达；0=不可达或数值异常。 */
	uint8_t ik_fail_reason; /* 失败原因：0无失败，1空指针/越界，2YZ不可达，3cos_q3不可达，4数值异常。 */

	float out_angle_rad[3]; /* 最终输出角。 */
} GaitIkDebug;

/*
 * 枚举：GaitIkStatus
 * 用途：描述逆运动学（IK）求解后的状态。
 */
typedef enum
{
	GAIT_IK_OK = 0,                 /* IK 求解成功，out_angle_rad 中结果有效。 */
	GAIT_IK_ERR_NULL = 1,           /* 空指针或索引越界等输入错误。 */
	GAIT_IK_ERR_UNREACHABLE = 2,    /* 目标超出可达空间。 */
	GAIT_IK_ERR_NUMERIC = 3,        /* 数值异常（非有限值）。 */

	/* 兼容旧命名，避免影响既有调用代码。 */
	GAIT_IK_INVALID_PARAM = GAIT_IK_ERR_NULL,
	GAIT_IK_UNREACHABLE = GAIT_IK_ERR_UNREACHABLE
} GaitIkStatus;

/*
 * 全局足端目标缓存（单位 mm）。
 * 下标与腿编号一致：0 左前，1 右前，2 左后，3 右后。
 * 坐标系：髋关节局部坐标（原点 motor1 轴心，+X前，+Y左，+Z上）。
 */
extern GaitFootPosMm g_foot_target_mm[ROBOT_LEG_NUM];

/* 全局 IK 调试缓存：保存最近一次 IK 调用的完整中间变量。 */
extern GaitIkDebug g_ik_debug;

/*
 * 函数：Gait_Init
 * 作用：初始化 gait 模块内部状态。
 * 具体做的事：
 * 1) 清空足端目标缓存；
 * 2) 清空上次 IK 解缓存；
 * 3) 关闭 IK 开关；
 * 4) 写入一组安全的默认足端目标（便于后续调试）。
 */
void Gait_Init(void);

/*
 * 函数：Gait_EnableIkControl
 * 作用：打开/关闭“足端目标 -> 关节角”的自动 IK 更新。
 * 参数：
 * - enable：
 *   - 0 表示关闭 IK（沿用外部直接给 Target_Angle 的模式）；
 *   - 非 0 表示开启 IK（由足端目标反解关节角）。
 */
void Gait_EnableIkControl(uint8_t enable);

/*
 * 函数：Gait_IsIkControlEnabled
 * 作用：读取 IK 控制开关状态。
 * 返回值：
 * - 1：IK 开启；
 * - 0：IK 关闭。
 */
uint8_t Gait_IsIkControlEnabled(void);

/*
 * 函数：Gait_SetLegFootTargetMm
 * 作用：设置某一条腿的足端目标点（单位 mm）。
 * 参数：
 * - leg_idx：腿编号（0~ROBOT_LEG_NUM-1）。
 * - x_mm：目标点 X 坐标（毫米）。
 * - y_mm：目标点 Y 坐标（毫米）。
 * - z_mm：目标点 Z 坐标（毫米）。
 * 返回值：
 * - 0：设置成功；
 * - 1：设置失败（腿编号越界）。
 */
uint8_t Gait_SetLegFootTargetMm(uint8_t leg_idx, float x_mm, float y_mm, float z_mm);


/*
 * 函数：Gait_LegIK
 * 作用：单腿逆运动学，把“足端目标位置”转换成“三关节角”。
 * 参数：
 * - leg_idx：腿编号（0~ROBOT_LEG_NUM-1）。
 * - foot_pos_mm：输入足端目标点（mm，髋关节局部坐标系）。
 * - seed_angle_rad：可选的参考关节角（可传 NULL），
 *   用于在双解情况下选择更接近当前姿态的那一支。
 * - out_angle_rad：输出关节角数组，长度 3，顺序：
 *   [0] hip_roll（髋外展/内收），
 *   [1] hip_pitch（髋俯仰），
 *   [2] knee_pitch（膝俯仰），单位均为 rad。
 * 返回值：
 * - GaitIkStatus：见上方枚举定义。
 */
GaitIkStatus Gait_LegIK(uint8_t leg_idx,
						const GaitFootPosMm *foot_pos_mm,
						const float seed_angle_rad[3],
						float out_angle_rad[3]);

/*
 * 函数：Gait_LegFK
 * 作用：单腿正运动学，把“三关节角”转换为“足端位置”。
 * 参数：
 * - leg_idx：腿编号（0~ROBOT_LEG_NUM-1）。
 * - angle_rad：输入关节角数组，长度 3，顺序：
 *   [0] hip_roll，
 *   [1] hip_pitch，
 *   [2] knee_pitch，单位 rad。
 * - out_foot_pos_mm：输出足端点（mm）。
 */
void Gait_LegFK(uint8_t leg_idx,
				const float angle_rad[3],
				GaitFootPosMm *out_foot_pos_mm);

/*
 * 函数：Gait_UpdateTargetAngleFromFootTarget
 * 作用：批量 IK，把 gait 内部缓存的 4 条腿足端目标反解到 target_angle。
 * 参数：
 * - target_angle：输出/更新目标关节角数组 [ROBOT_LEG_NUM][MOTORS_PER_LEG]，单位 rad。
 * - current_angle：当前关节角参考数组 [ROBOT_LEG_NUM][MOTORS_PER_LEG]（可传 NULL），
 *   用于 IK 分支选择与首次同步。
 * 返回值：
 * - IK 失败的腿数量（0~ROBOT_LEG_NUM）。
 */
uint8_t Gait_UpdateTargetAngleFromFootTarget(float target_angle[ROBOT_LEG_NUM][MOTORS_PER_LEG],
											 float current_angle[ROBOT_LEG_NUM][MOTORS_PER_LEG]);

#endif /* __GAIT_H：头文件结束。 */
