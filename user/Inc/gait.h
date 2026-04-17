#if !defined(__GAIT_H)
#define __GAIT_H

#include "robot_map.h"
#include <stdint.h>

#define GAIT_PI 3.14159265358979323846f

/* 腿编号约定：与 legs[] / Target_Angle[] 的下标保持一致。 */
typedef enum
{
	LEG_FL = 0U,
	LEG_FR = 1U,
	LEG_HL = 2U,
	LEG_HR = 3U
} GaitLegId;

/* 三关节索引：髋/大腿/小腿。 */
enum
{
	HIP_IDX = 0,
	THIGH_IDX = 1,
	CALF_IDX = 2
};

/* 三维坐标索引：x/y/z。 */
enum
{
	X_IDX = 0,
	Y_IDX = 1,
	Z_IDX = 2
};

/* 足端目标点（髋关节局部坐标系，单位 m）。 */
typedef struct
{
	float x_m;
	float y_m;
	float z_m;
} GaitFootPosM;

/* 4 条腿的足端目标缓存（单位 m）。 */
extern GaitFootPosM g_foot_target_m[ROBOT_LEG_NUM];
/* 4 条腿当前足端位置缓存（髋关节局部坐标系，单位 m）。 */
extern GaitFootPosM g_foot_current_m[ROBOT_LEG_NUM];
void gait_inverse_kinematics_core(uint8_t leg_id, const float foot_pos[3], float joint_pos[3]);
void leg_forward_kinematics(uint8_t leg_id, const float joint_pos[3], float foot_pos[3]);
void leg_jacobian(uint8_t leg_id, const float joint_pos[3], float jacobian[3][3]);
void leg_forward_kinematics_vel(uint8_t leg_id, const float joint_pos[3], const float joint_vel[3], float foot_vel[3]);
uint8_t Gait_SetLegFootTargetM(uint8_t leg_idx, float x_m, float y_m, float z_m);
void Gait_UpdateTargetAngleFromFootTarget(float target_angle[ROBOT_LEG_NUM][MOTORS_PER_LEG]);

#endif
