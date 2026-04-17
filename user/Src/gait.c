#include "gait.h"
#include "robot_map.h"
#include <math.h>
#include <string.h>
//这正逆运动学里面的角度都是弧度，所有的角度都是关节角度，也就是输出侧的角度
#define GAIT_NUMERIC_EPS 1.0e-6f
#define GAIT_IK_DOMAIN_EPS 1.0e-4f
#define PI 3.14159265358979323846f
/* 4 条腿的足端目标缓存（髋关节局部坐标系，单位 m）。 */
GaitFootPosM g_foot_target_m[ROBOT_LEG_NUM] = {0};
/* 4 条腿当前足端位置缓存（髋关节局部坐标系，单位 m）。 */
GaitFootPosM g_foot_current_m[ROBOT_LEG_NUM] = {0};
//x狗头方向，y左，z上

// 逆运动学，输入足端位置，输出关节角
void gait_inverse_kinematics_core(uint8_t leg_id, const float foot_pos[3], float joint_pos[3])
{
	float px = foot_pos[X_IDX];
	float py = foot_pos[Y_IDX];
	float pz = foot_pos[Z_IDX];
	int y_sign = (leg_id == LEG_FL || leg_id == LEG_HL) ? 1 : -1;
	float l1 = y_sign * leg_real_size.hip_length; // 髋关节偏移（考虑左右方向）
	float l2 = -leg_real_size.thigh_length;		  // 大腿长度
	float l3 = -leg_real_size.calf_length;		  // 小腿长度
	/*计算髋关节角度*/
	// 大腿连杆加小腿连杆在髋腿坐标系下yz平面的投影长度
	float L1 = sqrtf(py * py + pz * pz - l1 * l1);
	// 髋关节角度 - 使用atan2确保正确的象限
	float hip_angle = atan2f(pz * l1 + py * L1, py * l1 - pz * L1);

	/*计算小腿关节角度*/
	// 计算足端与大腿关节的距离
	float L2 = sqrtf(px * px + py * py + pz * pz - l1 * l1);
	// 应用余弦定理，确保参数在arccos的定义域[-1,1]内
	float temp = (l2 * l2 + l3 * l3 - L2 * L2) / (2 * fabs(l2 * l3));
	if (temp > 1)
		temp = 1;
	if (temp < -1)
		temp = -1;
	// 小腿关节角度 - 首先计算为[0,π]范围，然后转换到关节约束范围[-π,0]
	float calf_angle = acosf(temp);	 //[0, PI]
	calf_angle = -(PI - calf_angle); //[-PI, 0]

	/*计算大腿关节角度*/
	// 通过几何关系计算参数
	float a1 = py * sin(hip_angle) - pz * cos(hip_angle);
	float a2 = px;
	float m1 = l3 * sin(calf_angle);
	float m2 = l2 + l3 * cos(calf_angle);
	// 大腿关节角度 - 使用atan2确保正确的象限
	float thigh_angle = atan2f(m1 * a1 + m2 * a2, m1 * a2 - m2 * a1);

	// 存储计算结果
	joint_pos[HIP_IDX] = hip_angle;
	joint_pos[THIGH_IDX] = thigh_angle;
	joint_pos[CALF_IDX] = calf_angle;
}
// 正运动学，输入关节角，输出足端位置
void leg_forward_kinematics(uint8_t leg_id, const float joint_pos[3], float foot_pos[3])
{
	int y_sign = (leg_id == LEG_FL || leg_id == LEG_HL) ? 1 : -1;
	float l1 = y_sign * leg_real_size.hip_length; // 髋关节偏移（考虑左右方向）
	float l2 = -leg_real_size.thigh_length;		  // 大腿长度
	float l3 = -leg_real_size.calf_length;		  // 小腿长度

	// 计算三角函数值

	float s1 = sinf(joint_pos[HIP_IDX]);
	float s2 = sinf(joint_pos[THIGH_IDX]);
	float s3 = sinf(joint_pos[CALF_IDX]);

	float c1 = cosf(joint_pos[HIP_IDX]);
	float c2 = cosf(joint_pos[THIGH_IDX]);
	float c3 = cosf(joint_pos[CALF_IDX]);

	// 计算复合角的三角函数值
	float c23 = c2 * c3 - s2 * s3;
	float s23 = s2 * c3 + c2 * s3;

	// 应用运动学正解方程
	foot_pos[X_IDX] = l3 * s23 + l2 * s2;
	foot_pos[Y_IDX] = -l3 * s1 * c23 + l1 * c1 - l2 * c2 * s1;
	foot_pos[Z_IDX] = l3 * c1 * c23 + l1 * s1 + l2 * c1 * c2;
}
// 雅可比矩阵计算：输入关节角，输出雅可比矩阵（3x3）
void leg_jacobian(uint8_t leg_id, const float joint_pos[3], float jacobian[3][3])
{
	int y_sign = (leg_id == LEG_FL || leg_id == LEG_HL) ? 1 : -1;
	float l1 = y_sign * leg_real_size.hip_length; // 髋关节偏移（考虑左右方向）
	float l2 = -leg_real_size.thigh_length;		  // 大腿长度
	float l3 = -leg_real_size.calf_length;		  // 小腿长度

	float s1 = sinf(joint_pos[HIP_IDX]);
	float s2 = sinf(joint_pos[THIGH_IDX]);
	float s3 = sinf(joint_pos[CALF_IDX]);

	float c1 = cosf(joint_pos[HIP_IDX]);
	float c2 = cosf(joint_pos[THIGH_IDX]);
	float c3 = cosf(joint_pos[CALF_IDX]);

	float c23 = c2 * c3 - s2 * s3;
	float s23 = s2 * c3 + c2 * s3;
	jacobian[0][0] = 0;
	jacobian[1][0] = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1;
	jacobian[2][0] = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1;
	jacobian[0][1] = l3 * c23 + l2 * c2;
	jacobian[1][1] = l3 * s1 * s23 + l2 * s1 * s2;
	jacobian[2][1] = -l3 * c1 * s23 - l2 * c1 * s2;
	jacobian[0][2] = l3 * c23;
	jacobian[1][2] = l3 * s1 * s23;
	jacobian[2][2] = -l3 * c1 * s23;
}
// 雅可比建立速度关系
void leg_forward_kinematics_vel(uint8_t leg_id, const float joint_pos[3], const float joint_vel[3], float foot_vel[3])
{
	float jacobian[3][3];
	leg_jacobian(leg_id, joint_pos, jacobian);
	for (int i = 0U; i < 3U; i++)
	{
		foot_vel[i] = 0.0f;
		for (int j = 0U; j < 3U; j++)
		{
			foot_vel[i] += jacobian[i][j] * joint_vel[j];
		}
	}
}
// 给单腿发送足端位置
uint8_t Gait_SetLegFootTargetM(uint8_t leg_idx, float x_m, float y_m, float z_m)
{
	if (leg_idx >= ROBOT_LEG_NUM)
	{
		return 1U;
	}

	g_foot_target_m[leg_idx].x_m = x_m;
	g_foot_target_m[leg_idx].y_m = y_m;
	g_foot_target_m[leg_idx].z_m = z_m;
	return 0U;
}
//四个腿根据设置的足端位置设置关节位置
void Gait_UpdateTargetAngleFromFootTarget(float target_angle[ROBOT_LEG_NUM][MOTORS_PER_LEG])
{
	uint8_t leg_idx;
	for (leg_idx = 0U; leg_idx < ROBOT_LEG_NUM; leg_idx++)
	{
		float foot_pos[3];
		float joint_pos[3];
		foot_pos[X_IDX] = g_foot_target_m[leg_idx].x_m;
		foot_pos[Y_IDX] = g_foot_target_m[leg_idx].y_m;
		foot_pos[Z_IDX] = g_foot_target_m[leg_idx].z_m;

		gait_inverse_kinematics_core(leg_idx, foot_pos, joint_pos);
		target_angle[leg_idx][HIP_IDX] = joint_pos[HIP_IDX];
		target_angle[leg_idx][THIGH_IDX] = joint_pos[THIGH_IDX];
		target_angle[leg_idx][CALF_IDX] = joint_pos[CALF_IDX];
	}
}

