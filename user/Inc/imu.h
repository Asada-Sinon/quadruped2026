#ifndef __IMU_H__
#define __IMU_H__
#include "stdint.h"

typedef struct
{
    /* 加速度计三轴数据，通常按 [0]=X, [1]=Y, [2]=Z（单位取决于传感器配置，单位是g，也就是9.81 m/s^2）。 */
    float acc[3];
    /* 陀螺仪三轴角速度，通常按 [0]=X, [1]=Y, [2]=Z（单位取决于配置，deg/s）。 */
    float gyro[3];
    /* 姿态欧拉角三轴，通常按 [0]=Roll, [1]=Pitch, [2]=Yaw（单位取决于配置，deg）。 */
    float angle[3];
    /* 姿态四元数，4 个分量的顺序以传感器协议为准（常见为 [w, x, y, z]）。 */
    float quaternion[4];
    /* 四元数标量分量的原始值（常用作 w 分量的未处理/未归一化值，具体以传感器协议为准）。 */
    float w_original;
} IMU;

/* 机器人机身坐标系姿态数据，轴向与机身坐标系保持一致。 */
typedef struct
{
    /*
     * 机身坐标系下线加速度：
     * [0] = body X，前
     * [1] = body Y，左
     * [2] = body Z，上
     * 单位沿用当前 imu_recv.acc[]，目前是 G。
     */
    float acc[3];
    /*
     * 机身坐标系下角速度：
     * [0] = roll rate，绕机身 X
     * [1] = pitch rate，绕机身 Y
     * [2] = yaw rate，绕机身 Z
     * 单位沿用当前 imu_recv.gyro[]，目前是 deg/s。
     */
    float gyro[3];
    /*
     * 机身坐标系下欧拉角：
     * [0] = Roll
     * [1] = Pitch
     * [2] = Yaw
     * 单位沿用当前 imu_recv.angle[]，目前是 deg。
     */
    float angle[3];
} IMU_Body;

__packed typedef  struct
{
        uint8_t     tag;                /* 0x91 */
        uint8_t     id;
        uint8_t     rev[6];             /* reserved */
        uint32_t    ts;                 /* timestamp */
        float       acc[3];
        float       gyr[3];
        float       mag[3];
        float       eul[3];             /* eular angles:R/P/Y */
        float       quat[4];            /* quaternion */

} id0x91_t;

void imu_data_process(uint8_t *receive);

IMU* imu_get_data(void);
IMU_Body* imu_get_body_data(void);

/* UART8 DMA receive helpers. */
void IMU_Restart(void);
void IMU_RxEvent(uint16_t size);
#endif

