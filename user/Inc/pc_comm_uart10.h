#ifndef PC_COMM_UART10_H
#define PC_COMM_UART10_H

#include <stdint.h>
#include "robot_map.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PC_COMM_STATE_HEAD 0xFEFEU
#define PC_COMM_COMMAND_HEAD 0xA5A5U
#define PC_COMM_STATE_PERIOD_MS 20U
#define PC_COMM_COMMAND_TIMEOUT_MS 100U

#pragma pack(push, 1)
typedef struct
{
    uint16_t head;
    uint32_t tick_ms;
    uint8_t mode;
    uint8_t fault;

    float base_lin_vel[3];
    float base_ang_vel[3];
    float projected_gravity[3];
    float cmd[3];

    float joint_pos[J_NUM];
    float joint_vel[J_NUM];

    float battery_v;
    uint16_t crc;
} RobotStatePacket;

typedef struct
{
    uint16_t head;
    uint32_t tick_ms;
    uint8_t enable;
    uint8_t mode;

    float q_des[J_NUM];

    uint16_t crc;
} JointCommandPacket;
#pragma pack(pop)

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
#define PC_COMM_STATIC_ASSERT(expr, msg) _Static_assert((expr), msg)
#else
#define PC_COMM_STATIC_ASSERT_JOIN_(a, b) a##b
#define PC_COMM_STATIC_ASSERT_JOIN(a, b) PC_COMM_STATIC_ASSERT_JOIN_(a, b)
#define PC_COMM_STATIC_ASSERT(expr, msg) \
    typedef char PC_COMM_STATIC_ASSERT_JOIN(pc_comm_static_assert_, __LINE__)[(expr) ? 1 : -1]
#endif

PC_COMM_STATIC_ASSERT(sizeof(RobotStatePacket) == 158U, "RobotStatePacket size mismatch");
PC_COMM_STATIC_ASSERT(sizeof(JointCommandPacket) == 58U, "JointCommandPacket size mismatch");

uint16_t PCComm_CRC16_CCITT_FALSE(const uint8_t *data, uint16_t len);

void PCComm_Init(void);
void PCComm_StartReceive(void);
void PCComm_Task1ms(void);
void PCComm_SendState20ms(void);

uint8_t PCComm_IsCommandFresh(void);
uint32_t PCComm_GetLastCommandAgeMs(void);
uint8_t PCComm_GetEnable(void);
uint8_t PCComm_GetMode(void);
uint8_t PCComm_GetFault(void);
uint8_t PCComm_IsPolicyControlAllowed(void);
void PCComm_GetQDesUrdf(float q_des_out[J_NUM]);

void PCComm_OnUart10RxByte(uint8_t byte);
void PCComm_OnUart10RxCplt(void);
void PCComm_OnUart10TxCplt(void);
void PCComm_EStop(void);

#ifdef __cplusplus
}
#endif

#endif
