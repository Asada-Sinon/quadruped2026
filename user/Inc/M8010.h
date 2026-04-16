#ifndef __CRC_CCITT_H
#define __CRC_CCITT_H

#include <stdint.h>
#include <stdlib.h>

typedef struct Leg Leg;
/*
 * M8010 协议编解码模块
 *
 * 这个文件定义了：
 * 1) CRC-CCITT 校验函数；
 * 2) 电机收发数据帧格式（打包为 1 字节对齐）；
 * 3) 业务浮点量 <-> 总线定点量 的转换接口。
 *
 * 数据流：
 * MotorCmd_t(浮点目标) --modify_data--> RIS_ControlData_t(可发送字节流)
 * RIS_MotorData_t(接收字节流) --extract_data--> MotorData_t(浮点反馈)
 */

uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c);

/**
 *	crc_ccitt - recompute the CRC (CRC-CCITT variant) for the data
 *	buffer
 *	@crc: previous CRC value
 *	@buffer: data pointer
 *	@len: number of bytes in the buffer
 */
uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, size_t len);

/*
 * 按协议要求进行紧凑打包。
 * 注意：通信结构体字段顺序不能随意改动，否则会导致协议不兼容。
 */
#pragma pack(1)
/**
 * @brief 电机模式控制信息
 *
 */
typedef struct
{
    uint8_t id : 4;      // 电机ID: 0,1...,13,14 15表示向所有电机广播数据(此时无返回)
    uint8_t status : 3;  // 工作模式: 0.锁定 1.FOC闭环 2.编码器校准 3.保留
    uint8_t reserve : 1; // 保留位
} RIS_Mode_t;            // 控制模式 1Byte

/**
 * @brief 电机状态控制信息
 *
 */
typedef struct
{
    int16_t tor_des; // 期望关节输出扭矩 unit: N.m      (q8)
    int16_t spd_des; // 期望关节输出速度 unit: rad/s    (q8)
    int32_t pos_des; // 期望关节输出位置 unit: rad      (q15)
    int16_t k_pos;   // 期望关节刚度系数 unit: -1.0-1.0 (q15)
    int16_t k_spd;   // 期望关节阻尼系数 unit: -1.0-1.0 (q15)

} RIS_Comd_t; // 控制参数 12Byte

/**
 * @brief 电机状态反馈信息
 *
 */
typedef struct
{
    int16_t torque;      // 实际关节输出扭矩 unit: N.m     (q8)
    int16_t speed;       // 实际关节输出速度 unit: rad/s   (q8)
    int32_t pos;         // 实际关节输出位置 unit: rad     (q15)
    int8_t temp;         // 电机温度: -128~127°C
    uint8_t MError : 3;  // 电机错误标识: 0.正常 1.过热 2.过流 3.过压 4.编码器故障 5-7.保留
    uint16_t force : 12; // 足端气压传感器数据 12bit (0-4095)
    uint8_t none : 1;    // 保留位
} RIS_Fbk_t;             // 状态数据 11Byte

/**
 * @brief 控制数据包格式
 *
 */
typedef struct
{
    uint8_t head[2]; // 包头         2Byte
    RIS_Mode_t mode; // 电机控制模式  1Byte
    RIS_Comd_t comd; // 电机期望数据 12Byte
    uint16_t CRC16;  // CRC          2Byte

} RIS_ControlData_t; // 主机控制命令     17Byte

/**
 * @brief 电机反馈数据包格式
 *
 */
typedef struct
{
    uint8_t head[2]; // 包头         2Byte
    RIS_Mode_t mode; // 电机控制模式  1Byte
    RIS_Fbk_t fbk;   // 电机反馈数据 11Byte
    uint16_t CRC16;  // CRC          2Byte

} RIS_MotorData_t; // 电机返回数据     16Byte

#pragma pack()

/// @brief 电机指令结构体
typedef struct
{
    unsigned short id;   // 电机ID，15代表广播数据包
    unsigned short mode; // 0:空闲 1:FOC控制 2:电机标定
    float T;             // 期望关节的输出力矩(电机本身的力矩)(Nm)
    float W;             // 期望关节速度(电机本身的速度)(rad/s)
    float Pos;           // 期望关节位置(rad)
    float K_P;           // 关节刚度系数(0-25.599)
    float K_W;           // 关节速度系数(0-25.599)

    RIS_ControlData_t motor_send_data;

} MotorCmd_t;

/// @brief 电机反馈结构体
typedef struct
{
    unsigned char motor_id; // 电机ID
    unsigned char mode;     // 0:空闲 1:FOC控制 2:电机标定
    int Temp;               // 温度
    int MError;             // 错误码
    float T;                // 当前实际电机输出力矩(电机本身的力矩)(Nm)
    float W;                // 当前实际电机速度(电机本身的速度)(rad/s)
    float Pos;              // 当前电机位置(rad)，原始回传角度
    float PosRel;           // 首次有效回传置零后的相对位置(rad)
    float PosZero;          // 首次有效回传角度，作为零位基准
    unsigned char PosZeroInited; // 零位基准是否已锁定
    int correct;            // 接收数据是否完整(1完整，0不完整)
    int footForce;          // 足端力传感器原始数值

    uint16_t calc_crc;
    uint32_t timeout;                // 通讯超时 数量
    uint32_t bad_msg;                // CRC校验错误 数量
    RIS_MotorData_t motor_recv_data; // 电机接收数据结构体

} MotorData_t;

typedef struct
{
    MotorData_t motor_r;
    MotorCmd_t motor_s;
    int sign; // 1或-1，表示电机安装方向对控制量的影响
} M8010;

/*
 * 将上层使用的浮点控制量打包为协议帧：
 * - 自动做范围裁剪，防止越界
 * - 自动计算并写入 CRC16
 */
void modify_data(MotorCmd_t *motor_s);

/*
 * 将原始反馈帧解包为浮点量：
 * - 校验包头和 CRC
 * - 校验失败时更新错误计数并标记 correct=0
 */
void extract_data(MotorData_t *motor_r);
void cmd_init(void);
void cmd_single_test_init(void);
void send_data_all(Leg *leg);

/*
 * 按线性编号(0~11)更新某个电机的位置指令：
 * 1) 写入 cmd[cmd_idx].Pos
 * 2) 立即调用 modify_data 重新打包发送帧
 * 这样 send_data_all 下一次发送时就会用最新位置。
 */
void set_cmd_pos_by_index(uint8_t cmd_idx, float pos);


#endif
