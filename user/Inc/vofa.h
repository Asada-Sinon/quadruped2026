#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 最多发送多少个通道
 * 例如 4 表示一次最多发 4 个 float
 * 你后面想发 12 路电机数据，就改成 12 或更大
 * 小端序
 */
#ifndef VOFA_JF_MAX_CH
#define VOFA_JF_MAX_CH  16U
#endif

typedef struct
{
    UART_HandleTypeDef *huart;                         // 绑定的串口句柄，这里会绑定到 huart6
    uint8_t tx_buf[VOFA_JF_MAX_CH * 4U + 4U];         // 发送缓冲区：float数据 + 4字节帧尾
} VOFA_JF_DMA_HandleTypeDef;

/* 初始化 */
void VOFA_JF_DMA_Init(VOFA_JF_DMA_HandleTypeDef *hvofa, UART_HandleTypeDef *huart);

/* DMA发送一帧JustFloat数据
 * data  : float数组首地址
 * count : float通道数
 */
HAL_StatusTypeDef VOFA_JF_DMA_Send(VOFA_JF_DMA_HandleTypeDef *hvofa,
                                   const float *data,
                                   uint16_t count);

extern VOFA_JF_DMA_HandleTypeDef hvofa;   // 这个要定义成全局变量，方便外部调用
#ifdef __cplusplus
}
#endif




#endif /* __VOFA_H__ */


