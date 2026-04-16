#ifndef __RX_CALLBACK_H
#define __RX_CALLBACK_H

/*
 * HAL UART 回调转发模块
 *
 * 说明：
 * - HAL 库会在中断中调用 HAL_UART_RxCpltCallback / HAL_UART_ErrorCallback
 * - 本工程在 rx_callback.c 中重写这两个弱函数，把事件转发给 MotorBus
 */

#include "usart.h"

#endif


