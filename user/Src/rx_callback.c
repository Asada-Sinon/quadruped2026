#include "rx_callback.h"
#include "robot_map.h"

/*
 * UART DMA 接收完成回调。
 * 这里只做事件转发，具体数据处理在 MotorBus_Task1ms() 主流程完成，
 * 避免在中断里做耗时操作。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart , uint16_t Size)
{

}

/*
 * UART 错误回调。
 * 同样只打错误标志并转发，恢复逻辑由主流程统一处理。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{

}



