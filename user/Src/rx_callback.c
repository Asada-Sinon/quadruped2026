#include "rx_callback.h"
#include "robot_map.h"
#include "HT10A.h"
#include "FreeRTOS.h"
#include "task.h"
/*
 * UART DMA 接收完成回调。
 * 这里只做事件转发，具体数据处理在 MotorBus_Task1ms() 主流程完成，
 * 避免在中断里做耗时操作。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart , uint16_t Size)
{
    UBaseType_t irq_mask;
    (void)Size;
    if(huart->Instance == USART1) {
        /* ISR 场景下使用 FreeRTOS 中断临界区，保护共享缓冲区访问。 */
        irq_mask = taskENTER_CRITICAL_FROM_ISR();
        Teaching_Pendant_Restart();
        HT10A_process(Teaching_Pendant_buffer);
        taskEXIT_CRITICAL_FROM_ISR(irq_mask);
    }
}

/*
 * UART 错误回调。
 * 同样只打错误标志并转发，恢复逻辑由主流程统一处理。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	UBaseType_t irq_mask;
    if(huart->Instance == USART1) {
        /* ISR 场景下使用 FreeRTOS 中断临界区，保护共享缓冲区访问。 */
        irq_mask = taskENTER_CRITICAL_FROM_ISR();
        Teaching_Pendant_Restart();
        HT10A_process(Teaching_Pendant_buffer);
        taskEXIT_CRITICAL_FROM_ISR(irq_mask);
    }
}



