#include "rx_callback.h"
#include "robot_map.h"
#include "HT10A.h"
#include "imu.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pc_comm_uart10.h"
/*
 * UART DMA 接收完成回调。
 * 这里只处理 USART1 手柄数据。
 * 电机 485 接收改到 HAL_UART_RxCpltCallback()。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart , uint16_t Size)
{
    UBaseType_t irq_mask;
    if (huart == NULL)
    {
        return;
    }

    if(huart->Instance == USART1) {
        /* ISR 场景下使用 FreeRTOS 中断临界区，保护共享缓冲区访问。 */
        irq_mask = taskENTER_CRITICAL_FROM_ISR();
        Teaching_Pendant_Restart();
        HT10A_process(Teaching_Pendant_buffer);
        taskEXIT_CRITICAL_FROM_ISR(irq_mask);
        return;
    }
    if(huart->Instance == UART8) {
        IMU_RxEvent(Size);
        return;
    }
}

/*
 * 电机 DMA 发送完成回调：
 * 发送完成后立即启动 HAL_UART_Receive_DMA() 接收回包。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return;
    }

    if (huart->Instance == UART9)
    {
        //左前腿三个电机
        MotorBus_Restart(0U);
        return;
    }
    if (huart->Instance == USART3)
    {
        //右前腿三个电机
        MotorBus_Restart(1U);
        return;
    }
    if (huart->Instance == USART2)
    {
        //左后腿三个电机
        MotorBus_Restart(2U);
        return;
    }
    if (huart->Instance == UART7)
    {
        //右后腿三个电机
        MotorBus_Restart(3U);
        return;
    }
    if (huart->Instance == USART10)
    {
        /* PC 通信 UART10 发送完成：释放 busy 标志。 */
        PCComm_OnUart10TxCplt();
        return;
    }
}

/*
 * 电机 DMA 接收完成回调：
 * 这里做电机回包分发（按串口+电机ID）。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return;
    }

    if (huart->Instance == UART9)
    {
        MotorBus_Process(0U, sizeof(RIS_MotorData_t));
        return;
    }
    if (huart->Instance == USART3)
    {
        MotorBus_Process(1U, sizeof(RIS_MotorData_t));
        return;
    }
    if (huart->Instance == USART2)
    {
        MotorBus_Process(2U, sizeof(RIS_MotorData_t));
        return;
    }
    if (huart->Instance == UART7)
    {
        MotorBus_Process(3U, sizeof(RIS_MotorData_t));
        return;
    }
    if (huart->Instance == USART10)
    {
        /* PC 通信 UART10 接收完成：交给解析器并继续接收。 */
        PCComm_OnUart10RxCplt();
        return;
    }
}

/*
 * UART 错误回调。
 * 同样只打错误标志并转发，恢复逻辑由主流程统一处理。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	UBaseType_t irq_mask;
    if (huart == NULL)
    {
        return;
    }

    if(huart->Instance == USART1) {
        /* ISR 场景下使用 FreeRTOS 中断临界区，保护共享缓冲区访问。 */
        irq_mask = taskENTER_CRITICAL_FROM_ISR();
        Teaching_Pendant_Restart();
        HT10A_process(Teaching_Pendant_buffer);
        taskEXIT_CRITICAL_FROM_ISR(irq_mask);
        return;
    }
    if(huart->Instance == UART8) {
        IMU_Restart();
        return;
    }
    if(huart->Instance == UART9) {
        MotorBus_Restart(0U);
        return;
    }
    if(huart->Instance == USART3) {
        MotorBus_Restart(1U);
        return;
    }
    if(huart->Instance == USART2) {
        MotorBus_Restart(2U);
        return;
    }
    if(huart->Instance == UART7) {
        MotorBus_Restart(3U);
        return;
    }
    if(huart->Instance == USART10) {
        /* PC 通信 UART10 异常：重新挂起接收，避免卡死。 */
        PCComm_StartReceive();
        return;
    }
}



