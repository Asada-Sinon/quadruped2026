/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_robot.h"
#include "M8010.h"
#include "usart.h"
#include "vofa.h"
#include "gait.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
osThreadId myTask05Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// int test = 0;
// float test_angle[3] = {-5.15, -0.61, 5.47};
/* USER CODE END FunctionPrototypes */

void motor(void const * argument);
void vofa(void const * argument);
void calculate(void const * argument);
void motorsend(void const * argument);
void pc(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, motor, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, vofa, osPriorityIdle, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, calculate, osPriorityIdle, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, motorsend, osPriorityIdle, 0, 128);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* definition and creation of myTask05 */
  osThreadDef(myTask05, pc, osPriorityIdle, 0, 128);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_motor */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_motor */
void motor(void const * argument)
{
  /* USER CODE BEGIN motor */
  /*
   * motor 是 1ms 控制任务，只做 PC 通信状态机、状态包节拍、状态机、
   * 轨迹/IK/目标角计算和快照发布，不直接调用 send_data_all()。
   * 当前 FreeRTOSConfig.h 里 INCLUDE_vTaskDelayUntil=0，因此这里先保留
   * CMSIS 的 osDelay(1)；如果后续在 CubeMX 里启用 vTaskDelayUntil，
   * 可以再切到固定唤醒相位的 vTaskDelayUntil()。
   */
  (void)argument;

  /* Infinite loop */
  for (;;)
  {
    App_Robot_Loop1ms();
    osDelay(1);
  }
  /* USER CODE END motor */
}

/* USER CODE BEGIN Header_vofa */
/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vofa */
void vofa(void const * argument)
{
  /* USER CODE BEGIN vofa */
  /* Infinite loop */
  for (;;)
  {
    // App_Robot_Send_Loop();
    App_vofa_Send();
    osDelay(2);
  }
  /* USER CODE END vofa */
}

/* USER CODE BEGIN Header_calculate */
/**
 * @brief Function implementing the myTask03 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_calculate */
void calculate(void const * argument)
{
  /* USER CODE BEGIN calculate */
  /* Infinite loop */
  for (;;)
  {
    //不断通过正运动学解出来当前足端位置
    App_UpdateCurrentFootPosFromMotor(legs);
    osDelay(1);
  }
  /* USER CODE END calculate */
}

/* USER CODE BEGIN Header_motorsend */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motorsend */
void motorsend(void const * argument)
{
  /* USER CODE BEGIN motorsend */
  /*
   * motorsend 是唯一允许进入 App_Robot_MotorSendLoop()/send_data_all() 的任务。
   * 发送函数会等待 12 个主电机和额外 ID13 的回包，电机不上电时可能阻塞约 65ms，
   * 所以本任务周期先设为 5ms，且优先级必须低于 motor 控制任务，不能抢占 1ms 控制链路。
   */
  (void)argument;

  /* Infinite loop */
  for(;;)
  {
    App_Robot_MotorSendLoop();
    osDelay(5);
  }
  /* USER CODE END motorsend */
}

/* USER CODE BEGIN Header_pc */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pc */
void pc(void const * argument)
{
  /* USER CODE BEGIN pc */
  /*
   * PCComm_Task1ms() 和 PCComm_SendState20ms() 已经在 App_Robot_Loop1ms() 中执行。
   * 这个低优先级 pc 任务暂不重复发送 UART10 状态包，避免 g_tx_busy 状态被多任务交叉触发。
   */
  (void)argument;

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END pc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
