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
   * vTaskDelayUntil 鐢ㄤ簬鈥滃浐瀹氬懆鏈熲?濅换鍔★細
   * - xLastWakeTime锛氳?板綍涓婃?″敜閱掓椂鍒?
   * - xPeriodTicks锛氫换鍔″懆鏈燂紙杩欓噷鏄? 1ms锛?
   */
  //TickType_t xLastWakeTime;
  //const TickType_t xPeriodTicks = pdMS_TO_TICKS(1U);

  /* 鍙傛暟鏈?浣跨敤锛屾樉寮忔秷闄ゅ憡璀︺?? */
  //(void)argument;

  /* 鍚?鍔ㄥ墠鍏堟姄鍙栧綋鍓? tick 浣滀负鍛ㄦ湡鍩哄噯銆? */
  //xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for (;;)
  {
    // 瓒崇??鍗曚綅鏄痬
    App_Robot_Loop1ms();
    /* 鍥哄畾 1ms 鍛ㄦ湡杩愯?岋紝??屼笉鏄???滀粠褰撳墠鏃跺埢鍐嶅欢鏃? 1ms鈥濄?? */
    //vTaskDelayUntil(&xLastWakeTime, xPeriodTicks);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
