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
//把外设任务都交给freerots
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "button.h"
#include "bluetooth.h"
#include "control.h"
#include "gpio.h"
#include "infrared.h"
#include "manage.h"
#include "oled.h"
#include "ultrasonic.h"
#include <stdio.h>
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
/* 自定义任务句柄：FreeRTOS 用它们管理上层慢任务。 */
osThreadId motionTaskHandle;
osThreadId oledTaskHandle;
osThreadId debugTaskHandle;
osThreadId buttonTaskHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* 20ms 上层运动任务：超声波/红外只生成 speed/direct 目标，不直接写 PWM。 */
void StartMotionTask(void const * argument);
/* 200ms OLED 任务：低优先级刷新状态页，避免影响平衡控制。 */
void StartOledTask(void const * argument);
/* 50ms 地面站任务：通过 printf 输出 $...; 格式波形数据。 */
void StartDebugTask(void const * argument);
/* 10ms 按键任务：处理按键事件并触发机械零点校准。 */
void StartButtonTask(void const * argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
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
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
//默认任务
  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(motionTask, StartMotionTask, osPriorityAboveNormal, 0, 384);
  motionTaskHandle = osThreadCreate(osThread(motionTask), NULL);
//运动模式任务
  osThreadDef(oledTask, StartOledTask, osPriorityLow, 0, 512);
  oledTaskHandle = osThreadCreate(osThread(oledTask), NULL);
//oled显示任务
  osThreadDef(debugTask, StartDebugTask, osPriorityLow, 0, 512);
  debugTaskHandle = osThreadCreate(osThread(debugTask), NULL);
//地面站串口调试任务，优先级低，50ms 一次。
  osThreadDef(buttonTask, StartButtonTask, osPriorityBelowNormal, 0, 128);
  buttonTaskHandle = osThreadCreate(osThread(buttonTask), NULL);
// 按键任务，10ms 一次。
 /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartMotionTask(void const * argument)
	//20ms读取一次超声波
{
  (void)argument;

  for (;;)
  {
    /*
     * 20ms 运动模式任务：
     * 超声波测距、跟随/避障、红外循迹都属于上层运动目标生成。
     * 它们最终只调用 Steer() 设置 speed/direct 目标，不直接控制电机 PWM。
     */
    Read_Distane();

    if (g_CarRunningMode == ULTRA_FOLLOW_MODE)
    {
      UltraControl(0);
    }
    else if (g_CarRunningMode == ULTRA_AVOID_MODE)
    {
      UltraControl(1);
    }
    else if (g_CarRunningMode == INFRARED_TRACE_MODE)
    {
      InfraredTraceControl();
    }
    else if (g_CarRunningMode == CONTROL_MODE)
    {
      /* 手动模式不主动生成运动目标，只做蓝牙超时刹停保护。 */
      Bluetooth_ManualWatchdog();
    }

    osDelay(20);
  }
}

void StartOledTask(void const * argument)
{
  (void)argument;

  for (;;)
  {
    /* OLED 刷新很慢，所以放在低优先级任务，避免阻塞平衡控制。 */
    OLED_ShowHomePage();
    osDelay(200);
  }
}

void StartDebugTask(void const * argument)
{
  (void)argument;

  for (;;)
  {
    /* 地面站调试输出：角度、距离、速度目标、速度环输出、蓝牙和红外状态。 */
    printf("$%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f;",
           g_fCarAngle,
           Distance,
           g_fCarSpeedSet,
           g_fSpeedControlOut,
           (float)g_u8BluetoothLastByte,
           (float)g_u32BluetoothRxCount,
           (float)g_u8InfraredTraceState,
           (float)g_iInfraredTraceError);
    osDelay(50);
  }
}

void StartButtonTask(void const * argument)
{
  (void)argument;

  for (;;)
  {
    if (g_iButtonState == 1)
    {
      /* 按键短按：校准机械零点，同时 LED 翻转提示。 */
      ControlCalibrateZeroAngle();
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      g_iButtonState = 0;
    }

    osDelay(10);
  }
}

/* USER CODE END Application */

