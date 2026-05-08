/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

//此文件代表中断服务函数文件
//STM32 运行过程中，某些硬件事件突然发生时，CPU 会暂停当前事情，跳到这里执行对应函数
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "button.h"
#include "control.h"
#include "ultrasonic.h"
#include "bluetooth.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* 中断文件需要访问 TIM1/USART 句柄，把硬件中断转交给 HAL 和用户回调。 */
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
//小车心跳
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  if (g_u8ControlTickEnabled)
  {
    /*
     * 硬实时控制仍放在 1ms SysTick：
     * FreeRTOS 只管理 OLED/超声波/调试输出等低频任务，避免这些慢任务影响平衡。
     */
    ControlRampUpdate();
		//让蓝牙/超声波/红外给出的速度目标、转向目标慢慢变化
    g_nMainEventCount++;
		//时间基准时间加1
    g_nSpeedControlPeriod++;
		//算速度环从旧的值到新的值的时间，后续计算得到速度环目标输出，使得速度在变换的时候更加平滑
		//每 25ms 算出新的速度环输出然后把平滑计时器清零重新开始从 old 过渡到 new
    if (g_nMainEventCount >= 5)
    {
      g_nMainEventCount = 0;
			//大于5时间就清零，5ms作为一个循环
      GetMotorPulse();
			//读编码器数据
    }
    else if (g_nMainEventCount == 1)
    {
      GetMpuData();
      AngleCalculate();
			//读 MPU + 算角度
    }
    else if (g_nMainEventCount == 2)
    {
      AngleControl();
			//角度环控制，因为角度是内环，更重要所以控制时间比速度环周期短
    }
    else if (g_nMainEventCount == 3)
    {
      g_nSpeedControlCount++;
      if (g_nSpeedControlCount >= 5)
				//速度环25ms为周期重新计算
      {
        SpeedControl();
        g_nSpeedControlCount = 0;
        g_nSpeedControlPeriod = 0;
      }
    }
    else if (g_nMainEventCount == 4)
    {
      SpeedControlOutput();
      MotorManage();
      MotorOutput();
			//速度输出 + 电机输出
    }
    ButtonScan();
		//按键消抖，确认按下
  }
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
//让 HAL_GetTick() / HAL_Delay() 的时间继续走
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
		//让 FreeRTOS 的任务调度时间继续走
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
  /* USER CODE BEGIN SysTick_IRQn 1 */
HAL_SYSTICK_IRQHandler();
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
	//蓝牙中断，处理刚收到的蓝牙字节然后重新开启下一次接收
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/* 串口接收完成回调：USART3 收到蓝牙字节后解释命令，并重新打开下一字节接收。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    Bluetooth_OnByteReceived(g_u8BluetoothRxByte);
    HAL_UART_Receive_IT(&huart3, &g_u8BluetoothRxByte, 1);
  }
}

/* 串口错误回调：蓝牙串口出错时重新开启接收，避免遥控失联后不恢复。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    HAL_UART_Receive_IT(&huart3, &g_u8BluetoothRxByte, 1);
  }
}

/* TIM1 输入捕获回调：用 Echo 上升沿/下降沿测出超声波高电平宽度。 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
    if ((TIM1CH4_CAPTURE_STA & 0x80) == 0)
    {
      if (TIM1CH4_CAPTURE_STA & 0x40)
      {
        TIM1CH4_CAPTURE_VAL = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
        TIM1CH4_CAPTURE_STA |= 0x80;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
      }
      else
      {
        TIM1CH4_CAPTURE_STA = 0x40;
        TIM1CH4_CAPTURE_VAL = 0;
        __HAL_TIM_SET_COUNTER(htim, 0);
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
      }
    }
  }
}

/* TIM1 溢出回调：Echo 高电平超过 65535us 时累计溢出次数，防止测距时间截断。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    if ((TIM1CH4_CAPTURE_STA & 0x80) == 0)
    {
      if (TIM1CH4_CAPTURE_STA & 0x40)
      {
        if ((TIM1CH4_CAPTURE_STA & 0x3f) == 0x3f)
        {
          TIM1CH4_CAPTURE_STA |= 0x80;
          TIM1CH4_CAPTURE_VAL = 0xffff;
          __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
        }
        else
        {
          TIM1CH4_CAPTURE_STA++;
        }
      }
    }
  }
}

/* USER CODE END 1 */

