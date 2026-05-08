/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "button.h"
#include "mpu6050.h"
#include "control.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OLED_DC_Pin GPIO_PIN_13
#define OLED_DC_GPIO_Port GPIOC
#define TRIG_Pin GPIO_PIN_2
#define TRIG_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_3
#define BIN1_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_4
#define BIN2_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_0
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_1
#define AIN2_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOB
#define OLED_RST_Pin GPIO_PIN_13
#define OLED_RST_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_14
#define OLED_SDA_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_15
#define OLED_SCL_GPIO_Port GPIOB
#define Button_Pin GPIO_PIN_8
#define Button_GPIO_Port GPIOA
#define Ra_Pin GPIO_PIN_15
#define Ra_GPIO_Port GPIOA
#define Lb_Pin GPIO_PIN_3
#define Lb_GPIO_Port GPIOB
#define Rb_Pin GPIO_PIN_4
#define Rb_GPIO_Port GPIOB
#define La_Pin GPIO_PIN_5
#define La_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define Ra_Pin GPIO_PIN_15
#define Ra_GPIO_Port GPIOA
#define Rb_Pin GPIO_PIN_4
#define Rb_GPIO_Port GPIOB
#define La_Pin GPIO_PIN_5
#define La_GPIO_Port GPIOB
#define Lb_Pin GPIO_PIN_3
#define Lb_GPIO_Port GPIOB

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
