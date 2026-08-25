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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define LED_BLINKY_Pin GPIO_PIN_2
#define LED_BLINKY_GPIO_Port GPIOB
#define DRV_nFAULT_Pin GPIO_PIN_12
#define DRV_nFAULT_GPIO_Port GPIOB
#define DIP_SW_0_Pin GPIO_PIN_13
#define DIP_SW_0_GPIO_Port GPIOB
#define DRV_nSLEEP_Pin GPIO_PIN_5
#define DRV_nSLEEP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* PB6/PB7 are TIM4_CH1/CH2 (DRV8833 IN1+IN3 / IN2+IN4). CubeMX emits no
   defines for alternate-function pins, so they are named here — the pins have
   to be addressable as plain GPIO during startup, before TIM4 owns them. */
#define DRV_PWM_A_Pin        GPIO_PIN_6
#define DRV_PWM_A_GPIO_Port  GPIOB
#define DRV_PWM_B_Pin        GPIO_PIN_7
#define DRV_PWM_B_GPIO_Port  GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
