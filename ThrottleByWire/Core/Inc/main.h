/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENC1_T2CH1_Pin GPIO_PIN_0
#define ENC1_T2CH1_GPIO_Port GPIOA
#define ENC1_T2CH2_Pin GPIO_PIN_1
#define ENC1_T2CH2_GPIO_Port GPIOA
#define Current_sensor_Pin GPIO_PIN_4
#define Current_sensor_GPIO_Port GPIOA
#define ENC2_T3CH1_Pin GPIO_PIN_6
#define ENC2_T3CH1_GPIO_Port GPIOA
#define ENC2_T3CH2_Pin GPIO_PIN_7
#define ENC2_T3CH2_GPIO_Port GPIOA
#define ACT_DIR_Pin GPIO_PIN_9
#define ACT_DIR_GPIO_Port GPIOA
#define ACT_PWM_Pin GPIO_PIN_10
#define ACT_PWM_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
