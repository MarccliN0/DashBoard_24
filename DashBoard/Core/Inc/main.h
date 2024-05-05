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
#include "stm32g4xx_hal.h"

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
#define reset_button_Pin GPIO_PIN_10
#define reset_button_GPIO_Port GPIOG
#define rot_0_Pin GPIO_PIN_0
#define rot_0_GPIO_Port GPIOA
#define rot_1_Pin GPIO_PIN_1
#define rot_1_GPIO_Port GPIOA
#define joy_0_Pin GPIO_PIN_2
#define joy_0_GPIO_Port GPIOA
#define rot_2_Pin GPIO_PIN_4
#define rot_2_GPIO_Port GPIOA
#define res_out_Pin GPIO_PIN_5
#define res_out_GPIO_Port GPIOA
#define btn_0_Pin GPIO_PIN_6
#define btn_0_GPIO_Port GPIOA
#define btn_1_Pin GPIO_PIN_7
#define btn_1_GPIO_Port GPIOA
#define rot_3_Pin GPIO_PIN_11
#define rot_3_GPIO_Port GPIOB
#define btn_3_Pin GPIO_PIN_14
#define btn_3_GPIO_Port GPIOB
#define btn_2_Pin GPIO_PIN_15
#define btn_2_GPIO_Port GPIOB
#define joy_4_Pin GPIO_PIN_3
#define joy_4_GPIO_Port GPIOB
#define joy_3_Pin GPIO_PIN_4
#define joy_3_GPIO_Port GPIOB
#define joy_2_Pin GPIO_PIN_5
#define joy_2_GPIO_Port GPIOB
#define joy_1_Pin GPIO_PIN_6
#define joy_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
