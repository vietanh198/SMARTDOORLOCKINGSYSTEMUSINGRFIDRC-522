/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_Pin GPIO_PIN_4
#define CS_GPIO_Port GPIOA
#define LOA_Pin GPIO_PIN_12
#define LOA_GPIO_Port GPIOB
#define KHOA_Pin GPIO_PIN_13
#define KHOA_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_14
#define LED_GPIO_Port GPIOB
#define SELECCT_Pin GPIO_PIN_15
#define SELECCT_GPIO_Port GPIOB
#define MENU_Pin GPIO_PIN_8
#define MENU_GPIO_Port GPIOA
#define MENU_EXTI_IRQn EXTI9_5_IRQn
#define QR_Pin GPIO_PIN_10
#define QR_GPIO_Port GPIOA
#define D7_Pin GPIO_PIN_3
#define D7_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_4
#define D6_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_5
#define D5_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_6
#define D4_GPIO_Port GPIOB
#define EN_Pin GPIO_PIN_7
#define EN_GPIO_Port GPIOB
#define RW_Pin GPIO_PIN_8
#define RW_GPIO_Port GPIOB
#define RS_Pin GPIO_PIN_9
#define RS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
