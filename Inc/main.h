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
#include "stm32f7xx_hal.h"

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define START_Pin GPIO_PIN_14
#define START_GPIO_Port GPIOC
#define START_IR_Pin GPIO_PIN_15
#define START_IR_GPIO_Port GPIOC
#define SENSOR5_Pin GPIO_PIN_0
#define SENSOR5_GPIO_Port GPIOC
#define SENSOR4_Pin GPIO_PIN_2
#define SENSOR4_GPIO_Port GPIOA
#define FINAL_D_Pin GPIO_PIN_3
#define FINAL_D_GPIO_Port GPIOA
#define FINAL_D_EXTI_IRQn EXTI3_IRQn
#define ESC_DER_Pin GPIO_PIN_6
#define ESC_DER_GPIO_Port GPIOA
#define ESC_IZQ_Pin GPIO_PIN_7
#define ESC_IZQ_GPIO_Port GPIOA
#define INCL_1_Pin GPIO_PIN_13
#define INCL_1_GPIO_Port GPIOB
#define INCL_1_EXTI_IRQn EXTI15_10_IRQn
#define INCL_2_Pin GPIO_PIN_15
#define INCL_2_GPIO_Port GPIOB
#define INCL_2_EXTI_IRQn EXTI15_10_IRQn
#define SENSOR2_Pin GPIO_PIN_7
#define SENSOR2_GPIO_Port GPIOC
#define SENSOR1_Pin GPIO_PIN_8
#define SENSOR1_GPIO_Port GPIOC
#define FINAL_I_Pin GPIO_PIN_9
#define FINAL_I_GPIO_Port GPIOC
#define DIP5_Pin GPIO_PIN_3
#define DIP5_GPIO_Port GPIOB
#define DIP4_Pin GPIO_PIN_4
#define DIP4_GPIO_Port GPIOB
#define DIP3_Pin GPIO_PIN_5
#define DIP3_GPIO_Port GPIOB
#define DIP2_Pin GPIO_PIN_8
#define DIP2_GPIO_Port GPIOB
#define DIP1_Pin GPIO_PIN_9
#define DIP1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
