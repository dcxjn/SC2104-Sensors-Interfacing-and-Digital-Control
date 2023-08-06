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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OLED1_Pin GPIO_PIN_5
#define OLED1_GPIO_Port GPIOE
#define OLED2_Pin GPIO_PIN_6
#define OLED2_GPIO_Port GPIOE
#define AIN2_Pin GPIO_PIN_2
#define AIN2_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_3
#define AIN1_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_4
#define BIN1_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_5
#define BIN2_GPIO_Port GPIOA
#define OLED3_Pin GPIO_PIN_7
#define OLED3_GPIO_Port GPIOE
#define OLED4_Pin GPIO_PIN_8
#define OLED4_GPIO_Port GPIOE
#define LED_Pin GPIO_PIN_10
#define LED_GPIO_Port GPIOE
#define Buzzer_Pin GPIO_PIN_10
#define Buzzer_GPIO_Port GPIOB
#define DIN1_Pin GPIO_PIN_11
#define DIN1_GPIO_Port GPIOB
#define DIN2_Pin GPIO_PIN_15
#define DIN2_GPIO_Port GPIOB
#define USER_PB_Pin GPIO_PIN_8
#define USER_PB_GPIO_Port GPIOD
#define USER_PB_EXTI_IRQn EXTI9_5_IRQn
#define PWMA_Pin GPIO_PIN_6
#define PWMA_GPIO_Port GPIOC
#define PWMB_Pin GPIO_PIN_7
#define PWMB_GPIO_Port GPIOC
#define PWMD_Pin GPIO_PIN_9
#define PWMD_GPIO_Port GPIOC
#define Ch_B_Pin GPIO_PIN_15
#define Ch_B_GPIO_Port GPIOA
#define Ch_B_EXTI_IRQn EXTI15_10_IRQn
#define Ch_A_Pin GPIO_PIN_3
#define Ch_A_GPIO_Port GPIOB
#define Ch_A_EXTI_IRQn EXTI3_IRQn
#define IMU_INT_Pin GPIO_PIN_1
#define IMU_INT_GPIO_Port GPIOE
#define IMU_INT_EXTI_IRQn EXTI1_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
