/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define RAIN_Pin GPIO_PIN_15
#define RAIN_GPIO_Port GPIOC
#define DHT_Pin GPIO_PIN_1
#define DHT_GPIO_Port GPIOA
#define pumpControl_Pin GPIO_PIN_2
#define pumpControl_GPIO_Port GPIOA
#define pumpControl_EXTI_IRQn EXTI2_IRQn
#define fanControl_Pin GPIO_PIN_3
#define fanControl_GPIO_Port GPIOA
#define fanControl_EXTI_IRQn EXTI3_IRQn
#define ledControl_Pin GPIO_PIN_4
#define ledControl_GPIO_Port GPIOA
#define ledControl_EXTI_IRQn EXTI4_IRQn
#define coverControl_Pin GPIO_PIN_5
#define coverControl_GPIO_Port GPIOA
#define coverControl_EXTI_IRQn EXTI9_5_IRQn
#define LCD_D7_Pin GPIO_PIN_12
#define LCD_D7_GPIO_Port GPIOB
#define LCD_D6_Pin GPIO_PIN_14
#define LCD_D6_GPIO_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_15
#define LCD_D5_GPIO_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_8
#define LCD_D4_GPIO_Port GPIOA
#define TX_Pin GPIO_PIN_9
#define TX_GPIO_Port GPIOA
#define RX_Pin GPIO_PIN_10
#define RX_GPIO_Port GPIOA
#define LCD_EN_Pin GPIO_PIN_11
#define LCD_EN_GPIO_Port GPIOA
#define LCD_RS_Pin GPIO_PIN_12
#define LCD_RS_GPIO_Port GPIOA
#define switch1_Pin GPIO_PIN_3
#define switch1_GPIO_Port GPIOB
#define switch2_Pin GPIO_PIN_4
#define switch2_GPIO_Port GPIOB
#define cover1_Pin GPIO_PIN_5
#define cover1_GPIO_Port GPIOB
#define cover2_Pin GPIO_PIN_6
#define cover2_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_7
#define LED_GPIO_Port GPIOB
#define FAN_Pin GPIO_PIN_8
#define FAN_GPIO_Port GPIOB
#define PUMP_Pin GPIO_PIN_9
#define PUMP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
