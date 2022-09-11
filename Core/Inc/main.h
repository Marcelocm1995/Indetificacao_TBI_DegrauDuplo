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

#define LED1_ON() HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1)
#define LED1_OFF() HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0)
#define LED1_TOGGLE() HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin)

#define LED2_ON() HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1)
#define LED2_OFF() HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0)
#define LED2_TOGGLE() HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin)

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_6
#define LD1_GPIO_Port GPIOA
#define IN2_H_Pin GPIO_PIN_10
#define IN2_H_GPIO_Port GPIOB
#define DC_LCD_Pin GPIO_PIN_12
#define DC_LCD_GPIO_Port GPIOB
#define CS_LCD_Pin GPIO_PIN_6
#define CS_LCD_GPIO_Port GPIOC
#define RST_LCD_Pin GPIO_PIN_7
#define RST_LCD_GPIO_Port GPIOC
#define IN1_H_Pin GPIO_PIN_8
#define IN1_H_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
