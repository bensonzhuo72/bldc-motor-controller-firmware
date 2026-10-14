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
#define MOTOR_EN_Pin GPIO_PIN_1
#define MOTOR_EN_GPIO_Port GPIOC
#define LED_STATUS_Pin GPIO_PIN_3
#define LED_STATUS_GPIO_Port GPIOC
#define SPI1_CS_ENCODER_Pin GPIO_PIN_4
#define SPI1_CS_ENCODER_GPIO_Port GPIOA
#define SPI1_SCLK_ENCODER_Pin GPIO_PIN_5
#define SPI1_SCLK_ENCODER_GPIO_Port GPIOA
#define SPI1_MISO_ENCODER_Pin GPIO_PIN_6
#define SPI1_MISO_ENCODER_GPIO_Port GPIOA
#define SPI1_MOSI_ENCODER_Pin GPIO_PIN_7
#define SPI1_MOSI_ENCODER_GPIO_Port GPIOA
#define ADC_IN_C_Pin GPIO_PIN_0
#define ADC_IN_C_GPIO_Port GPIOB
#define ADC_IN_B_Pin GPIO_PIN_1
#define ADC_IN_B_GPIO_Port GPIOB
#define ADC_IN_A_Pin GPIO_PIN_2
#define ADC_IN_A_GPIO_Port GPIOB
#define VM_SENSE_Pin GPIO_PIN_11
#define VM_SENSE_GPIO_Port GPIOB
#define ENABLE_Pin GPIO_PIN_8
#define ENABLE_GPIO_Port GPIOC
#define CAL_Pin GPIO_PIN_9
#define CAL_GPIO_Port GPIOC
#define SPI3_SCLK_Pin GPIO_PIN_10
#define SPI3_SCLK_GPIO_Port GPIOC
#define SPI3_MISO_Pin GPIO_PIN_11
#define SPI3_MISO_GPIO_Port GPIOC
#define SPI3_CS_MOSI_Pin GPIO_PIN_12
#define SPI3_CS_MOSI_GPIO_Port GPIOC
#define SPI3_CS_Pin GPIO_PIN_2
#define SPI3_CS_GPIO_Port GPIOD
#define INT1_Pin GPIO_PIN_4
#define INT1_GPIO_Port GPIOB
#define INT2_Pin GPIO_PIN_7
#define INT2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
