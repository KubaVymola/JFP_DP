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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NRF24_IRQ_Pin GPIO_PIN_13
#define NRF24_IRQ_GPIO_Port GPIOC
#define NRF24_CSN_Pin GPIO_PIN_14
#define NRF24_CSN_GPIO_Port GPIOC
#define NRF24_CE_Pin GPIO_PIN_15
#define NRF24_CE_GPIO_Port GPIOC
#define ESC_VIN_ADC_Pin GPIO_PIN_0
#define ESC_VIN_ADC_GPIO_Port GPIOC
#define SPI2_CS_Pin GPIO_PIN_1
#define SPI2_CS_GPIO_Port GPIOC
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define HP203B_INT1_Pin GPIO_PIN_6
#define HP203B_INT1_GPIO_Port GPIOC
#define STATUS_LED_Pin GPIO_PIN_8
#define STATUS_LED_GPIO_Port GPIOA
#define BMI160_INT1_Pin GPIO_PIN_15
#define BMI160_INT1_GPIO_Port GPIOA
#define BMI160_CS_Pin GPIO_PIN_2
#define BMI160_CS_GPIO_Port GPIOD
#define BMI160_INT2_Pin GPIO_PIN_3
#define BMI160_INT2_GPIO_Port GPIOB
#define MOTOR_1_Pin GPIO_PIN_4
#define MOTOR_1_GPIO_Port GPIOB
#define MOTOR_2_Pin GPIO_PIN_5
#define MOTOR_2_GPIO_Port GPIOB
#define MOTOR_3_Pin GPIO_PIN_6
#define MOTOR_3_GPIO_Port GPIOB
#define MOTOR_4_Pin GPIO_PIN_7
#define MOTOR_4_GPIO_Port GPIOB
#define PPM_INPUT_Pin GPIO_PIN_9
#define PPM_INPUT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
