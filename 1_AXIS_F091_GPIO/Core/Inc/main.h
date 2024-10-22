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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SSM1ST2420HC.h"

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
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern ADC_HandleTypeDef hadc;
extern CAN_HandleTypeDef hcan;
extern int Ax;
extern uint16_t Angles[4100];
extern int AMS_ENB;
extern int ENC_ENB;
extern uint8_t AMS_Ready;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AIN_MCU_Pin GPIO_PIN_0
#define AIN_MCU_GPIO_Port GPIOA
#define CAN_STB_Pin GPIO_PIN_2
#define CAN_STB_GPIO_Port GPIOA
#define AMS_CS_Pin GPIO_PIN_4
#define AMS_CS_GPIO_Port GPIOA
#define AMS_SCK_Pin GPIO_PIN_5
#define AMS_SCK_GPIO_Port GPIOA
#define AMS_MISO_Pin GPIO_PIN_6
#define AMS_MISO_GPIO_Port GPIOA
#define AMS_MOSI_Pin GPIO_PIN_7
#define AMS_MOSI_GPIO_Port GPIOA
#define AMS_INT_Pin GPIO_PIN_0
#define AMS_INT_GPIO_Port GPIOB
#define AMS_INT_EXTI_IRQn EXTI0_1_IRQn
#define TMC_CS_Pin GPIO_PIN_12
#define TMC_CS_GPIO_Port GPIOB
#define TMC_SCK_Pin GPIO_PIN_13
#define TMC_SCK_GPIO_Port GPIOB
#define TMC_MISO_Pin GPIO_PIN_14
#define TMC_MISO_GPIO_Port GPIOB
#define TMC_MOSI_Pin GPIO_PIN_15
#define TMC_MOSI_GPIO_Port GPIOB
#define DRV_ENN_Pin GPIO_PIN_8
#define DRV_ENN_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define EXT_OUT_1_Pin GPIO_PIN_4
#define EXT_OUT_1_GPIO_Port GPIOB
#define EXT_OUT_2_Pin GPIO_PIN_5
#define EXT_OUT_2_GPIO_Port GPIOB
#define REFL_UC_Pin GPIO_PIN_6
#define REFL_UC_GPIO_Port GPIOB
#define REFR_UC_Pin GPIO_PIN_7
#define REFR_UC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
