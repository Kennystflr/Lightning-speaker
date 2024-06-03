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

#include "stm32g4xx_ll_ucpd.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

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
#define A1_Pin GPIO_PIN_13
#define A1_GPIO_Port GPIOC
#define A2_Pin GPIO_PIN_14
#define A2_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_15
#define B1_GPIO_Port GPIOC
#define B2_Pin GPIO_PIN_0
#define B2_GPIO_Port GPIOF
#define V_sense_Pin GPIO_PIN_1
#define V_sense_GPIO_Port GPIOF
#define Out_L_Pin GPIO_PIN_0
#define Out_L_GPIO_Port GPIOA
#define Out_R_Pin GPIO_PIN_1
#define Out_R_GPIO_Port GPIOA
#define pot_volume_Pin GPIO_PIN_2
#define pot_volume_GPIO_Port GPIOA
#define light_Pin GPIO_PIN_4
#define light_GPIO_Port GPIOA
#define eq_band_btn_Pin GPIO_PIN_11
#define eq_band_btn_GPIO_Port GPIOB
#define band3_Pin GPIO_PIN_12
#define band3_GPIO_Port GPIOB
#define band2_Pin GPIO_PIN_13
#define band2_GPIO_Port GPIOB
#define band1_Pin GPIO_PIN_14
#define band1_GPIO_Port GPIOB
#define band0_Pin GPIO_PIN_15
#define band0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
