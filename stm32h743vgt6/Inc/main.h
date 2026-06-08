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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "stdint.h"
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
#define P_DO_DIR_Pin GPIO_PIN_5
#define P_DO_DIR_GPIO_Port GPIOE
#define P_LEDWK0_Pin GPIO_PIN_14
#define P_LEDWK0_GPIO_Port GPIOC
#define P_LEDWK1_Pin GPIO_PIN_15
#define P_LEDWK1_GPIO_Port GPIOC
#define P_DI1_Pin GPIO_PIN_0
#define P_DI1_GPIO_Port GPIOC
#define P_DI2_Pin GPIO_PIN_1
#define P_DI2_GPIO_Port GPIOC
#define P_485DE_Pin GPIO_PIN_3
#define P_485DE_GPIO_Port GPIOC
#define P_A2_Pin GPIO_PIN_0
#define P_A2_GPIO_Port GPIOA
#define P_B2_Pin GPIO_PIN_1
#define P_B2_GPIO_Port GPIOA
#define P_W5500_RSTN_Pin GPIO_PIN_2
#define P_W5500_RSTN_GPIO_Port GPIOA
#define P_W5500_INTN_Pin GPIO_PIN_3
#define P_W5500_INTN_GPIO_Port GPIOA
#define P_W5500_SCSN_Pin GPIO_PIN_4
#define P_W5500_SCSN_GPIO_Port GPIOA
#define P_W5500_SCLK_Pin GPIO_PIN_5
#define P_W5500_SCLK_GPIO_Port GPIOA
#define P_W5500_MISO_Pin GPIO_PIN_6
#define P_W5500_MISO_GPIO_Port GPIOA
#define P_W5500_MOSI_Pin GPIO_PIN_7
#define P_W5500_MOSI_GPIO_Port GPIOA
#define P_PMODE0_Pin GPIO_PIN_4
#define P_PMODE0_GPIO_Port GPIOC
#define P_PMODE1_Pin GPIO_PIN_5
#define P_PMODE1_GPIO_Port GPIOC
#define P_PMODE2_Pin GPIO_PIN_0
#define P_PMODE2_GPIO_Port GPIOB
#define P_DI3_Pin GPIO_PIN_1
#define P_DI3_GPIO_Port GPIOB
#define P_DI4_Pin GPIO_PIN_7
#define P_DI4_GPIO_Port GPIOE
#define P_DI5_Pin GPIO_PIN_8
#define P_DI5_GPIO_Port GPIOE
#define P_A_Pin GPIO_PIN_9
#define P_A_GPIO_Port GPIOE
#define P_B_Pin GPIO_PIN_11
#define P_B_GPIO_Port GPIOE
#define P_DI6_Pin GPIO_PIN_12
#define P_DI6_GPIO_Port GPIOE
#define P_CON1_Pin GPIO_PIN_13
#define P_CON1_GPIO_Port GPIOE
#define P_ADJ1_Pin GPIO_PIN_14
#define P_ADJ1_GPIO_Port GPIOE
#define P_DAC_CS_Pin GPIO_PIN_12
#define P_DAC_CS_GPIO_Port GPIOB
#define P_DAC_SCLK_Pin GPIO_PIN_13
#define P_DAC_SCLK_GPIO_Port GPIOB
#define P_DAC_SDI_Pin GPIO_PIN_15
#define P_DAC_SDI_GPIO_Port GPIOB
#define P_ADJ3_Pin GPIO_PIN_8
#define P_ADJ3_GPIO_Port GPIOD
#define P_CON3_Pin GPIO_PIN_9
#define P_CON3_GPIO_Port GPIOD
#define P_ADJ2_Pin GPIO_PIN_10
#define P_ADJ2_GPIO_Port GPIOD
#define P_CON2_Pin GPIO_PIN_11
#define P_CON2_GPIO_Port GPIOD
#define P_A1_Pin GPIO_PIN_12
#define P_A1_GPIO_Port GPIOD
#define P_B1_Pin GPIO_PIN_13
#define P_B1_GPIO_Port GPIOD
#define P_MRAM_CS_Pin GPIO_PIN_14
#define P_MRAM_CS_GPIO_Port GPIOD
#define P_MRAM_WP_Pin GPIO_PIN_15
#define P_MRAM_WP_GPIO_Port GPIOD
#define P_DO_PUL_Pin GPIO_PIN_6
#define P_DO_PUL_GPIO_Port GPIOC
#define P_DO5_RL_Pin GPIO_PIN_7
#define P_DO5_RL_GPIO_Port GPIOC
#define P_DO4_RL_Pin GPIO_PIN_8
#define P_DO4_RL_GPIO_Port GPIOC
#define P_DI10_Pin GPIO_PIN_9
#define P_DI10_GPIO_Port GPIOC
#define P_DI9_Pin GPIO_PIN_8
#define P_DI9_GPIO_Port GPIOA
#define P_DI8_Pin GPIO_PIN_11
#define P_DI8_GPIO_Port GPIOA
#define P_DI7_Pin GPIO_PIN_12
#define P_DI7_GPIO_Port GPIOA
#define P_ADCCLK_Pin GPIO_PIN_15
#define P_ADCCLK_GPIO_Port GPIOA
#define P_ADCCS3_Pin GPIO_PIN_0
#define P_ADCCS3_GPIO_Port GPIOD
#define P_ADCSYNC_Pin GPIO_PIN_1
#define P_ADCSYNC_GPIO_Port GPIOD
#define P_ADCDRDY_Pin GPIO_PIN_2
#define P_ADCDRDY_GPIO_Port GPIOD
#define P_ADCCS2_Pin GPIO_PIN_3
#define P_ADCCS2_GPIO_Port GPIOD
#define P_ADCCS1_Pin GPIO_PIN_4
#define P_ADCCS1_GPIO_Port GPIOD
#define P_DO1_Pin GPIO_PIN_7
#define P_DO1_GPIO_Port GPIOD
#define P_DO2_Pin GPIO_PIN_3
#define P_DO2_GPIO_Port GPIOB
#define P_DO3_Pin GPIO_PIN_4
#define P_DO3_GPIO_Port GPIOB
#define P_DO6_Pin GPIO_PIN_0
#define P_DO6_GPIO_Port GPIOE
#define P_DI11_Pin GPIO_PIN_1
#define P_DI11_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
extern uint16_t tim6us;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
