/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PH0-OSC_IN (PH0)   ------> RCC_OSC_IN
     PH1-OSC_OUT (PH1)   ------> RCC_OSC_OUT
     PA13 (JTMS/SWDIO)   ------> DEBUG_JTMS-SWDIO
     PA14 (JTCK/SWCLK)   ------> DEBUG_JTCK-SWCLK
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, P_LEDWK0_Pin|P_LEDWK1_Pin|P_485DE_Pin|P_PMODE0_Pin
                          |P_DO5_RL_Pin|P_DO4_RL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, P_W5500_RSTN_Pin|P_W5500_SCSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(P_PMODE1_GPIO_Port, P_PMODE1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, P_PMODE2_Pin|P_DO2_Pin|P_DO3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(P_DAC_CS_GPIO_Port, P_DAC_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, P_MRAM_CS_Pin|P_MRAM_WP_Pin|P_ADCCS3_Pin|P_ADCCS2_Pin
                          |P_ADCCS1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, P_ADCSYNC_Pin|P_DO1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(P_DO6_GPIO_Port, P_DO6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : P_LEDWK0_Pin P_LEDWK1_Pin P_485DE_Pin P_PMODE0_Pin
                           P_DO5_RL_Pin P_DO4_RL_Pin */
  GPIO_InitStruct.Pin = P_LEDWK0_Pin|P_LEDWK1_Pin|P_485DE_Pin|P_PMODE0_Pin
                          |P_DO5_RL_Pin|P_DO4_RL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : P_DI1_Pin P_DI2_Pin P_DI10_Pin */
  GPIO_InitStruct.Pin = P_DI1_Pin|P_DI2_Pin|P_DI10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : P_W5500_RSTN_Pin P_W5500_SCSN_Pin */
  GPIO_InitStruct.Pin = P_W5500_RSTN_Pin|P_W5500_SCSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : P_W5500_INTN_Pin P_DI9_Pin P_DI8_Pin P_DI7_Pin */
  GPIO_InitStruct.Pin = P_W5500_INTN_Pin|P_DI9_Pin|P_DI8_Pin|P_DI7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : P_PMODE1_Pin */
  GPIO_InitStruct.Pin = P_PMODE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(P_PMODE1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : P_PMODE2_Pin P_DO2_Pin P_DO3_Pin */
  GPIO_InitStruct.Pin = P_PMODE2_Pin|P_DO2_Pin|P_DO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : P_DI3_Pin PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = P_DI3_Pin|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : P_DI4_Pin P_DI5_Pin P_DI6_Pin P_CON1_Pin
                           P_ADJ1_Pin */
  GPIO_InitStruct.Pin = P_DI4_Pin|P_DI5_Pin|P_DI6_Pin|P_CON1_Pin
                          |P_ADJ1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : P_DAC_CS_Pin */
  GPIO_InitStruct.Pin = P_DAC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(P_DAC_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : P_ADJ3_Pin P_CON3_Pin P_ADJ2_Pin P_CON2_Pin */
  GPIO_InitStruct.Pin = P_ADJ3_Pin|P_CON3_Pin|P_ADJ2_Pin|P_CON2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : P_MRAM_CS_Pin P_MRAM_WP_Pin P_ADCCS3_Pin P_ADCCS2_Pin
                           P_ADCCS1_Pin */
  GPIO_InitStruct.Pin = P_MRAM_CS_Pin|P_MRAM_WP_Pin|P_ADCCS3_Pin|P_ADCCS2_Pin
                          |P_ADCCS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : P_ADCSYNC_Pin P_DO1_Pin */
  GPIO_InitStruct.Pin = P_ADCSYNC_Pin|P_DO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : P_ADCDRDY_Pin */
  GPIO_InitStruct.Pin = P_ADCDRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(P_ADCDRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : P_DO6_Pin */
  GPIO_InitStruct.Pin = P_DO6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(P_DO6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : P_DI11_Pin */
  GPIO_InitStruct.Pin = P_DI11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(P_DI11_GPIO_Port, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
