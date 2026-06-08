/**
******************************************************************************
* @file         DAC8831.h
* @version      V1.0
* @date         2023-09-04                  
* @brief        DAC8831.c的头文件
******************************************************************************
*/
#ifndef _DAC8831_H_
#define _DAC8831_H_
#include "stm32h7xx_hal.h"
#include "gpio.h"
#include "spi.h"
/***************----- DAC GPIO定义 -----***************/
#define DAC_CS		GPIO_PIN_12	// DAC8831 CS 引脚	 
#define DAC_CS_PORT	GPIOB

#define DAC_CS_RESET HAL_GPIO_WritePin(P_DAC_CS_GPIO_Port, P_DAC_CS_Pin, GPIO_PIN_RESET)
#define DAC_CS_SET	 HAL_GPIO_WritePin(P_DAC_CS_GPIO_Port, P_DAC_CS_Pin, GPIO_PIN_SET)

void DAC8831_CS_HIGH(void);
void DAC8831_Write(int32_t data);

#endif

