/**
******************************************************************************
* @file         bsp_spi.h
* @version      V1.0
* @date         2020-06-05                  
* @brief        bsp_spi.c的头文件
******************************************************************************
*/
#ifndef _BSP_SPI_H_
#define _BSP_SPI_H_
#include "types.h"
#include "spi.h"
#include "stm32h7xx_hal.h"
/***************----- W5500 GPIO定义 -----***************/
#define W5500_SCS		GPIO_PIN_4	//定义W5500的CS引脚	 
#define W5500_SCS_PORT	GPIOA
	
#define W5500_RST		GPIO_PIN_2	//定义W5500的RST引脚
#define W5500_RST_PORT	GPIOA

#define W5500_INT		GPIO_PIN_3	//定义W5500的INT引脚
#define W5500_INT_PORT	GPIOA


#define hspix hspi1 //***网上找的是用SPI1的 便于代码通用性 使用SPIx代替
#define SPIx SPI1

void SPI_WriteByte(uint8_t TxData);
uint8_t SPI_ReadByte(void);
void SPI_CS_Select(void);//cs_low
void SPI_CS_Deselect(void);//cs_high
void SPI_CrisEnter(void);
void SPI_CrisExit(void);
#endif
