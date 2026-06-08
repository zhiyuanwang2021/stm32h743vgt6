#ifndef _MRAM_H_
#define _MRAM_H_

#include "main.h"
#define hspi_MRAM hspi2
#define CS_PIN_PORT GPIOD
#define CS_PIN GPIO_PIN_14
#define USE_HOLD 0
#define USE_WP 1


#define WREN 0x06 //Write Enable
#define WRDI 0x04 //Write Disable
#define RDSR 0x05 //Read Status Register 
#define WRSR 0x01 //Write Status Register
#define READ 0x03 //Read Data Bytes 
#define WRITE 0x02 //Write Data Bytes
#define SLEEP 0xB9 //Enter Sleep Mode
#define WAKE 0xAB //Exit Sleep Mode

#define BLOCKS_SR_WRITABLE 0x82

#define CS_LOW HAL_GPIO_WritePin(CS_PIN_PORT,CS_PIN,GPIO_PIN_RESET)
#define CS_HIGH HAL_GPIO_WritePin(CS_PIN_PORT,CS_PIN,GPIO_PIN_SET)

#define MRAM_WP_LOW HAL_GPIO_WritePin(P_MRAM_WP_GPIO_Port,P_MRAM_WP_Pin,GPIO_PIN_RESET)
#define MRAM_WP_HIGH HAL_GPIO_WritePin(P_MRAM_WP_GPIO_Port,P_MRAM_WP_Pin,GPIO_PIN_SET)

#if USE_HOLD
#define HOLD_LOW HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET)
#define HOLD_HIGH HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET)
#endif

#if USE_WP
#define WP_LOW HAL_GPIO_WritePin(P_MRAM_WP_GPIO_Port,P_MRAM_WP_Pin,GPIO_PIN_RESET)
#define WP_HIGH HAL_GPIO_WritePin(P_MRAM_WP_GPIO_Port,P_MRAM_WP_Pin,GPIO_PIN_SET)
#endif

extern void MRAM_ReadStatusRegister(void);
extern void MRAM_WriteStatusRegister(void);
extern void MRAM_WriteEnable(void);
extern void MRAM_CS_HIGH(void);
extern uint8_t MRAM_READ(uint32_t address,uint8_t* Data,uint16_t size);
extern uint8_t MRAM_WRITE(uint32_t address,uint8_t* Data,uint16_t size);

#endif
