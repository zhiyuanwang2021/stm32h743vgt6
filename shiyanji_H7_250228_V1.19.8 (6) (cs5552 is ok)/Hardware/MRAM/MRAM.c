#include "MRAM.h"
#include "spi.h"
#include "usart.h"

/**
 * @brief read MRAM status register
 * 
*/
void MRAM_ReadStatusRegister(void)
{
	uint8_t send,recv;
	CS_LOW;
#if USE_HOLD
	HOLD_HIGH;
#endif
#if USE_WP
	WP_HIGH;
#endif
	send = RDSR;
	HAL_SPI_Transmit(&hspi_MRAM,&send,1,1000);
	HAL_SPI_Receive(&hspi_MRAM,&recv,1,1000);
	CS_HIGH;
#if USE_HOLD
	HOLD_LOW;
#endif
#if USE_WP
	WP_LOW;
#endif
	printf("recv:%2x\n",recv);
}

/**
 * @brief write MRAM status register
 * 
*/
void MRAM_WriteStatusRegister(void)
{
	uint8_t send[2];
	CS_LOW;
#if USE_HOLD
	HOLD_HIGH;
#endif
#if USE_WP
	WP_HIGH;
#endif
	send[0] = WRSR;
	send[1] = BLOCKS_SR_WRITABLE;
	HAL_SPI_Transmit(&hspi_MRAM,send,2,10);
	CS_HIGH;
#if USE_HOLD
	HOLD_LOW;
#endif
#if USE_WP
	WP_LOW;
#endif
}

/**
 * @brief write MRAM ENBALE
 * 
*/
void MRAM_WriteEnable(void)
{
	uint8_t send;
	CS_LOW;
#if USE_HOLD
	HOLD_HIGH;
#endif
#if USE_WP
	WP_HIGH;
#endif
	send = WREN;
	HAL_SPI_Transmit(&hspi_MRAM,&send,1,10);
	CS_HIGH;
#if USE_HOLD
	HOLD_LOW;
#endif
#if USE_WP
	WP_LOW;
#endif
}

/**
 * @brief CS of MRAM draw high
 * 
*/
void MRAM_CS_HIGH(void)
{
	CS_HIGH;
}

/**
 * @brief write data to MRAM 
 * @param address MRAM interal register address
 * @param Data pointer to data needed to be writed
 * @param size  the size of Data
*/
uint8_t MRAM_WRITE(uint32_t address,uint8_t* Data,uint16_t size)
{
	uint8_t address_buf[3],send,err = 0;
	send = WRITE;
	address_buf[0]=(address>>16)&0xff;
	address_buf[1]=(address>>8)&0xff;
	address_buf[2]=address&0xff;
	CS_LOW;
#if USE_HOLD
	HOLD_HIGH;
#endif
#if USE_WP
	WP_HIGH;
#endif
	err += HAL_SPI_Transmit(&hspi_MRAM,&send,1,10);
	err += HAL_SPI_Transmit(&hspi_MRAM,address_buf,3,10);
	err += HAL_SPI_Transmit(&hspi_MRAM,Data,size,10);
	CS_HIGH;
#if USE_HOLD
	HOLD_LOW;
#endif
#if USE_WP
	WP_LOW;
#endif
	return err;
}

/**
 * @brief read data from MRAM 
 * @param address MRAM interal register address
 * @param Data pointer to data needed to be read
 * @param size  the size of Data
*/
uint8_t MRAM_READ(uint32_t address,uint8_t* Data,uint16_t size)
{
	uint8_t address_buf[3],send,err = 0;
	send = READ;
	address_buf[0]=(address>>16)&0xff;
	address_buf[1]=(address>>8)&0xff;
	address_buf[2]=address&0xff;
	CS_LOW;
#if USE_HOLD
	HOLD_HIGH;
#endif
#if USE_WP
	WP_HIGH;
#endif
	err += HAL_SPI_Transmit(&hspi_MRAM,&send,1,10);
	err += HAL_SPI_Transmit(&hspi_MRAM,address_buf,3,10);
	err += HAL_SPI_Receive(&hspi_MRAM,Data,size,10);
	CS_HIGH;
#if USE_HOLD
	HOLD_LOW;
#endif
#if USE_WP
	WP_LOW;
#endif
	return err;
}

