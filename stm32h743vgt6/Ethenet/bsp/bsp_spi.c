/**
******************************************************************************
* @file         bsp_spi.c
* @version      V1.0
* @date         2020-06-05
* @brief        配置与W5500通信的SPI管脚与SPI配置
******************************************************************************
*/
#include "bsp_spi.h"


/**
  * @brief  写1字节数据到SPI总线
  * @param  TxData 写到总线的数据
  * @retval None
  */
void SPI_WriteByte(uint8_t TxData)
{
	HAL_SPI_Transmit(&hspix, &TxData, 1, 0xff);
}


/**
  * @brief  从SPI总线读取1字节数据
  * @retval 读到的数据
  */
uint8_t SPI_ReadByte(void)
{
	uint8_t i;
		HAL_SPI_Receive(&hspix, &i, 1, 0xff);
	return i;
}

//uint8 spi_read_send_byte(uint8 byte)
//{
//	uint8 i;
//		HAL_SPI_TransmitReceive(&hspix,&byte,&i,1,0xff);
//	return i;
//}


void SPI_CS_Select(void)//cs_low
{
  HAL_GPIO_WritePin(W5500_SCS_PORT,W5500_SCS,GPIO_PIN_RESET);
}

void SPI_CS_Deselect(void) //cs_high
{
  HAL_GPIO_WritePin(W5500_SCS_PORT, W5500_SCS, GPIO_PIN_SET);
}

/**
  * @brief  进入临界区
  * @retval None
  */
void SPI_CrisEnter(void)
{
	__set_PRIMASK(1);
}


/**
  * @brief  退出临界区
  * @retval None
  */
void SPI_CrisExit(void)
{
	__set_PRIMASK(0);
}
