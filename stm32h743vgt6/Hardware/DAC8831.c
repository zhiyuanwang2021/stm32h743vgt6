/*
		DAC8831输出范围-2.5V~+2.5V
		经过运放放大4倍 输出AO：-10V~10V
		DAC8831精度 Vref*(1/32768)   =>   2.5/32768 = 0.0000762939453125  最小分辨率约为0.0763mV
		实际AO输出 最小分辨率为 0.3mV
		
 @input -10V~10V 对应 -100,000~100,000  分辨率3.051758f
*/
#include "DAC8831.h"

#define hspix hspi2
#define SPIx SPI2


void DAC8831_CS_HIGH(void)
{
	DAC_CS_SET;
}

/*********************
*
* @para data 输入范围  -100,000~100,000
*
**********************/
int16_t data_offset=-6; 
void DAC8831_Write(int32_t data)
{
	
	uint8_t buf[2]={0};
	uint16_t temp=0;
	
	data = data + data_offset;
	if(data > 0)
	{	
		temp = (uint16_t)(data/3.051758f+0.5f);
		if(temp > 0x7FFF) temp=0x7FFF;
		temp |=0x8000;
	}
	else
	{
		data *=-1;
		temp = (uint16_t)(data/3.051758f+0.5f);
		if(temp > 0x8000) temp=0x8000;
		temp = 0x8000 - temp;
	}
	
	buf[0] = (uint8_t)(temp>>8);
	buf[1] = (uint8_t)(temp&0xff);
	DAC_CS_RESET;
	HAL_SPI_Transmit(&hspix, buf, 2,1000);
	DAC_CS_SET;
}


