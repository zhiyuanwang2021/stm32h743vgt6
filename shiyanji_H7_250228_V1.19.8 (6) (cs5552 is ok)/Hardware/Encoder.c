#include "Encoder.h"
#include "tim.h"
#include "stm32h7xx_hal_tim.h"
#include "usart.h"
#include "math.h"

ENCODER_STRUCT encoder;

void Encoder_Init(void)
{
	//P_A1 P_B1
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);   //开启编码器定时器
	__HAL_TIM_ENABLE_IT(&htim4,TIM_IT_UPDATE);      //开启编码器定时器更新中断,防溢出处理
	__HAL_TIM_SET_COUNTER(&htim4, 30000);
	//P_A/SSI_DATA  P_B
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);   //开启编码器定时器
	__HAL_TIM_ENABLE_IT(&htim1,TIM_IT_UPDATE);      //开启编码器定时器更新中断,防溢出处理
	__HAL_TIM_SET_COUNTER(&htim1, 30000);
	//P_A2 P_B2
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);   //开启编码器定时器
	__HAL_TIM_ENABLE_IT(&htim5,TIM_IT_UPDATE);      //开启编码器定时器更新中断,防溢出处理
	__HAL_TIM_SET_COUNTER(&htim5, 30000);
}

/**
 * @brief when system init,init count value according to tare or code storaged in mram
*/
void encoderCountInit(int32_t count0,int32_t count1,int32_t count2,ENCODER_STRUCT *encoder_struct)
{
	encoder_struct->count0 = count0;
	encoder_struct->count1 = count1;
	encoder_struct->count2 = count2;
}

void Encoder_Get(uint8_t channel,ENCODER_STRUCT *encoder_struct)
{
	int32_t counter = 0;
		switch(channel)
		{
			case Encoder0: 
				counter = (__HAL_TIM_GetCounter(&htim1)-30000);
				counter = counter;
				encoder_struct->count0 += counter;
				__HAL_TIM_SetCounter(&htim1, 30000);
			//printf("turns:%d  count0:%d\r\n",encoder_struct->turns0,encoder_struct->count0);
			break;
			case Encoder1:
				counter = (__HAL_TIM_GetCounter(&htim4)-30000);
				counter = counter;
				encoder_struct->count1 += counter;
				__HAL_TIM_SetCounter(&htim4, 30000);
			break;
			case Encoder2:
				counter = (__HAL_TIM_GetCounter(&htim5)-30000);
				counter = counter;
				encoder_struct->count2 += counter;
				__HAL_TIM_SetCounter(&htim5, 30000);
			default:
			break;
		}
}




