#include "Servo_driver.h"
#include "tim.h"
#include "gpio.h"
#include "stdint.h"
#include "gParameter.h"
#include "elog.h"
#include "in_out.h"

servoPulse_t servoPulse;

/*
	参数选择Pr0.07为3  脉冲列+符号
*/
void Mod_Fclk_PULSE(uint32_t fclk);
void modFclkPULSE(uint32_t fclk);
void modFclkNumberPULSE(uint32_t fclk);
void modFclkPeriod(float fclk);
void modServoFclk(float fclk);

void Servo_DIR_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_GPIO_WritePin(DIR_Port, DIR_Pin, GPIO_PIN_RESET);
	
	GPIO_InitStruct.Pin = DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DIR_Port, &GPIO_InitStruct);
}

void Servo_PWM_Enable(uint8_t channel) //打开对应定时器对应通道的PWM
{
	switch(channel)
	{
		case PULSE:
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);//PULSE
		break;
		case SERVO_DIR:
	HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);//DIR
		break;
		default:
		break;
	}
//	HAL_TIM_Base_Start_IT(&htim7);//SSI_CLK 定时器模拟PWM
}

void Servo_PWM_Disable(uint8_t channel)//关闭对应定时器对应通道的PWM
{
	switch(channel)
	{
		case PULSE:
	HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);//PULSE
		break;
		case SERVO_DIR:
	HAL_TIM_PWM_Stop(&htim15,TIM_CHANNEL_1);//DIR
		break;
		default:
		break;
	}
}

void Servo_mod_fclk(uint8_t channel,float fclk) //更改PWM频率
{
	// static uint8_t fclkMode = 0;
	switch(channel)
	{
		case PULSE:
			if(posCtrlPeriod.count == 0){
				modServoFclk(fclk);
			}
			//if(AL.cycleT == 0)
				//Mod_Fclk_PULSE(fclk);
			//modFclkNumberPULSE(fclk);
		break;
		case SERVO_DIR:

		break;
		default:
		break;
	}

}

void Mod_Fclk_PULSE(uint32_t fclk)
{
	TIM_OC_InitTypeDef sConfigOC = {0};
	uint16_t reloadVal,reloadValHalf,psc;
	// static uint8_t i = 0;
	// static uint32_t fclkcounters = 0;
	// static uint16_t counter = 0;

	// if(fclk % 1000 != 0){
	// 	fclkcounters += (fclk % 1000);
	// }

	// if(fclkcounters >= 1000){
	// 	j = fclkcounters / 1000;
	// 	fclkcounters = fclkcounters % 1000;
	// 	fclk += j*1000;
	// }

	if( fclk <= 10000){
		// if(AL.movectrl == POS_MODE && fclk < 1000)
		// 	fclk = 1000;
		if(AL.movectrl == LOAD_MODE && fclk < 50)
			fclk = 50;
		psc = 80-1;
		reloadVal = (uint16_t) ((TIM8MCLK/((psc+1)*1.0f)/(fclk*1.0f)) - 1);
		reloadValHalf =(uint16_t) (((TIM8MCLK/((psc+1)*1.0f)/(fclk*1.0f)) - 1)*0.5f);
	}else if(fclk <= 20000){
		psc = 4-1;
		reloadVal = (uint16_t) ((TIM8MCLK/((psc+1)*1.0f)/(fclk*1.0f)) - 1);
		reloadValHalf =(uint16_t) (((TIM8MCLK/((psc+1)*1.0f)/(fclk*1.0f)) - 1)*0.5f);
	}else{
		psc = 0;
		reloadVal = (uint16_t) ((TIM8MCLK/((psc+1)*1.0f)/(fclk*1.0f)) - 1);
		reloadValHalf =(uint16_t) (((TIM8MCLK/((psc+1)*1.0f)/(fclk*1.0f)) - 1)*0.5f);
	}
	test.fr = 1.0f * TIM8MCLK / ((psc + 1)*1.0f)/((reloadVal + 1)*1.0f);
	test.pcs = psc;
	test.reloadVal = reloadVal;
	test.reloadValHalf = reloadValHalf;
	// if(i++ >= 10){
	// 	i = 0;
	// printf("%d,%d,%d\r\n",fclk,psc,reloadVal);
	// }
	if((AL.cycleT == 0 && AL.movectrl == LOAD_MODE) || AL.movectrl == POS_MODE){
		// HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);//PULSE
		// __HAL_TIM_PRESCALER(&htim8,  (uint16_t)psc);//modfiy  psc
		// __HAL_TIM_SetAutoreload(&htim8 , (uint16_t)reloadVal);//modfiy reload
		// __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, (uint16_t)reloadValHalf);//modify compare
		// HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);//PULSE
		htim8.Init.Prescaler = psc;
		htim8.Init.Period =reloadVal;
		sConfigOC.Pulse = reloadValHalf;
		test.pcs = psc;
		test.reloadVal = reloadVal;
		test.reloadValHalf = reloadValHalf;
		test.fclk = fclk;

	htim8.Init.RepetitionCounter = 0;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_TIM_MspPostInit(&htim8);
	}
	test.fclk = (__HAL_TIM_GetCounter(&htim8));
	Servo_PWM_Enable(PULSE);
}

void modFclkNumberPULSEInit(uint32_t fclk){
TIM_OC_InitTypeDef sConfigOC = {0};
	uint16_t reloadVal,reloadValHalf,psc;
	psc = 350-1;
	reloadVal = (uint16_t) ((TIM8MCLK/((psc+1)*1.0f)/(fclk*1.0f)) - 1);
	reloadValHalf =(uint16_t) (reloadVal*0.5);

	htim8.Init.Prescaler = psc;
	htim8.Init.Period =reloadVal;
	sConfigOC.Pulse = reloadValHalf;
	test.pcs = psc;
	test.reloadVal = reloadVal;
	test.reloadValHalf = reloadValHalf;
	test.fclk = fclk;

	htim8.Init.RepetitionCounter = 0;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_TIM_MspPostInit(&htim8);
}

void modFclkNumberPULSE(uint32_t fclk)
{
	// uint16_t psc;
	// static uint8_t i = 0;
	if(AL.cycleT == 0 && AL.movectrl == POS_MODE){
			if(fclk >= 1000){
				hal_output.pwmCounter = fclk / 1000;
				hal_output.pwmIntCounter = 0;
				HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);//PULSE
			}

			if(fclk < 1000){
				HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);//PULSE
				hal_output.pwmCounter = 0;
				hal_output.pwmIntCounter = 0;
			}
		//}
	}else if (AL.movectrl == LOAD_MODE)
	{
		if(fclk >= 1000){
			hal_output.pwmCounter = fclk / 1000;
			hal_output.pwmIntCounter = 0;
			HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);//PULSE
		}

		if(fclk < 1000){
			HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);//PULSE
			hal_output.pwmCounter = 0;
			hal_output.pwmIntCounter = 0;
		}
	}
}

void modFclkPULSE(uint32_t fclk)
{
	uint16_t reloadVal,reloadValHalf,psc;
	// static uint8_t i = 0;
	if(fclk >= 1 && fclk <= 100){
		psc = 50000-1;
		reloadVal = (uint16_t) ((TIM8MCLK/((psc+1)*1.0f)/(fclk*1.0f)) - 1);
		reloadValHalf =(uint16_t) (reloadVal*0.5);
	}else if(fclk <= 1000){
		psc = 40-1;
		reloadVal = (uint16_t) ((TIM8MCLK/((psc+1)*1.0f)/(fclk*1.0f)) - 1);
		reloadValHalf =(uint16_t) (reloadVal*0.5);
	}else if(fclk <= 20000){
		psc = 4-1;
		reloadVal = (uint16_t) ((TIM8MCLK/((psc+1)*1.0f)/(fclk*1.0f)) - 1);
		reloadValHalf =(uint16_t) (reloadVal*0.5);
	}else{
		psc = 0;
		reloadVal = (uint16_t) ((TIM8MCLK/((psc+1)*1.0f)/(fclk*1.0f)) - 1);
		reloadValHalf =(uint16_t) (reloadVal*0.5);
	}

	// if(i++ >= 10){
	// 	i = 0;
	// printf("%d,%d,%d\r\n",fclk,psc,reloadVal);
	// }
	HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);//PULSE
	__HAL_TIM_PRESCALER(&htim8,  (uint16_t)psc);//modfiy  psc
	__HAL_TIM_SetAutoreload(&htim8 , (uint16_t)reloadVal);//modfiy reload
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, (uint16_t)reloadValHalf);//modify compare
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);//PULSE
	if(fclk < 1){
		HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);//PULSE
		__HAL_TIM_PRESCALER(&htim8,  (uint16_t)49999);//modfiy  psc
		__HAL_TIM_SetAutoreload(&htim8 , (uint16_t)3999);//modfiy reload
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, (uint16_t)2000);//modify compare
	}
	test.pcs = psc;
	test.reloadVal = reloadVal;
	test.reloadValHalf = reloadValHalf;
	test.fclk = fclk;
}


void modFclkPeriod(float fclk)
{
	TIM_OC_InitTypeDef sConfigOC = {0};
	uint16_t reloadVal,reloadValHalf,psc;
	if(fclk >= 1){
		if(fclk >= 1 && fclk <= 1000){
			psc = 5000-1;
			reloadVal = (uint16_t) ((TIM8MCLK/((psc+1)*1.0f)/(fclk*1.0f)) - 1);
			if(reloadVal > 65534) reloadVal = 65534;
			reloadValHalf = reloadVal/2;
		}else if(fclk <= 10000){
			psc = 200-1;
			reloadVal = (uint16_t) ((TIM8MCLK/((psc+1)*1.0f)/(fclk*1.0f)) - 1);
			if(reloadVal > 65534) reloadVal = 65534;
			reloadValHalf = reloadVal/2;
		}else if(fclk <= 20000){
			psc = 4-1;
			reloadVal = (uint16_t) ((TIM8MCLK/((psc+1)*1.0f)/(fclk*1.0f)) - 1);
			reloadValHalf = reloadVal/2;
		}else{
			psc = 0;
			reloadVal = (uint16_t) ((TIM8MCLK/((psc+1)*1.0f)/(fclk*1.0f)) - 1);
			reloadValHalf = reloadVal/2;
		}
		test.fr = 1.0f * TIM8MCLK / ((psc + 1)*1.0f)/((reloadVal + 1)*1.0f);
		test.pcs = psc;
		test.reloadVal = reloadVal;
		test.reloadValHalf = reloadValHalf;

		htim8.Init.Prescaler = psc;
		htim8.Init.Period =reloadVal;
		sConfigOC.Pulse = reloadValHalf;
		test.pcs = psc;
		test.reloadVal = reloadVal;
		test.reloadValHalf = reloadValHalf;
		test.fclk = fclk;

		htim8.Init.RepetitionCounter = 0;
		if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
		{
			Error_Handler();
		}
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
		if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
		{
			Error_Handler();
		}
		HAL_TIM_MspPostInit(&htim8);
		//test.fclk = (__HAL_TIM_GetCounter(&htim8));
		Servo_PWM_Enable(PULSE);
	}else if(fclk < 1){
		// htim8.Init.Prescaler = psc;
		// htim8.Init.Period = 150;
		// sConfigOC.Pulse = 75;
		// htim8.Init.RepetitionCounter = 0;
		// if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
		// {
		// 	Error_Handler();
		// }
		// sConfigOC.OCMode = TIM_OCMODE_PWM1;
		// sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		// sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		// sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		// sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		// sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
		// test.fclk = fclk;
		// if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
		// {
		// 	Error_Handler();
		// }
		// HAL_TIM_MspPostInit(&htim8);
		Servo_PWM_Disable(PULSE);
	}


}

float pulseHaveNoOut(const float originalPeriod){
	uint32_t remainingRCR = 0;
	Servo_PWM_Disable(PULSE);
	remainingRCR = htim8.Instance->RCR + 1;
	// 3. 计算剩余脉冲数（原配置为10ms输出1个脉冲）
	// int pulses_emitted = (int)(elapsed_time_ms / originalPeriod);
	// int pulses_remaining = 1 - pulses_emitted;
	return remainingRCR;
}

uint32_t SFH_fclk_1[6] = {SFH_fclk6000_1,SFH_fclk2000_1,SFH_fclk100_1,SFH_fclk50_1,SFH_fclk4_1,SFH_fclk0_1};
uint32_t SFH_fclk_2[6] = {SFH_fclk6000_2,SFH_fclk2000_2,SFH_fclk100_2,SFH_fclk50_2,SFH_fclk4_2,SFH_fclk0_2};
uint8_t servoRegionSelcet(const uint8_t _nowRegion,const float _fclk);

void modServoFclk(float fclk)
{
	TIM_OC_InitTypeDef sConfigOC = {0};
	uint16_t reloadVal,reloadValHalf,psc;
	static uint16_t pscLast = 0;
	static uint8_t sfhRegion = SF_pcs6000;
	float outFclk = 0;
	// static int64_t fclkInt64 = 0,
	// 			totalInt64 = 0,
	// 			remainderInt64 = 0,
	// 			outInt64 = 0;
	// double outDouble = 0;
	
	//计算当前周期理论脉冲数
	// fclkInt64 = (int64_t)(fclk * 10000.0f);
	// totalInt64 = (double)fclkInt64 / 1000.0 + remainderInt64;
	// outInt64 = totalInt64 / 10000;
	// remainderInt64 = totalInt64 - outInt64 * 10000;

	// servoPulse.total = totalInt64 /10000.0f;
	// servoPulse.out = outInt64 / 10000.0f;
	// servoPulse.remainder = remainderInt64 / 10000.0f;
	// outFclk = servoPulse.out * 1000.0f;

	//未输出完的脉冲累积到remainder
	//servoPulse.remainder += pulseHaveNoOut(0);
	//计算当前周期理论脉冲数
	servoPulse.total = fclk * 0.001f + servoPulse.remainder;
	servoPulse.out = (uint32_t)(servoPulse.total);
	servoPulse.remainder = servoPulse.total - servoPulse.out;
	outFclk = servoPulse.out * 1000.0f;

	test.monitor4 = (float)servoPulse.total;
	test.monitor5 = (float)servoPulse.out;
	test.monitor6 = (float)servoPulse.remainder;

	if(servoPulse.out > 0){
		sfhRegion = servoRegionSelcet(sfhRegion,outFclk);
		switch (sfhRegion)
		{
		case SF_pcs6000:
			pscLast = psc;
			psc = 5999;
			reloadVal = (uint16_t) ((TIM8MCLK/((psc+1)*1.0f)/(outFclk*1.0f)) - 1);
			if(reloadVal > 65534) reloadVal = 65534;
			reloadValHalf = reloadVal/2;
			break;
		case SF_pcs2000:
			pscLast = psc;
			psc = 1999;
			reloadVal = (uint16_t) ((TIM8MCLK/((psc+1)*1.0f)/(outFclk*1.0f)) - 1);
			if(reloadVal > 65534) reloadVal = 65534;
			reloadValHalf = reloadVal/2;
			break;
		case SF_pcs100:
			pscLast = psc;
			psc = 99;
			reloadVal = (uint16_t) ((TIM8MCLK/((psc+1)*1.0f)/(outFclk*1.0f)) - 1);
			if(reloadVal > 65534) reloadVal = 65534;
			reloadValHalf = reloadVal/2;
			break;
		case SF_pcs50:
			pscLast = psc;
			psc = 49;
			reloadVal = (uint16_t) ((TIM8MCLK/((psc+1)*1.0f)/(outFclk*1.0f)) - 1);
			if(reloadVal > 65534) reloadVal = 65534;
			reloadValHalf = reloadVal/2;
			break;
		case SF_pcs4:
			pscLast = psc;
			psc = 3;
			reloadVal = (uint16_t) ((TIM8MCLK/((psc+1)*1.0f)/(outFclk*1.0f)) - 1);
			if(reloadVal > 65534) reloadVal = 65534;
			reloadValHalf = reloadVal/2;
			break;
		case SF_pcs0:
			pscLast = psc;
			psc = 0;
			reloadVal = (uint16_t) ((TIM8MCLK/((psc+1)*1.0f)/(outFclk*1.0f)) - 1);
			if(reloadVal > 65534) reloadVal = 65534;
			reloadValHalf = reloadVal/2;
			break;
		default:
			break;
		}

		test.monitor1 = (float)psc;
		test.monitor2 = (float)reloadVal;
		test.monitor3 = (float)reloadValHalf;
		test.fclk = fclk;
		
		if(psc != pscLast){
			__HAL_TIM_SET_COUNTER(&htim8,0);
			htim8.Init.Prescaler = psc;
			htim8.Init.Period =reloadVal;
			sConfigOC.Pulse = reloadValHalf;
			//htim8.Init.RepetitionCounter = servoPulse.out - 1;
			// test.pcs = psc;
			// test.reloadVal = reloadVal;
			// test.reloadValHalf = reloadValHalf;
			// test.fclk = fclk;
			// if (HAL_TIM_OnePulse_Init(&htim8, TIM_OPMODE_SINGLE) != HAL_OK) {
			// 	Error_Handler();
			//   } 
			if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
			{
				Error_Handler();
			} 
			sConfigOC.OCMode = TIM_OCMODE_PWM1;
			sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
			sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
			sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
			sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
			sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
			if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
			{
				Error_Handler();
			}
			HAL_TIM_MspPostInit(&htim8);
			//test.fclk = (__HAL_TIM_GetCounter(&htim8));
		}else{
			Servo_PWM_Disable(PULSE);
			// //配置PSC预分频值
			// __HAL_TIM_SET_PRESCALER(&htim8, psc);
			__HAL_TIM_SET_COUNTER(&htim8, 0);
			//配置PWM频率 ARR
			__HAL_TIM_SetAutoreload(&htim8, reloadVal);
			//配置PWM占空比
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, reloadValHalf);//modify compare
			//设置RCR
			//htim8.Instance->RCR = servoPulse.out - 1;
		}
		Servo_PWM_Enable(PULSE);
	}else{
		test.fclk = fclk;
		Servo_PWM_Disable(PULSE);
	}
}

uint8_t servoRegionSelcet(const uint8_t _nowRegion,const float _fclk){
	uint8_t _dir = 2,i;//dir: 0 means left <-  ,1 means right ->,2 means no change
	switch (_nowRegion)
	{
	case SF_pcs6000:
		if(_fclk > SFH_fclk6000_2){
			_dir = 1;
		}
		break;
	case SF_pcs2000:
		if(_fclk < SFH_fclk2000_1){
			_dir = 0;
		}else if(_fclk > SFH_fclk2000_2){
			_dir = 1;
		}
		break;
	case SF_pcs100:
		if(_fclk < SFH_fclk100_1){
			_dir = 0;
		}else if(_fclk > SFH_fclk100_2){
			_dir = 1;
		}
		break;
	case SF_pcs50:
		if(_fclk < SFH_fclk50_1){
			_dir = 0;
		}else if(_fclk > SFH_fclk50_2){
			_dir = 1;
		}
		break;
	case SF_pcs4:
		if(_fclk < SFH_fclk4_1){
			_dir = 0;
		}else if(_fclk > SFH_fclk4_2){
			_dir = 1;
		}
		break;
	case SF_pcs0:
		if(_fclk < SFH_fclk0_1){
			_dir = 0;
		}
		break;
	default:
		break;
	}
	// dir: 0 means left ;1 means right
	if(_dir == 0){
		for(i = _nowRegion; i > 0; i--){
			if(_fclk >= SFH_fclk_1[i - 1]){
				i = i - 1;
				break;
			}
		}
	}else if(_dir == 1){
		for(i = _nowRegion+1;i<6;i++){
			if(_fclk <= SFH_fclk_2[i]){
				break;
			}
		}
	}
	return i;
}



