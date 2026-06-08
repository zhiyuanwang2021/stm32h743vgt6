#include "DO.h"

DOTypeDef DO;
//继电器设置
void Relay_Write(uint16_t GPIO_Pin,GPIO_PinState PinState)
{
	switch(GPIO_Pin)
	{
		case P_DO4_RL_Pin:
	 HAL_GPIO_WritePin(P_DO4_RL_GPIO_Port, P_DO4_RL_Pin,PinState);
		DO.DO4_RL = PinState;
		break;
		case P_DO5_RL_Pin:
		HAL_GPIO_WritePin(P_DO5_RL_GPIO_Port, P_DO5_RL_Pin,PinState);
		DO.DO5_RL = PinState;
		break;
		default:break;
	}
}
//达林顿驱动管设置
void Darlington_Write(uint16_t GPIO_Pin,GPIO_PinState PinState)
{
	switch(GPIO_Pin)
	{
		case P_DO1_Pin:
	 HAL_GPIO_WritePin(P_DO1_GPIO_Port, P_DO1_Pin,PinState);
		DO.DO1 = PinState;
		break;
		case P_DO2_Pin:
		HAL_GPIO_WritePin(P_DO2_GPIO_Port, P_DO2_Pin,PinState);
		DO.DO2 = PinState;
		break;
		case P_DO3_Pin:
		HAL_GPIO_WritePin(P_DO3_GPIO_Port, P_DO3_Pin,PinState);
		DO.DO3 = PinState;
		break;
		default:break;
	}
}
//光耦设置
// void Optocoupler_Write(uint16_t GPIO_Pin,GPIO_PinState PinState)
// {
// 	switch(GPIO_Pin)
// 	{
// 		case P_DO6_Pin:
// 	 HAL_GPIO_WritePin(P_DO6_GPIO_Port, P_DO6_Pin,PinState);
// 		break;
// 		default:break;
// 	}
// }

