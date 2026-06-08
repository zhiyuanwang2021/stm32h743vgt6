#ifndef _DO_H_
#define _DO_H_
#include "stm32h7xx_hal.h"
#include "gpio.h"
#include "main.h"

typedef struct{
GPIO_PinState DO1;
GPIO_PinState DO2;
GPIO_PinState DO3;
GPIO_PinState DO4_RL;	
GPIO_PinState DO5_RL;
GPIO_PinState SON;
GPIO_PinState DIR;
}DOTypeDef;
extern DOTypeDef DO;

void Relay_Write(uint16_t GPIO_Pin,GPIO_PinState PinState);
void Darlington_Write(uint16_t GPIO_Pin,GPIO_PinState PinState);
//void Optocoupler_Write(uint16_t GPIO_Pin,GPIO_PinState PinState);


#endif

