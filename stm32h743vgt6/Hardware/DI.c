#include "DI.h"
#include "main.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <elog.h>
#include "sensor.h"

HAL_DI_STRUCT halDI;
DITypeDef DI;
HAL_DB9_STRUCT halDB9;
DB9Typedef DB9;

GPIO_PinState DIDetect(GPIO_PinState PinState,DI_DETECT_STRUCT* x,GPIO_PinState PinStatePress);

//DI dectect to eliminate the dither of DI
void DIDetect_ReadALL(void){
	DIDetect(HAL_GPIO_ReadPin(P_DI1_GPIO_Port,P_DI1_Pin),&halDI.DI1,GPIO_PIN_RESET);
	DIDetect(HAL_GPIO_ReadPin(P_DI2_GPIO_Port,P_DI2_Pin),&halDI.DI2,GPIO_PIN_RESET);
	DIDetect(HAL_GPIO_ReadPin(P_DI3_GPIO_Port,P_DI3_Pin),&halDI.DI3,GPIO_PIN_RESET);
	DIDetect(HAL_GPIO_ReadPin(P_DI4_GPIO_Port,P_DI4_Pin),&halDI.DI4,GPIO_PIN_RESET);
	DIDetect(HAL_GPIO_ReadPin(P_DI5_GPIO_Port,P_DI5_Pin),&halDI.DI5,GPIO_PIN_RESET);
	DIDetect(HAL_GPIO_ReadPin(P_DI6_GPIO_Port,P_DI6_Pin),&halDI.DI6,GPIO_PIN_RESET);
	DIDetect(HAL_GPIO_ReadPin(P_DI7_GPIO_Port,P_DI7_Pin),&halDI.DI7,GPIO_PIN_RESET);
	DIDetect(HAL_GPIO_ReadPin(P_DI8_GPIO_Port,P_DI8_Pin),&halDI.DI8,GPIO_PIN_RESET);
	DIDetect(HAL_GPIO_ReadPin(P_DI9_GPIO_Port,P_DI9_Pin),&halDI.DI9,GPIO_PIN_RESET);
	DIDetect(HAL_GPIO_ReadPin(P_DI10_GPIO_Port,P_DI10_Pin),&halDI.DI10,GPIO_PIN_RESET);
	DIDetect(HAL_GPIO_ReadPin(P_DI11_GPIO_Port,P_DI11_Pin),&halDI.ALM,GPIO_PIN_RESET);
	halDI.ALL = (halDI.ALM.value<<10) + (halDI.DI10.value<<9) + (halDI.DI9.value<<8) + (halDI.DI8.value<<7) 
	+ (halDI.DI7.value<<6) + (halDI.DI6.value<<5) + (halDI.DI5.value<<4) + (halDI.DI4.value<<3) + (halDI.DI3.value<<2) + (halDI.DI2.value<<1) + halDI.DI1.value;
	//printf("%d,%d,%d\r\n",halDI.DI3.value,halDI.DI3.pressTime,halDI.DI3.stateMachine);
}

//DB9 CON and ADJ dectect to eliminate the dither of DI
void DB9Detect_ReadALL(void){
//sensor connect check by CON
#if USE_DB9_CON
	DIDetect(HAL_GPIO_ReadPin(P_CON1_GPIO_Port,P_CON1_Pin),&halDB9.CON1,GPIO_PIN_SET);
	DIDetect(HAL_GPIO_ReadPin(P_CON2_GPIO_Port,P_CON2_Pin),&halDB9.CON2,GPIO_PIN_SET);
	DIDetect(HAL_GPIO_ReadPin(P_CON3_GPIO_Port,P_CON3_Pin),&halDB9.CON3,GPIO_PIN_SET);
#else //sensor connect check by EEPROM
	//sensorConnectCheckByEeprom(&halDB9);
#endif
	DIDetect(HAL_GPIO_ReadPin(P_ADJ1_GPIO_Port,P_ADJ1_Pin),&halDB9.ADJ1,GPIO_PIN_SET);
	DIDetect(HAL_GPIO_ReadPin(P_ADJ2_GPIO_Port,P_ADJ2_Pin),&halDB9.ADJ2,GPIO_PIN_SET);
	DIDetect(HAL_GPIO_ReadPin(P_ADJ3_GPIO_Port,P_ADJ3_Pin),&halDB9.ADJ3,GPIO_PIN_SET);
	halDB9.ALL = (halDB9.CON3.value<<5)+(halDB9.ADJ3.value<<4)+(halDB9.CON2.value<<3)+(halDB9.ADJ2.value<<2)+(halDB9.CON1.value<<1)+halDB9.ADJ1.value;
	//printf("%d,%d,%d\r\n",halDB9.CON1.value,halDB9.CON1.pressTime,halDB9.CON1.stateMachine);
}

#define NO_PRESS 0 			//no pressed
#define DELAY_PRESS 1		//eliminate dither when detect press
#define YES_PRESS 2			//has been pressed
#define DELAY_UNPRESS 3		//eliminate dither when detect unpress
/**
 * @brief the state machine is used to eliminate the dither of DI
 * @param[in]		PinState Pinstate of halDI
 * @param[in]		x DI_DETECT_STRUCT,Used to handle state machine processes锛?
 * 					record press time and pinstate after eliminating dither
 * @param[in]		PinStatePress the state after DI being pressed
*/
GPIO_PinState DIDetect(GPIO_PinState PinState,DI_DETECT_STRUCT* x,GPIO_PinState PinStatePress){
    uint32_t TimTmp;
    TickType_t currentTick;
    GPIO_PinState PinStateRelease;

    currentTick = xTaskGetTickCount();
    PinStateRelease = (PinStatePress == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    switch (x->stateMachine)
    {
    case NO_PRESS:
        x->value = PinStateRelease;
        x->unpressTime += (currentTick - x->timRecordNoPress);
        if(x->unpressTime > 4000000000U)
            x->unpressTime = 4000000000U;
        x->timRecordNoPress = currentTick;
        if(PinState == PinStatePress){
            x->stateMachine = DELAY_PRESS;
            x->timRecord = currentTick;
        }
        break;
    case DELAY_PRESS:
        x->value = PinStateRelease;
        x->unpressTime += (currentTick - x->timRecordNoPress);
        if(x->unpressTime > 4000000000U)
            x->unpressTime = 4000000000U;
        x->timRecordNoPress = currentTick;
        TimTmp = currentTick - x->timRecord;
        if(TimTmp > 15U){
            x->timRecord = 0;
            if(PinState == PinStatePress){
                x->stateMachine = YES_PRESS;
                x->timRecordYesPress = currentTick;
            //    log_d("YES_PRESS");
            }
            else
                x->stateMachine = NO_PRESS;
        }
        break;
    case YES_PRESS:
        x->value = PinStatePress;
        x->pressTime += (currentTick - x->timRecordYesPress);
        if(x->pressTime > 4000000000U)//4,000,000s 鈮?46days
            x->pressTime = 4000000000U;
        x->timRecordYesPress = currentTick;
        if(PinState == PinStateRelease){
            x->timRecord = currentTick;
            x->stateMachine = DELAY_UNPRESS;
        }
        break;
    case DELAY_UNPRESS:
        x->value = PinStatePress;
        x->pressTime += (currentTick - x->timRecordYesPress);
        if(x->pressTime > 4000000000U)
            x->pressTime = 4000000000U;
        x->timRecordYesPress = currentTick;
        TimTmp = currentTick - x->timRecord;
        if(TimTmp > 15U){
            x->timRecord = 0;
            if(PinState == PinStateRelease){
                x->stateMachine = NO_PRESS;
                x->pressTime = 0;
                x->timRecordYesPress = 0;
            //    log_d("UNPRESS");
            }
            else
                x->stateMachine = YES_PRESS;
        }
        break;
    default:
        break;
    }
    return x->value;
 }








