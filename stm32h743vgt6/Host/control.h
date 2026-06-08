#ifndef  _CONTROL_H_
#define  _CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif
	
#include "stm32h7xx_hal.h"
//#include "pid.h"
	


//extern PIDCONTROLLER posPID,pressPID,forcePID;
#ifdef __cplusplus
}
#endif

extern float MAINTANCE_test;

void controlLoop(void);
void EnableState_Update(void);
void ETH_HeartbeatWatchdog(uint8_t* flag,uint16_t threshold);
void CommandUpdate(void);

#endif
