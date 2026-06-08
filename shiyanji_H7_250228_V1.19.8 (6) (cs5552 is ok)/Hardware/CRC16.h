#ifndef _CRC16_H_
#define _CRC16_H_
#include "stm32h7xx_hal.h"
#include <stdbool.h>

uint16_t CRC16(uint8_t *puchMsg, uint16_t usDataLen);
	
#endif /* _CRC16_H_*/
