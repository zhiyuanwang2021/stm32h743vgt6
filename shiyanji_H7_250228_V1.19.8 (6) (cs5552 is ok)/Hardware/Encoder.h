#ifndef  _ENCODER_H_
#define  _ENCODER_H_
#include "stm32h7xx_hal.h"
#include "stdint.h"

#define Encoder0 0
#define Encoder1 1
#define Encoder2 2

typedef struct{
int32_t count0;
int32_t count1;
int32_t count2;
} ENCODER_STRUCT;
extern ENCODER_STRUCT encoder;

void Encoder_Init(void);
void Encoder_Get(uint8_t channel,ENCODER_STRUCT *encoder);
void encoderCountInit(int32_t count0,int32_t count1,int32_t count2,ENCODER_STRUCT *encoder_struct);


#endif

