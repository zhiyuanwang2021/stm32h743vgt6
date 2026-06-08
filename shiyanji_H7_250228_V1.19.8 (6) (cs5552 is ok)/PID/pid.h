#ifndef _PID_H_
#define _PID_H_
#include "stdint.h"


typedef struct{
    uint8_t index;
    uint8_t calcuRun;           //calculate PID output
    uint32_t uMAx;
    uint32_t uMin;
    float out;                  //output
    float outIntegral;
    float outIntegralRecord;
    double outMAx;
    double outMin;
    double outdeta;
    double endValue;
    double setValue;             //set value
    double actualValue;          //actual value
    double err;                  //error
    double errLast;              //last error
    double kp,ki,kd;             //p i d parameter
    double kpSeparate,kiSeparate,kdSeparate;             //p i d parameter
    double integral;             //intergral item
	double iThreshold;
}pid_t;

extern pid_t pid_pos,pid_load,pid_ext;

void pidInitPos(void);
void pidInitLoad(void);
void pidInitExt(void);
void pidParaUpdatePos(void);
void pidParaUpdateLoad(void);
void pidParaUpdateExt(void);

float pidCalPos(float setValue,float realValue,pid_t* pid);
float pidCalLoad(double setValue,double realValue,pid_t* pid);
float pidCalExt(double setValue,double realValue,pid_t* pid);

#endif
