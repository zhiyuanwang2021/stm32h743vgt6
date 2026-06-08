#ifndef _ADRC_H_
#define _ADRC_H_

#include "stdint.h"

typedef struct
{
uint16_t N0;//����΢��������ٶȳ���h0=N*h
double x1;//����΢����״̬��
double x2;//����΢����״̬��΢����
double r;//ʱ��߶�?
double h;//ADRCϵͳ����ʱ��
double h0;
double fh;//����΢�ּ��ٶȸ�����
}FHAN_STRUCT;

extern FHAN_STRUCT fhan;

typedef struct 
{
   double beta1;
   double beta2;
   double z1;
   double z2;
   double e;
   double fe1;
   double fe2;
   double u;
   double y;
   double h;
   double uPos;
}leso_t;

extern leso_t leso;

void adrcLESO(double U,double Y,double B0,double H,leso_t* Leso);
void adrcFhanInputUpdate(FHAN_STRUCT *fhan,double x1,double x2,double r,double h);
float adrcFhan(FHAN_STRUCT *fhan_Input);
void adrcInit(leso_t* Leso,float initPose);
void adrcControlLawLoadInterlPos(float setPose,float realPose,float setSpeed,float realSpeed);
void adrcControlLawPos(float setPose,float realPose,float setSpeedMax,float setSpeed,float realSpeed);
void adrcControlLawPoswithCreep(const float endPose,const float setPose,const float realPose,
                        const float setSpeedMax,const float setSpeed,const float realSpeed);
#endif 
