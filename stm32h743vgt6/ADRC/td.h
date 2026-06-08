#ifndef _TD_H_
#define _TD_H_

#include <math.h>
#include <string.h>
#include <stdint.h>
#include "main.h"
 
#define timeradio 200000.f  //200000000/1000
#define sign(x) ((x)>0?1:((x)<0?-1:1))
/*****************************/
typedef struct 
{
	float Target; //Ŀ��ֵ
	float Lastnot0_target;  // ���һ�η����Ŀ��ֵ
	struct 
	{
		float R1;  // ����΢������ǰ����ֵ
		float R2;  // ����΢������ǰ�����ٶ�
		float V1;  // ����΢����Ԥ���ĸ����ٶ�
		float V2;  // ����΢����Ԥ���ĸ��ټ��ٶ�
		float R;   // *����΢�����������ٶ�
	}td;  // ����΢�����ӽṹ
   struct 
	{
		uint32_t Time_p;  // �ϴο��Ƶ�ʱ��
		uint32_t Time_n;  // ���ο��Ƶ�ʱ��
		float Dtime;      // ���ο���֮���ʱ��
	}Time;
}TD_conctrol;

extern TD_conctrol load_td;
extern float load_tdvalue;
extern float loadspeed_tdvalue;
 
static void getTimeStamp(TD_conctrol * TD); // ��¼��ǰ��ʱ���
static void TD_cal(TD_conctrol * TD);  // �������΢��������
static void TD_inputStatus(TD_conctrol * TD, float target);  // ���뵱ǰ״̬
void TD_Run(TD_conctrol * TD, float target);  //����Ŀ��ֵ����
static void reset_TD(TD_conctrol * TD, float new_R);      	// �������΢��������
static void TD_param_init(TD_conctrol * TD, float R);
float TD_get_R1(TD_conctrol * TD);
float TD_get_R2(TD_conctrol * TD);
void create_TD(TD_conctrol* TD,float R);
 
#endif

