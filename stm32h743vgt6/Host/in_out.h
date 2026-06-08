#ifndef  _IN_OUT_H_
#define  _IN_OUT_H_
#include "DO.h"
#include "DI.h"
#include "gParameter.h"
typedef struct{
	DOTypeDef 	DO;
	//uint8_t 	DO;
	float 	PWM;
	float 	PWM_last;
	int32_t 	AO;
	uint32_t 	pwmCounter;
	uint32_t 	pwmIntCounter;
}HAL_OUTPUT;

extern HAL_OUTPUT hal_output;

typedef struct{
	int32_t code;
	int32_t code_last;
	int32_t codeErr;
	int32_t codeFiter;
	int32_t codeFiter_last;
	float	orig;
	float	origTare;
	float	filterTransTare;
	float	orig_last;
	float	filter;
	float   filterTrans;  //transmit to PC
	float	filter_last;
	float	pre;
}MDSTRUCT;//measure data

extern MDSTRUCT pose;
extern MDSTRUCT speedPose;
extern MDSTRUCT bigDeformationUpper;
extern MDSTRUCT bigDeformationLower;
extern MDSTRUCT bigDeformationSum;
extern MDSTRUCT force;
extern MDSTRUCT forceLsm;
extern MDSTRUCT speedForce;
extern MDSTRUCT strain1;
extern MDSTRUCT strain2;
extern MDSTRUCT strain1Lsm;
extern MDSTRUCT strain2Lsm;
extern MDSTRUCT speedStrain;
extern MDSTRUCT forceSL2;

typedef struct{
	void (*func)(PINSTATE_MAP_STRUCT*);
}DIMAPSTRUCT;

extern DIMAPSTRUCT DIMap[10];

typedef enum{
	manualBoxDI_Type = 0,
	manualBox485_Type
}manualBoxType_e;

typedef struct{
	uint8_t type;
	float speed;
	float rapidSpeed;
	void(*up)(float speed);
	void(*down)(float speed);
	void(*reset)(float speed);
	void(*halt)(void);
}MANUAL_BOX_STRUCT;

extern MANUAL_BOX_STRUCT manualBox;

void inputGetValue(void);
void inputMapping(void); 
void outputMapping(void);
void outputControl(void);
void output_param_update(void);
void poseCalcu(void);

void manualBoxInit(void);
void manualBoxTypeMonitor(MANUAL_BOX_STRUCT *_manualBox);

void poseCodeCalculate_Int(MDSTRUCT *pose,const int32_t count);
void poseSpeedFilter_Int(FILTER_INT *_filter,MDSTRUCT *_pose,MDSTRUCT *_speedPose,const uint8_t _filter_depth);

void openloop_parameters_update(void);

#endif
