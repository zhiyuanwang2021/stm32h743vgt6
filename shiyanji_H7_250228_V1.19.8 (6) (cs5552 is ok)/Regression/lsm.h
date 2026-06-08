#ifndef _LSM_H_
#define _LSM_H_

#include "main.h"

#define LSM_DEPTH 500

typedef struct{
	uint32_t i_xm;
	uint32_t i_ym;
	uint32_t i_ls;
	double x_m;
	double y_m;
	double xytemp;
	double x2temp;
	double x_mtemp;
	double y_mtemp;
	double x_mbuf[LSM_DEPTH];
	double y_mbuf[LSM_DEPTH];
	double x_buf[LSM_DEPTH];
	double y_buf[LSM_DEPTH]; 
	double xytempbuf[LSM_DEPTH];
	double x2tempbuf[LSM_DEPTH];
	double k;
	double kFilter;
	double b;
}LS_SLIDE_STRUCT;
extern LS_SLIDE_STRUCT ls_slide_t;
extern LS_SLIDE_STRUCT lsRigidity;
extern LS_SLIDE_STRUCT lsSlideExt;
extern LS_SLIDE_STRUCT lsSlideExt2;

typedef struct{
	double y_pre;//predicte y(n)
	double y_pre_last;//predicte y(n)
	double y_expect;//expect y(n+1)
	double delta_x;//delta x(n+1)
	double k;
}LSM_PREDICTOR_STRUCT;
extern LSM_PREDICTOR_STRUCT lsm_pre_load;
extern LSM_PREDICTOR_STRUCT lsm_pre_rigidity;
extern LSM_PREDICTOR_STRUCT lsm_pre_ext;
extern LSM_PREDICTOR_STRUCT lsm_pre_ext2;

typedef struct{
	uint16_t loadLen;//mid average filter
	uint16_t loadLenLs;//least square method
	uint16_t strainLen;//mid average filter
	uint16_t strainLenLs;//least square method
	uint16_t strain2Len;//mid average filter
	uint16_t strain2LenLs;//least square method
}FILTER_LEN_STRUCT;
extern FILTER_LEN_STRUCT filterLen;

typedef struct {
    double S_xy;   // 保存x*y的加权和
    double S_xx;   // 保存x2的加权和
	double k;
} RLSState_t;
extern RLSState_t rigidityRLS;

extern double mean_fast(uint32_t num,uint32_t* i,double* mtemp,double* m_buf,double input);
extern double least_square_fast(uint32_t num,LS_SLIDE_STRUCT* ls,double x,double y);
extern uint16_t sigmodLen(double a,double k,uint16_t maxLen);
uint16_t switchLen(double a,double k,uint16_t maxLen,uint16_t lsLen,float nominalValue,float input);
double leastSquareStandard(uint32_t num,LS_SLIDE_STRUCT* ls,double x,double y);
double rigidityUpdateRLS(RLSState_t *state,const double x,const double y,const double c);

#endif //_LSM_H_



