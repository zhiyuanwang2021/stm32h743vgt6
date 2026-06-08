#include "lsm.h"
#include "in_out.h"

LS_SLIDE_STRUCT ls_slide_t;
LS_SLIDE_STRUCT lsRigidity;
LS_SLIDE_STRUCT lsSlideExt;
LS_SLIDE_STRUCT lsSlideExt2;
LSM_PREDICTOR_STRUCT lsm_pre_load;
LSM_PREDICTOR_STRUCT lsm_pre_rigidity;
LSM_PREDICTOR_STRUCT lsm_pre_ext;
LSM_PREDICTOR_STRUCT lsm_pre_ext2;
FILTER_LEN_STRUCT filterLen;
RLSState_t rigidityRLS;

double mean_fast(uint32_t num,uint32_t* i,double* mtemp,double* m_buf,double input)
{
 	(*i)++;
 	if((*i) >= num) *i=0;
	 
	*mtemp -= m_buf[*i];
	m_buf[*i] = input;
	*mtemp += m_buf[*i];
	
	 return ((*mtemp)/(double)num);
}
 
 double least_square_fast(uint32_t num,LS_SLIDE_STRUCT* ls,double x,double y)
 {
 	uint32_t i=0,x_i=0;
 	double den;//denominator
 	double y_pre;
 	
 	ls->y_buf[ls->i_ls] = y;
 		
 	ls->xytemp = 0;
 	for(i=0;i<num;i++)
 	{
		x_i = (i + 1 + num - ls->i_ls)%num;
 		if(x_i == 0) x_i = num;
 		ls->xytemp += ls->y_buf[i]*(double)x_i;
	}
	ls->x_m = ((double)num+1.0f)/2.0f;
 	ls->x2temp = (double)num*((double)num+1.0f)*(2.0f*(double)num+1.0f)/6.0f;
 	//printf("x2sum=%f",ls->x2temp);
 	den = ls->x2temp - (double)num*ls->x_m*ls->x_m;
 	//printf("den=%f,xytemp=%f,nxy=%f\n",den,ls->xytemp,(double)num*ls->x_m*ls->y_m);
	if(den >= 0 && den < 1e-8) den = 1e-8;
	else if(den <0 && den>-1e-8) den = -1e-8;
	
	ls->k = (ls->xytemp - (double)num*ls->x_m*ls->y_m)/den;
	if(ls->k >= 0 && ls->k < 1e-8) ls->k = 1e-8;
	else if(ls->k <0 && ls->k>-1e-8) ls->k = -1e-8;
	
	ls->b = ls->y_m - ls->k*ls->x_m;
	
	y_pre = ls->k * num + ls->b;
	
	(ls->i_ls)++;
 	if((ls->i_ls) >= num) ls->i_ls=0;

	return y_pre;	
 }

uint16_t sigmodLen(double a,double k,uint16_t maxLen){
	uint16_t out;
	// if(k > a){
	// 	out = (uint16_t)((1/(1+exp(k - 6.907755 - a)) + 0.001)*maxLen);
	// }else if(k < -a){
	// 	out = (uint16_t)((1/(1+exp(-k - 6.907755 - a)) + 0.001)*maxLen);
	// }else{
	// 	out = maxLen;
	// }
	if(k > a){
		out = (uint16_t)((1/(1+exp((k- 6.907755 - a)*0.5)) + 0.001)*maxLen);
	}else if(k < -a){
		out = (uint16_t)((1/(1+exp((-k - 6.907755 - a)*0.5)) + 0.001)*maxLen);
	}else{
		out = maxLen;
	}
	return out;
 }

 uint16_t switchLen(double a,double k,uint16_t maxLen,uint16_t lsLen,float nominalValue,float input){
	uint16_t out;
	static uint16_t counter = 0;
	if( fabs(input) <= nominalValue*0.1f && fabs(k) < a){
		counter++;
	}else{
		counter = 0;
	}

	if(counter >= maxLen){
		counter = maxLen;
		out = maxLen;
	}else{
		if(counter > lsLen)
			out = counter;
		else
			out = lsLen;
	}
	return out;
 }

 double leastSquareStandard(uint32_t num,LS_SLIDE_STRUCT* ls,double x,double y)
 {
 	uint32_t i=0;
 	double den;//denominator
 	double y_pre = 0;
 	
 	ls->y_buf[ls->i_ls] = y;
 	ls->x_buf[ls->i_ls] = x;
 	ls->xytemp = 0;
	ls->x2temp  = 0;
 	for(;i<num;i++)
 	{
 		ls->xytemp += ls->y_buf[i]*ls->x_buf[i];
		ls->x2temp += ls->x_buf[i]*ls->x_buf[i];
	}
 	den = ls->x2temp - (double)num*ls->x_m*ls->x_m;
	if(den >= 0 && den < 1e-8) den = 1e-8;
	else if(den <0 && den>-1e-8) den = -1e-8;
	
	ls->k = (ls->xytemp - (double)num*ls->x_m*ls->y_m)/den;
	if(ls->k >= 0 && ls->k < 1e-8) ls->k = 1e-8;
	else if(ls->k <0 && ls->k>-1e-8) ls->k = -1e-8;
	//printf("den=%f,xytemp=%f,nxy=%f,k=%f\r\n",den,ls->xytemp,(double)num*ls->x_m*ls->y_m,ls->k);
	// ls->b = ls->y_m - ls->k*ls->x_m;
	
	// y_pre = ls->k * num + ls->b;
	
	(ls->i_ls)++;
 	if((ls->i_ls) >= num) ls->i_ls=0;

	return y_pre;	
 }


void initRLS(RLSState_t *_state) {
    _state->S_xy = 0.0;
    _state->S_xx = 1e-8;  // 初始化为小正数防止除零
}

// 更新RLS状态并返回估计的刚度k
double rigidityUpdateRLS(RLSState_t *state,const double x,const double y,const double c) {
    // 应用遗忘因子更新累积量
    state->S_xy = c * state->S_xy + x * y;
    state->S_xx = c * state->S_xx + x * x;
    state->k = state->S_xy / state->S_xx;
    // 计算并返回新的刚度估计值
    return state->k;
}

