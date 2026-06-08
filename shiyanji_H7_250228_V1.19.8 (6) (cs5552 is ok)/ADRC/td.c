/*跟踪微分信号器，简称TD（time delay）算法*/
//对信号起到过渡作用//
/*使用步骤*/
/*
定义
TD_conctrol ABCD[4];
 
初始化
for(uint i=0;i<4;i++)
create_TD(&ABCD[i],parameter);
 
进行输入和计算
TD_Run(&ABCD[i],given_value);
 
获取输出结果
x=TD_get_R1(&ABCD[i]);
xx=TD_get_R2(&ABCD[i]);
单环PID采用ABCD[i].R1作为输入，双环PID采用R1为外环输入，R2为内环输入
 
参数整定
只需改变parameter,parameter越大,收敛速度越快，反之亦然


	TD_Run(&load_td,force.orig);
	
	force.filter=TD_get_R1(&load_td);
	AL.loadCtrl.pgSpeed=TD_get_R2(&load_td);

*/
 
#include "td.h"
 
TD_conctrol load_td;
float load_tdvalue=0;
float loadspeed_tdvalue=0;
 
float fst(float x1, float x2, float r, float h)//期望  跟踪速度  最大跟踪速度  控制的时间间隔
{
 
float y, d, a0, a1, a, sy, sa, result;
	y = x1 + h * x2;
	d = r * h*h;
	d = (d==0) ? 0.000001f : d;
	a0 = sqrtf(d*(d + 8.f * fabsf(y)));
	a1 = 0.5f *(a0 - d)* sign(y) + h * x2;
	sy = 0.5f *(sign(y+d)- sign(y-d));
	a = (h * x2 + y - a1)* sy + a1;
	sa = 0.5f *(sign(a+d)- sign(a-d));
	result = -r *((a / d - sign(a))* sa + sign(a));
 
	return result;
}
 
static void TD_cal(TD_conctrol * TD)   //计算得出预估的跟踪速度 预估的跟踪加速度 当前跟踪值 当前跟踪速度  
{
	TD->td.V1 = TD->td.R2;
	TD->td.V2 = fst(TD->td.R1-TD->Target, TD->td.R2, TD->td.R, TD->Time.Dtime);
//	TD->td.R1 += TD->Time.Dtime * TD->td.V1;
//	TD->td.R2 += TD->Time.Dtime * TD->td.V2;
	TD->td.R1 += 0.001f * TD->td.V1;
	TD->td.R2 += 0.001f * TD->td.V2;
}
 
 
static void getTimeStamp(TD_conctrol * TD)  
{
	TD->Time.Time_p = TD->Time.Time_n;
	TD->Time.Time_n = HAL_GetTick();
	TD->Time.Dtime = (TD->Time.Time_n - TD->Time.Time_p)/ timeradio;
}
 
static void TD_param_init(TD_conctrol * TD, float R)
{
	memset(TD, 0, sizeof(TD_conctrol));
	TD->td.R = R;
}
 
static void TD_inputStatus(TD_conctrol * TD, float target)
{
	TD->Lastnot0_target = TD->Target==0 ? TD->Lastnot0_target : TD->Target;
	TD->Target = target;
	getTimeStamp(TD);
}
 
void TD_Run(TD_conctrol * TD, float target)
{
	TD_inputStatus(TD, target);
	TD_cal(TD);
 
}
 
static void reset_TD(TD_conctrol * TD, float R)
{
	TD->td.R = R;
}
 
void create_TD(TD_conctrol* TD,float R)
{
	reset_TD(TD,R);
	TD_param_init(TD,R);
 
}
 
float TD_get_R1(TD_conctrol * TD)
{
	return TD->td.R1;
}
 
float TD_get_R2(TD_conctrol * TD)
{
	return TD->td.R2;
}
