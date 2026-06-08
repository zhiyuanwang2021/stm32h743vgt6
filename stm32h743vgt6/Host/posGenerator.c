#include <math.h>
#include "posGenerator.h"
#include "usart.h"
#include "gParameter.h"
#include "communicate.h"
#include <string.h>
#include <elog.h>

PosGenerator pg,pg_last;//位置轨迹发生器
PosGenerator pgLoad,pgLoad_last;//力轨迹发生器
PosGenerator pgExt,pgExt_last;//应变轨迹发生器
PGCYCLE pgcycle;//循环指令位置发生器附加部分
PGDYN pgdyn;
DYNCYCLE_MODIFT dyncycle_modify;
DYNCYCLE_NORMAL dyncycle_normal;
DYNCYCLE_HALTTIME dyncycle_halttime;

static int8_t sign(float x);

uint8_t pg_init(PosGenerator* PG)
{
	// float		dm;//总位移
	volatile float		vm;//只加减速对应的最大速度
	int64_t s0 = PG->s0*100000;
	int64_t sg = PG->sg*100000;
	int64_t dmInt;
	if(PG->a<0.00001f || PG->d>-0.00001f || PG->vmax <0.00001f || fabs(PG->s0 - PG->sg)<0.00001f)
	{	
		log_e("[err1]set fail! a=%0.3f, d=%0.3f, vmax=%0.3f, s0=%0.3f, sg=%0.3f,movecommand:%2x\r\n", PG->a,PG->d,PG->vmax,PG->s0,PG->sg,commBuf.Command);
//		PG->ok =0;
		return 1;
	}
	//设置错误	
	if(sg > s0)//目标位置大于初始位置
	{	
		if(PG->v0 > PG->vmax)//初速度＞最大速度，则减速、匀速、减速
		{
			PG->a1 = PG->d;
			PG->a3 = PG->d;			
		}
		else//加速、匀速、减速
		{
			PG->a1 = PG->a;
			PG->a3 = PG->d;			
		}
			
	}
	else//目标位置小于初始位置
	{
		if(PG->v0 < -PG->vmax)//初速度＞最大速度，则减速、匀速、减速
		{
			PG->a1 = -PG->d;
			PG->a3 = -PG->d;			
		}
		else//加速、匀速、减速
		{
			PG->a1 = -PG->a;
			PG->a3 = -PG->d;			
		}		
	}
//	PG->vc		= 0;
//	PG->dc		= 0;
	dmInt = sg - s0;
	//dm	= PG->sg - PG->s0;
	if((PG->a3-PG->a1)<0.1f && (PG->a3-PG->a1)>-0.1f)
	{
		vm = PG->vmax;
	}
	else
	{
		vm = sqrtf((dmInt/100000.0*2.0f*PG->a1*PG->a3+PG->a3*PG->v0*PG->v0)/(PG->a3-PG->a1));
		if(vm>PG->vmax) 
			vm = PG->vmax;		
	}
	if(sg > s0)
		PG->vm = vm;
	else
		PG->vm = -vm;
	
	
	PG->T1 = (PG->vm - PG->v0)*1000.0f/PG->a1;
	PG->T3	= -PG->vm*1000.0f/PG->a3;
	PG->d1 = (PG->vm*PG->vm-PG->v0*PG->v0)/(2.0f*PG->a1);
	PG->d3	= -PG->vm*PG->vm/(2.0f*PG->a3);
	PG->d2 = dmInt/100000.0 - PG->d1 - PG->d3;
	if((PG->d2 <0.0001f) && (PG->d2>-0.0001f)) PG->d2=0;
	if((dmInt/100000.0>0 && PG->d2<0)||(dmInt/100000.0<0 && PG->d2>0))
	{	
		log_e("[err2]set fail dm=%f, d2=%f\r\n",dmInt/100000.0,PG->d2);
		log_e("[err2]set fail d1=%f, d2=%f, d3=%f\r\n",PG->d1,PG->d2,PG->d3);
//		PG->ok =0;
		return 2;
	}
	PG->T2 = PG->d2*1000.0f/PG->vm;
	PG->t1 = PG->T1;
	PG->t2 = PG->t1 + PG->T2;
	PG->t3 = PG->t2 + PG->T3;
	PG->ttotal =PG->t3;
	PG->s1 = s0/100000.0 + PG->d1;
	PG->s2 = PG->s1 + PG->d2;
	PG->s3 = sg/100000.0;
	
	//printf部分应该单独有标志位推入到 串口传输任务中传输  
	
	// log_i("s0=%f,sg=%f,v0=%f,a1=%f,a3=%f,vmax=%f",PG->s0,PG->sg,PG->v0,PG->a1,PG->a3,PG->vmax);
	// log_i("d1=%f,d2=%f,d3=%f",PG->d1,PG->d2,PG->d3);
	// log_i("dm=%f,vm=%f,T1=%d,T2=%d,T3=%d,t1=%d,t2=%d,t3=%d,s1=%f,s2=%f",dmInt/100000.0,PG->vm,PG->T1,PG->T2,PG->T3,PG->t1,PG->t2,PG->t3,PG->s1,PG->s2);
//	log_i("pg init Command:%2x Success!",commBuf.Command);
	PG->t=0;//时间初始化
//	PG->ok =1;//初始化成功
	return 0;
}
/*函数名：位置指令值发生器
输入：t ms, 相对初始位置时刻的相对时间
输出：pg.vt m/s,pg.st mm, 浮点型
*/
float pos_generator(uint32_t t,PosGenerator* PG)
{
	float st;
	//for(t=0;t<=PG->ttotal;t++)
	//{
		if(t<PG->t1)
		{
			PG->vt = PG->v0+PG->a1*t/1000.0f;
			st = PG->s0+(PG->vt*PG->vt-PG->v0*PG->v0)/(2*PG->a1);
			PG->aa = PG->a1;
		}			
		else if(t<PG->t2)
		{
			PG->vt = PG->vm;
			st = PG->s1 + PG->vm * (t-PG->t1)/1000.0f;
			PG->aa = 0;
		}
		else if(t<PG->t3)
		{
			PG->vt = PG->vm + PG->a3*(t-PG->t2)/1000.0f;
			st = PG->s2 +(PG->vt*PG->vt-PG->vm*PG->vm)/(2*PG->a3);
			PG->aa = PG->a3;
		}
		else
		{
			PG->vt =0;
			st = PG->s3;// + PG->vt *(t-PG->t3)/1000.0f;
			PG->aa = 0;
		}
		return st;
//		printf("%d	%f	%f\r\n",t,PG->vt,PG->st);
//	}
}

/**
 * @brief normal move posgenerator init
 * @param	PG type:PosGenerator*
 * @param[in] mode current Movectrl
*/
uint8_t pgNormalInit(PosGenerator* PG,uint8_t mode){
	uint8_t err;
	//normal acc/dec/speed limit
	switch (mode)
	{
	case POS_MODE:
		PG->a = constraint(PG->a,posAllCtrlPara.Nominal.Acc,0);
		PG->d = constraint(PG->d,0,-posAllCtrlPara.Nominal.Acc);
		PG->vmax = constraint(PG->vmax,posAllCtrlPara.Nominal.Speed,0);
		break;
	case LOAD_MODE:
		PG->a = constraint(PG->a,loadAllCtrlPara.Nominal.Acc,0);
		PG->d = constraint(PG->d,0,-loadAllCtrlPara.Nominal.Acc);
		PG->vmax = constraint(PG->vmax,loadAllCtrlPara.Nominal.Speed,0);
		break;
	case EXTEN_MODE:
		PG->a = constraint(PG->a,extAllCtrlPara.Nominal.Acc,0);
		PG->d = constraint(PG->d,0,-extAllCtrlPara.Nominal.Acc);
		PG->vmax = constraint(PG->vmax,extAllCtrlPara.Nominal.Speed,0);
		break;
	default:
		break;
	}
	err = pg_init(PG);
	return err;
}

/**
 * @brief		循环指令规划初始化
 * @param		x	——循环指令参数结构体
				PG	——位置生成器结构体
 * @retval		void
*/
void pgcycle_init(PGCYCLE* x,PosGenerator* PG)
{	
	if(++x->count_step > 2)
	{
		x->count_step=1;
		x->count_cycle++;
	}
	if(x->count_cycle < x->cycles)
	{
		switch(x->count_step)
		{
			case 1:
							PG->sg 	=x->destination1;
							PG->vmax	 = x->speed1;
							switch(AL.movectrl)
							{
								case POS_MODE:	AL.posCtrl.setPos	=	PG->sg;break;
								case LOAD_MODE:	AL.loadCtrl.setPos	=	PG->sg;break;
								case EXTEN_MODE:		break;
							}
							AL.halt	=x->halt1;
				break;
			case 2:
							PG->sg 	=x->destination2;
							PG->vmax	 = x->speed2;
							switch(AL.movectrl)
							{
								case POS_MODE:	AL.posCtrl.setPos	=	PG->sg;break;
								case LOAD_MODE:	AL.loadCtrl.setPos	=	PG->sg;break;
								case EXTEN_MODE:		break;
							}
							AL.halt	=x->halt2;
				break;
			default:
				break;
		}
	}
	else if(x->count_cycle >= x->cycles)	//结束运动循环 移动至最终目标位置
	{
		x->count_step=0;
		PG->sg	= x->destination;
		switch(AL.movectrl)
		{
			case POS_MODE:	AL.posCtrl.setPos	=	PG->sg;break;
			case LOAD_MODE:	AL.loadCtrl.setPos	=	PG->sg;break;
			case EXTEN_MODE:		break;
		}
		PG->vmax	 = x->speed;
	}
	//printf("PG->sg:%f,PG->vmax:%f,AL.halt:%f",PG->sg,PG->vmax,AL.halt);
	//printf("count_step:%d,count_cycle:%d",x->count_step,x->count_cycle);
	
}

/*函数名：位置指令现场恢复 
输入：x要被恢复的结构体 x_last存储上一周期的现场
输出：NULL
*/
void pgResume(PosGenerator* x,PosGenerator* x_last)
{
	memcpy(x,x_last,sizeof(*x));
}
/*函数名：位置指令现场存储 
输入：x现场结构体 x_last要现场备份
输出：NULL
*/
void pg_save(PosGenerator* x,PosGenerator* x_last)
{
	memcpy(x_last,x,sizeof(*x_last));
}


#define COSINE 				0x01
#define TRIANGLE			0x02
#define RECTANGLE			0x04
#define SAWTOOTH    		0x08
#define SAWTOOTH_INVERSE 	0x10
#define PULSE				0x20

#define PI        3.14159265359f
#define _2PI      6.28318530718f
#define _4PI2     39.478417604357f
/*
	关于如何到达正峰值开始进入halfcycle计数
有两种情况：1、如果SpeedToStart设定了非0值  		以SpeedToStart速度到达正峰值处
			2、如果SpeedToStart=0  					直接按照该波形达到正峰值处
*/
float DynCycle_wavecalcu(uint8_t waveform,float amplitude,float frequency,uint32_t t);

void test_DynCycle_para_init()
{
	pgdyn.amplitude=10;
	pgdyn.frequency=0.1;
	pgdyn.offset = 0;
	pgdyn.halfcycles=50;
	pgdyn.waveform=COSINE;
	pgdyn.t_nowturn=0;
	pgdyn.count_cycle=0;
	pgdyn.speedtostart = 1;//开始速度
	
	pgdyn.speedtodestination = 2;//结束速度
	pgdyn.destination = 0;
	
	pgdyn.relativedestination = 0;//相对位移

	pgdyn.modify = 0;//修改活动周期的参数允许位
	
	pgdyn.haltatplusamplitude = 500;//ms
	pgdyn.haltatminusamplitude = 250;//ms
	

}

void DynCycle_planinit()//DynCycle轨迹规划
{
	memset(&dyncycle_halttime,0,sizeof(dyncycle_halttime));//haltatamplitude清0
	memset(&dyncycle_normal,0,sizeof(dyncycle_normal));//dyncycle_normal清0
	memset(&dyncycle_modify,0,sizeof(dyncycle_modify));//dyncycle_modify清0

	if(pgdyn.modify != 0)
		DynCycle_modify_planinit(&pgdyn);//modify=1的轨迹规划 
	else
		DynCycle_normal_planinit(&pgdyn);//modify=0的轨迹规划
	pg.t=0;
	pgLoad.t=0;	
	printf("DynCycle_planinit\r\n");
}

float DynCycle_posgenerator(PGDYN* x,uint32_t t)
{
	float ret=0;
	if(x->modify != 0) //modify 修改活动周期的参数允许位
	ret = DynCycle_modify_posgenerator(x,&dyncycle_modify,t);
	else		
	ret = DynCycle_normal_posgenerator(x,&dyncycle_normal,&dyncycle_halttime,t);
	
	if(x->relativedestination != 0)
	ret += x->offset;
	
	return ret;
}
/*
DynCycle_normal_planinit 
11.13 添加t3循环结束后到Destination轨迹
*/
uint32_t DynCycle_normal_t3calcu(PGDYN* x);//循环完成后由正峰值处到Destination

void DynCycle_normal_planinit(PGDYN* x)
{
	//11.11假设所有规划初始位置都是0
	uint32_t halt_time=0;
	switch(x->waveform)
	{
		case COSINE:
			if(x->speedtostart != 0)
			{
			dyncycle_normal.t1 = x->amplitude*1000.0f/x->speedtostart;//从初始值到达正峰值时间 ms
			}
			else
			dyncycle_normal.t1 = 250.0f/x->frequency;//从初始值0到达正峰值时间 ms
			halt_time = (x->halfcycles/2+1)*x->haltatplusamplitude + (x->halfcycles-x->halfcycles/2)*x->haltatminusamplitude;// ms
			dyncycle_normal.t2 = x->halfcycles/(2*x->frequency)*1000 + halt_time + dyncycle_normal.t1;//周期循环时间 + 峰值暂停时间  信号时间 ms
			dyncycle_normal.t3 =  DynCycle_normal_t3calcu(&pgdyn)+dyncycle_normal.t2;//结束时间 ms 

		break;
		case TRIANGLE:
			if(x->speedtostart != 0)
			{
			dyncycle_normal.t1 = x->amplitude*1000.0f/x->speedtostart;//从初始值到达正峰值时间 ms
			}
			else
			dyncycle_normal.t1 = 250/x->frequency;//从初始值到达正峰值时间 ms
			halt_time = (x->halfcycles/2+1)*x->haltatplusamplitude + (x->halfcycles-x->halfcycles/2)*x->haltatminusamplitude;// ms
			dyncycle_normal.t2 = x->halfcycles/(2*x->frequency)*1000 + halt_time + dyncycle_normal.t1;//信号时间 ms
			dyncycle_normal.t3 = DynCycle_normal_t3calcu(&pgdyn)+dyncycle_normal.t2;//结束时间 ms 
		break;
		case RECTANGLE:
			if(x->speedtostart != 0)
			{
			dyncycle_normal.t1 = x->amplitude*1000.0f/x->speedtostart;//从初始值到达正峰值时间 ms
			}
			else
			dyncycle_normal.t1 = 0;//从初始值到达正峰值时间 ms
			halt_time = (x->halfcycles/2+1)*x->haltatplusamplitude + (x->halfcycles-x->halfcycles/2)*x->haltatminusamplitude;// ms
			dyncycle_normal.t2 = x->halfcycles/(2*x->frequency)*1000 + halt_time + dyncycle_normal.t1;//信号时间 ms
			dyncycle_normal.t3 = DynCycle_normal_t3calcu(&pgdyn)+dyncycle_normal.t2;//结束时间 ms 
		break;
		case SAWTOOTH:
			if(x->speedtostart != 0)
			{
			dyncycle_normal.t1 = x->amplitude*1000.0f/x->speedtostart;//从初始值到达正峰值时间 ms
			}
			else
			dyncycle_normal.t1 = 0;//从初始值到达正峰值时间 ms
			halt_time = (x->halfcycles/2+1)*x->haltatplusamplitude + (x->halfcycles-x->halfcycles/2)*x->haltatminusamplitude;// ms
			dyncycle_normal.t2 = x->halfcycles/(2*x->frequency)*1000 + halt_time + dyncycle_normal.t1;//信号时间 ms
			dyncycle_normal.t3 = DynCycle_normal_t3calcu(&pgdyn)+dyncycle_normal.t2;//结束时间 ms 
		break;
		case SAWTOOTH_INVERSE:
			if(x->speedtostart != 0)
			{
			dyncycle_normal.t1 = x->amplitude*1000.0f/x->speedtostart;//从初始值到达正峰值时间 ms
			}
			else
			dyncycle_normal.t1 = 500/x->frequency;//从初始值到达正峰值时间 ms
			halt_time = (x->halfcycles/2+1)*x->haltatplusamplitude + (x->halfcycles-x->halfcycles/2)*x->haltatminusamplitude;// ms
			dyncycle_normal.t2 = x->halfcycles/(2*x->frequency)*1000 + halt_time + dyncycle_normal.t1;//信号时间 ms
			dyncycle_normal.t3 = DynCycle_normal_t3calcu(&pgdyn)+dyncycle_normal.t2;//结束时间 ms 
		break;
		case PULSE://PULSE 不具备speedtostart特性
			dyncycle_normal.t1 = 0;//从初始值到达正峰值时间 ms
			//只存在正峰值
			halt_time = (x->halfcycles-x->halfcycles/2)*x->haltatplusamplitude;// ms
			dyncycle_normal.t2 = x->halfcycles/(2*x->frequency)*1000 + halt_time + dyncycle_normal.t1;//信号时间 ms
			dyncycle_normal.t3 = DynCycle_normal_t3calcu(&pgdyn)+dyncycle_normal.t2;//结束时间 ms 
		break;
	}	
}

/*
DynCycle_normal_posgenerator 
*/
#define NOHALT 		0
#define HALTATPLUS 	1
#define HALTATMINUS 2

float DynCycle_normal_posgenerator(PGDYN* x,DYNCYCLE_NORMAL *tim_node,DYNCYCLE_HALTTIME *ht,uint32_t t)
{
	static float ret=0;
	int32_t t2_input=0;
	//t1
	if(t<tim_node->t1)//从初始值到达正峰值
	{
		if(x->speedtostart != 0)
		ret = x->speedtostart*t/1000.0f;
		else
		ret = DynCycle_wavecalcu(x->waveform,x->amplitude,x->frequency,(1000.0f/x->frequency-tim_node->t1)+t);	
	}
	//t2
	else if(t<tim_node->t2)
	{
	ht->haltatamplitude_time = ht->haltatplusamplitude_counter*x->haltatplusamplitude 
						 + ht->haltatminusamplitude_counter*x->haltatminusamplitude;
	t2_input = t-tim_node->t1-ht->haltatamplitude_time;
	if(t2_input < 0) t2_input = 0;
	ret = DynCycle_wavecalcu(x->waveform,x->amplitude,x->frequency,t2_input);
		//halt判断 
		
	switch(x->waveform)
	{
		case COSINE:
		case TRIANGLE:
		case RECTANGLE:
			if((ht->wave_time == (uint32_t)(500.0f /x->frequency) || t == tim_node->t1) 
				&& ht->haltamplitude_countingflag == NOHALT)//半周期判断
				{
					if(x->haltatplusamplitude != 0 && ret > 0)
					{
					ht->haltamplitude_countingflag = HALTATPLUS;//
					ht->wave_time=0;
					ht->haltatamplitude_timestamp = t;
					//	printf("haltatamplitude_timestamp:%d\r\n",haltatamplitude_timestamp);
					}
					else if(x->haltatminusamplitude != 0 && ret < 0)
					{
					ht->haltamplitude_countingflag = HALTATMINUS;//
					ht->wave_time=0;
					ht->haltatamplitude_timestamp = t;
					//	printf("haltatamplitude_timestamp:%d\r\n",haltatamplitude_timestamp);
					}
				}
		break;
		case SAWTOOTH:
			if(t == tim_node->t1 && x->haltatplusamplitude != 0 && ret > 0)
			{
			ht->haltamplitude_countingflag = HALTATPLUS;//
			ht->wave_time=0;
			ht->haltatamplitude_timestamp = t;
			//printf("haltatamplitude_timestamp:%d\r\n",haltatamplitude_timestamp);
			}
			if(ht->wave_time == (uint32_t)(1000.0f /x->frequency) && ht->haltamplitude_countingflag == NOHALT && x->haltatminusamplitude != 0 )//半周期判断
			{
				ret = -x->amplitude;
				ht->haltamplitude_countingflag = HALTATMINUS;//
				ht->wave_time=0;
				ht->haltatamplitude_timestamp = t;
			//	printf("haltatamplitude_timestamp:%d\r\n",haltatamplitude_timestamp);
			}
		break;
		case SAWTOOTH_INVERSE:
			if((ht->wave_time == (uint32_t)(1000.0f /x->frequency) || t == tim_node->t1) 
				&& ht->haltamplitude_countingflag == NOHALT
				&& x->haltatplusamplitude != 0)//半周期判断
			{
				ret = x->amplitude;
				ht->haltamplitude_countingflag = HALTATPLUS;//
				ht->wave_time=0;
				ht->haltatamplitude_timestamp = t;
				//	printf("haltatamplitude_timestamp:%d\r\n",haltatamplitude_timestamp);
			}
		break;
		case PULSE:
			if(	(ht->wave_time == (uint32_t)(1000.0f /x->frequency) ||
				t == tim_node->t1 +	(uint32_t)(1000.0f /(12.0f*x->frequency)))	
				&& ht->haltamplitude_countingflag == NOHALT
				&& x->haltatplusamplitude != 0)//半周期判断
			{
				ret = x->amplitude;
				ht->haltamplitude_countingflag = HALTATPLUS;//
				ht->wave_time=0;
				ht->haltatamplitude_timestamp = t;
				//	printf("haltatamplitude_timestamp:%d\r\n",haltatamplitude_timestamp);
			}
		break;	
			
	}
			//halt执行
			if(ht->haltamplitude_countingflag == HALTATPLUS)
			{
				//printf("HALTATPLUS %d\r\n",t);
				if((t-ht->haltatamplitude_timestamp) >= x->haltatplusamplitude-1)
				{
					ht->haltatplusamplitude_counter++;
					ht->haltamplitude_countingflag = NOHALT;
					if(x->waveform == SAWTOOTH_INVERSE)
					{
					ht->haltamplitude_countingflag = HALTATMINUS;
					ht->wave_time=0;
					ht->haltatamplitude_timestamp = t+1;	
					}
				//	printf("haltatamplitude_complete + :%d\r\n",t);
				}
				ret = x->amplitude;
			}
			else if(ht->haltamplitude_countingflag == HALTATMINUS)
			{
				//printf("HALTATMINUS %d\r\n",t);
				if((t-ht->haltatamplitude_timestamp) >= x->haltatminusamplitude-1)
				{
					ht->haltatminusamplitude_counter++;
					ht->haltamplitude_countingflag = NOHALT;
					if(x->waveform == SAWTOOTH)
					{
					ht->haltamplitude_countingflag = HALTATPLUS;
					ht->wave_time=0;
					ht->haltatamplitude_timestamp = t+1;	
					}
				//	printf("haltatamplitude_complete - :%d\r\n",t);
				}
				ret = -x->amplitude;
			}
			else if(ht->haltamplitude_countingflag == NOHALT)//记录正常波形的时间
			{
				ht->wave_time++;
				ht->haltatamplitude_timestamp = t;
			}
		
	}
	//t3
	else if(t<=tim_node->t3)//从正峰值到达Destination
	{
		if(x->speedtodestination != 0)
		{
			if(x->waveform != PULSE )
			ret = x->amplitude - x->speedtodestination*(t-tim_node->t2)/1000.0f;
			else
			ret = sign(x->destination)*x->speedtodestination*(t-tim_node->t2)/1000.0f;
		}
		else
		ret = DynCycle_wavecalcu(x->waveform,x->amplitude,x->frequency,t-tim_node->t2);	
		
		if(t == tim_node->t3)
		ret = x->destination;
	}
	else if(t>tim_node->t3)//可以添加 终点补偿
	{
//		if( (x->pos - x->destination) != 0)
//		{
//			i +=sign(x->pos - x->destination);
//			if(x->speedtodestination != 0)
//			{
//			ret = x->amplitude - x->speedtodestination*(tim_node->t3-tim_node->t2+i)/1000.0f;
//			}
//			else
//			ret = DynCycle_wavecalcu(x->waveform,x->amplitude,x->frequency,tim_node->t3-tim_node->t2+i);	
//		}
	}
	
	return (ret+x->offset);
}

uint32_t DynCycle_normal_t3calcu(PGDYN* x)
{
	uint32_t t3=0;
	switch(x->waveform)
	{
		case COSINE:
			if(x->speedtodestination != 0)
			{
			t3 = fabs(x->amplitude-x->destination)*1000.0f/x->speedtodestination;//从达正峰值到Destination时间 ms
			}
			else //若没有指定speedtodestination，那么Destination必须只能在-x->amplitude ~ x->amplitude之间
			t3 = acosf(x->destination/x->amplitude)/(2.0f*PI*x->frequency)*1000.0f;//结束时间 ms 

		break;
		case TRIANGLE:
			if(x->speedtodestination != 0)
			{
			t3 = fabs(x->amplitude-x->destination)*1000.0f/x->speedtodestination;//从达正峰值到Destination时间 ms
			}
			else //若没有指定speedtodestination，那么Destination必须只能在-x->amplitude ~ x->amplitude之间
			t3 = (x->amplitude-x->destination)/(4.0f*x->frequency*x->amplitude)*1000.0f;//结束时间 ms 
		break;
		case RECTANGLE:
			if(x->speedtodestination != 0)
			{
			t3 = fabs(x->amplitude-x->destination)*1000.0f/x->speedtodestination;//从达正峰值到Destination时间 ms
			}
			else //延迟半周期回到Destination处，那么Destination必须只能在-x->amplitude ~ x->amplitude之间
			t3 = 500.0f/x->frequency;//从达正峰值到Destination时间 ms
		break;
		case SAWTOOTH:
			if(x->speedtodestination != 0)
			{
			t3 = fabs(x->amplitude-x->destination)*1000.0f/x->speedtodestination;//从达正峰值到Destination时间 ms
			}
			else
			t3 = (x->amplitude-x->destination)/(2.0f*x->amplitude*x->frequency)*1000.0f;//从达正峰值到Destination时间 ms 
		break;
		case SAWTOOTH_INVERSE:
			if(x->speedtodestination != 0)
			{
			t3 = fabs(x->amplitude-x->destination)*1000.0f/x->speedtodestination;//从达正峰值到Destination时间 ms
			}
			else
			t3 = (x->amplitude+x->destination)/(2.0f*x->amplitude*x->frequency)*1000.0f;//从达正峰值到Destination时间 ms
		break;
		case PULSE://PULSE 不具备speedtodestination特性
			if(x->speedtodestination != 0)
			{
			t3 = fabs(x->destination)*1000.0f/x->speedtodestination;//从达正峰值到Destination时间 ms
			}
			else
			t3 = 0;//不根据PULSE波形到Destination指令，而是进行直接跳转，所以慎用
		break;
	}	
	return t3;
}

/*
DynCycle_modify相关函数
	目标实现两次周期波的输出
	第一次 幅值=1 频率=1 偏移值=0
	第二次 幅值=amplitude 频率=frequency 偏移值=offset

	11.13 考虑t3过渡时间加入speedtostart != 0 情况
	11.13 添加t5循环结束后到Destination轨迹
	11.15 normal的halt做好了 modify的还没搞 可以参照normal的进行加入 
*/
#define t3_use_speedtostart 1  //选择modify t3波形切换时 是否使用speedtostart切换 0否 1是
uint32_t DynCycle_modify_t3calcu(PGDYN* x);
uint32_t DynCycle_modify_t5calcu(PGDYN* x);

void DynCycle_modify_planinit(PGDYN* x)
{
	//11.11假设所有规划初始位置都是0
	switch(x->waveform)
	{
		case COSINE:
			if(x->speedtostart != 0)
			{
			dyncycle_modify.t1 = 1000.0f/x->speedtostart;//从初始值到达正峰值时间 ms
			}
			else
			dyncycle_modify.t1 = 250;//从初始值到达正峰值时间 ms
			
			dyncycle_modify.t2 = x->halfcycles/2*1000+dyncycle_modify.t1;//原始信号时间 ms
			dyncycle_modify.t3 = DynCycle_modify_t3calcu(&pgdyn)+dyncycle_modify.t2 ;//过渡时间 用Modify信号实现
			dyncycle_modify.t4 = x->halfcycles/(2*x->frequency)*1000+dyncycle_modify.t3;//Modify信号时间 ms
			dyncycle_modify.t5 = DynCycle_modify_t5calcu(&pgdyn)+dyncycle_modify.t4;//Modify信号时间 ms
		break;
		case TRIANGLE:
			if(x->speedtostart != 0)
			{
			dyncycle_modify.t1 = 1000.0f/x->speedtostart;//从初始值到达正峰值时间 ms
			}
			else
			dyncycle_modify.t1 = 250;//从初始值到达正峰值时间 ms
			dyncycle_modify.t2 = x->halfcycles/2*1000+dyncycle_modify.t1;//原始信号时间 ms
			dyncycle_modify.t3 = DynCycle_modify_t3calcu(&pgdyn)+dyncycle_modify.t2;//过渡时间 用Modify信号实现
			dyncycle_modify.t4 = x->halfcycles/(2*x->frequency)*1000+dyncycle_modify.t3;//Modify信号时间 ms
			dyncycle_modify.t5 = DynCycle_modify_t5calcu(&pgdyn)+dyncycle_modify.t4;//Modify信号时间 ms
		break;
		case RECTANGLE:
			if(x->speedtostart != 0)
			{
			dyncycle_modify.t1 = 1000.0f/x->speedtostart;//从初始值到达正峰值时间 ms
			}
			else
			dyncycle_modify.t1 = 0;//从初始值到达正峰值时间 ms
			dyncycle_modify.t2 = x->halfcycles/2*1000+dyncycle_modify.t1;//原始信号时间 ms
			dyncycle_modify.t3 = DynCycle_modify_t3calcu(&pgdyn)+dyncycle_modify.t2;//过渡时间 用Modify信号实现
			dyncycle_modify.t4 = x->halfcycles/(2*x->frequency)*1000+dyncycle_modify.t3;//Modify信号时间 ms  +2确保最后波形收尾在正峰值处
			dyncycle_modify.t5 = DynCycle_modify_t5calcu(&pgdyn)+dyncycle_modify.t4;//Modify信号时间 ms
		break;
		case SAWTOOTH:
			if(x->speedtostart != 0)
			{
			dyncycle_modify.t1 = 1000.0f/x->speedtostart;//从初始值到达正峰值时间 ms
			}
			else
			dyncycle_modify.t1 = 0;//从初始值到达正峰值时间 ms
			
			dyncycle_modify.t2 = x->halfcycles/2*1000+dyncycle_modify.t1;//原始信号时间 ms
			dyncycle_modify.t3 = DynCycle_modify_t3calcu(&pgdyn)+dyncycle_modify.t2;//过渡时间 用Modify信号实现
			dyncycle_modify.t4 = x->halfcycles/(2*x->frequency)*1000+dyncycle_modify.t3;//Modify信号时间 ms  +2确保最后波形收尾在正峰值处
			dyncycle_modify.t5 = DynCycle_modify_t5calcu(&pgdyn)+dyncycle_modify.t4;//Modify信号时间 ms
		break;
		case SAWTOOTH_INVERSE:
			if(x->speedtostart != 0)
			{
			dyncycle_modify.t1 = 1000.0f/x->speedtostart;//从初始值到达正峰值时间 ms
			}
			else
			dyncycle_modify.t1 = 500;//从初始值到达正峰值时间 ms
			dyncycle_modify.t2 = x->halfcycles/2*1000+dyncycle_modify.t1;//原始信号时间 ms
			dyncycle_modify.t3 = DynCycle_modify_t3calcu(&pgdyn)+dyncycle_modify.t2;//过渡时间 用Modify信号实现
			dyncycle_modify.t4 = x->halfcycles/(2*x->frequency)*1000+dyncycle_modify.t3;//Modify信号时间 ms
			dyncycle_modify.t5 = DynCycle_modify_t5calcu(&pgdyn)+dyncycle_modify.t4;//Modify信号时间 ms
		break;
		case PULSE://PULSE 不具备speedtostart特性
			dyncycle_modify.t1 = 0;//从初始值到达正峰值时间 ms
			dyncycle_modify.t2 = x->halfcycles/2*1000+dyncycle_modify.t1;//原始信号时间 ms
			dyncycle_modify.t3 = 0+dyncycle_modify.t2;//过渡时间 用Modify信号实现
			dyncycle_modify.t4 = x->halfcycles/(2*x->frequency)*1000+dyncycle_modify.t3;//Modify信号时间 ms
			dyncycle_modify.t5 = DynCycle_modify_t5calcu(&pgdyn)+dyncycle_modify.t4;//Modify信号时间 ms
		break;
	}	
}

/*
pgdyn.halfcycles 暂时只能为偶数 奇数会出现负值切换还没写11.12
*/
float DynCycle_modify_posgenerator(PGDYN* x,DYNCYCLE_MODIFT *tim_node,uint32_t t)
{
	static float ret=0;
	//t1
	if(t<tim_node->t1)
	{
		    if(x->speedtostart != 0)
			{
			ret = x->speedtostart*t/1000.0f;
			}
			else
			ret = DynCycle_wavecalcu(x->waveform,1,1,(1000-tim_node->t1)+t);
	}
	//t2
	else if(t<tim_node->t2)
	{
	ret = DynCycle_wavecalcu(x->waveform,1,1,(t-tim_node->t1));
	}
	//t3
	else if(t<tim_node->t3)
	{
#if (t3_use_speedtostart)
		if(x->speedtostart != 0)
		{
			if(x->amplitude > 1)
			ret = 1+x->speedtostart*(t-tim_node->t2)/1000.0f;
			else if(x->amplitude<1)
			ret = 1-x->speedtostart*(t-tim_node->t2)/1000.0f;
		}
		else
#endif
		{
			if(x->amplitude > 1)
			ret = DynCycle_wavecalcu(x->waveform,x->amplitude,x->frequency,(t-tim_node->t2+1000.0f/x->frequency-(tim_node->t3-tim_node->t2)));
			else if(x->amplitude<1)
			ret = DynCycle_wavecalcu(x->waveform,1,1,t-tim_node->t2);
		}
	}
	//t4
	else if(t<tim_node->t4)
	{
	ret = DynCycle_wavecalcu(x->waveform,x->amplitude,x->frequency,t-tim_node->t3);
	}
	else if(t<=tim_node->t5)//从正峰值到达Destination
	{
		if(x->speedtodestination != 0)
		{
			if(x->waveform != PULSE )
			ret = x->amplitude - x->speedtodestination*(t-tim_node->t4)/1000.0f;
			else
			ret = sign(x->destination)*x->speedtodestination*(t-tim_node->t4)/1000.0f;
		}
		else
		ret = DynCycle_wavecalcu(x->waveform,x->amplitude,x->frequency,t-tim_node->t4);	
		
		if(t == tim_node->t5)
		ret = x->destination;
	}
	else if(t>tim_node->t5)
	{
	
	}

	return (ret+x->offset);
}

uint32_t DynCycle_modify_t3calcu(PGDYN* x)
{
	uint32_t t3=0;
#if (t3_use_speedtostart)
	if(x->speedtostart != 0)
	{
		switch(x->waveform)
		{
			case COSINE:
			case TRIANGLE:
			case SAWTOOTH:
			case SAWTOOTH_INVERSE:
			case RECTANGLE:
			case PULSE:
					t3 = (uint32_t)(fabs(x->amplitude-1)*1000/x->speedtostart);
				break;
			default:
				break;
		}
	}
	else
#endif
	{
		switch(x->waveform)
		{
			case COSINE:
					if(x->amplitude > 1)
					t3 = (uint32_t)(acosf(1.0f/x->amplitude)/(2.0f*PI*x->frequency)*1000.0f);
					else if(x->amplitude<1)
					t3 = (uint32_t)(acosf(x->amplitude)/(2.0f*PI)*1000.0f);
					else
					t3 = 0;
				break;
			case TRIANGLE:
					if(x->amplitude > 1)
					t3 = (uint32_t)(1/(4.0f*x->frequency)*(1-1/x->amplitude)*1000);
					else if(x->amplitude<1)
					t3 = (uint32_t)(1/(4.0f)*(1-x->amplitude)*1000);
					else
					t3 = 0;
				break;
			case RECTANGLE:
					if(x->amplitude > 1)
					t3 = (uint32_t)((x->amplitude-1)*1000.0f/x->frequency);
					else if(x->amplitude<=1)
					t3 = (uint32_t)((1-x->amplitude)*1000.0f/x->frequency);
				break;
			case SAWTOOTH:
					if(x->amplitude >= 1)
					t3 = 0;
					else if(x->amplitude < 1)
					t3 = (uint32_t)(0.5f*(1-x->amplitude)*1000);
				break;
			case SAWTOOTH_INVERSE:
					if(x->amplitude > 1)
					t3 = (uint32_t)(1/(2.0f*x->amplitude*x->frequency)*(x->amplitude-1)*1000);
					else if(x->amplitude<=1)
					t3 = 0;
				break;
			case PULSE:
					t3 = 0;
				break;
			default:
				break;
		}
	}
	return t3;
}

uint32_t DynCycle_modify_t5calcu(PGDYN* x)
{
	uint32_t t5=0;
	switch(x->waveform)
	{
		case COSINE:
			if(x->speedtodestination != 0)
			{
			t5 = fabs(x->amplitude-x->destination)*1000.0f/x->speedtodestination;//从达正峰值到Destination时间 ms
			}
			else //若没有指定speedtodestination，那么Destination必须只能在-x->amplitude ~ x->amplitude之间
			t5 = acosf(x->destination/x->amplitude)/(2.0f*PI*x->frequency)*1000.0f;//结束时间 ms 

		break;
		case TRIANGLE:
			if(x->speedtodestination != 0)
			{
			t5 = fabs(x->amplitude-x->destination)*1000.0f/x->speedtodestination;//从达正峰值到Destination时间 ms
			}
			else //若没有指定speedtodestination，那么Destination必须只能在-x->amplitude ~ x->amplitude之间
			t5 = (x->amplitude-x->destination)/(4.0f*x->frequency*x->amplitude)*1000.0f;//结束时间 ms 
		break;
		case RECTANGLE:
			if(x->speedtodestination != 0)
			{
			t5 = fabs(x->amplitude-x->destination)*1000.0f/x->speedtodestination;//从达正峰值到Destination时间 ms
			}
			else //延迟半周期回到Destination处，那么Destination必须只能在-x->amplitude ~ x->amplitude之间
			t5 = 500.0f/x->frequency;//从达正峰值到Destination时间 ms
		break;
		case SAWTOOTH:
			if(x->speedtodestination != 0)
			{
			t5 = fabs(x->amplitude-x->destination)*1000.0f/x->speedtodestination;//从达正峰值到Destination时间 ms
			}
			else
			t5 = (x->amplitude-x->destination)/(2.0f*x->amplitude*x->frequency)*1000.0f;//从达正峰值到Destination时间 ms 
		break;
		case SAWTOOTH_INVERSE:
			if(x->speedtodestination != 0)
			{
			t5 = fabs(x->amplitude-x->destination)*1000.0f/x->speedtodestination;//从达正峰值到Destination时间 ms
			}
			else
			t5 = (x->amplitude+x->destination)/(2.0f*x->amplitude*x->frequency)*1000.0f;//从达正峰值到Destination时间 ms
		break;
		case PULSE://PULSE 不具备speedtodestination特性
			if(x->speedtodestination != 0)
			{
			t5 = fabs(x->destination)*1000.0f/x->speedtodestination;//从达正峰值到Destination时间 ms
			}
			else
			t5 = 0;//不根据PULSE波形到Destination指令，而是进行直接跳转，所以慎用
		break;
	}	
	return t5;
}

/*
波形峰值最小距离计算 计算峰值左右一拍的值 与 峰值的差
*/
float AmplitudeMinDistance_calcu(PGDYN* x)
{
	float ret=0;
	switch(x->waveform)
	{
		case COSINE:
			ret = (1.0f-cosf(2.0f*PI*x->frequency*0.001f))*x->amplitude;
		break;
		case TRIANGLE:
			ret = 4.0f*x->frequency*0.001f*x->amplitude;
		break;
		case RECTANGLE:
			ret =0;
		break;
		case SAWTOOTH:
		case SAWTOOTH_INVERSE:
			ret = 2.0f*x->frequency*0.001f*x->amplitude;
		break;
		case PULSE://PULSE 不具备speedtodestination特性
			ret = 12.0f*x->frequency*0.001f*x->amplitude;
		break;
	}	
	return ret;
}

/*
DynCycle_wavecalcu 基础波形生成 
以及相关波形函数
*/
float DynCycle_wavecalcu(uint8_t waveform,float amplitude,float frequency,uint32_t t) //基础波形位置计算
{
	float ret=0;
	switch(waveform)
	{
		case COSINE:
			ret = amplitude*CosineWave(frequency,t/1000.0f);
			break;
		case TRIANGLE:
			ret = amplitude*TriangleWave(frequency,t/1000.0f);
			break;
		case RECTANGLE:
			ret = amplitude*RectangleWave(frequency,t/1000.0f);
			break;
		case SAWTOOTH:
			ret = amplitude*SawtoothWave(frequency,t/1000.0f);
			break;
		case SAWTOOTH_INVERSE:
			ret = amplitude*Sawtooth_InverseWave(frequency,t/1000.0f);
			break;
		case PULSE:
			ret = amplitude*PulseWave(frequency,t/1000.0f);
			break;
		default:
			break;
	}
	return ret;
}

//余弦波
float CosineWave(float freq,float t)//hz s
{
	float mag=0;
	t = t - ((uint32_t)(t*freq))/freq;
	mag = cosf(2.0f*PI*t*freq);
	return mag;
}

//余弦波 acc/speed/pose generator
float cosWaveASP(COS_STRUCT* x)//hz s
{
	float mag,w,wt,t;
	w = _2PI * x->f;
	t = x->t - ((uint32_t)(x->t * x->f)) / x->f;
	wt = w * t;
	x->st = x->A * cosf(wt);
	x->vt = - x->A * w * sinf(wt);
	x->at = - x->A * w * w * cosf(wt);
	mag = x->st;
	return mag;
}

//三角波
float TriangleWave(float freq,float t) //hz s
{
	float mag=0;
	t = t -((uint32_t)(t*freq))/freq;
	if(t<1/(2.0f*freq))
		mag = -4*freq*t+1;
	else if(t<1.0f/freq)
		mag = 4.0f*freq*t-3;
	return mag;
}
//矩形波
float RectangleWave(float freq,float t) //hz s
{
	float mag=0;
	t = t -((uint32_t)(t*freq))/freq;
	if(t<1/(2.0f*freq))
		mag = 1;
	else
		mag = -1;
	return mag;
}
//锯齿波
float SawtoothWave(float freq,float t) //hz s
{
	float mag=0;
	t = t -((uint32_t)(t*freq))/freq;
	mag=-2*freq*t+1;
	return mag;
}
//逆锯齿波
float Sawtooth_InverseWave(float freq,float t) //hz s
{
	float mag=0;
	t = t -((uint32_t)(t*freq))/freq;
	mag=2*freq*t-1;
	return mag;
}
//脉冲波
float PulseWave(float freq,float t) //hz s
{
	float mag=0;
	t = t -((uint32_t)(t*freq))/freq;
	if(t<0.083f/freq)
		mag=12*freq*t;
	else if(t<0.166f/freq)
		mag=-12*freq*t+2;
	else
		mag=0;
	return mag;
}

static int8_t sign(float x){
	return x>0 ? 1:(x==0 ? 0:-1);
}

/**
 * @brief cos pos generator init function
 * @param maxAcc	accelerate nominal value
 * @param maxSpeed	speed nominal value
 * @param A			amplitude of cos wave
 * @param f			frequency
 * @param x 		type:COS_STRUCT*	
*/
uint8_t cosPgInit(float maxAcc,float maxSpeed,float A,float f,COS_STRUCT* x){
	uint8_t err=0;
	x->ok = 0;
	x->am = A * _4PI2 * f * f;
	x->vm = A * _2PI * f;
	if(maxAcc < fabs(x->am))
		err |= beyondAcc;
	if(maxSpeed < fabs(x->vm))
		err |= beyondSpeed;
	if(err == 0){
		x->A = A;
		x->f = f;
		x->ok = 1;
	}
	else{
		x->ok = 0;
		log_e("Dyn cosPgInit fail:%d",err);
	}
	return err;
} 

/**
 * @brief generate cos pose with time
 * @param x type:COS_STRUCT*
*/
float cosPosGenerator(COS_STRUCT* x){
	float st;
	st = cosWaveASP(x);
	return st;
}



/* *****END OF FILE****/
