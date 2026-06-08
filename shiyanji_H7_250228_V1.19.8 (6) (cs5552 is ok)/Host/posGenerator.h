/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _POSGENERATOR_H_
#define _POSGENERATOR_H_

#include <stdint.h>

typedef struct
{
/*位移单位mm
速度单位mm/s
加减速度单位mm/s2,减速度为负值
*/	
	float 	s0;//初始位置mm
	float		s1;//第一段加/减速结束位置
	float		s2;//第二段匀速结束位置
	float		s3;//第三段减速结束位置
	float		v0;//初始速度mm/s
	float   	a;//加速度值 
	float		d;//减速度值
	float   	a1;//第一段加速度值 
	float		a3;//第三段加速度值
	float 		vmax;//最大速度
	float 		vm;//实际最大速度
	float		sg;//目标位置
	float		vc;//末爬行速度
	float  		dc;//末爬行位移
	float		d1;//第一段加/减速位移
	float		d2;//匀速位移
	float		d3;//减速位移
	uint32_t		T1;//第一段持续时间
	uint32_t		T2;//第二段持续时间
	uint32_t		T3;//第三段持续时间
	uint32_t		Tc;//末爬行段持续时间
	uint32_t		t1;//第一段结束时刻
	uint32_t		t2;//第二段结束时刻
	uint32_t		t3;//第三段结束时刻
	uint32_t		ttotal;//总时间
	uint32_t		t;//相对时间
	float		vt;//速度随时间的变化
	float		st;//位置随时间的变化
	float		st_last;
	uint8_t 	ok;//1位置发生器初始化成功,0初始化失败
	float		aa;
//	int32_t 		
}PosGenerator;
extern PosGenerator pg,pg_last;//位置轨迹发生器  pg_last用于保护上一周期的现场，位置初始化失败后，应该恢复现场
extern PosGenerator pgLoad,pgLoad_last;//力轨迹发生器  pgLoad_last用于保护上一周期的现场，位置初始化失败后，应该恢复现场
extern PosGenerator pgExt,pgExt_last;//力轨迹发生器  pgLoad_last用于保护上一周期的现场，位置初始化失败后，应该恢复现场

typedef struct{
	uint8_t count_step;
	uint8_t count_cycle;
	uint8_t cycles;
	float halt1;
	float halt2;
	float speed;
	float speed1;
	float speed2;
	float destination;
	float destination1;
	float destination2;
}PGCYCLE;
extern PGCYCLE pgcycle;

typedef struct{
	uint8_t modify;//修改活动周期的参数允许位
	uint8_t peakctrl;//峰值控制循环
	uint8_t movectrl;//控制模式
	uint8_t relativedestination;//相对位移允许位
	
	uint8_t waveform;			//波形形式
	uint8_t halfcycles;			//半循环次数

	float amplitude;			//波形幅值
	float haltatplusamplitude;	//正峰值暂停时间
	float haltatminusamplitude;	//负峰值暂停时间
	float frequency;			//波形频率
	float speedtodestination;	//最终目标速度
	float destination;			//最终目标位置
	float speedtostart;			//开始速度
	float offset;				//偏移量
	
	float pos;					//位置生成器的规划位置
	uint8_t count_cycle;		//已规划的周期
	uint32_t t_nowturn;			//当前周期时间
}PGDYN;
extern PGDYN pgdyn;

typedef struct{
	uint32_t t1;
	uint32_t t2;
	uint32_t t3;
	uint32_t t4;
	uint32_t t5;
}DYNCYCLE_MODIFT;
extern DYNCYCLE_MODIFT dyncycle_modify;

typedef struct{
	uint32_t t1;//0->+amplitude
	uint32_t t2;//循环
	uint32_t t3;//+amplitude->0
}DYNCYCLE_NORMAL;
extern DYNCYCLE_NORMAL dyncycle_normal;

typedef struct{
	uint32_t haltatplusamplitude_counter;
	uint32_t haltatminusamplitude_counter;
	uint32_t haltatamplitude_timestamp;
	uint32_t haltatamplitude_time;
	uint32_t wave_time;
	uint8_t haltamplitude_countingflag;
}DYNCYCLE_HALTTIME;
extern DYNCYCLE_HALTTIME dyncycle_halttime;

enum COSPG_ENUM{
	beyondAcc = 1,//cos max acc beyond nominal acc
	beyondSpeed = 1<<1,//cos max speed beyond nomianal speed
};

typedef struct{
/*位移单位mm
速度单位mm/s
加减速度单位mm/s2,减速度为负值
*/	
	uint8_t ok;//cos wave pos generator init successfully 
	float   A;//cos wave amplitude
	float	f;//cos wave frequency
	float 	vm;//max velocity
	float	am;//max accelerate

	float 	at;//accelerate changes with time
	float	vt;//velocity changes with time
	float	st;//pose changes with time
	float	t;//reltive time
	uint32_t	tTotal;//total time
}COS_STRUCT;

uint8_t pg_init(PosGenerator* PG);
uint8_t pgNormalInit(PosGenerator* PG,uint8_t mode);
float pos_generator(uint32_t t,PosGenerator* PG);

void pgcycle_init(PGCYCLE* x,PosGenerator* PG);

void pgResume(PosGenerator* x,PosGenerator* x_last);
void pg_save(PosGenerator* x,PosGenerator* x_last);


void DynCycle_planinit(void);//DynCycle轨迹规划
float DynCycle_posgenerator(PGDYN* x,uint32_t t);
void DynCycle_normal_planinit(PGDYN* x);
void DynCycle_modify_planinit(PGDYN* x);
float DynCycle_normal_posgenerator(PGDYN* x,DYNCYCLE_NORMAL *tim_node,DYNCYCLE_HALTTIME *ht,uint32_t t);
float DynCycle_modify_posgenerator(PGDYN* x,DYNCYCLE_MODIFT *tim_node,uint32_t t);
void test_DynCycle_para_init(void);

float AmplitudeMinDistance_calcu(PGDYN* x);
float CosineWave(float freq,float t);
float TriangleWave(float freq,float t); //hz s
float RectangleWave(float freq,float t);
float SawtoothWave(float freq,float t); //hz s
float Sawtooth_InverseWave(float freq,float t); //hz s
float PulseWave(float freq,float t);

#endif /* _POSGENERATOR_H_ */
/* *****END OF FILE****/

