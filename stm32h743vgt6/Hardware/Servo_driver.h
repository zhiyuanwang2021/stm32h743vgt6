#ifndef  _SERVO_DRIVER_H_
#define  _SERVO_DRIVER_H_

#include "stm32h7xx_hal.h"
#include "main.h"
/*
	参数选择Pr0.07为3  脉冲列+符号
	0或2			90°位相差2相脉冲列
	1					正方向脉冲列 + 负方向脉冲列
	3					脉冲列 + 符号
*/
#define ServoPr007 3

#define PULSE   0
#define SERVO_DIR     1
#define SSI_CLK 2

#define SON_Pin				P_DO6_Pin 			//启动引脚
#define SON_Port			P_DO6_GPIO_Port
#define ALM_Pin 			P_DI11_Pin 			//警报引脚
#define ALM_Port			P_DI11_GPIO_Port 
#define DIR_Pin    GPIO_PIN_5					//方向引脚
#define DIR_Port 	 GPIOE

//SRV_ON 伺服开启/关闭输入
#define SRV_ON        (HAL_GPIO_WritePin(SON_Port,SON_Pin,GPIO_PIN_SET))
#define SRV_OFF				(HAL_GPIO_WritePin(SON_Port,SON_Pin,GPIO_PIN_RESET))

//DIR 电机旋转方向控制
#define DIR_SET        (HAL_GPIO_WritePin(DIR_Port,DIR_Pin,GPIO_PIN_SET))
#define DIR_RESET				(HAL_GPIO_WritePin(DIR_Port,DIR_Pin,GPIO_PIN_RESET))

//返回1 ALM为高电平 返回0 ALM为低电平(伺服驱动器内部光耦导通) 
#define ALM_Monitor   ((uint8_t)HAL_GPIO_ReadPin(ALM_Port,ALM_Pin))

#define TIM8MCLK 200000000.0f

typedef enum{
	SF_pcs6000 = 0,//SF means servo frequency
	SF_pcs2000 = 1,//SF means servo frequency
	SF_pcs100 = 2,//SF means servo frequency
	SF_pcs50 = 3,//SF means servo frequency
	SF_pcs4 = 4,//SF means servo frequency
	SF_pcs0 = 5,//SF means servo frequency
}servoFreq_e;

typedef enum{
	SFH_fclk6000_1 = 	0,//SFH means servo frequency hysteresis
	SFH_fclk6000_2 = 	6000,//SFH means servo frequency hysteresis
	SFH_fclk2000_1 = 	4000,//SFH means servo frequency hysteresis
	SFH_fclk2000_2 = 	20000,//SFH means servo frequency hysteresis
	SFH_fclk100_1 = 	12000,//SFH means servo frequency hysteresis
	SFH_fclk100_2 = 	100000,//SFH means servo frequency hysteresis
	SFH_fclk50_1 = 		80000,//SFH means servo frequency hysteresis
	SFH_fclk50_2 = 		250000,//SFH means servo frequency hysteresis
	SFH_fclk4_1 = 		200000,//SFH means servo frequency hysteresis
	SFH_fclk4_2 = 		500000,//SFH means servo frequency hysteresis
	SFH_fclk0_1 = 		300000,//SFH means servo frequency hysteresis
	SFH_fclk0_2 = 		2000000,//SFH means servo frequency hysteresis
}servoFreqHysteresis_e;

typedef struct{
	uint32_t out;
	double total;
	double remainder;
}servoPulse_t;

extern servoPulse_t servoPulse;

void Servo_DIR_GPIO_Init(void);//初始化模式3下方向控制引脚
void Servo_PWM_Enable(uint8_t channel);//打开对应定时器对应通道的PWM
void Servo_PWM_Disable(uint8_t channel);//关闭对应定时器对应通道的PWM
void Servo_mod_fclk(uint8_t channel,float fclk); //更改PWM频率
void modFclkNumberPULSEInit(uint32_t fclk);
#endif

