/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "control.h"
#include "in_out.h"
#include "trackplaning.h"
#include "userdebug.h"
#include "RS485.h"
#include "ETHw5500.h"
#include "EthProtocol.h"
#include "usart.h"
#include "Servo_driver.h"
#include "posGenerator.h"
#include "gParameter.h"
#include "Encoder.h"
#include "communicate.h"
#include "Eeprom_manage.h"
#include "sensor.h"
#include "td.h"
#include "DI.h"
#include "lsm.h"
#include "pid.h"
#include <elog.h>
#include "mram_manage.h"
#include "adrc.h"
#include "CS5552.h"
// #include "CS553X.h"  // Legacy CS5530 header disabled during CS5552 migration.

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIM6_PRINT_10MS_INTERVAL_MS  10U
#define TIM6_PRINT_1S_INTERVAL_MS    1000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint32_t loop_t=0;
uint16_t time13Count = 0;
uint32_t okCounter = 0;
volatile uint32_t tim6SignalCounter = 0;
volatile uint8_t tim6PrintFlag = 0;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId loopTaskHandle;
osThreadId watchdogTaskHandle;
osThreadId comTaskHandle;
osThreadId waveTaskHandle;
osThreadId ethernetTaskHandle;
osThreadId cdatatranstaskHandle;
osThreadId eepromTaskHandle;
osMessageQId lpusTANQueueHandle;
osMutexId Controller_statusMutexHandle;
osMutexId pgMutexHandle;
osMutexId moveparaMutexHandle;
osSemaphoreId cs5552SemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Controller_status_update(uint16_t status_all);//???????????????????±?????????§???????è??????ì????????????????????????  ?????????ì????????è???????????è??????°????????????????????
static uint32_t Tim6GetUpdateFrequencyHz(void);
static uint32_t Tim6GetIntervalCount(uint32_t interval_ms, uint32_t update_hz);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void LoopTask(void const * argument);
void WatchdogTask(void const * argument);
void CommuicationTask(void const * argument);
void WaveTask(void const * argument);
void EthernetTask(void const * argument);
void CDataTransTask(void const * argument);
void EepromTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
static uint32_t Tim6GetUpdateFrequencyHz(void)
{
  uint32_t pclk1_hz = HAL_RCC_GetPCLK1Freq();
  uint32_t tim6_clock_hz = pclk1_hz;
  uint32_t timer_divisor;

  if (HAL_RCC_GetHCLKFreq() != pclk1_hz)
  {
    tim6_clock_hz *= 2u;
  }

  timer_divisor = (htim6.Init.Prescaler + 1u) * (htim6.Init.Period + 1u);
  if (timer_divisor == 0u)
  {
    return 1u;
  }

  return tim6_clock_hz / timer_divisor;
}

static uint32_t Tim6GetIntervalCount(uint32_t interval_ms, uint32_t update_hz)
{
  uint32_t interval_count;

  interval_count = (update_hz * interval_ms + 999u) / 1000u;
  if (interval_count == 0u)
  {
    interval_count = 1u;
  }

  return interval_count;
}

/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  Static_Set_Output_Log_Format();
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of Controller_statusMutex */
  osMutexDef(Controller_statusMutex);
  Controller_statusMutexHandle = osMutexCreate(osMutex(Controller_statusMutex));

  /* definition and creation of pgMutex */
  osMutexDef(pgMutex);
  pgMutexHandle = osMutexCreate(osMutex(pgMutex));

  /* definition and creation of moveparaMutex */
  osMutexDef(moveparaMutex);
  moveparaMutexHandle = osMutexCreate(osMutex(moveparaMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of cs5552Sem */
  osSemaphoreDef(cs5552Sem);
  cs5552SemHandle = osSemaphoreCreate(osSemaphore(cs5552Sem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of lpusTANQueue */
  osMessageQDef(lpusTANQueue, 16, uint16_t);
  lpusTANQueueHandle = osMessageCreate(osMessageQ(lpusTANQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityRealtime, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of loopTask */
  osThreadDef(loopTask, LoopTask, osPriorityRealtime, 0, 2048);
  loopTaskHandle = osThreadCreate(osThread(loopTask), NULL);

  /* definition and creation of watchdogTask */
  osThreadDef(watchdogTask, WatchdogTask, osPriorityLow, 0, 128);
  watchdogTaskHandle = osThreadCreate(osThread(watchdogTask), NULL);

  /* definition and creation of comTask */
  osThreadDef(comTask, CommuicationTask, osPriorityNormal, 0, 512);
  comTaskHandle = osThreadCreate(osThread(comTask), NULL);

  /* definition and creation of waveTask */
  osThreadDef(waveTask, WaveTask, osPriorityHigh, 0, 256);
  waveTaskHandle = osThreadCreate(osThread(waveTask), NULL);

  /* definition and creation of ethernetTask */
  osThreadDef(ethernetTask, EthernetTask, osPriorityNormal, 0, 1024);
  ethernetTaskHandle = osThreadCreate(osThread(ethernetTask), NULL);

  /* definition and creation of cdatatranstask */
  osThreadDef(cdatatranstask, CDataTransTask, osPriorityNormal, 0, 256);
  cdatatranstaskHandle = osThreadCreate(osThread(cdatatranstask), NULL);

  /* definition and creation of eepromTask */
  osThreadDef(eepromTask, EepromTask, osPriorityLow, 0, 512);
  eepromTaskHandle = osThreadCreate(osThread(eepromTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  uint32_t tim6UpdateHz = Tim6GetUpdateFrequencyHz();
  uint32_t tim6Print10msCount = Tim6GetIntervalCount(TIM6_PRINT_10MS_INTERVAL_MS, tim6UpdateHz);
  uint32_t tim6Print1sCount = Tim6GetIntervalCount(TIM6_PRINT_1S_INTERVAL_MS, tim6UpdateHz);

  /* Clear the initial token created by the binary semaphore. */
  osSemaphoreWait(cs5552SemHandle, 0);

  /* Infinite loop */
  for(;;)
  {
        if (osSemaphoreWait(cs5552SemHandle, osWaitForever) == osOK)
        {
          // taskENTER_CRITICAL();
          // HAL_GPIO_WritePin(P_ADCCS1_GPIO_Port, P_ADCCS1_Pin, GPIO_PIN_SET);
          
          CS5552_Tim6SampleAndFilter();
          MD_CODE_map(sensorCheck.MapNum);
          // HAL_GPIO_WritePin(P_ADCCS1_GPIO_Port, P_ADCCS1_Pin, GPIO_PIN_RESET);
          // taskEXIT_CRITICAL();
          tim6SignalCounter++;
          tim6PrintFlag = 1u;

          if ((tim6SignalCounter % tim6Print10msCount) == 0u)
          {
            printf("%.6f\r\n", g_voltage_filtered_ch0);
          }

          if (tim6SignalCounter >= tim6Print1sCount)
          {
            printf("-----%lu-----%lu-----\r\n",
                   (unsigned long)ch0_valid_sample_count,
                   (unsigned long)ch0_invalid_sample_count);
            ch0_valid_sample_count = 0u;
            ch0_invalid_sample_count = 0u;
            tim6SignalCounter = 0u;
          }
        }
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_LoopTask */
/**
* @brief Function implementing the loopTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LoopTask */
void LoopTask(void const * argument)
{
  /* USER CODE BEGIN LoopTask */
	uint16_t i=0;
  uint8_t sensorcode_counter=0;
	//test_DynCycle_para_init();
  pidInitPos();
  pidInitLoad();
  pidInitExt();
	//DynCycle_planinit();
  /* Infinite loop */
  for(;;)
  {
	  // loop_t++;
	  // if(loop_t >= dyncycle_modify.t5+500)
	  // //if(loop_t >= dyncycle_normal.t3+100)
	  // {
		// pgdyn.waveform++;
		//   if(pgdyn.waveform > 5)
		// 	  pgdyn.waveform = 0;
		// loop_t =0;
		//   DynCycle_planinit();
	  // }
	  // pgdyn.pos=DynCycle_posgenerator(&pgdyn,loop_t);//test for dynCycle

    //eth heart beat watchdog
	  //ETH_HeartbeatWatchdog(&stateFlag.PC_ack_state,ETH_HEARTBEAT_THRESHOLD);
    
    //mointor work mode
    workModeMonitor(&wm,&ws);
	  
	  EnableState_Update();//??è??????§???????è???????????¤????????????°??
	  
    //openloop param update
	  openloop_parameters_update();
		
		//??è??????????????????§Sendata????????????°??
		if(mySemaphore.comm.sendataUpdate == 1)
		{
      log_i("sendata_update() run before");
			sendata_update();//?????????????????°???§?????????°??????????????????????? ????¤??????????????????????????§?????????????????????????????????????????§?????
			//??????????????EEPROM???????????????è???????????¤??????	
      if(sensorConnector == ch0Pose)
        mySemaphore.mram.write_sendataPose = 1;
      else if(sensorConnector == ch1Bd)
        mySemaphore.mram.write_sendataBigDerformation = 1;
      else
        mySemaphore.eeprom.write_sendataOther = 1;

		}
		stateFlag.OpenDevice_state = Device_Open;
		if(stateFlag.OpenDevice_state == Device_Open)//Device_Open  
		{
      if(++sensorcode_counter>=10)      
      {
        sensorcode_counter=0;
        mySemaphore.mram.write_sensorcode = 1;
      }
			inputGetValue();
			inputMapping(); 
      //scram button == 0 stop srv
			if(stateFlag.DoPE_state == DoPE_On)//DoPE_On  
			{
				switch(stateFlag.DoPESetCtrl)//DoPESetCtrl  
				{
					case CtrlTRUE:
								controlLoop();
								if(i++ >= 100){i = 0;LEDlight_Tog();}
					break;
					case CtrlFALSE:
								
								svPWM.pos=0;
								if(i++ >= 1000){i = 0;LEDlight_Tog();}
					break;
				}
			}
			else
			{
				svPWM.pos = 0;
				svAO.pos = 0;
				if(i++ >= 2000){i = 0;LEDlight_Tog();}
			} 

			outputMapping();
			outputControl();
		}
		else
		{
			LEDlight_Off();
		}
    osDelay(1);
  }
  /* USER CODE END LoopTask */
}

/* USER CODE BEGIN Header_WatchdogTask */
/**
* @brief Function implementing the watchdogTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WatchdogTask */
void WatchdogTask(void const * argument)
{
  /* USER CODE BEGIN WatchdogTask */
	uint16_t loopCounter = 0;
  /* Infinite loop */
  for(;;)
  {
	  if(loopCounter++ >= 20){
		  loopCounter = 0;
		 // log_i("monitor running");
	  }
	  //system reset
    if(mySemaphore.systemReset.reset == 1){
      stm32SoftReset();
    }

	  //Eth task monitor
	  if(taskMonitor.ethFood == 1){
		taskMonitor.ethFood = 0;
		taskMonitor.ethDog = 0;
	  }else{
		taskMonitor.ethDog++;
		  if(taskMonitor.ethDog >= 40){
			  taskMonitor.ethDog = 0;
			  log_e("Eth taskMonitor Trigger,Restart Eth task");
			//restart task
			if( ethernetTaskHandle != NULL )
			{
				vTaskDelete( ethernetTaskHandle );
				//Eth init
				ETHw5500_init();
				osThreadDef(ethernetTask, EthernetTask, osPriorityNormal, 0, 1024);
				ethernetTaskHandle = osThreadCreate(osThread(ethernetTask), NULL);
				log_i("Success Restart Eth task");
			}else{
				osThreadDef(ethernetTask, EthernetTask, osPriorityNormal, 0, 1024);
				ethernetTaskHandle = osThreadCreate(osThread(ethernetTask), NULL);
				log_i("Success Restart Eth task");
				
			}
		  }
		  
	  }
	  
	  //Comm task monitor
	  if(taskMonitor.commFood == 1){
		taskMonitor.commFood = 0;
		taskMonitor.commDog = 0;
	  }else{
		taskMonitor.commDog++;
		  if(taskMonitor.commDog >= 40){
			  taskMonitor.commDog = 0;
			  log_e("Comm taskMonitor Trigger,Restart Comm task");
			//restart task
			if( comTaskHandle != NULL )
			{
				vTaskDelete( comTaskHandle );
				osThreadDef(comTask, CommuicationTask, osPriorityLow, 0, 512);
				comTaskHandle = osThreadCreate(osThread(comTask), NULL);
				log_i("Success Restart Comm task");
			}else{
				osThreadDef(comTask, CommuicationTask, osPriorityLow, 0, 512);
				comTaskHandle = osThreadCreate(osThread(comTask), NULL);
				log_i("Success Restart Comm task");
			}
			
		  }
		  
	  }
	  
    osDelay(50);
  }
  /* USER CODE END WatchdogTask */
}

/* USER CODE BEGIN Header_CommuicationTask */
/**
* @brief Function implementing the comTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CommuicationTask */
void CommuicationTask(void const * argument)
{
  /* USER CODE BEGIN CommuicationTask */
	uint16_t loopCounter = 0;
  portTASK_USES_FLOATING_POINT();
  /* Infinite loop */
  for(;;)
  {
	  taskMonitor.commFood = 1;
		ETH_Communicate_Process();
    // if(loopCounter % 300 == 0){
		//   RS485_test();
    // }
	  if(loopCounter++ >= 900){
		  loopCounter = 0;
		//log_d("CommuicationTask running normal");
	  }
    osDelay(1);
  }
  /* USER CODE END CommuicationTask */
}

/* USER CODE BEGIN Header_WaveTask */
extern void forceTareTestPrint(void);
void taskListPrint(char * buf){
  vTaskList(buf);
  printf("Task List\r\n");
  printf("Name\t\tState\tPriority\tStack\tNum\t\r\n");
  printf("%s", buf);
  printf("B:Block  R:Ready  D:Delete  S:suspend  X:running \r\n");
  vTaskGetRunTimeStats(buf); 
  printf("Name\t\trunCount\t\tCPU_usage \r\n");
  printf("%s\r\n",buf);
}
uint8_t taskListBuffer[500];
/**
* @brief Function implementing the waveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WaveTask */
void WaveTask(void const * argument)
{
  /* USER CODE BEGIN WaveTask */
	//static char buffer[1024] = {0};
  static uint32_t counter = 0;
  // static uint32_t counterInt = 0;
  /* Infinite loop */
  for(;;)
  {
    //printf("DB9.ADJ1.value:%d,DB9.ADJ2.value:%d,DB9.ADJ3.value:%d,cdatatrans.SensorKeyPressed:%d\r\n",DB9.ADJ1.value,DB9.ADJ2.value,DB9.ADJ3.value,cdatatrans.SensorKeyPressed);
    // printf("%d,%d,%d,%f,%f,%f,%d,%d,%d,%f\r\n",cs5530.Code[cs5530Channel1],cs5530.Code[cs5530Channel2],cs5530.Code[cs5530Channel3],
    //                               cs5530.Voltage[cs5530Channel1],cs5530.Voltage[cs5530Channel2],cs5530.Voltage[cs5530Channel3],
    //                               time13Count,okCounter,force.code,force.orig);
  
  
  counter++;
  if(counter >= 100){
      counter = 0;
      
      //printf("%d,%d,%f,%f,%f\r\n",bigDeformationLower.code,bigDeformationUpper.code,bigDeformationLower.orig,bigDeformationUpper.orig,bigDeformationSum.orig);
      //printf("PWM:%f\r\n",hal_output.PWM);
      //printf("workMode:%d,%d,%x,%x,%x,%x,%f,%f,%f\r\n",wm.now,wm.last,ws.moveReady,ws.emergency,ws.limitTypeWord,ws.paraConfig,extAllCtrlPara.Sft.UpperSft,extAllCtrlPara.Sft.LowerSft,strain1.filter);
      //printf("type:%d\r\n",manualBox.type);
      //printf("value:%d,fvalue:%f\r\n",AL.tare.value[ch4Load],AL.tare.fValue[ch4Load]);
    }
    // extern void rigidityLsmIdentify(uint16_t filter_depth,const float _xOrig,const float _yOrig);
    // rigidityLsmIdentify(100,pose.orig,force.filter);
    // rigidityUpdateRLS(&rigidityRLS,pose.orig,force.filter,0.95);
    //printf("%f,%f,%f,%f,%f,%f,%f\r\n",rigidityRLS.S_xx,rigidityRLS.S_xy,rigidityRLS.k,lsRigidity.k,lsRigidity.kFilter,lsRigidity.x_m,lsRigidity.y_m);

    // printf("%f,%d,%f,%d,%f,%f,%f,%f,%f,%f,%f\r\n",hal_output.PWM,pose.code,pose.orig,AL.openloopFrq,
    //   test.monitor1,test.monitor2,test.monitor3,test.monitor4,test.monitor5,test.monitor6,test.fclk );
//#define DEBUG_LOADCRTOL_250310
#ifdef DEBUG_LOADCRTOL_250310
  printf("%f,%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f\r\n",
    pose.orig,
    pgLoad.st,
    force.filter,
    AL.loadCtrl.detaLoopCtrlOut,
    AL.loadCtrl.posLoopCtrlOut,
    svPWM.load,
    pid_load.out,
    posCtrlPeriod.real,
    speedForce.filter,
    pid_load.kiSeparate,
    pid_load.kpSeparate,
    lsRigidity.k);
#endif

//#define DEBUG_EXTCRTOL_250320
#ifdef DEBUG_EXTCRTOL_250320
  printf("%f,%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%f\r\n",
    pose.orig,
    pgExt.st,
    strain1.filter,
    AL.ext1Ctrl.detaLoopCtrlOut,
    AL.ext1Ctrl.posLoopCtrlOut,
    svPWM.ext,
    pid_ext.out,
    posCtrlPeriod.real,
    speedStrain.filter,
    pid_ext.kp,
    pid_ext.ki,
    pid_ext.out,
    pid_ext.err,
    pid_ext.setValue);
#endif

//#define DEBUG_POS_CALIBRATE_250110
#ifdef DEBUG_POS_CALIBRATE_250110
  printf("%d,%.6f,%d,%.15f,%f,%f,%f,%d,%d,%d\r\n",pose.code,pose.orig,sensorCalibrate.zeroCode[ch0Pose],AL.posCtrl.NominalSensitive,
                                   test.monitor1,test.monitor2,test.monitor3,test.monitorCode1,test.monitorCode2,test.index);
#endif

//#define DEBUG_LOAD_250108
#ifdef DEBUG_LOAD_250108
  printf("%f,%f,%f,%d,%f,%f,%f,%f,%f\r\n",pgLoad.st,force.filterTrans,force.filter,force.code,AL.tare.fValue[ch4Load],AL.loadCtrl.NominalSensitive,AL.loadCtrl.NominalValue,pid_load.kiSeparate,pid_load.kpSeparate);
#endif



//#define DEBUG_PULSE_250103
#ifdef DEBUG_PULSE_250103
  printf("%f,%f,%d,%d,%d\r\n",(pg.st-pose.orig),test.fclk,test.reloadVal,test.reloadValHalf,test.pcs);
#endif

//#define DEBUG_POSEXT
#ifdef DEBUG_POSEXT
  if((sign(posext.Destination)*(posext.Destination - force.filter) <= 0)
                && mySemaphore.move.pgInit == 0)
  printf("%f,%f,%d,%f\r\n",posext.Destination,force.filter,mySemaphore.move.pgInit,sign(posext.Destination)*(posext.Destination - force.filter));
#endif


//#define DEBUG_STRIAN
#ifdef DEBUG_STRIAN
  printf("%f,%f,%f,%d\r\n",pgExt.st,strain1.orig,strain1Lsm.filter,strain1.code);
#endif

//#define DEBUG_POSE_UP
#ifdef DEBUG_POSE_UP
    printf("%f,%f,%f,%f\r\n",pg.st,pose.orig,pg.vt,speedPose.filter);
#endif

//#define DEBUG_TASKLIST
#ifdef DEBUG_TASKLIST
  if(++countTaskList>=10){
    countTaskList = 0;
    taskListPrint((char *)taskListBuffer);
  }
#endif
//#define DEBUG_CRASH_241011
#ifdef DEBUG_CRASH_241011
    printf("%f,%f,%f,%f\r\n",svPWM.pos,pose.orig,force.filter,force.filterTrans);
#endif

#define DEBUG_LOAD_SENSOR
#ifdef DEBUG_LOAD_SENSOR
    // printf("%d,%f,%f,%f,%f,%d,%f,%f,%f,%d,%d\r\n",force.code,force.orig,force.filter,forceLsm.filter,force.filterTrans,AL.tare.value[ch4Load],AL.tare.fValue[ch4Load],
    //                     AL.loadCtrl.NominalSensitive,AL.loadCtrl.NominalValue,filterLen.loadLen,SenData[ch4Load].sensorIntgr);
#endif

//#define DEBUG_TEST
#ifdef DEBUG_TEST
    printf("%.10f,%.10f,%.10f,%d,%d\r\n",test.monitor1,test.monitor2,test.monitor3,test.monitorCode1,test.monitorCode2);
#endif

//#define DEUBG_PID_POS
#ifdef DEUBG_PID_POS
    printf("%f,%lf,%lf,%f,%f,%d,%d\r\n",AL.posCtrl.posLoopCtrlOut,leso.z2,leso.z1,pg.st_last,pose.orig,servoPulse.out,pid_pos.calcuRun);
    //printf("%lf,%lf,%lf,%f,%f,%f,%d,%d,",leso.u,leso.y,leso.e,hal_output.PWM,svPWM.pos,test.fclk,__HAL_TIM_GET_COUNTER(&htim8),test.esoCounter);
    //printf("%d,%f,%f\r\n",counterInt,force.filter,force.filterTrans);
    //printf("posCtrlPeriod.real %d vm:%f\r\n",posCtrlPeriod.real,pg.vm);
#endif // DEBUG

    //printf("%f,%f,%f,%f,%f,%d,%f \r\n",leso.z1,pg.st_last,pose.orig,leso.z2,svPWM.pos,AL.complete_state.flag,AL.servoOutput);
    //printf("%d,%d,%d,%d,%f\r\n",wm.now,wm.last,ws.emergency,ws.loopCtrl,speedPose.filter);
//#define DEBUG_LOAD
#ifdef DEBUG_LOAD
    printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d\r\n",AL.loadCtrl.detaLoopCtrlOut,AL.loadCtrl.posLoopCtrlOut,AL.loadCtrl.speedLoopCtrlOut,force.filter,pgLoad.st_last,pgLoad.st_last-force.filter,svPWM.load,pose.orig,pg.st_last,hal_output.PWM,hal_output.pwmIntCounter,test.pcs,test.reloadVal,test.reloadValHalf,test.fclk,posCtrlPeriod.real);
#endif // DEBUG
//#define DEBUG1_9_2
#ifdef DEBUG1_9_2 
    //printf("%f,%f,%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%d,%d,%d,%f,%f,%f,%f,%f,%d\r\n",pg.st_last,pose.orig,fhan.x1,fhan.x2,fhan.fh,pg.vt,svPWM.pos,leso.z2,tim6us,AL.servoOutput,forceLsm.filter,force.filter,hal_output.PWM,hal_output.pwmIntCounter,AL.complete_state.flag,pwms1,pwms2,pwms3,pid_pos.out,AL.posCtrl.posLoopCtrlOut,posCtrlPeriod.real);
    printf("%f,%f,%f,%f,%f,%d,%d,%f \r\n",pgLoad.st_last,force.filter,pg.st_last,pose.orig,svPWM.pos,hal_output.PWM,posCtrlPeriod.real,svPWM.pos * posCtrlPeriod.real / 1000.0);

#endif

//#define DEBUG1_9_4
#ifdef DEBUG1_9_4
    printf("%d,%d\r\n",SenData[ch4Load].sensorIntgr,SenData[ch0Pose].sensorIntgr);
#endif
    //forceTareTestPrint();
	  osDelay(10);
  }
  /* USER CODE END WaveTask */
}

/* USER CODE BEGIN Header_EthernetTask */
/**
* @brief Function implementing the ethernetTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EthernetTask */
void EthernetTask(void const * argument)
{
  /* USER CODE BEGIN EthernetTask */
  portTASK_USES_FLOATING_POINT();
	uint16_t loopCounter = 0;
  /* Infinite loop */
  for(;;)
  {
	  taskMonitor.ethFood = 1;
	  
		ETHw5500_Func();
	  if(loopCounter++ >= 200){
		  loopCounter = 0;
		//log_d("EthernetTask running normal");
	  }
    //log_d("EthernetTask running normal");
    osDelay(5);
  }
  /* USER CODE END EthernetTask */
}

/* USER CODE BEGIN Header_CDataTransTask */
/**
* @brief Function implementing the cdatatranstask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CDataTransTask */
void CDataTransTask(void const * argument)
{
  /* USER CODE BEGIN CDataTransTask */
	// uint16_t loopCounter = 0;
  /* Infinite loop */
  for(;;)
  {	  
	if(stateFlag.OpenDevice_state == Device_Open)//Device_Open  
	{
//		if(stateFlag.DoPE_state == DoPE_On)//DoPE_On  
//		{  
			if(stateFlag.DoPETransmitData == TRUE)
			{	
				CurrentData_Fifo_Push();
        //printf("periodic data trans running\r\n");
			}
//		}
	} 
	
	if(mySemaphore.comm.transperiodUpdate == 1)
	{
		SysPara.TransmitDataPeriod = commBuf.SysPara.TransmitDataPeriod;
		mySemaphore.mram.write_transmitdataperiod = 1;
		mySemaphore.comm.transperiodUpdate = 0;
		
	}
//	if(loopCounter++ >= 5){
//		loopCounter = 0;
//		log_d("CDataTransTask running normal");
//	  }
    osDelay(SysPara.TransmitDataPeriod);
  }
  /* USER CODE END CDataTransTask */
}

/* USER CODE BEGIN Header_EepromTask */
/**
* @brief Function implementing the eepromTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EepromTask */
void EepromTask(void const * argument)
{
  /* USER CODE BEGIN EepromTask */
  uint16_t initWaitCounter = 0;
  /* Infinite loop */
  for(;;)
  {
    //DB9 senser Type Map corresponding SENSOR_CHANNEL
#if USE_DB9_CON
	  sensorTypeMap(&mySemaphore.db9);
#else
    //need wait 1s after power on
    if(initWaitCounter++ >= 100){
      initWaitCounter = 100;
      sensorConnectCheckByEeprom(&halDB9);
    }
#endif
    manualBoxTypeMonitor(&manualBox);
    mram_write_monitor(&mySemaphore.mram);
    eeprom_write_monitor(&mySemaphore.eeprom);
    // CS5552 compatibility recovery hook is currently disabled here.
  // CS5552_CompatResetMonitor(&cs5552_compat_ctx);
  
  
    osDelay(10);
  }
  /* USER CODE END EepromTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
