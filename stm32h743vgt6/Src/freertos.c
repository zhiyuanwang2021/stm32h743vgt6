/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  /* Infinite loop */
  for(;;)
  {
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CommuicationTask */
}

/* USER CODE BEGIN Header_WaveTask */
/**
* @brief Function implementing the waveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WaveTask */
void WaveTask(void const * argument)
{
  /* USER CODE BEGIN WaveTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END EepromTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
