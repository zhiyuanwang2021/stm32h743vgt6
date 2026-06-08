#ifndef  _COMMUNICATE_H_
#define  _COMMUNICATE_H_
#include "EthProtocol.h"
#include "freertos.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

extern osMutexId Controller_statusMutexHandle;
extern osMutexId pgMutexHandle;
extern osMutexId moveparaMutexHandle;
void ETH_Communicate_Process(void);
void CurrentData_Fifo_Push(void);

#endif

