# elog V 0.1 2024-4-2 12:32:24 

此版本实现功能：easylogger优先级log，多任务同步输出，带有互斥量

参照移植文档pdf  [easylogger移植freertos.pdf](..\..\..\easylogger移植freertos.pdf) 

问题：基本原理还是在此时刻调用printf，我们需要的是实现在空闲任务中输出printf，实现多任务队列异步输出。





# elog V 0.2 2024-4-2 23:47:12 

此版本实现功能：easylogger多任务队列异步输出



问题：多任务过于频繁的进队，会导致log信息错位



## easylogger异步输出移植方法

主要工作是将rt-thread的API换成freertos的API，所有移植工作都在文件elog_async.c中进行

### 1.替换#include

原代码

```
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
```

替换代码

```
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os.h"
```



### 2.替换任务通知信号量（这里源代码没有使用任务通知，而是使用信号量）

原代码

```
static sem_t output_notice;
```

替换代码

```
static osSemaphoreId output_noticeHandle;
```



### 3.替换异步输出线程句柄

原代码

```
static pthread_t async_output_thread;
```

替换代码

```
static osThreadId elogasynctaskHandle;
```



### 4.替换异步输出通知函数内部

原代码

```
void elog_async_output_notice(void) {
    sem_post(&output_notice);
}
```

替换代码

```
void elog_async_output_notice(void) {
    xSemaphoreGive(output_noticeHandle);
}
```



### 5.替换异步输出线程函数

原代码

```
static void *async_output(void *arg) {
    size_t get_log_size = 0;
    static char poll_get_buf[ELOG_ASYNC_POLL_GET_LOG_BUF_SIZE];

    while(thread_running) {
        /* waiting log */
        //sem_wait(&output_notice);
        xSemaphoreTake(output_noticeHandle,portMAX_DELAY);
        /* polling gets and outputs the log */
        while(true) {

#ifdef ELOG_ASYNC_LINE_OUTPUT
            get_log_size = elog_async_get_line_log(poll_get_buf, ELOG_ASYNC_POLL_GET_LOG_BUF_SIZE);
#else
            get_log_size = elog_async_get_log(poll_get_buf, ELOG_ASYNC_POLL_GET_LOG_BUF_SIZE);
#endif

            if (get_log_size) {
                elog_port_output(poll_get_buf, get_log_size);
            } else {
                break;
            }
        }
    }
    return NULL;
}
```

替换代码

```
static void ElogAsyncTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  size_t get_log_size = 0;
  static char poll_get_buf[ELOG_ASYNC_POLL_GET_LOG_BUF_SIZE];
  /* Infinite loop */
    while(thread_running) {
        /* waiting log */
        //sem_wait(&output_notice);
        xSemaphoreTake(output_noticeHandle,portMAX_DELAY);
        /* polling gets and outputs the log */
        while(true) {

#ifdef ELOG_ASYNC_LINE_OUTPUT
            get_log_size = elog_async_get_line_log(poll_get_buf, ELOG_ASYNC_POLL_GET_LOG_BUF_SIZE);
#else
            get_log_size = elog_async_get_log(poll_get_buf, ELOG_ASYNC_POLL_GET_LOG_BUF_SIZE);
#endif

            if (get_log_size) {
                elog_port_output(poll_get_buf, get_log_size);
            } else {
                break;
            }
        }
    }
  /* USER CODE END StartDefaultTask */
}
```



### 6.替换elog异步初始化

原代码

```
ElogErrCode elog_async_init(void) {
    ElogErrCode result = ELOG_NO_ERR;

    if (init_ok) {
        return result;
    }

#ifdef ELOG_ASYNC_OUTPUT_USING_PTHREAD
    pthread_attr_t thread_attr;
    struct sched_param thread_sched_param;

    sem_init(&output_notice, 0, 0);

    thread_running = true;

    pthread_attr_init(&thread_attr);
    //pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED);
    pthread_attr_setstacksize(&thread_attr, ELOG_ASYNC_OUTPUT_PTHREAD_STACK_SIZE);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_RR);
    thread_sched_param.sched_priority = ELOG_ASYNC_OUTPUT_PTHREAD_PRIORITY;
    pthread_attr_setschedparam(&thread_attr, &thread_sched_param);
    pthread_create(&async_output_thread, &thread_attr, async_output, NULL);
    pthread_attr_destroy(&thread_attr);
#endif

    init_ok = true;

    return result;
}
```

替换代码

```
ElogErrCode elog_async_init(void) {
    ElogErrCode result = ELOG_NO_ERR;

    if (init_ok) {
        return result;
    }
#ifdef ELOG_ASYNC_OUTPUT_USING_FREERTOS
  //semaphore creat
  osSemaphoreDef(output_notice);
  output_noticeHandle = osSemaphoreCreate(osSemaphore(output_notice), 1);
  thread_running = true;
  //elogtask creat
  osThreadDef(elogasynctask, ElogAsyncTask, osPriorityLow, 0, ELOG_ASYNC_OUTPUT_PTHREAD_STACK_SIZE);
  elogasynctaskHandle = osThreadCreate(osThread(elogasynctask), NULL);
#endif

    init_ok = true;

    return result;
}
```



### 7.替换elog异步反初始化

原代码

```
void elog_async_deinit(void) {
    if (!init_ok) {
        return ;
    }

    #ifdef ELOG_ASYNC_OUTPUT_USING_PTHREAD
        thread_running = false;

        elog_async_output_notice();

        pthread_join(async_output_thread, NULL);
        
        sem_destroy(&output_notice);
    #endif

    init_ok = false;
}
```

替换代码

```
void elog_async_deinit(void) {
    if (!init_ok) {
        return ;
    }
    
#ifdef ELOG_ASYNC_OUTPUT_USING_FREERTOS
    thread_running = false;

    elog_async_output_notice();

    //待添加使async_output线程执行完毕的操作系统函数，也可以使用信号量机制来同步完成
    //pthread_join(async_output_thread, NULL);
    
    vSemaphoreDelete(output_noticeHandle);
#endif

    init_ok = false;
}
```



### 注意

1.在进行异步elog移植之前，需要先实现带有互斥量的同步elog移植工作！参照pdf [easylogger移植freertos.pdf](..\..\..\easylogger移植freertos.pdf) 

2.记得在文件最前进行宏定义 #define ELOG_ASYNC_OUTPUT_USING_FREERTOS

3.后期会把ELOG_ASYNC_OUTPUT_USING_PTHREAD改成以ELOG_ASYNC_OUTPUT_USING_FREERTOS为主的预编译



### 总结

大致完成了异步elog，具体细节和代码格式还需要完善。



# elog V 0.3 2024-4-4 00:16:48



## 针对elog V0.2进行格式整理，完善代码细节



### 1.在文件elg_cfg.h中，定义ELOG_ASYNC_OUTPUT_USING_FREERTOS宏定义参数，使能基于freertos的async

/* asynchronous output mode using POSIX pthread implementation */

//#define ELOG_ASYNC_OUTPUT_USING_PTHREAD

/* asynchronous output mode using freertos elogasynctask implementation */

\#define ELOG_ASYNC_OUTPUT_USING_FREERTOS



