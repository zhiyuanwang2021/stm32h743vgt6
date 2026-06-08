/**
  ******************************************************************************
  * @file    myFifo.h
  * @author  HCX
  * @version V1.0.0
  * @date    2022-11-6
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#ifndef _MYFIFO_H_
#define _MYFIFO_H_
#include "main.h"
#include "Ethw5500.h"

#define REV_FIFO_SIZE  10 
#define SEND_FIFO_SIZE  10
#define MAX_REV_SIZE 			DATA_BUF_SIZE
#define MAX_SEND_SIZE  		DATA_BUF_SIZE

extern uint8_t	rev_fifo_data[REV_FIFO_SIZE][MAX_REV_SIZE];//用于缓存10s数据
extern uint8_t buf_data_save[MAX_REV_SIZE];
extern uint8_t 	send_fifo_data[SEND_FIFO_SIZE][MAX_SEND_SIZE];
extern uint8_t 	buf_data_send[MAX_SEND_SIZE];

typedef enum 
{
	FIFO_EMPTY,	
	FIFO_NORMAL,		    
	FIFO_FULL	
}FIFO_State;

typedef struct
{
	uint8_t head;
	uint8_t tail;
	uint8_t count;  //队列计数
	uint8_t busy;//出队、入队不可以同时操作
	FIFO_State state;	
} STRUCT_FIFO;
extern STRUCT_FIFO rev_fifo;
extern STRUCT_FIFO send_fifo;
extern STRUCT_FIFO files_fifo;

void fifo_init(STRUCT_FIFO *fifo);
void rev_ring_fifo_push(uint16_t buf_data_points,uint8_t *buf_data);//入队
void rev_ring_fifo_pop(uint8_t *buf_data);//出队
void send_ring_fifo_push(uint16_t buf_data_points,uint8_t* buf_data,uint8_t FuncNum);//入队
void send_ring_fifo_pop(uint8_t* buf_data);//出队
#endif /* _MYFIFO_H_ */
  
