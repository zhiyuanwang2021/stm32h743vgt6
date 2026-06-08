/**
  ******************************************************************************
  * @file    myFifo.c
  * @author  HCX
  * @version V1.0.0
  * @date    2022-11-6
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "myFifo.h"
#include "usart.h"
#include <cstring>
#include "elog.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
STRUCT_FIFO rev_fifo;
STRUCT_FIFO send_fifo;
STRUCT_FIFO files_fifo;
uint8_t	rev_fifo_data[REV_FIFO_SIZE][MAX_REV_SIZE];//用于缓存10s数据
uint8_t buf_data_save[MAX_REV_SIZE];
uint8_t 	send_fifo_data[SEND_FIFO_SIZE][MAX_SEND_SIZE];
uint8_t 	buf_data_send[MAX_SEND_SIZE];
/*
函数名：fifo_init
功能：fifo参数初始化
*/
void fifo_init(STRUCT_FIFO *fifo)
{	
//	printf("fifo_init\r\n");
	while((*fifo).busy ==1)
	{
		HAL_Delay(1);
	}
	(*fifo).busy =1;
	
	(*fifo).count = 0;
	(*fifo).head =0;
	(*fifo).tail =0;
	(*fifo).state = FIFO_EMPTY;
	(*fifo).busy =0;
}

/*
函数名：rev_ring_fifo_push
功能：入队
输入值：
float buf_data_float[MAX_SAMPLERATE];
uint16_t buf_data_points;// 每个数据包的数据点数（解析值）
*/
void rev_ring_fifo_push(uint16_t buf_data_points,uint8_t *buf_data)
{
	uint16_t i;
	
	while(rev_fifo.busy ==1)
	{
		osDelay(1);
	}
	rev_fifo.busy =1;
	
	//输入插入队尾rev_fifo_data[rev_fifo.tail]
	memset(rev_fifo_data[rev_fifo.tail],0,sizeof(rev_fifo_data[rev_fifo.tail]));
		for(i=0;i<buf_data_points;i++)
		{
			rev_fifo_data[rev_fifo.tail][i]=buf_data[i];
		}
	
	//调整队尾队头位置
	rev_fifo.tail = (rev_fifo.tail+1)%REV_FIFO_SIZE;
	
	//状态判断
	rev_fifo.count++;
//	printf("rev_fifo.count=%d\r\n",rev_fifo.count);
	if(rev_fifo.count<REV_FIFO_SIZE)
	{
		rev_fifo.state = FIFO_NORMAL; 
	}
	else
	{
		rev_fifo.count =	REV_FIFO_SIZE;
		rev_fifo.state = FIFO_FULL;
		rev_fifo.head 	= rev_fifo.tail;	//当出现rev_fifo.coun>REV_FIFO_SIZE，要确保head指针与tail指针同步，保正队列的顺序
	}
	
	rev_fifo.busy =0;
}

/*
函数名：rev_ring_fifo_pop
功能：出队
输出值：
uint8_t buf_data_save[MAX_SEND_SIZE];
uint16_t buf_data_points;// 每个数据包的数据点数（解析值）
*/
void rev_ring_fifo_pop(uint8_t *buf_data)
{
	uint16_t i,DataLen;

	while(rev_fifo.busy ==1)
	{   
		osDelay(1);
	}
	if(rev_fifo.state == FIFO_EMPTY) return;	
	
	rev_fifo.busy =1;	
	
	//队头出队 rev_fifo_data[myFifo.head]-->
	DataLen = (rev_fifo_data[rev_fifo.head][3]*256+rev_fifo_data[rev_fifo.head][4])+5;
//	printf("poplen:%d\r\n",DataLen);
		for(i=0;i<DataLen;i++)
		{
			buf_data[i]=rev_fifo_data[rev_fifo.head][i];
		}
	memset(rev_fifo_data[rev_fifo.head],0,sizeof(rev_fifo_data[rev_fifo.head]));
	//调整队头位置
	rev_fifo.head = (rev_fifo.head+1)%REV_FIFO_SIZE;	
	
	//状态判断
	rev_fifo.count--;
	if(rev_fifo.count>0)
	{
		rev_fifo.state = FIFO_NORMAL; 
	}
	else
	{
		rev_fifo.state = FIFO_EMPTY; 
	}	
	
	rev_fifo.busy =0;
}

/*
函数名：send_ring_fifo_push
功能：入队
输入值：
char 	buf_data_send_json[MAX_SEND_PACK_SIZE];
输出值：
char 	send_fifo_data[SEND_FIFO_SIZE][MAX_SEND_PACK_SIZE];
*/
void send_ring_fifo_push(uint16_t buf_data_points,uint8_t* buf_data,uint8_t FuncNum)
{
	uint16_t i;
	
	while(send_fifo.busy ==1)
	{
		osDelay(1);
	}	
	//if(send_fifo.state == FIFO_FULL) return;//如果满了则加不进去	
	
	send_fifo.busy =1;
	send_fifo_data[send_fifo.tail][0] = FuncNum;//功能码
	send_fifo_data[send_fifo.tail][1] =(uint8_t)(buf_data_points>>8);//有效载荷数据长度
	send_fifo_data[send_fifo.tail][2] =(uint8_t)(buf_data_points);
	
	//输入插入队尾send_fifo_data[send_fifo.tail][MAX_SEND_PACK_SIZE];
	for(i=0;i<buf_data_points;i++)
	{
		send_fifo_data[send_fifo.tail][i+3]=buf_data[i];
	}
	
	//调整队尾队头位置
	send_fifo.tail = (send_fifo.tail+1)%SEND_FIFO_SIZE;
	
	//状态判断
	send_fifo.count++;
	if(send_fifo.count<SEND_FIFO_SIZE)
	{
		send_fifo.state = FIFO_NORMAL; 
	}
	else
	{
		send_fifo.count =	SEND_FIFO_SIZE;
		send_fifo.state = FIFO_FULL; 
		send_fifo.head 	= (send_fifo.tail+1)%SEND_FIFO_SIZE;
	}
	
	send_fifo.busy =0;
	
//	printf("state=%d  count=%d  head=%d  tail=%d\r\n",send_fifo.state,send_fifo.count,send_fifo.head,send_fifo.tail);
}

/*
函数名：send_ring_fifo_pop
功能：出队
输入：char 	send_fifo_data[SEND_FIFO_SIZE][MAX_SEND_PACK_SIZE];
输出：char 	buf_data_send[MAX_SEND_PACK_SIZE];
*/
void send_ring_fifo_pop(uint8_t* buf_data)
{
	uint16_t i;
	uint16_t buf_data_points;

	while(send_fifo.busy ==1)
	{
		osDelay(1);
	}
	
	if(send_fifo.state == FIFO_EMPTY) return;	
	
	send_fifo.busy =1;	
	
	buf_data_points = (send_fifo_data[send_fifo.head][1] << 8) + send_fifo_data[send_fifo.head][2];
	//队头出队 send_fifo_data[send_fifo.head]
	for(i=0;i<buf_data_points+3;i++)
	{
		buf_data[i]=send_fifo_data[send_fifo.head][i];
	}
	memset(send_fifo_data[send_fifo.head],0,sizeof(send_fifo_data[send_fifo.head]));//出队后清楚对应行
	//调整队头位置
	send_fifo.head = (send_fifo.head+1)%SEND_FIFO_SIZE;	
	
	//状态判断
	send_fifo.count--;
	if(send_fifo.count>0)
	{
		send_fifo.state = FIFO_NORMAL; 
	}
	else
	{
		send_fifo.state = FIFO_EMPTY;
	}	
	
	send_fifo.busy =0;
	//printf("state=%d  count=%d  head=%d  tail=%d\r\n",send_fifo.state,send_fifo.count,send_fifo.head,send_fifo.tail);
}
/*--------------End of myFifo.c-----------------*/

