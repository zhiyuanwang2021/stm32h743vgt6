/**
******************************************************************************
* @file         ETHw5500.h
* @version      V1.0
* @date         2023-08-23                
* @brief        ETHw5500.c的头文件
******************************************************************************
*/
#ifndef _ETHw5500_H_
#define _ETHw5500_H_
#include "types.h"
#include "stm32h7xx_hal.h"
#include "socket.h"
#include "bsp_spi.h"
#include "loopback.h"

#define SOCK_TCPC        0
#define SOCK_TCPS        1
#define SOCK_UDPS        2
#define SOCK_UDPS3       3
#define SOCK_UDPS4			 4

#define TCP 0x00
#define UDP 0x01

#define DATA_BUF_SIZE   2048
typedef struct{
uint8_t  Mode;
uint8_t  sn;
uint8_t  Revbuf[DATA_BUF_SIZE];
uint16_t revsize;
uint8_t  Sendbuf[DATA_BUF_SIZE];
uint16_t sendsize;
}UTC_ETH;
extern UTC_ETH utc_eth;

extern wiz_NetInfo gWIZNETINFO;
extern uint8_t dest_ip[4];
extern uint16_t dest_port;		


void ETHw5500_init(void);

void ETHw5500_Looptest(void);	

void ETHw5500_Func(void);

void ETH_StatePrintf(void);



#endif //_ETHw5500_H_
