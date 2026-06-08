#ifndef _Computer_vision_H_
#define _Computer_vision_H_

#include "main.h"
#include "usart.h"

#define  uart_putbuff(WARE_UART, buf,size)  HAL_UART_Transmit(&WARE_UART, buf, size, 0xfff)

void vcan_sendware(void *wareaddr, uint32_t waresize);
void vcan_sendimg(void *imgaddr, uint32_t imgsize);
void vcan_sendccd(void *ccdaddr, uint32_t ccdsize);
#endif //_Computer_vision_H_
