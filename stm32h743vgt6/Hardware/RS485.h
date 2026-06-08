#ifndef _RS485_H_
#define _RS485_H_

#include "main.h"
#include "gParameter.h"

#define RS485_EN_SET    HAL_GPIO_WritePin(P_485DE_GPIO_Port,P_485DE_Pin,GPIO_PIN_SET)
#define RS485_EN_RESET  HAL_GPIO_WritePin(P_485DE_GPIO_Port,P_485DE_Pin, GPIO_PIN_RESET)
	
#define rs485_rxbuf_size 100
#define rs485_txbuf_size 100

typedef enum{
    MB485_mainBoardAddr = 0x0001,
    MB485_manualBoxAddr = 0x0002,
    MB485_regAdd = 0x0001,//MB485 means manual box 485
    MB485_regLen = 0x0008,//MB485 means manual box 485
    MB485_errFunc10 = 0x10, //func10 err
    MB485_errFunc05 = 0x05, //func05 err
    MB485_noErr = 0x00,
    MB485_coilUp = 0x101,
    MB485_coilRapidUp = 0x102,
    MB485_coilDown = 0x103,
    MB485_coilRapidDown = 0x104,
    MB485_coilTestStart = 0x105,
    MB485_coilReset = 0x106,
}manualBox485_e;

typedef enum{
    MB485C_halt = 0x000,//MB485C means manual box 485 coil
    MB485C_up = 0x101,
    MB485C_rapidUp = 0x102,
    MB485C_down = 0x103,
    MB485C_rapidDown = 0x104,
    MB485C_testStart = 0x105,
    MB485C_reset = 0x106,
}manualBox485Coil_e;

typedef struct{
    uint8_t  mainBoardAddr;
    uint8_t  manualBoxAddr;
    uint8_t  rcbuf[100];   //modbus接受缓冲区
 	uint8_t  recount;      //modbus端口接收到的数据个数
 	uint8_t  reflag;       //modbus一帧数据接受完成标志位
 	uint8_t  sendbuf[100]; //modbus接发送缓冲区
}manualBox485_t;

typedef struct{
    uint16_t Bps;// baud rate/100
    uint16_t poseH16;
    uint16_t poseL16;
    uint16_t poseSpeedH16;
    uint16_t poseSpeedL16;
    uint16_t loadH16;
    uint16_t loadL16;
    uint16_t strainH16;
    uint16_t strainL16;
}manualBoxHoldreg_t;

typedef struct
{
    bool DO1;
    bool DO2;
    bool DO3;
    bool DO4;
    bool DO5;
    bool DO6;
    uint16_t word;
}manualBoxCoil_t;

extern manualBox485_t       manualBox485;
extern manualBoxHoldreg_t   manualBoxHoldreg;
extern manualBoxCoil_t      manualBoxCoil;

extern uint8_t rs485_rxbuf[rs485_rxbuf_size];//串口单字节接收缓冲区
extern uint8_t rs485_txbuf[rs485_txbuf_size];//串口单字节接收上位机Modbus指令缓冲区

extern uint8_t buf_485[3];

void RS485_init(UART_HandleTypeDef *huartx);
void RS485_test(void);

// the request of writing multi holdregs
void manualBox485Func10Request(manualBox485_t *_mb485,manualBoxHoldreg_t *_mbHoldreg,
		const float _pose,const float _poseSpeed,const float _load,const float _strain);
// the response callBack of writing multi holdregs
manualBox485_e manualBox485Func10ResponseCallBack(manualBox485_t *_mb485);
// the response of writing multi holdregs
manualBox485_e manualBox485Func05Response(manualBox485_t *_mb485,manualBoxCoil_t *_mbCoil);

uint8_t manualBoxCoilUpdate(const uint8_t _coilNum,const bool _state);

#endif
