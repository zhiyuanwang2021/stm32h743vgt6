#ifndef _EEPROM_H_
#define _EEPROM_H_
#include "stm32h7xx_hal.h"
#include "i2c.h"
#include "usart.h"
#include "cmsis_os.h"
#include "gParameter.h"

//A2 A1 A0 WR
//8  4  2  1
#define M24C02_0000 0xA0  //0xA0
#define M24C02_Controller M24C02_0000
#define M24C02_0010 0xA2  //0xA0
#define M24C02_0100 0xA4  //0xA0 
#define M24C02_0110 0xA6  //0xA0
#define M24C02_1000 0xA8  //0xA0
#define M24C02_1010 0xAA  //0xA0
#define M24C02_1100 0xAC  //0xA0
#define M24C02_1110 0xAE  //0xA0

uint8_t At24cxx_Write_Byte(uint16_t Device_Addr, uint16_t addr, uint8_t* dat);
uint8_t At24cxx_Write_Word(uint16_t Device_Addr, uint16_t addr, uint16_t* dat);
uint8_t At24cxx_Write_Dword(uint16_t Device_Addr, uint16_t addr, uint32_t* dat);
void At24cxx_Write(uint16_t Device_Addr,uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite);
void At24cxx_WriteLenByte(uint16_t Device_Addr,uint16_t WriteAddr,uint32_t DataToWrite,uint8_t Len);
uint8_t At24cxx_Read_Byte(uint16_t Device_Addr, uint16_t addr, uint8_t* read_buf);
uint8_t At24cxx_Read_Word(uint16_t Device_Addr, uint16_t addr, uint16_t* read_buf);
uint8_t At24cxx_Read_Dword(uint16_t Device_Addr, uint16_t addr, uint32_t* read_buf);
uint8_t ATt24cxx_Read(uint16_t Device_Addr, uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead);
uint32_t At24cxx_ReadLenByte(uint16_t Device_Addr,uint16_t ReadAddr,uint8_t Len);



#endif //_EEPROM_H_
