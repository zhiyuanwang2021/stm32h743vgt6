#include "EEPROM.h"
#define i2cx hi2c2
#define Write_Timeout 20
#define Read_Timeout 20

#define EEPROM_DEBUG


/**
 * @brief		AT24Cxx任意地址写一个字节数据
 * @param		addr —— 写数据的地址（0-32767）
 * @param		dat  —— 存放写入数据的地址
 * @retval		成功 —— HAL_OK
*/
uint8_t At24cxx_Write_Byte(uint16_t Device_Addr, uint16_t addr, uint8_t* dat)
{
	uint8_t ret;
	ret = HAL_I2C_Mem_Write(&i2cx, Device_Addr, addr, I2C_MEMADD_SIZE_16BIT, dat, 1, 0xFFFF);
	if(stateFlag.os_start==0)
		HAL_Delay(5);//此行不可少
	else
		osDelay(5);
	return ret;
}

uint8_t At24cxx_Write_Word(uint16_t Device_Addr, uint16_t addr, uint16_t* dat)
{
	uint8_t byte[2],i;
	
	byte[0]=(*dat & 0xFF00)>>8;
	byte[1]=(*dat & 0x00FF);
	for(i=0;i<2;i++)
	{
		if(At24cxx_Write_Byte(Device_Addr,addr+i,&(byte[i]))) return 1;		
	}	
	
	return 0;
}

uint8_t At24cxx_Write_Dword(uint16_t Device_Addr, uint16_t addr, uint32_t* dat)
{
	uint8_t byte[4],i;
	
	byte[0]=(*dat & 0xFF000000)>>24;
	byte[1]=(*dat & 0x00FF0000)>>16;
	byte[2]=(*dat & 0x0000FF00)>>8;
	byte[3]=(*dat & 0x000000FF);
	for(i=0;i<4;i++)
	{
		if(At24cxx_Write_Byte(Device_Addr,addr+i,&(byte[i]))) return 1;		
	}	
	
	return 0;
}

/**
 * @brief		在AT24C256里面的指定地址开始写入指定个数的数据
 * @param		WriteAddr :开始写入的地址 对24c02为0~32767
 * @param 		pBuffer  :数据数组首地址
 * @param		NumToWrite:要写入数据的个数
 * @retval		成功 —— HAL_OK
*/
void At24cxx_Write(uint16_t Device_Addr,uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)
{
	HAL_I2C_Mem_Write(&i2cx, Device_Addr, WriteAddr, I2C_MEMADD_SIZE_16BIT, pBuffer, NumToWrite, 0xFFFF);
}

/**
 * @brief		在AT24C256里面的指定地址开始写入长度为Len的数据
 * @param		该函数用于写入16bit或者32bit的数据.
 * @param 		WriteAddr  :开始写入的地址  
 * @param		DataToWrite:数据数组首地址
 * @param		Len        :要写入数据的长度2,4
*/

void At24cxx_WriteLenByte(uint16_t Device_Addr,uint16_t WriteAddr,uint32_t DataToWrite,uint8_t Len)
{  	
	uint8_t dat[2]={0,0};
	uint8_t dat4[4]={0,0,0,0};
	
	if(Len==2)
	{
		dat[0]=DataToWrite & 0xFF;		
		dat[1]=(DataToWrite>>8) & 0xFF;
		At24cxx_Write(Device_Addr,WriteAddr,dat,Len);
	}
	else if(Len==4)
	{
		dat4[0]=DataToWrite & 0xFF;		
		dat4[1]=(DataToWrite>>8) & 0xFF;
		dat4[2]=(DataToWrite>>16) & 0xFF;
		dat4[3]=(DataToWrite>>24) & 0xFF;
		At24cxx_Write(Device_Addr,WriteAddr,dat4,Len);
	}									    
}


/**
 * @brief		AT24Cxx任意地址读一个字节数据
 * @param		addr —— 读数据的地址（0-32767）
 * @param		read_buf —— 存放读取数据的地址
 * @retval		成功 —— HAL_OK
*/
uint8_t At24cxx_Read_Byte(uint16_t Device_Addr, uint16_t addr, uint8_t* read_buf)
{
	if(stateFlag.os_start==0)
		HAL_Delay(5);//此行不可少
	else
		osDelay(5);
	return HAL_I2C_Mem_Read(&i2cx, Device_Addr, addr, I2C_MEMADD_SIZE_16BIT, read_buf, 1, Read_Timeout);
}

uint8_t At24cxx_Read_Word(uint16_t Device_Addr, uint16_t addr, uint16_t* read_buf)
{
	uint8_t byte[2],i;
	
	for(i=0;i<2;i++)
	{		
		if(At24cxx_Read_Byte(Device_Addr,addr+i,&(byte[i]))) return 1;
	}	
	*read_buf = (byte[0]<<8)+byte[1];
	
	return 0;
}

uint8_t At24cxx_Read_Dword(uint16_t Device_Addr, uint16_t addr, uint32_t* read_buf)
{
	uint8_t byte[4],i;
	
	for(i=0;i<4;i++)
	{		
		if(At24cxx_Read_Byte(Device_Addr,addr+i,&(byte[i]))) return 1;
	}	
	*read_buf = (byte[0]<<24)+(byte[1]<<16)+(byte[2]<<8)+byte[3];
	
	return 0;
}

/**
 * @brief		在AT24C256里面的指定地址开始读出指定个数的数据
 * @param		ReadAddr :开始读出的地址 对24c02为0~32767
 * @param 		pBuffer  :数据数组首地址
 * @param		NumToRead:要读出数据的个数
 * @retval		成功 —— HAL_OK
*/
uint8_t ATt24cxx_Read(uint16_t Device_Addr, uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)
{
	return HAL_I2C_Mem_Read(&i2cx, Device_Addr, ReadAddr, I2C_MEMADD_SIZE_16BIT, pBuffer, NumToRead, 0xFFFF);

}  
/**
 * @brief		在AT24C256里面的指定地址开始读出长度为Len的数据
 * @param		ReadAddr:开始读出的地址 对24c02为0~32767
 * @param 		Len  	:要读出数据的长度2,4
 * @retval		读出的数据
*/
uint32_t At24cxx_ReadLenByte(uint16_t Device_Addr,uint16_t ReadAddr,uint8_t Len)
{  	
	uint8_t t;
	uint8_t dat;
	uint32_t temp=0;
	
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		At24cxx_Read_Byte(Device_Addr,ReadAddr+Len-t-1,&dat);
		temp+=dat; 	 				   
	}
	return temp;												    
}














