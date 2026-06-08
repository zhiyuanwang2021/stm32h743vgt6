# Modbus移植

## 1.文件移植

### 1）将Modbus整个文件夹移入工程文件下



### 2）keil mdk中添加 ./Modbus/inlude 头文件目录



### 3）keil mdk中，在project中添加 modbus group，加入外部.c文件

```
modbus_slave.c
modbus_slave_rtu.c
modbus_common.c
modbus_slave_tcp.c
```



## 2.初始化

### 1）在main.c中引入头文件

```
#include "modbus_slave.h"
```



### 2）将modbusInit()、串口空闲中断，放在main函数初始化部分（这里rs485使用串口2）

```
/* USER CODE BEGIN 2 */
  modbusInit();
  HAL_UARTEx_ReceiveToIdle_IT(&huart2, modbusRxbuf, modbusRxBuffSize);
/* USER CODE END 2 */
```



### 3）串口空闲回调中添加modbus_poll_rec(uint8_t *data, uint16_t len)函数

```
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) 
{
	if(huart->Instance == huart2.Instance)              //判定：串口2接收
	{ 
		modbus_poll_rec(modbusRxbuf, Size);
		
		HAL_UARTEx_ReceiveToIdle_IT(&huart2, modbusRxbuf, modbusRxBuffSize);      //重新开启串口接收     	
	}
}
```



## 3.应用层业务代码编写

### 1）用户在对应的功能码的API中进行代码编写，接口如下：

```
slave_rsp_code_def Slave_ReadHoldReg(uint16_t regAddr, uint16_t regNum, uint8_t *retData);

slave_rsp_code_def Slave_ReadInputReg(uint16_t regAddr, uint16_t regNum, uint8_t *retData);

slave_rsp_code_def Slave_ReadCoilState(uint16_t regAddr, uint16_t num, bool *retData);

slave_rsp_code_def Slave_ReadInputState(uint16_t regAddr, uint16_t regNum, bool *retData);

slave_rsp_code_def Slave_WriteOneReg(uint16_t regAddr, uint16_t data);

slave_rsp_code_def Slave_WriteMulReg(uint16_t regAddr, uint16_t regNum, uint8_t *data);

slave_rsp_code_def Slave_WriteMulCoils(uint16_t regAddr, uint16_t regNum, bool *data);

slave_rsp_code_def Slave_WriteOneCoil(uint16_t regAddr, bool val);
```



### 2）具体业务代码请在对应的函数中实现

针对正位装置已拟协议，初步完成了库的验证（0x04与0x01还未验证）



## 4.注意：

### 1）请勿在业务代码中添加长延时内容

如果有长延时内容请以标志位、信号量等方式放在其他任务或主循环中实现

### 2）本文件最好在UTF-8格式下打开，GB2312可能会乱码

