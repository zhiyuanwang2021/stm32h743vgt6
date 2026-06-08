/*******************************************************************************
 *  Copyright (c) [scl]。保留所有权利。
 *     本文仅供个人学习和研究使用，禁止用于商业用途。
 ******************************************************************************/

#ifndef STM32_LIB_MODBUS_SLAVE_H
#define STM32_LIB_MODBUS_SLAVE_H

#include "modbus_common.h"

#define modbusRxBuffSize 100

extern uint8_t modbusRxbuf[modbusRxBuffSize];//串口单字节接收上位机Modbus指令缓冲区

typedef enum {
    rsp_ok = 0x0,
    // 不支持的功能码
    rsp_nut_support_cmd = 0x1,
    // 寄存器地址错误
    rsp_err_reg_addr = 0x2,
    // 数据值域错误
    rsp_err_value = 0x3,
    // 写入失败
    rsp_err_write = 0x4,
} slave_rsp_code_def;
typedef struct {
    uint8_t slave_addr;
    uint8_t func_code;
    uint16_t reg_addr;
    uint16_t reg_num;
    uint8_t *data;
    uint16_t data_len;
} mb_slave_parse_data_t; /*modbus从机解析数据结构体定义*/
typedef struct {
    uint8_t slave_addr; /*从机地址*/
    modbus_trans_mode trans_mode; /*传输模式*/
    struct {
        void (*send)(uint8_t *in_data, uint16_t len);/*发送数据*/
        uint8_t *(*receive)(uint16_t *out_data_len); /*接收数据*/
    } driver; /*驱动*/
    struct {
        slave_rsp_code_def (*read_coil_state)(uint16_t reg_addr, uint16_t reg_num, bool *ret_data);/*01: 读线圈状态*/
        slave_rsp_code_def (*read_input_state)(uint16_t reg_addr, uint16_t reg_num, bool *ret_data);/*02: 读输入状态*/
        slave_rsp_code_def (*read_hold_reg)(uint16_t reg_addr, uint16_t reg_num, uint8_t *ret_data);/*03: 读保持寄存器*/
        slave_rsp_code_def (*read_input_reg)(uint16_t reg_addr, uint16_t reg_num, uint8_t *ret_data);/*04: 读输入寄存器*/
        slave_rsp_code_def (*write_one_reg)(uint16_t reg_addr, uint16_t data);/*06:写一个寄存器*/
        slave_rsp_code_def (*write_mul_coils)(uint16_t reg_addr, uint16_t reg_num, bool *data);/*0F:写多个线圈*/
        slave_rsp_code_def (*write_mul_reg)(uint16_t reg_addr, uint16_t reg_num, uint8_t *data);/*10:写多个寄存器*/
        slave_rsp_code_def (*write_one_coil)(uint16_t reg_addr, bool val);   /*05:写一个线圈*/
    } api; /*api接口*/

} modbus_slave_t; /*从机*/

#define MAX_REG_BUF_LEN 200
#define LOG_LEN 30
typedef union{
    uint16_t buf[MAX_REG_BUF_LEN];
    struct{
        uint16_t baudRate;// baud rate/100
        uint16_t poseH16;
        uint16_t poseL16;
        uint16_t poseSpeedH16;
        uint16_t poseSpeedL16;
        uint16_t loadH16;
        uint16_t loadL16;
        uint16_t strainH16;
        uint16_t strainL16;     
        uint16_t slowSpeedSetH16;
        uint16_t slowSpeedSetL16;
        uint16_t fastSpeedSetH16;
        uint16_t fastSpeedSetL16; 
    }name;
}regHoldreg_u;
/**
 * @brief modbus从机初始化
 * @param slave
 */
void modbus_slave_init(modbus_slave_t *slave);

/**
 * @brief modbus从机入口函数
 */
void modbus_poll(void);

/**
 * @brief modbus从机入口函数，数据源来自外部
 */
void modbus_poll_rec(uint8_t *data, uint16_t len);

void modbusInit(void);

slave_rsp_code_def Slave_ReadHoldReg(uint16_t regAddr, uint16_t regNum, uint8_t *retData);

slave_rsp_code_def Slave_ReadInputReg(uint16_t regAddr, uint16_t regNum, uint8_t *retData);

slave_rsp_code_def Slave_ReadCoilState(uint16_t regAddr, uint16_t num, bool *retData);

slave_rsp_code_def Slave_ReadInputState(uint16_t regAddr, uint16_t regNum, bool *retData);

slave_rsp_code_def Slave_WriteOneReg(uint16_t regAddr, uint16_t data);

slave_rsp_code_def Slave_WriteMulReg(uint16_t regAddr, uint16_t regNum, uint8_t *data);

slave_rsp_code_def Slave_WriteMulCoils(uint16_t regAddr, uint16_t regNum, bool *data);

slave_rsp_code_def Slave_WriteOneCoil(uint16_t regAddr, bool val);

#endif //STM32_LIB_MODBUS_SLAVE_H
