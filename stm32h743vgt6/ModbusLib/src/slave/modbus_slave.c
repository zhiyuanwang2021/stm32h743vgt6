/*******************************************************************************
 *  Copyright (c) [scl]。保留所有权利。
 *     本文仅供个人学习和研究使用，禁止用于商业用途。
 ******************************************************************************/

#include "modbus_slave.h"
#include "usart.h"

uint8_t modbusRxbuf[modbusRxBuffSize] = {0};//串口单字节接收上位机Modbus指令缓冲区

static modbus_slave_t mb_slave;
static uint8_t data_buf_cache[256];
static bool status_buf_cache[256] = {0}; /*针对状态量的缓存*/

/**
 * @brief register modbus send function
 */
static void mobusSendFunction(uint8_t *in_data, uint16_t len){
    HAL_UART_Transmit(&huart2,in_data,len,1);
    //HAL_UART_Transmit_DMA(&huart2,in_data,len);
}

static uint8_t *modbusRevFunction(uint16_t *out_data_len){
    *out_data_len = 0;
    return NULL;
}

/**
 * @brief modbus slave struct init
 */
static void modbusSlaveStructInit(void){
    mb_slave.driver.send = mobusSendFunction;
    mb_slave.driver.receive = modbusRevFunction;
    mb_slave.slave_addr = 0x01;
    mb_slave.trans_mode = rtu_mode;
    mb_slave.api.read_coil_state = Slave_ReadCoilState;
    mb_slave.api.read_hold_reg = Slave_ReadHoldReg;
    mb_slave.api.read_input_reg = Slave_ReadInputReg;
    mb_slave.api.read_input_state = Slave_ReadInputState;
    mb_slave.api.write_one_coil = Slave_WriteOneCoil;
    mb_slave.api.write_one_reg = Slave_WriteOneReg;
    mb_slave.api.write_mul_coils = Slave_WriteMulCoils;
    mb_slave.api.write_mul_reg = Slave_WriteMulReg;
}

/**
 * @brief
 * @param slave
 */
static void modbus_slave_init(modbus_slave_t *slave) {
    if (slave->trans_mode == tcp_mode) {
        // printf("debug:modbus slave mode is tcp mode");
        // extern void modbus_slave_tcp_init(modbus_slave_t *slave, uint8_t *in_cache_buf, bool *in_status_cache);
        // modbus_slave_tcp_init(slave, data_buf_cache, status_buf_cache);
    } else if (slave->trans_mode == rtu_mode) {
        // printf("debug:modbus slave mode is rtu mode");
        extern void modbus_slave_rtu_init(modbus_slave_t *slave, uint8_t *in_cache_buf, bool *in_status_cache);
        modbus_slave_rtu_init(slave, data_buf_cache, status_buf_cache);
    }
}

/**
 * @brief modbus init
 */
void modbusInit(void){
    modbusSlaveStructInit();
    modbus_slave_init(&mb_slave);
}

void modbus_poll(void) {
    sys_assert_void(&mb_slave != NULL);
    sys_assert_void(mb_slave.driver.receive != NULL);
    uint16_t rec_len = 0;
    uint8_t *p_data = mb_slave.driver.receive(&rec_len);
    /*读取数据为NULL,直接返回*/
    sys_assert_void(p_data == NULL);
    modbus_poll_rec(p_data, rec_len);
}

void modbus_poll_rec(uint8_t *data, uint16_t len) {
    sys_assert_void(&mb_slave != NULL);
    if (mb_slave.trans_mode == rtu_mode) {
        extern void modbus_rtu_poll(uint8_t *data, uint16_t len);
        modbus_rtu_poll(data, len);
    } else if (mb_slave.trans_mode == tcp_mode) {
        // extern void modbus_tcp_poll(uint8_t *data, uint16_t len);
        // modbus_tcp_poll(data, len);
    } else {
        // @note 暂不支持
    }
}

