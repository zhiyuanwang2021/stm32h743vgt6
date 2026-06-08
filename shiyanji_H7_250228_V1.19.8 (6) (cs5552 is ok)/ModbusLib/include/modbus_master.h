/*******************************************************************************
 *  Copyright (c) [scl]。保留所有权利。
 *     本文仅供个人学习和研究使用，禁止用于商业用途。
 ******************************************************************************/
#ifndef STM32_F1XX_TEMPLATE_MODBUS_MASTER_H
#define STM32_F1XX_TEMPLATE_MODBUS_MASTER_H

#include "modbus_common.h"

typedef enum {
    ok_code = 0x0,
    rec_time_out = 0xFD,
    crc_error_code = 0xFE,
    data_len_error_code = 0xFF,
} resp_status_code;
typedef struct {
    uint8_t resp_code;
    union { /*共用*/
        uint8_t rd_data[256];
        bool rd_flag_data[256];
    };
    uint16_t rd_data_len;
} master_op_result_t; /*主机操作结果*/

typedef struct {
    modbus_trans_mode trans_mode; /*传输模式*/
    uint8_t slave_addr; /*从机地址*/
    struct {
        void (*send)(uint8_t *data, uint16_t len);/*发送*/
        uint16_t (*receive)(uint8_t *out_data);/*接收*/
        void (*delay)(uint32_t ms);/*延迟*/
        void (*log)(char *fmt, ...); /*调试*/
    } driver;

    struct {
        /*06:写一个寄存器*/
        master_op_result_t *(*write_one_register)(uint16_t reg_addr, uint16_t val);

        /*10:写多个寄存器*/
        master_op_result_t *(*write_mul_register)(uint16_t reg_addr, uint8_t *data, uint16_t data_len);

        /*0x05 写一个线圈*/
        master_op_result_t *(*write_one_coil)(uint16_t reg_addr, bool val);

        /*0x0F write one coil*/
        master_op_result_t *(*write_mul_coil)(uint16_t reg_addr, bool *data, uint16_t data_len);

        /*0x3 read register*/
        master_op_result_t *(*read_hold_register)(uint16_t reg_addr, uint16_t reg_num);

        /*0x4 read input register*/
        master_op_result_t *(*read_input_register)(uint16_t reg_addr, uint16_t reg_num);

        /*0x2 read input status register*/
        master_op_result_t *(*read_input_status_register)(uint16_t reg_addr, uint16_t reg_num);

        /*0x1 read coil status register*/
        master_op_result_t *(*read_coil_status_register)(uint16_t reg_addr, uint16_t reg_num);

    } api;
} modbus_master_t;

void modbus_master_init(modbus_master_t *master);

#endif //STM32_F1XX_TEMPLATE_MODBUS_MASTER_H
