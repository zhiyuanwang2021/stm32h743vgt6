/*******************************************************************************
 *  Copyright (c) [scl]。保留所有权利。
 *     本文仅供个人学习和研究使用，禁止用于商业用途。
 ******************************************************************************/

#ifndef STM32_LIB_MODBUS_COMMON_H
#define STM32_LIB_MODBUS_COMMON_H

#include "stdint.h"
#include "stdio.h"
#include "stdbool.h"
#include "string.h"

#ifndef sys_assert_void
#define sys_assert_void(condition) if (!(condition)) return
#endif
typedef enum {
    /**
    * @brief 读线圈状态
    */
    read_coil_state_code = 0x1,
    /**
    * @brief 读输入状态
    */
    read_input_state_code = 0x2,
    /**
    * @brief 读保持寄存器
    */
    read_hold_reg_code = 0x3,
    /**
    * @brief 读输入寄存器
    */
    read_input_reg_code = 0x4,
    /**
    * @brief 写一个线圈
    */
    write_one_coil_code = 0x05,
    /**
    * @brief 写一个寄存器
    */
    write_one_reg_code = 0x06,
    /**
    * @brief 写多个寄存器
    */
    write_mul_reg_code = 0x10,
    /**
    * @brief 写多个线圈
    */
    write_mul_coils_reg_code = 0x0F,
} modbus_func_code;
typedef enum {
    rtu_mode, /*RTU 模式*/
    tcp_mode,/*TCP 模式*/
} modbus_trans_mode; /*传输模式定义*/
#define cv_u8_to_16(arr) ((uint16_t) ((uint16_t) (((arr)[0]) << 8) + ((arr)[1])))
#define modbus_u8_to_u16(a, b) (uint16_t)((uint16_t)((a)<<8)+(b))

uint16_t pack_bool_array(uint8_t *dst, const bool *data, uint16_t data_len);

void un_pack_bool(bool *dst, uint16_t cnt, const uint8_t *data, uint16_t data_len);

/**
 * @brief CRC CHECK SUM
 * @param buf
 * @param buf_len
 * @return
 */
uint16_t modbus_crc_compute(const uint8_t *buf, uint16_t buf_len);

#endif //STM32_LIB_MODBUS_COMMON_H
