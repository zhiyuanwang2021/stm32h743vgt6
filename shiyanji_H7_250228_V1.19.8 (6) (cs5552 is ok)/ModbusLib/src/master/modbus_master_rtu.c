/*******************************************************************************
 *  Copyright (c) [scl]。保留所有权利。
 *     本文仅供个人学习和研究使用，禁止用于商业用途。
 ******************************************************************************/

#include "modbus_master.h"

#define RTU_DELAY_MS 10
#define ERROR_RESP_DATA_LEN (5) // 错误指令响应长度
#define ERROR_RESP_CODE_IDX (2)   // 错误响应码下标位置
static uint8_t *rtu_cache_buf;
static modbus_master_t *rtu_driver;
static master_op_result_t *rtu_op_result;

static master_op_result_t *write_one_register(uint16_t reg_addr, uint16_t val);

static master_op_result_t *write_mul_coil(uint16_t reg_addr, bool *data, uint16_t data_len);/*0x05 write one coil*/
static master_op_result_t *write_one_coil(uint16_t reg_addr, bool val); /*0x05 写一个线圈*/
static master_op_result_t *write_mul_register(uint16_t reg_addr, uint8_t *data, uint16_t data_len);

static master_op_result_t *read_hold_register(uint16_t reg_addr, uint16_t reg_num);/*0x3 read register*/
static master_op_result_t *read_input_register(uint16_t reg_addr, uint16_t reg_num);/*0x4 read input register*/
static master_op_result_t *read_input_status_register(uint16_t reg_addr, uint16_t reg_num);

static master_op_result_t *read_coil_status_register(uint16_t reg_addr, uint16_t reg_num);

/**
 * @brief 分析数据
 * @param code 功能码
 * @param data 接收的数据
 * @param len 数据长度
 * @return
 */
static master_op_result_t *rtu_analysis_rec_data(uint8_t code, uint8_t *data, uint16_t len);

void master_rtu_init(modbus_master_t *master, master_op_result_t *op_result, uint8_t *cache) {
    rtu_driver = master;
    rtu_cache_buf = cache;
    rtu_op_result = op_result;
    master->api.write_one_register = write_one_register;
    master->api.write_mul_register = write_mul_register;
    master->api.write_one_coil = write_one_coil;
    master->api.write_mul_coil = write_mul_coil;
    master->api.read_hold_register = read_hold_register;
    master->api.read_input_register = read_input_register;
    master->api.read_input_status_register = read_input_status_register;
    master->api.read_coil_status_register = read_coil_status_register;
}

static master_op_result_t *common_send_handle(uint8_t code, uint8_t *data, uint16_t cmd_len) {
    /*发送*/
    rtu_driver->driver.send(data, cmd_len);
    if (rtu_driver->driver.delay != NULL) rtu_driver->driver.delay(RTU_DELAY_MS);
    /*接收数据*/
    uint16_t len = rtu_driver->driver.receive(rtu_cache_buf);
    return rtu_analysis_rec_data(code, rtu_cache_buf, len);
}

static master_op_result_t *common_read_send_handle(uint8_t code, uint16_t reg_addr, uint16_t reg_num) {
    uint8_t idx = 0;
    rtu_cache_buf[idx++] = rtu_driver->slave_addr;
    rtu_cache_buf[idx++] = code;
    rtu_cache_buf[idx++] = (uint8_t) (reg_addr >> 8);
    rtu_cache_buf[idx++] = (uint8_t) (reg_addr);
    rtu_cache_buf[idx++] = (uint8_t) (reg_num >> 8);
    rtu_cache_buf[idx++] = (uint8_t) (reg_num);
    const uint16_t crc = modbus_crc_compute(rtu_cache_buf, idx);
    rtu_cache_buf[idx++] = (uint8_t) (crc);
    rtu_cache_buf[idx++] = (uint8_t) (crc >> 8);
    return common_send_handle(code, rtu_cache_buf, idx);
}

static master_op_result_t *write_one_register(uint16_t reg_addr, uint16_t val) {
    /*指令构建*/
    uint8_t idx = 0;
    rtu_cache_buf[idx++] = rtu_driver->slave_addr;
    rtu_cache_buf[idx++] = 0x06;
    rtu_cache_buf[idx++] = reg_addr >> 8;
    rtu_cache_buf[idx++] = reg_addr;
    rtu_cache_buf[idx++] = val >> 8;
    rtu_cache_buf[idx++] = val;
    const uint16_t crc = modbus_crc_compute(rtu_cache_buf, idx);
    rtu_cache_buf[idx++] = (uint8_t) (crc);
    rtu_cache_buf[idx++] = (uint8_t) (crc >> 8);
    return common_send_handle(0x06, rtu_cache_buf, idx);
}

static master_op_result_t *write_one_coil(uint16_t reg_addr, bool val) {
    uint8_t wr_buf[2] = {0};
    if (val)wr_buf[0] = 0xFF;
    uint8_t idx = 0;
    rtu_cache_buf[idx++] = rtu_driver->slave_addr;
    rtu_cache_buf[idx++] = 0x05;
    rtu_cache_buf[idx++] = (uint8_t) (reg_addr >> 8);
    rtu_cache_buf[idx++] = (uint8_t) (reg_addr);
    rtu_cache_buf[idx++] = val ? 0xff : 0x00;
    rtu_cache_buf[idx++] = 0x00;
    const uint16_t crc = modbus_crc_compute(rtu_cache_buf, idx);
    rtu_cache_buf[idx++] = (uint8_t) (crc);
    rtu_cache_buf[idx++] = (uint8_t) (crc >> 8);
    return common_send_handle(0x05, rtu_cache_buf, idx);
}

static master_op_result_t *write_mul_coil(uint16_t reg_addr, bool *data, uint16_t data_len) {
    uint8_t idx = 0;
    uint16_t real_data_len = pack_bool_array(rtu_cache_buf + 7, data, data_len);
    rtu_cache_buf[idx++] = rtu_driver->slave_addr;
    rtu_cache_buf[idx++] = 0x0F;
    rtu_cache_buf[idx++] = (uint8_t) (reg_addr >> 8);
    rtu_cache_buf[idx++] = (uint8_t) (reg_addr);
    rtu_cache_buf[idx++] = (uint8_t) (data_len >> 8);
    rtu_cache_buf[idx++] = (uint8_t) (data_len);
    rtu_cache_buf[idx++] = (uint8_t) (real_data_len);
    idx += real_data_len;
    const uint16_t crc = modbus_crc_compute(rtu_cache_buf, idx);
    rtu_cache_buf[idx++] = (uint8_t) (crc);
    rtu_cache_buf[idx++] = (uint8_t) (crc >> 8);
    return common_send_handle(0x05, rtu_cache_buf, idx);
}

static master_op_result_t *write_mul_register(uint16_t reg_addr, uint8_t *data, uint16_t data_len) {
    /*指令构建*/
    uint8_t idx = 0;
    rtu_cache_buf[idx++] = rtu_driver->slave_addr;
    rtu_cache_buf[idx++] = 0x10;
    rtu_cache_buf[idx++] = (uint8_t) (reg_addr >> 8);
    rtu_cache_buf[idx++] = (uint8_t) (reg_addr);
    const uint16_t reg_num = (uint16_t) (data_len / 2);
    rtu_cache_buf[idx++] = (uint8_t) (reg_num >> 8);
    rtu_cache_buf[idx++] = (uint8_t) (reg_num);
    rtu_cache_buf[idx++] = (uint8_t) (data_len);
    for (int i = 0; i < data_len; ++i) {
        rtu_cache_buf[idx++] = data[i];
    }
    const uint16_t crc = modbus_crc_compute(rtu_cache_buf, idx);
    rtu_cache_buf[idx++] = (uint8_t) (crc);
    rtu_cache_buf[idx++] = (uint8_t) (crc >> 8);
    return common_send_handle(0x10, rtu_cache_buf, idx);
}

static master_op_result_t *read_hold_register(uint16_t reg_addr, uint16_t reg_num) {
    return common_read_send_handle(0x03, reg_addr, reg_num);
}

static master_op_result_t *read_input_register(uint16_t reg_addr, uint16_t reg_num) {
    return common_read_send_handle(0x4, reg_addr, reg_num);
}

static master_op_result_t *read_input_status_register(uint16_t reg_addr, uint16_t reg_num) {
    master_op_result_t *ptr = common_read_send_handle(0x2, reg_addr, reg_num);
    if (ptr->resp_code == ok_code) {
        un_pack_bool(ptr->rd_flag_data, reg_num, ptr->rd_data, ptr->rd_data_len);
        ptr->rd_data_len = reg_num;
    }
    return ptr;
}

static master_op_result_t *read_coil_status_register(uint16_t reg_addr, uint16_t reg_num) {
    master_op_result_t *ptr = common_read_send_handle(0x1, reg_addr, reg_num);
    if (ptr->resp_code == ok_code) {
        un_pack_bool(ptr->rd_flag_data, reg_num, ptr->rd_data, ptr->rd_data_len);
        ptr->rd_data_len = reg_num;
    }
    return ptr;
}

static master_op_result_t *rtu_analysis_rec_data(uint8_t code, uint8_t *data, uint16_t len) {
    rtu_op_result->resp_code = ok_code;
    switch (code) {
        case 0x0F: // 写多个线圈
        case 0x05: // 写一个线圈
        case 0x06: // 写一个寄存器
        case 0x10: // 写多个寄存器
        {
            if (modbus_crc_compute(data, len) == 0) {
                if (len == ERROR_RESP_DATA_LEN) {
                    rtu_op_result->resp_code = data[ERROR_RESP_CODE_IDX];
                }
            } else {
                rtu_op_result->resp_code = crc_error_code;
            }
            break;
        }

        case 0x3: // 读保持寄存器
        case 0x4: // 读输入寄存器
        case 0x2:// 读输入状态
        case 0x1: // 读线圈状态
        {
            if (len == 0) {// 验证CRC
                rtu_op_result->resp_code = rec_time_out;
            } else if (modbus_crc_compute(data, len) == 0) {
                if (len == ERROR_RESP_DATA_LEN) {
                    rtu_op_result->resp_code = data[ERROR_RESP_CODE_IDX];
                } else {
                    rtu_op_result->rd_data_len = data[2];
                    memcpy(rtu_op_result->rd_data, data + 3, rtu_op_result->rd_data_len);
                }
            } else {
                rtu_op_result->resp_code = crc_error_code;
            }
            break;
        }
        default:
            rtu_op_result->resp_code = rec_time_out;
            break;
    }
    return rtu_op_result;
}