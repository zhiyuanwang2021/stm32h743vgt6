/*******************************************************************************
 *  Copyright (c) [scl]。保留所有权利。
 *     本文仅供个人学习和研究使用，禁止用于商业用途。
 ******************************************************************************/
#include "modbus_slave.h"


/*最小数据帧长度*/
#define data_frame_min_len 12
/*固定帧头长度*/
#define fix_frame_head_len 6

/*本地数据解析对象*/
static mb_slave_parse_data_t packet;
static modbus_slave_t *tcp_slave;
static bool *cache_status_buf;
static uint8_t *cache_buf;
#define mb_tcp_ret(cond, fn)  (cond) ? fn: rsp_nut_support_cmd

void modbus_slave_tcp_init(modbus_slave_t *slave, uint8_t *in_cache_buf, bool *in_status_cache) {
    tcp_slave = slave;
    cache_buf = in_cache_buf;
    cache_status_buf = in_status_cache;
}


static inline void data_parse(uint8_t *data, mb_slave_parse_data_t *ptr);

/**
 * @brief tcp 模式接收数据
 * @param data 数据
 * @param len 数据长度
 */
void modbus_tcp_poll(uint8_t *data, uint16_t len) {
    /*验证数据长度是否不小于最小帧长*/
    sys_assert_void(len >= data_frame_min_len);
    sys_assert_void(tcp_slave != NULL);
    sys_assert_void(tcp_slave->driver.send != NULL);
    data_parse(data + fix_frame_head_len, &packet);
    /*数据处理*/
    uint16_t resp_len = fix_frame_head_len;/*响应数据长度*/
    uint8_t resp_code = rsp_nut_support_cmd;/*响应码*/
    if (cache_buf != NULL) {
        /*复制起始固定帧*/
        memcpy(cache_buf, data, resp_len);
        switch (packet.func_code) {
            case read_coil_state_code: /*读线圈状态*/
            case read_input_state_code: /*读输入状态*/
            {
                if (packet.func_code == read_input_state_code) { /*读输入状态*/
                    resp_code = mb_tcp_ret(tcp_slave->api.read_input_state != NULL,
                                           tcp_slave->api.read_input_state(packet.reg_addr, packet.reg_num,
                                                                           cache_status_buf));
                } else {
                    resp_code = mb_tcp_ret(tcp_slave->api.read_coil_state != NULL,
                                           tcp_slave->api.read_coil_state(packet.reg_addr, packet.reg_num,
                                                                          cache_status_buf));
                }
                if (resp_code == rsp_ok) {
                    uint8_t byte_num;
                    // 返回的字节数
                    if (packet.reg_num % 8 == 0) {
                        byte_num = packet.reg_num / 8;
                    } else {
                        byte_num = 1 + packet.reg_num / 8;
                    }
                    cache_buf[resp_len - 1] = 3 + byte_num;
                    cache_buf[resp_len++] = packet.slave_addr;// 从机地址
                    cache_buf[resp_len++] = packet.func_code; // 功能码
                    cache_buf[resp_len++] = byte_num; // 字节数
                    resp_len += pack_bool_array(cache_buf + resp_len, cache_status_buf, packet.reg_num);
                }
            }
                break;

            case read_hold_reg_code: /*读保持寄存器*/
            case read_input_reg_code: /*读输入寄存器*/
            {
                if (packet.func_code == read_hold_reg_code) {
                    resp_code = mb_tcp_ret(tcp_slave->api.read_hold_reg != NULL,
                                           tcp_slave->api.read_hold_reg(packet.reg_addr, packet.reg_num,
                                                                        cache_buf + 9));
                } else {
                    resp_code = mb_tcp_ret(tcp_slave->api.read_input_reg != NULL,
                                           tcp_slave->api.read_input_reg(packet.reg_addr, packet.reg_num,
                                                                         cache_buf + 9));
                }
                if (resp_code == rsp_ok) {
                    cache_buf[resp_len - 1] = 3 + packet.reg_num * 2;
                    cache_buf[resp_len++] = packet.slave_addr;
                    cache_buf[resp_len++] = packet.func_code;
                    cache_buf[resp_len++] = packet.reg_num * 2;
                    resp_len += (packet.reg_num * 2);

                }
                break;
            }
            case write_one_reg_code:/*写一个寄存器*/
            case write_one_coil_code:/*写一个线圈*/
            {
                if (packet.func_code == write_one_coil_code) {
                    resp_code = mb_tcp_ret(tcp_slave->api.write_one_coil != NULL,
                                           tcp_slave->api.write_one_coil(packet.reg_addr,
                                                                         cv_u8_to_16(packet.data) == 0xFF00));
                } else {
                    resp_code = mb_tcp_ret(tcp_slave->api.write_one_reg != NULL,
                                           tcp_slave->api.write_one_reg(packet.reg_addr, cv_u8_to_16(packet.data)));
                }
                if (resp_code == rsp_ok) {
                    tcp_slave->driver.send(data, len);
                    return;
                }
            }
                break;
            case write_mul_reg_code:/*写多个寄存器*/
            case write_mul_coils_reg_code:/*写多个线圈*/
            {
                if (packet.func_code == write_mul_coils_reg_code) {
                    un_pack_bool(cache_status_buf, packet.reg_num, packet.data, packet.data_len);
                    resp_code = mb_tcp_ret(tcp_slave->api.write_mul_coils != NULL,
                                           tcp_slave->api.write_mul_coils(packet.reg_addr, packet.reg_num,
                                                                              cache_status_buf));
                } else {
                    resp_code = mb_tcp_ret(tcp_slave->api.write_mul_reg != NULL,
                                           tcp_slave->api.write_mul_reg(packet.reg_addr, packet.reg_num, packet.data));
                }

                if (resp_code == rsp_ok) {
                    cache_buf[resp_len - 1] = 6;
                    cache_buf[resp_len++] = packet.slave_addr;
                    cache_buf[resp_len++] = packet.func_code;
                    cache_buf[resp_len++] = packet.reg_addr >> 8;
                    cache_buf[resp_len++] = packet.reg_addr;
                    cache_buf[resp_len++] = packet.reg_num >> 8;
                    cache_buf[resp_len++] = packet.reg_num;
                }

                break;
            }
            default: {
                printf("warning:modbus func code not support");
            }
        }

        if (resp_code != rsp_ok) {
            cache_buf[resp_len - 1] = 0x3;
            cache_buf[resp_len++] = packet.slave_addr;
            cache_buf[resp_len++] = packet.func_code | 0x80;
            cache_buf[resp_len++] = resp_code;
        }
        tcp_slave->driver.send(cache_buf, resp_len);
    } else {
        printf("error:modbus_slave_init is not call");
    }

}


static inline void data_parse(uint8_t *data, mb_slave_parse_data_t *ptr) {
    uint8_t idx = 0;
    ptr->slave_addr = data[idx++];
    ptr->func_code = data[idx++];
    ptr->reg_addr = cv_u8_to_16(data + idx);
    idx += 2;
    switch (ptr->func_code) {
        case read_hold_reg_code: /*读保持寄存器*/
        case read_input_reg_code: /*读输入寄存器*/
        case read_coil_state_code: /*读线圈状态*/
        case read_input_state_code: /*读输入状态*/
            ptr->reg_num = cv_u8_to_16(data + idx);
            break;
        case write_one_reg_code:/*写一个寄存器*/
        case write_one_coil_code:/*写一个线圈*/
            ptr->data = data + idx;
            ptr->data_len = 2;
            break;
        case write_mul_reg_code:/*写多个寄存器*/
        case write_mul_coils_reg_code:/*写多个线圈*/
            ptr->reg_num = cv_u8_to_16(data + idx);
            idx += 2;
            ptr->data_len = data[idx++];
            ptr->data = data + idx;
            break;
    }
}

