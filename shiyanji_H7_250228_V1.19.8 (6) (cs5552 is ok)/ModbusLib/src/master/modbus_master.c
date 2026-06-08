/*******************************************************************************
 *  Copyright (c) [scl]。保留所有权利。
 *     本文仅供个人学习和研究使用，禁止用于商业用途。
 ******************************************************************************/
#include "modbus_master.h"

static uint8_t cmd_cache_buf[256]; /*指令缓存数组*/
static modbus_master_t *master_driver = NULL;
static master_op_result_t op_result;

void modbus_master_init(modbus_master_t *master) {
    sys_assert_void(master != NULL);
    master_driver = master;
    switch (master->trans_mode) {
        case rtu_mode: {
            extern void master_rtu_init(modbus_master_t *master, master_op_result_t *op_result, uint8_t *cache);
            master_rtu_init(master, &op_result, cmd_cache_buf);
            break;
        }
        case tcp_mode:
            break;
    }
}