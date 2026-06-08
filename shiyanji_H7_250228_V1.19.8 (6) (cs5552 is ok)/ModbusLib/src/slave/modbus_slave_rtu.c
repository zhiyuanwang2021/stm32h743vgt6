/*******************************************************************************
 *  Copyright (c) [scl]闁靛棗鍊风换姘舵偩濞嗘劕顣查柡鍫濐槹濞煎牓宕氶埡瀣?
 *     闁跨噦鎷�?闁哄倸娲ｇ划搴㈢瑹濞戞鍤嬪ù婊呭皑閹凤拷?闁跨喐鍨濈弧鍕椽瀹€鈧悥铏圭矚閺堥潧鈻忛柨鐕傛嫹?闁挎稑鐭夐幏锟�?闁跨喐鏋婚幏锟�?闁跨喓鏅弫銈嗙鎼粹剝娅屽☉鎾存皑閺併倝鏌呴弮鎾村?
 ******************************************************************************/
#include "modbus_slave.h"
#include "in_out.h"
#include "RS485.h"

#ifndef sign
	#define sign(x) ((x)>0?1:((x)<0?-1:1))
#endif

#define data_frame_min_len 8
static mb_slave_parse_data_t packet;
static modbus_slave_t *rtu_slave;
static uint8_t *cache_buf;
#define mb_rtu_ret(cond, fn)  (cond) ? fn: rsp_nut_support_cmd

void modbus_slave_rtu_init(modbus_slave_t *slave, uint8_t *in_cache_buf, bool *in_status_cache) {
    rtu_slave = slave;
    cache_buf = in_cache_buf;
    (void)in_status_cache;
}

static inline void data_parse(uint8_t *data, mb_slave_parse_data_t *ptr);

static inline uint16_t rtu_add_crc(uint8_t *ret_data, uint16_t len);

void modbus_rtu_poll(uint8_t *data, uint16_t len) {
    /*濡ょ姴鐭侀惁澶愬极閻楀牆绁﹂梻鈧崹顔碱唺闁跨噦鎷�?闁告熬缂氱粭澶屼焊韫囧海鑹鹃柡鍫氬亾閻忓繐绻愰幎姘舵晸閿燂拷?*/
    sys_assert_void(len >= data_frame_min_len);
    /*濡ょ姴鐭侀惁澶嬬鎼淬垺绨氶柛锔芥緲濞煎啴鏁撻敓锟�?闁告熬缂氱粩鎾晸閿燂拷?*/
    sys_assert_void(data[0] == rtu_slave->slave_addr);
    /*闁轰胶澧楀畵涔c闁哄稄绻濋悰锟�*/
    sys_assert_void(modbus_crc_compute(data, len) == 0);
    data_parse(data, &packet);
    /*闁轰胶澧楀畵浣瑰緞閸曨厽鍊�*/
    uint16_t resp_len = 0;/*闁告繂绉寸花鏌ュ极閻楀牆绁﹂梻鈧崹顔碱唺*/
    uint8_t resp_code = rsp_nut_support_cmd;/*闁告繂绉寸花鏌ユ晸閿燂拷?*/
    switch (packet.func_code) {
        case 0x3: /*閻犲洩顔婄换姘跺箰娴ｅ摜妲戦悗娑櫭▍锟�*/
        case 0x4: /*閻犲洦妲掔欢顓㈠礂閵夈儳妲戦悗娑櫭▍锟�*/
        {
            if (packet.func_code == 0x3) {
                resp_code = mb_rtu_ret(rtu_slave->api.read_hold_reg != NULL,
                                       rtu_slave->api.read_hold_reg(packet.slave_addr, packet.reg_num,
                                                                    cache_buf + 3));
            } else {
                resp_code = mb_rtu_ret(rtu_slave->api.read_input_reg != NULL,
                                       rtu_slave->api.read_input_reg(packet.slave_addr, packet.reg_num,
                                                                     cache_buf + 3));
            }
            if (resp_code == rsp_ok) {
                cache_buf[resp_len++] = packet.slave_addr;
                cache_buf[resp_len++] = packet.func_code;
                cache_buf[resp_len++] = packet.reg_num * 2;
                resp_len += (packet.reg_num * 2);

            }
            break;
        }
        case 0x1: /*閻犲洩宕甸崵搴ㄥ捶閸垹笑闁跨噦鎷�?*/
        case 0x2: /*閻犲洦妲掔欢顓㈠礂閵壯冃﹂柨鐕傛嫹?*/
        {
            // if (packet.func_code == 0x1) {
            //     resp_code = mb_rtu_ret(rtu_slave->api.read_coil_state != NULL,
            //                            rtu_slave->api.read_coil_state(packet.reg_addr, packet.reg_num,
            //                                                           cache_status_buf));
            // } else {
            //     resp_code = mb_rtu_ret(rtu_slave->api.read_input_state != NULL,
            //                            rtu_slave->api.read_input_state(packet.reg_addr, packet.reg_num,
            //                                                            cache_status_buf));
            // }
            // if (resp_code == rsp_ok) {
            //     cache_buf[resp_len++] = packet.slave_addr;
            //     cache_buf[resp_len++] = packet.func_code;
            //     // 閺夆晜鏌ㄥú鏍儍閸曨偆鎽熼柤鍝勫€归弳锟�
            //     if (packet.reg_num % 8 == 0) {
            //         cache_buf[resp_len++] = packet.reg_num / 8;
            //     } else {
            //         cache_buf[resp_len++] = packet.reg_num / 8 + 1;
            //     }
            //     resp_len += pack_bool_array(cache_buf + resp_len, cache_status_buf, packet.reg_num);
            // }
        }
            break;
        case 0x06:/*闁告劖鐟ょ粩鎾晸閿燂拷?閻庨潧瀚悺銊╂晸閿燂拷?*/
        case 0x05:/*闁告劖鐟ょ粩鎾晸閿燂拷?缂佹儳鐏濆﹢鈧�*/
        {
            if (packet.func_code == 0x05) {
                resp_code = mb_rtu_ret(rtu_slave->api.write_one_coil != NULL,
                                       rtu_slave->api.write_one_coil(packet.reg_addr,
                                                                     cv_u8_to_16(packet.data) == 0xFF00));
            } else {
                resp_code = mb_rtu_ret(rtu_slave->api.write_one_reg != NULL,
                                       rtu_slave->api.write_one_reg(packet.reg_addr, cv_u8_to_16(packet.data)));
            }
            if (resp_code == rsp_ok) {
                rtu_slave->driver.send(data, len);
                return;
            }
            break;
        }
        case 0x10:/*闁告劖鐟遍幏锟�?闁跨喐鍨濋柌婊呪偓闈涘閻°劑鏁撻敓锟�?*/
        case 0x0F:/*闁告劖鐟遍幏锟�?闁跨喐鍨濋柌婊呯棯閸喗绠�*/
        {
            // if (packet.func_code == 0x0F) {
            //     un_pack_bool(cache_status_buf, packet.reg_num, packet.data, packet.data_len);
            //     resp_code = mb_rtu_ret(rtu_slave->api.write_mul_coils != NULL,
            //                            rtu_slave->api.write_mul_coils(packet.reg_addr, packet.reg_num,
            //                                                               cache_status_buf));
            // } else {
            //     resp_code = mb_rtu_ret(rtu_slave->api.write_mul_reg != NULL,
            //                            rtu_slave->api.write_mul_reg(packet.reg_addr, packet.reg_num, packet.data));
            // }
            // if (resp_code == rsp_ok) {
            //     cache_buf[resp_len++] = packet.slave_addr;
            //     cache_buf[resp_len++] = packet.func_code;
            //     cache_buf[resp_len++] = packet.reg_addr >> 8;
            //     cache_buf[resp_len++] = packet.reg_addr;
            //     cache_buf[resp_len++] = packet.reg_num >> 8;
            //     cache_buf[resp_len++] = packet.reg_num;
            // }
            break;
        }
    }
    if (resp_code != rsp_ok) {
        uint8_t idx = 0;
        cache_buf[idx++] = packet.slave_addr;
        cache_buf[idx++] = packet.func_code | 0x80;
        cache_buf[idx++] = resp_code;
        resp_len = idx;
    }
    resp_len = rtu_add_crc(cache_buf, resp_len);
    rtu_slave->driver.send(cache_buf, resp_len);
}

static inline uint16_t rtu_add_crc(uint8_t *ret_data, uint16_t len) {
    const uint16_t crc = modbus_crc_compute(ret_data, len);
    ret_data[len++] = (uint8_t) crc;
    ret_data[len++] = (uint8_t) (crc >> 8);
    return len;
}

static inline void data_parse(uint8_t *data, mb_slave_parse_data_t *ptr) {
    uint8_t idx = 0;
    ptr->slave_addr = data[idx++];
    ptr->func_code = data[idx++];
    ptr->reg_addr = cv_u8_to_16(data + idx);
    idx += 2;
    switch (ptr->func_code) {
        case 0x3: /*閻犲洩顔婄换姘跺箰娴ｅ摜妲戦悗娑櫭▍锟�*/
        case 0x4: /*閻犲洦妲掔欢顓㈠礂閵夈儳妲戦悗娑櫭▍锟�*/
        case 0x1: /*閻犲洩宕甸崵搴ㄥ捶閸垹笑闁跨噦鎷�?*/
        case 0x2: /*閻犲洦妲掔欢顓㈠礂閵壯冃﹂柨鐕傛嫹?*/
            ptr->reg_num = cv_u8_to_16(data + idx);
            idx += 2;
            break;
        case 0x06:/*闁告劖鐟ょ粩鎾晸閿燂拷?閻庨潧瀚悺銊╂晸閿燂拷?*/
        case 0x05:/*闁告劖鐟ょ粩鎾晸閿燂拷?缂佹儳鐏濆﹢鈧�*/
            ptr->data = data + idx;
            ptr->data_len = 2;
            break;
        case 0x10:/*闁告劖鐟遍幏锟�?闁跨喐鍨濋柌婊呪偓闈涘閻°劑鏁撻敓锟�?*/
        case 0x0F:/*闁告劖鐟遍幏锟�?闁跨喐鍨濋柌婊呯棯閸喗绠�*/
            ptr->reg_num = cv_u8_to_16(data + idx);
            idx += 2;
            ptr->data_len = data[idx++];
            ptr->data = data + idx;
            break;
    }
}


/***************************************************************闁稿繗娓圭紞瀣偓鍦仧楠烇拷********************************************************/
uint16_t reg_start_addr = 0x000;

regHoldreg_u holdreg={
    .name.baudRate = 1152,
    .name.poseH16 = 0x0000,
    .name.poseL16 = 0x0000,
    .name.poseSpeedH16 = 0x0000,
    .name.poseSpeedL16 = 0x0000,
    .name.loadH16 = 0x0000,
    .name.loadL16 = 0x0000,
    .name.strainH16 = 0x0000,
    .name.strainL16 = 0x0000,
    .name.slowSpeedSetH16 = 0x0000,
    .name.slowSpeedSetL16 = 0x0000,
    .name.fastSpeedSetH16 = 0x0000,
    .name.fastSpeedSetL16 = 0x0000,
};

uint8_t reg_buf[MAX_REG_BUF_LEN] = {0x00, 0x01,0x00,0x00};//note storage format 

/**
 * @brief read input reg 0x04
 * @param regAddr
 * @param regNum
 * @param retData
 */
slave_rsp_code_def Slave_ReadInputReg(uint16_t regAddr, uint16_t regNum, uint8_t *retData) {
    printf("Slave_ReadInputReg reg: %X; num:%d\n", regAddr, regNum);
	return rsp_ok;
}

/**
 * @brief read coil state 0x01
 * @param regAddr
 * @param regNum
 * @param retData
 */
slave_rsp_code_def Slave_ReadCoilState(uint16_t regAddr, uint16_t num, bool *retData) {
	printf("Slave_ReadCoilState reg: %X; num:%d\n", regAddr, num);
	for (int i = 0; i < num; ++i) {
		retData[i] = true;
	}
	return rsp_ok;
}

//#define DEBUG_WRITE_MUL_REG
/**
 * @brief write_mul_reg 0x10
 * @param regAddr
 * @param regNum
 * @param data
 */
slave_rsp_code_def Slave_WriteMulReg(uint16_t regAddr, uint16_t regNum, uint8_t *data) {
	uint8_t idx = regAddr * 2;
    uint16_t *p;
    if(idx >= MAX_REG_BUF_LEN)
        return rsp_err_reg_addr;
    for(int i = 0; i < regNum*2;){
        reg_buf[idx++] = data[i++];
        reg_buf[idx++] = data[i++];
        p = (uint16_t*)(&holdreg.buf[idx-2]);
        *p = (modbus_u8_to_u16(reg_buf[idx-2], reg_buf[idx-1]));
    }
#ifdef DEBUG_WRITE_MUL_REG
    printf("Slave_WriteMulReg regAddr:%4x,regNum:%4x\r\n,",regAddr,regNum);
    for (int i = 0; i < regNum; ++i) {
		printf("val:%4x\n", modbus_u8_to_u16(data[i*2], data[i*2+1]));
	}
#endif
    return rsp_ok;
}

//#define DEBUG_WRITE_ONE_REG
/**
 * @brief write_one_reg 0x06
 * @param regAddr
 * @param data
 */
slave_rsp_code_def Slave_WriteOneReg(uint16_t regAddr, uint16_t data) {
    uint8_t idx = regAddr * 2;
    uint16_t *p;
    if(idx >= MAX_REG_BUF_LEN )
        return rsp_err_reg_addr;
	reg_buf[idx] = (uint8_t)(data >> 8);
    reg_buf[++idx] = (uint8_t)(data & 0xFF);
    p = (uint16_t*)(&holdreg.buf[idx-2]);
    *p = (modbus_u8_to_u16(reg_buf[idx-2], reg_buf[idx-1]));

#ifdef DEBUG_WRITE_ONE_REG
    printf("Slave_WriteOneReg regAddr: 0x%2X; data:0x%2X; reg[%d]:0x%x; reg[%d]:0x%x\r\n", regAddr, data, 
                                                                                    idx-1,reg_buf[idx -1],idx ,reg_buf[idx]);
#endif

    return rsp_ok;
}


//#define DEBUG_READ_HOLD_REG
/**
 * @brief read_hold_reg 0x03
 * @param regAddr register start address
 * @param regNum  register length 
 * @param retData return data,reData[0]-holdReg[0+regAddr],reData[1]-holdReg[1+regAddr] ...and so on
 * @return
 */
slave_rsp_code_def Slave_ReadHoldReg(uint16_t regAddr, uint16_t regNum, uint8_t *retData) {
    uint8_t i = 0,k=0;
    uint32_t poseInt32,poseSpeedInt32,loadInt32,strainInt32;
    int32_t poseSign, poseSpeedSign, loadSign, strainSign;
    float _pose,_poseSpeed,_load,_strain;
	if (regAddr < reg_start_addr) {
		return rsp_err_reg_addr;
	} else {
        manualBox.type = manualBox485_Type;
        _pose = pose.orig;
        _poseSpeed = speedPose.filter;
        _load = force.filterTrans;
        _strain = strain1.filterTrans;
        poseSign = sign(_pose);
        poseSpeedSign = sign(_poseSpeed);
        loadSign = sign(_load);
        strainSign = sign(_strain);

        poseInt32 		= (uint32_t)((_pose * 1e6f) * poseSign);
        if(poseSign == -1){
            poseInt32 = poseInt32 | 0x80000000;
        }
        poseSpeedInt32 	= (uint32_t)((_poseSpeed * 1e6f) * poseSpeedSign);
        if(poseSpeedSign == -1){
            poseSpeedInt32 = poseSpeedInt32 | 0x80000000;
        }
        loadInt32 		= (uint32_t)((_load * 1e3f) * loadSign);
        if(loadSign == -1){
            loadInt32 = loadInt32 | 0x80000000;
        }
        strainInt32 	= (uint32_t)((_strain * 1e6f) * strainSign);
        if(strainSign == -1){
            strainInt32 = strainInt32 | 0x80000000;
        }

        holdreg.name.poseH16 		= (uint16_t)((poseInt32 >> 16)&0xFFFF);
        holdreg.name.poseL16 		= (uint16_t)((poseInt32)&0xFFFF);
        holdreg.name.poseSpeedH16 	= (uint16_t)((poseSpeedInt32 >> 16)&0xFFFF);
        holdreg.name.poseSpeedL16 	= (uint16_t)((poseSpeedInt32)&0xFFFF);
        holdreg.name.loadH16 		= (uint16_t)((loadInt32 >> 16)&0xFFFF);
        holdreg.name.loadL16 		= (uint16_t)((loadInt32)&0xFFFF);
        holdreg.name.strainH16 		= (uint16_t)((strainInt32 >> 16)&0xFFFF);
        holdreg.name.strainL16 		= (uint16_t)((strainInt32)&0xFFFF);
        k = regAddr;
        for(i=0;i<regNum*2;k++){
            retData[i++] = (uint8_t)((holdreg.buf[k]>>8) &0xFF);
            retData[i++] = (uint8_t)(holdreg.buf[k]&0xFF);
        }
		return rsp_ok;
	}
#ifdef DEBUG_READ_HOLD_REG
    printf("Slave_ReadHoldReg regAddr: %X; regNum:%d\r\n", regAddr, regNum);
#endif

}

//#define DEBUG_READ_INPUT_STATE
/**
 * @brief read_input_state  0x02
 * @param regAddr register start address
 * @param regNum  register length 
 * @param retData return data,reData[0]-coil[0+regAddr],reData[1]-coil[1+regAddr] ...and so on
 */
slave_rsp_code_def Slave_ReadInputState(uint16_t regAddr, uint16_t regNum, bool *retData) {
    uint8_t i = 0;
    const bool coil_1 = false;
    const bool coil_2 = true;
    switch (regAddr)
    {
    case 0x00:
        /*********application code*********/
        //e.g.
        retData[i++] = coil_1;
        /*********application code*********/
        if(--regNum <= 0)
            break;
    case 0x01:
        /*********application code*********/
        //e.g.
        retData[i++] = coil_2;
        /*********application code*********/
        if(--regNum <= 0)
            break;
    default:
        return rsp_err_reg_addr;
    }
#ifdef DEBUG_READ_INPUT_STATE
    printf("Slave_ReadInputState regAddr: %X; regNum:%d\r\n", regAddr, regNum);
#endif
	return rsp_ok;
}

//#define DEBUG_WRITE_MUL_COILS
/**
 * @brief write_mul_coils 0x0F
 * @param regAddr register start address
 * @param regNum  register length 
 * @param data    data[0]-coil0 data[1]-coil1 ...and so on
 * @note 
 */
slave_rsp_code_def Slave_WriteMulCoils(uint16_t regAddr, uint16_t regNum, bool *data) {
    uint8_t i;
    for(i = 0; i < regNum; i++){
        switch (i+regAddr)
        {
        case 0x00:
            if(data[i] == true){
                printf("ON:start correction position\r\n");
            }else{
                printf("OFF:start correction position\r\n");
            }
            break;
        case 0x01:
            if(data[i] == true){
                printf("ON:start reset position\r\n");
            }else{
                printf("OFF:start reset position\r\n");
            }
            break;
        case 0x02:
            if(data[i] == true){
                printf("ON:force stop\r\n");
            }else{
                printf("OFF:force stop\r\n");
            }
            break;
        default:
            return rsp_err_reg_addr;
        }

    }
#ifdef DEBUG_WRITE_MUL_COILS
	printf("Slave_WriteMulCoils regAddr: %X; regNum:%d\n", regAddr, regNum);
    for(i = 0;i < regNum;i++){
        printf("coil %d:%d\r\n",i+regAddr,data[i]);
    }
#endif 
	return rsp_ok;
}

//#define DEBUG_WRITE_ONE_COIL
/**
 * @brief write_one_coil 0x05
 * @param regAddr register start address
 * @param val     coil state, true represent ON, false represent OFF.
 * @note add application code in if or else.
 */
slave_rsp_code_def Slave_WriteOneCoil(uint16_t regAddr, bool val) {
    uint8_t err = 0;
    err = manualBoxCoilUpdate(regAddr,val);
    if(err == 1)
        return rsp_err_reg_addr;
#ifdef DEBUG_WRITE_ONE_COIL
	printf("Slave_WriteOneCoil coli: %X; value:%d\r\n", regAddr, val);
#endif 
    return rsp_ok;
}

















