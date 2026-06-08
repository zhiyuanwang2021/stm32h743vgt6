#include "user.h"
#include <stdio.h>
#include <stdbool.h>

static uint32_t s_us_ticks = 0;

/* 多芯片片选管理 */
static uint8_t  cs5552_chip = 0;
static GPIO_TypeDef* cs_ports[2];
static uint16_t     cs_pins[2];

void CS5552_SelectChip(uint8_t chip) {
    cs5552_chip = (chip > 1) ? 0 : chip;
}

void CS5552_CS_LOW(void) {
    HAL_GPIO_WritePin(cs_ports[cs5552_chip], cs_pins[cs5552_chip], GPIO_PIN_RESET);
}

void CS5552_CS_HIGH(void) {
    HAL_GPIO_WritePin(cs_ports[cs5552_chip], cs_pins[cs5552_chip], GPIO_PIN_SET);
}

void CS5552_Delay_Init(void) {
    s_us_ticks = SystemCoreClock / 1000000U;
}

void CS5552_Delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

void CS5552_Delay_us(uint32_t us) {
    if (us == 0 || s_us_ticks == 0) {
        return;
    }

    uint32_t max_us = 0xFFFFFFFFU / s_us_ticks;
    if (us > max_us) {
        us = max_us;
    }
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * s_us_ticks;

    while ((DWT->CYCCNT - start) < ticks);
}

static HAL_StatusTypeDef SPI_TxRxByte(uint8_t txData, uint8_t *rxData) {
    return HAL_SPI_TransmitReceive(CS5552_HSPI, &txData, rxData, 1, CS5552_SPI_TIMEOUT_MS);
}

bool SPI_SendByte(uint8_t data) {
    uint8_t dummy = 0;
    return (SPI_TxRxByte(data, &dummy) == HAL_OK);
}

uint8_t SPI_ReceiveByte(void) {
    uint8_t data = 0x00;
    (void)SPI_TxRxByte(0xFF, &data);
    return data;
}

static uint8_t CS5552_CalcParity(uint8_t cmd) {
    uint8_t temp = cmd & 0xFEu;

    uint8_t count = temp;
    count = count - ((count >> 1) & 0x55u);
    count = (count & 0x33u) + ((count >> 2) & 0x33u);
    count = (count + (count >> 4)) & 0x0Fu;
    return (count & 0x01u) ? (cmd | 0x01u) : (cmd & 0xFEu);
}

void CS5552_Reset_SPI(void) {
    CS5552_CS_LOW();
    CS5552_Delay_us(5);
    (void)SPI_SendByte(0x00);
    (void)SPI_SendByte(0xA5);
    (void)SPI_SendByte(0xFF);
    (void)SPI_SendByte(0x5A);
    CS5552_CS_HIGH();
    CS5552_Delay_ms(2);
}

#define CS5552_REG_24BIT_MODE   0
bool CS5552_WriteReg(uint8_t reg_addr, uint32_t data) {
    if (reg_addr > REG_MAX_ADDR) {
        return false;
    }
    uint8_t cmd = (reg_addr & 0x0Fu) << 3;
    cmd = CS5552_CalcParity(cmd);
#if CS5552_REG_24BIT_MODE

    uint8_t txBuf[4] = {
        cmd,
        (uint8_t)(data >> 16),
        (uint8_t)(data >> 8),
        (uint8_t)(data)
    };
    uint8_t rxBuf[4] = {0};
    CS5552_CS_LOW();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(CS5552_HSPI, txBuf, rxBuf, 4, CS5552_SPI_TIMEOUT_MS);
#else

    uint8_t txBuf[5] = {
        cmd,
        (uint8_t)(data >> 24),
        (uint8_t)(data >> 16),
        (uint8_t)(data >> 8),
        (uint8_t)(data)
    };
    uint8_t rxBuf[5] = {0};
    CS5552_CS_LOW();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(CS5552_HSPI, txBuf, rxBuf, 5, CS5552_SPI_TIMEOUT_MS);
#endif
    CS5552_CS_HIGH();

    CS5552_Delay_us(10);
    return (status == HAL_OK);
}

bool CS5552_ReadReg(uint8_t reg_addr, uint32_t *out_data) {
    if (reg_addr > REG_MAX_ADDR || out_data == NULL) {
        return false;
    }
    uint8_t cmd = ((reg_addr & 0x0Fu) << 3) | (1u << 2);
    cmd = CS5552_CalcParity(cmd);
#if CS5552_REG_24BIT_MODE

    uint8_t txBuf[4] = {cmd, 0xFF, 0xFF, 0xFF};
    uint8_t rxBuf[4] = {0};
    CS5552_CS_LOW();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(CS5552_HSPI, txBuf, rxBuf, 4, CS5552_SPI_TIMEOUT_MS);
#else

    uint8_t txBuf[5] = {cmd, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t rxBuf[5] = {0};
    CS5552_CS_LOW();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(CS5552_HSPI, txBuf, rxBuf, 5, CS5552_SPI_TIMEOUT_MS);
#endif
    CS5552_CS_HIGH();
    if (status != HAL_OK) {
        return false;
    }
#if CS5552_REG_24BIT_MODE
    *out_data = ((uint32_t)rxBuf[1] << 16) |
                ((uint32_t)rxBuf[2] << 8)  |
                ((uint32_t)rxBuf[3]);
#else
    *out_data = ((uint32_t)rxBuf[1] << 24) |
                ((uint32_t)rxBuf[2] << 16) |
                ((uint32_t)rxBuf[3] << 8)  |
                ((uint32_t)rxBuf[4]);
#endif
    return true;
}

/**
 * @brief 配置单个通道 (清零失调/增益 + 写入 CONV_CONF)
 * @param channel 0=CH0, 1=CH1
 * @return true 成功, false 失败
 * @note  统一使用 DR=12.5Hz, PGA=128x，便于电桥/称重等弱信号采集
 */
static bool CS5552_ConfigChannel(uint8_t channel) {
    uint8_t  reg_os, reg_gain, reg_conv;
    uint32_t conv_conf;

    if (channel == 0) {
        reg_os    = REG_OS_CH0;
        reg_gain  = REG_GAIN_CH0;
        reg_conv  = REG_CONV_CONF0;
        conv_conf = CONV_CONF_CHN_CH0 | CONV_CONF_DR_800HZ | CONV_CONF_GA_128X; /* 0x000000C1 */
    } else if (channel == 1) {
        reg_os    = REG_OS_CH1;
        reg_gain  = REG_GAIN_CH1;
        reg_conv  = REG_CONV_CONF1;
        conv_conf = CONV_CONF_CHN_CH1 | CONV_CONF_DR_800HZ | CONV_CONF_GA_128X; /* 0x000010C1 */
    } else {
        return false;
    }

    /* 清零失调寄存器 */
    if (!CS5552_WriteReg(reg_os, 0x00000000)) {
        printf("CS5552 Init: CH%u OS write failed\r\n", channel);
        return false;
    }
    /* 增益寄存器写入 1.0 (定点 2.22 格式) */
    if (!CS5552_WriteReg(reg_gain, 0x02000000)) {
        printf("CS5552 Init: CH%u GAIN write failed\r\n", channel);
        return false;
    }
    /* 转换参数寄存器：通道选择 + DR + PGA */
    if (!CS5552_WriteReg(reg_conv, conv_conf)) {
        printf("CS5552 Init: CH%u CONV_CONF write failed\r\n", channel);
        return false;
    }
    return true;
}



/**
 * @brief 初始化 CS5552 芯片 (双通道版本)
 * @return true 成功, false 失败
 * @note  完成：复位 → 双通道参数配置 → 系统寄存器配置 → 异常检查 →
 *        双通道 Offset 自校准 → 稳定延时
 */
bool CS5552_Init(void) {
    static bool pin_map_done = false;
    if (!pin_map_done) {
        cs_ports[CS5552_CHIP_0] = P_ADCCS2_GPIO_Port;
        cs_pins[CS5552_CHIP_0]  = P_ADCCS2_Pin;
        cs_ports[CS5552_CHIP_1] = P_ADCCS3_GPIO_Port;
        cs_pins[CS5552_CHIP_1]  = P_ADCCS3_Pin;
        pin_map_done = true;
    }

    /* 初始化 DWT 延时参数 */
    CS5552_Delay_Init();

    CS5552_Reset_SPI();

    /* 1. 触发全局软件复位 */
    if (!CS5552_WriteReg(REG_SYS_CONF0, 0x80000000)) {
        printf("CS5552 Init: REG_SYS_CONF0 write failed\r\n");
        return false;
    }
    CS5552_Delay_ms(50);  /* 复位稳定：内部基准源 + 振荡器启动 */

    /* 2. 双通道参数配置 (校准寄存器清零 + CONV_CONFx) */
    if (!CS5552_ConfigChannel(0)) {
        return false;
    }
    if (!CS5552_ConfigChannel(1)) {
        return false;
    }

    /* 3. 清除复位位，进入正常工作态 */
    if (!CS5552_WriteReg(REG_SYS_CONF0, 0x00010000)) {
        printf("CS5552 Init: REG_SYS_CONF0 clear failed\r\n");
        return false;
    }

    /* 4. 系统配置寄存器 (使能 ADC 模拟核心 / 电流模式 / 时钟) */
    uint32_t sys_conf1 = 0x80110210;
    if (!CS5552_WriteReg(REG_SYS_CONF1, sys_conf1)) {
        printf("CS5552 Init: REG_SYS_CONF1 write failed\r\n");
        return false;
    }
    if (!CS5552_WriteReg(REG_SYS_CONF2, 0x00020000)) {
        printf("CS5552 Init: REG_SYS_CONF2 write failed\r\n");
        return false;
    }

    /* 5. 异常标志检查 */
    uint32_t sys0 = 0;
    if (!CS5552_ReadReg(REG_SYS_CONF0, &sys0)) {
        printf("CS5552 Init: REG_SYS_CONF0 read failed\r\n");
        return false;
    }
    if (sys0 & (SYS_CONF0_ERR_CKS | SYS_CONF0_ERR_C)) {
        printf("CS5552 Init: error flags set, SYS_CONF0=%08X\r\n", (unsigned)sys0);
        return false;
    }

    /* 6. 回读 SYS_CONF1 验证写入完整性 */
    uint32_t readback = 0;
    if (!CS5552_ReadReg(REG_SYS_CONF1, &readback)) {
        printf("CS5552 Init: REG_SYS_CONF1 readback failed\r\n");
        return false;
    }
    if (readback != sys_conf1) {
        printf("CS5552 Init: REG_SYS_CONF1 mismatch W:%08X R:%08X\r\n",
               (unsigned)sys_conf1, (unsigned)readback);
        return false;
    }

    /* 7. 双通道 Offset 自校准 (上电后必须执行一次以消除内部失调) */
    // printf("CS5552 Init: CH0 offset calibrating...\r\n");
    if (!CS5552_Offset_SelfCalibration(CS5552_CONFIG_IDX_CH0)) {
        printf("CS5552 Init: CH0 offset cal failed\r\n");
        return false;
    }
    // printf("CS5552 Init: CH1 offset calibrating...\r\n");
    if (!CS5552_Offset_SelfCalibration(CS5552_CONFIG_IDX_CH1)) {
        printf("CS5552 Init: CH1 offset cal failed\r\n");
        return false;
    }

    /* 8. 校准完成后稳定延时，确保后续转换数据有效 */
    CS5552_Delay_ms(20);

    // printf("CS5552 Init: dual-channel ready\r\n");
    return true;
}

static bool CS5552_WaitSDO_Low(uint32_t timeout_ms) {
    uint32_t tickstart = HAL_GetTick();
    while (CS5552_SDO_READ() != GPIO_PIN_RESET) {
        if ((HAL_GetTick() - tickstart) > timeout_ms) {
            return false;
        }
    }
    return true;
}

bool CS5552_Offset_SelfCalibration(uint8_t conv_conf_idx) {
    if (conv_conf_idx > 0x03) {
        return false;
    }
    uint8_t cmd = (1u << 7) | ((conv_conf_idx & 0x03u) << 4) | (CMD_MOD_OFFSET_CAL << 1);
    cmd = CS5552_CalcParity(cmd);
    CS5552_CS_LOW();
    if (!SPI_SendByte(cmd)) {
        CS5552_CS_HIGH();
        return false;
    }
    if (!CS5552_WaitSDO_Low(CS5552_CALIB_TIMEOUT_MS)) {
        CS5552_CS_HIGH();
        return false;
    }

    CS5552_SDO_READ() != GPIO_PIN_RESET;

    uint8_t txBuf[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t rxBuf[5] = {0};
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(CS5552_HSPI, txBuf, rxBuf, 5, CS5552_SPI_TIMEOUT_MS);
    CS5552_CS_HIGH();
    return (status == HAL_OK);
}

bool CS5552_StartContConv(uint8_t conv_conf_idx) {
    if (conv_conf_idx > 0x03) {
        return false;
    }
    uint8_t cmd = (1u << 7) | ((conv_conf_idx & 0x03u) << 4) | (CMD_MOD_CONT_CONV << 1);
    cmd = CS5552_CalcParity(cmd);
    CS5552_CS_LOW();
    CS5552_Delay_us(10);
    if (!SPI_SendByte(cmd)) {
        CS5552_CS_HIGH();
        printf("ContConv: cmd send failed\r\n");
        return false;
    }
    CS5552_Delay_us(20);

    /* CSHIGH_MODE=1 时，命令锁存后 CS 可释放，ADC 内部继续转换 */
    CS5552_CS_HIGH();
    return true;
}

/**
 * @brief  CSHIGH_MODE 下的连续转换数据读取
 * @param  out_data 输出有符号 ADC 码值
 * @retval true 成功, false 超时或 SPI 错误
 * @note   时序：CS↓ → 等 SDO↓(数据就绪) → 读 32-bit → CS↑(释放总线)
 *         前置条件：SYS_CONF0 CSHIGH_MODE=1, CKS_EN=0, 已 StartContConv
 *         CS 高期间 SPI 总线可复用于其他芯片
 */
bool CS5552_ReadContData(int32_t *out_data) {
    if (out_data == NULL) {
        return false;
    }

    /* 1. 拉低 CS，选中芯片 */
    CS5552_CS_LOW();

    /* 2. 等待 SDO 拉低（转换数据就绪） */
    if (!CS5552_WaitSDO_Low(CS5552_SDO_TIMEOUT_MS)) {
        CS5552_CS_HIGH();
        return false;
    }

    /* 3. 读取 32-bit 数据帧 */
    if (!CS5552_ReadADC(out_data)) {
        CS5552_CS_HIGH();
        return false;
    }

    /* 4. 拉高 CS，释放 SPI 总线供其他芯片使用 */
    CS5552_CS_HIGH();
    return true;
}

/**
 * @brief  启动单次转换（时序与 CS5552_StartContConv 完全一致）
 * @param  conv_conf_idx 转换配置索引
 * @retval true 成功, false 失败
 * @note   CS↓ → delay 10us → 发 SINGLE_CONV 命令 → delay 20us → 返回（CS 保持低）
 *         返回后需等 SDO↓ 再读数据，读完需 CS↑
 */
bool CS5552_StartSingleConv(uint8_t conv_conf_idx) {
    if (conv_conf_idx > 0x03) {
        return false;
    }
    uint8_t cmd = (1u << 7) | ((conv_conf_idx & 0x03u) << 4) | (CMD_MOD_SINGLE_CONV << 1);
    cmd = CS5552_CalcParity(cmd);
    CS5552_CS_LOW();
    CS5552_Delay_us(10);
    if (!SPI_SendByte(cmd)) {
        CS5552_CS_HIGH();
        return false;
    }
    CS5552_Delay_us(20);
    return true;
}

bool CS5552_ReadADC(int32_t *out_data) {
    if (out_data == NULL) {
        return false;
    }

    uint8_t txBuf[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t rxBuf[5] = {0};
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(CS5552_HSPI, txBuf, rxBuf, 5, CS5552_SPI_TIMEOUT_MS);
    if (status != HAL_OK) {
        printf("ADC: SPI read failed, status=%d\r\n", (int)status);
        return false;
    }
    uint32_t raw_data = ((uint32_t)rxBuf[1] << 24) |
                        ((uint32_t)rxBuf[2] << 16) |
                        ((uint32_t)rxBuf[3] << 8)  |
                        ((uint32_t)rxBuf[4]);
    int32_t signed_data = (int32_t)(raw_data & 0xFFFFFFFEu);
    signed_data >>= 1;
    *out_data = signed_data;
    return true;
}

/**
 * @brief  解析并校验 CONV_DATA 寄存器原始值
 * @param  raw_data         CONV_DATA 寄存器 32-bit 原始值
 * @param  expected_channel 期望通道 (0=CH0, 1=CH1, 其他=不校验通道)
 * @param  out_data         输出有符号 31-bit ADC 码值 (bit[31:1] 经掩码移位)
 * @param  out_channel      输出实际通道标识 (可选, 传 NULL 忽略)
 * @retval true 数据格式有效, false 校验失败
 * @note   数据格式参考 Table 5-11: bit[31:1]=ADC DATA, bit[0]=CHL
 *         满量程: 0x3FFFFFFF=+1.00, 0xC0000000=-1.00
 */
bool CS5552_ParseConvData(uint32_t raw_data, uint8_t expected_channel,
                          int32_t *out_data, uint8_t *out_channel) {
    if (out_data == NULL) {
        return false;
    }

    /* 提取 bit[0] 通道标识 */
    uint8_t channel = (raw_data & CS5552_DATA_CHANNEL_BIT) ? 1 : 0;
    if (out_channel != NULL) {
        *out_channel = channel;
    }

    /* 校验通道匹配 */
    if (expected_channel <= 1 && channel != expected_channel) {
        printf("ADC Data: channel mismatch exp=%u got=%u raw=0x%08lX\r\n",
               expected_channel, channel, (unsigned long)raw_data);
        return false;
    }

    /* 掩码 bit[0] 后算术右移 1 位, 得到 31-bit 有符号值 */
    int32_t signed_data = (int32_t)(raw_data & 0xFFFFFFFEu);
    signed_data >>= 1;
    *out_data = signed_data;

    /* 校验数据范围: 不应超出满量程 */
    if (signed_data > CS5552_DATA_MAX_VALID || signed_data < CS5552_DATA_MIN_VALID) {
        printf("ADC Data: out of range val=%ld raw=0x%08lX\r\n",
               (long)signed_data, (unsigned long)raw_data);
        return false;
    }

    return true;
}

/**
 * @brief  单次转换模式读取单通道 ADC 数据
 * @param  conv_conf_idx 转换配置索引 (CS5552_CONFIG_IDX_CH0/CH1)
 * @param  out_data      输出有符号 ADC 码值
 * @param  out_channel   输出通道标识 (可为 NULL)
 * @retval true 成功, false 失败
 * @note   时序：StartSingleConv(CS↓→cmd→CS保持低) → 等 SDO↓ → 读数据 → CS↑
 *         数据帧格式与 CS5552_ReadADC 一致
 */
bool CS5552_SingleChannelRead(uint8_t conv_conf_idx, int32_t *out_data, uint8_t *out_channel) {
    if (out_data == NULL || conv_conf_idx > 0x03) {
        return false;
    }

    /* 1. 启动单次转换（时序与 StartContConv 一致，CS 保持低） */
    if (!CS5552_StartSingleConv(conv_conf_idx)) {
        return false;
    }

    /* 2. 等待转换完成：SDO↓ 表示数据就绪 */
    // uint32_t sdo_wait_start = HAL_GetTick();
    if (!CS5552_WaitSDO_Low(CS5552_SDO_TIMEOUT_MS)) {
        // printf("ADC: SDO timeout, waited=%lums\r\n",
        //        (unsigned long)(HAL_GetTick() - sdo_wait_start));
        CS5552_CS_HIGH();
        return false;
    }
    // CS5552_Delay_ms(1);
    // printf("SDO is %d\r\n", CS5552_SDO_READ() != GPIO_PIN_RESET);
    return CS5552_ReadADC(out_data);   
}

float CS5552_ConvertToVoltage(int32_t raw_code, uint8_t pga_gain) {
    if (pga_gain == 0) {
        return 0.0f;
    }

    float full_scale_mv = (float)CS5552_VREF_MV / (float)pga_gain;
    return ((float)raw_code * full_scale_mv) / (float)(1L << 23);
}


void LED_disp(uint8_t led) {
    HAL_GPIO_WritePin(P_LEDWK0_GPIO_Port, P_LEDWK0_Pin, (led & 0x01) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(P_LEDWK1_GPIO_Port, P_LEDWK1_Pin, (led & 0x02) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
static uint32_t led_tick = 0;

void LED_proc(void) {
    if (HAL_GetTick() - led_tick < 200) {
        return;
    }
    led_tick = HAL_GetTick();
    HAL_GPIO_TogglePin(P_LEDWK0_GPIO_Port, P_LEDWK0_Pin);
    HAL_GPIO_TogglePin(P_LEDWK1_GPIO_Port, P_LEDWK1_Pin);
}
