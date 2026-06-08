#include "user.h"
#include <stdio.h>
#include <stdbool.h>

static uint32_t s_us_ticks = 0;
static uint8_t cs5552_chip = 0;
static GPIO_TypeDef *cs_ports[2];
static uint16_t cs_pins[2];
static bool s_checksum_enabled[2] = {false, false};
static uint32_t s_spi_error_count[2] = {0u, 0u};
static uint32_t s_checksum_error_count[2] = {0u, 0u};
static uint32_t s_data_error_count[2] = {0u, 0u};

static HAL_StatusTypeDef SPI_TxRxByte(uint8_t txData, uint8_t *rxData) {
    return HAL_SPI_TransmitReceive(CS5552_HSPI, &txData, rxData, 1, CS5552_SPI_TIMEOUT_MS);
}

static uint8_t CS5552_CalcParity(uint8_t cmd) {
    uint8_t temp = cmd & 0xFEu;
    uint8_t count = temp;

    count = count - ((count >> 1) & 0x55u);
    count = (count & 0x33u) + ((count >> 2) & 0x33u);
    count = (count + (count >> 4)) & 0x0Fu;
    return (count & 0x01u) ? (cmd | 0x01u) : (cmd & 0xFEu);
}

static bool CS5552_ChecksumEnabled(void) {
    return s_checksum_enabled[cs5552_chip];
}

static void CS5552_SetChecksumEnabled(bool enabled) {
    s_checksum_enabled[cs5552_chip] = enabled;
}

static void CS5552_RecordSpiError(void) {
    s_spi_error_count[cs5552_chip]++;
}

static void CS5552_RecordChecksumError(void) {
    s_checksum_error_count[cs5552_chip]++;
}

static void CS5552_RecordDataError(void) {
    s_data_error_count[cs5552_chip]++;
}

static void CS5552_PackU32(uint32_t data, uint8_t bytes[4]) {
    bytes[0] = (uint8_t)(data >> 24);
    bytes[1] = (uint8_t)(data >> 16);
    bytes[2] = (uint8_t)(data >> 8);
    bytes[3] = (uint8_t)data;
}

static uint32_t CS5552_UnpackU32(const uint8_t bytes[4]) {
    return ((uint32_t)bytes[0] << 24) |
           ((uint32_t)bytes[1] << 16) |
           ((uint32_t)bytes[2] << 8)  |
           ((uint32_t)bytes[3]);
}

static uint8_t CS5552_CalcDataChecksum(const uint8_t data_bytes[4]) {
    uint32_t sum = (uint32_t)CS5552_CHECKSUM_SEED +
                   (uint32_t)data_bytes[0] +
                   (uint32_t)data_bytes[1] +
                   (uint32_t)data_bytes[2] +
                   (uint32_t)data_bytes[3];
    return (uint8_t)sum;
}

static bool CS5552_VerifyDataChecksum(const uint8_t data_bytes[4], uint8_t checksum, const char *context) {
    uint8_t expected = CS5552_CalcDataChecksum(data_bytes);
    if (checksum == expected) {
        return true;
    }

    CS5552_RecordChecksumError();
    // printf("%s checksum mismatch: got=0x%02X exp=0x%02X\r\n", context, checksum, expected);
    return false;
}

static bool CS5552_ReadConversionFrame(uint32_t *out_raw_data) {
    bool use_checksum;
    uint8_t txBuf[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t rxBuf[6] = {0};
    uint16_t frame_len;
    HAL_StatusTypeDef status;

    if (out_raw_data == NULL) {
        return false;
    }

    use_checksum = CS5552_ChecksumEnabled();
    frame_len = use_checksum ? 6u : 5u;
    status = HAL_SPI_TransmitReceive(CS5552_HSPI, txBuf, rxBuf, frame_len, CS5552_SPI_TIMEOUT_MS);
    if (status != HAL_OK) {
        CS5552_RecordSpiError();
        printf("ADC: SPI read failed, status=%d\r\n", (int)status);
        return false;
    }

    *out_raw_data = CS5552_UnpackU32(&rxBuf[1]);
    if (use_checksum && !CS5552_VerifyDataChecksum(&rxBuf[1], rxBuf[5], "ADC")) {
        return false;
    }

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

static bool CS5552_ConfigChannel(uint8_t channel) {
    uint8_t reg_os;
    uint8_t reg_gain;
    uint8_t reg_conv;
    uint32_t conv_conf;

    if (channel == 0u) {
        reg_os = REG_OS_CH0;
        reg_gain = REG_GAIN_CH0;
        reg_conv = REG_CONV_CONF0;
        conv_conf = CONV_CONF_CHN_CH0 | CONV_CONF_DR_3200HZ | CONV_CONF_GA_64X;
    } else if (channel == 1u) {
        reg_os = REG_OS_CH1;
        reg_gain = REG_GAIN_CH1;
        reg_conv = REG_CONV_CONF1;
        conv_conf = CONV_CONF_CHN_CH1 | CONV_CONF_DR_3200HZ | CONV_CONF_GA_64X;
    } else {
        return false;
    }

    if (!CS5552_WriteReg(reg_os, 0x00000000u)) {
        printf("CS5552 Init: CH%u OS write failed\r\n", channel);
        return false;
    }
    if (!CS5552_WriteReg(reg_gain, 0x02000000u)) {
        printf("CS5552 Init: CH%u GAIN write failed\r\n", channel);
        return false;
    }
    if (!CS5552_WriteReg(reg_conv, conv_conf)) {
        printf("CS5552 Init: CH%u CONV_CONF write failed\r\n", channel);
        return false;
    }

    return true;
}

void CS5552_SelectChip(uint8_t chip) {
    cs5552_chip = (chip > 1u) ? 0u : chip;
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
    uint32_t max_us;
    uint32_t start;
    uint32_t ticks;

    if ((us == 0u) || (s_us_ticks == 0u)) {
        return;
    }

    max_us = 0xFFFFFFFFU / s_us_ticks;
    if (us > max_us) {
        us = max_us;
    }

    start = DWT->CYCCNT;
    ticks = us * s_us_ticks;
    while ((DWT->CYCCNT - start) < ticks) {
    }
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

void CS5552_Reset_SPI(void) {
    CS5552_CS_LOW();
    CS5552_Delay_us(5);
    (void)SPI_SendByte(0x00);
    (void)SPI_SendByte(0xA5);
    (void)SPI_SendByte(0xFF);
    (void)SPI_SendByte(0x5A);
    CS5552_CS_HIGH();
    CS5552_Delay_ms(2);
    CS5552_SetChecksumEnabled(false);
}

bool CS5552_WriteReg(uint8_t reg_addr, uint32_t data) {
    uint8_t cmd;
    bool use_checksum;
    bool enable_checksum_after_write;
    bool disable_checksum_after_write;
    uint8_t txBuf[6] = {0};
    uint8_t rxBuf[6] = {0};
    uint16_t frame_len;
    HAL_StatusTypeDef status;

    if (reg_addr > REG_MAX_ADDR) {
        return false;
    }

    cmd = (uint8_t)((reg_addr & 0x0Fu) << 3);
    cmd = CS5552_CalcParity(cmd);
    use_checksum = CS5552_ChecksumEnabled();
    enable_checksum_after_write = (!use_checksum) &&
                                  (reg_addr == REG_SYS_CONF0) &&
                                  ((data & SYS_CONF0_CKS_EN) != 0u);
    disable_checksum_after_write = use_checksum &&
                                   (reg_addr == REG_SYS_CONF0) &&
                                   ((data & SYS_CONF0_CKS_EN) == 0u);

    txBuf[0] = cmd;
    CS5552_PackU32(data, &txBuf[1]);
    if (use_checksum) {
        txBuf[5] = CS5552_CalcDataChecksum(&txBuf[1]);
    }

    frame_len = use_checksum ? 6u : 5u;
    CS5552_CS_LOW();
    status = HAL_SPI_TransmitReceive(CS5552_HSPI, txBuf, rxBuf, frame_len, CS5552_SPI_TIMEOUT_MS);
    CS5552_CS_HIGH();

    if (status != HAL_OK) {
        CS5552_RecordSpiError();
    } else if (enable_checksum_after_write) {
        CS5552_SetChecksumEnabled(true);
    } else if (disable_checksum_after_write) {
        CS5552_SetChecksumEnabled(false);
    }

    CS5552_Delay_us(10);
    return (status == HAL_OK);
}

bool CS5552_ReadReg(uint8_t reg_addr, uint32_t *out_data) {
    uint8_t cmd;
    bool use_checksum;
    uint8_t txBuf[6] = {0};
    uint8_t rxBuf[6] = {0};
    uint16_t frame_len;
    HAL_StatusTypeDef status;

    if ((reg_addr > REG_MAX_ADDR) || (out_data == NULL)) {
        return false;
    }

    cmd = (uint8_t)(((reg_addr & 0x0Fu) << 3) | (1u << 2));
    cmd = CS5552_CalcParity(cmd);
    use_checksum = CS5552_ChecksumEnabled();
    frame_len = use_checksum ? 6u : 5u;

    txBuf[0] = cmd;
    txBuf[1] = 0xFF;
    txBuf[2] = 0xFF;
    txBuf[3] = 0xFF;
    txBuf[4] = 0xFF;
    txBuf[5] = 0xFF;

    CS5552_CS_LOW();
    status = HAL_SPI_TransmitReceive(CS5552_HSPI, txBuf, rxBuf, frame_len, CS5552_SPI_TIMEOUT_MS);
    CS5552_CS_HIGH();
    if (status != HAL_OK) {
        CS5552_RecordSpiError();
        return false;
    }

    if (use_checksum && !CS5552_VerifyDataChecksum(&rxBuf[1], rxBuf[5], "REG")) {
        return false;
    }

    *out_data = CS5552_UnpackU32(&rxBuf[1]);
    return true;
}

bool CS5552_Init(void) {
    static bool pin_map_done = false;
    uint32_t sys_conf1 = 0x80110210u;
    uint32_t sys0 = 0;
    uint32_t readback = 0;

    if (!pin_map_done) {
        cs_ports[CS5552_CHIP_0] = P_ADCCS2_GPIO_Port;
        cs_pins[CS5552_CHIP_0] = P_ADCCS2_Pin;
        cs_ports[CS5552_CHIP_1] = P_ADCCS3_GPIO_Port;
        cs_pins[CS5552_CHIP_1] = P_ADCCS3_Pin;
        pin_map_done = true;
    }

    CS5552_Delay_Init();
    CS5552_Reset_SPI();

    if (!CS5552_WriteReg(REG_SYS_CONF0, SYS_CONF0_RESET)) {
        printf("CS5552 Init: REG_SYS_CONF0 write failed\r\n");
        return false;
    }
    CS5552_Delay_ms(50);

    if (!CS5552_WriteReg(REG_SYS_CONF0, SYS_CONF0_CKS_EN | SYS_CONF0_CSHIGH_MODE)) {
        printf("CS5552 Init: REG_SYS_CONF0 checksum enable failed\r\n");
        return false;
    }
    if (!CS5552_ReadReg(REG_SYS_CONF0, &sys0)) {
        printf("CS5552 Init: REG_SYS_CONF0 checksum verify failed\r\n");
        return false;
    }
    if ((sys0 & (SYS_CONF0_CKS_EN | SYS_CONF0_CSHIGH_MODE)) !=
        (SYS_CONF0_CKS_EN | SYS_CONF0_CSHIGH_MODE)) {
        printf("CS5552 Init: SYS_CONF0 missing flags, SYS_CONF0=%08X\r\n", (unsigned)sys0);
        return false;
    }

    if (!CS5552_ConfigChannel(0u)) {
        return false;
    }
    if (!CS5552_ConfigChannel(1u)) {
        return false;
    }

    if (!CS5552_WriteReg(REG_SYS_CONF1, sys_conf1)) {
        printf("CS5552 Init: REG_SYS_CONF1 write failed\r\n");
        return false;
    }
    if (!CS5552_WriteReg(REG_SYS_CONF2, 0x00020000u)) {
        printf("CS5552 Init: REG_SYS_CONF2 write failed\r\n");
        return false;
    }

    if (!CS5552_ReadReg(REG_SYS_CONF0, &sys0)) {
        printf("CS5552 Init: REG_SYS_CONF0 read failed\r\n");
        return false;
    }
    if ((sys0 & (SYS_CONF0_CKS_EN | SYS_CONF0_CSHIGH_MODE)) !=
        (SYS_CONF0_CKS_EN | SYS_CONF0_CSHIGH_MODE)) {
        printf("CS5552 Init: SYS_CONF0 missing flags, SYS_CONF0=%08X\r\n", (unsigned)sys0);
        return false;
    }
    if (sys0 & (SYS_CONF0_ERR_CKS | SYS_CONF0_ERR_C)) {
        printf("CS5552 Init: error flags set, SYS_CONF0=%08X\r\n", (unsigned)sys0);
        return false;
    }

    if (!CS5552_ReadReg(REG_SYS_CONF1, &readback)) {
        printf("CS5552 Init: REG_SYS_CONF1 readback failed\r\n");
        return false;
    }
    if (readback != sys_conf1) {
        printf("CS5552 Init: REG_SYS_CONF1 mismatch W:%08X R:%08X\r\n",
               (unsigned)sys_conf1, (unsigned)readback);
        return false;
    }

    if (!CS5552_Offset_SelfCalibration(CS5552_CONFIG_IDX_CH0)) {
        printf("CS5552 Init: CH0 offset cal failed\r\n");
        return false;
    }
    if (!CS5552_Offset_SelfCalibration(CS5552_CONFIG_IDX_CH1)) {
        printf("CS5552 Init: CH1 offset cal failed\r\n");
        return false;
    }

    CS5552_Delay_ms(20);
    return true;
}

bool CS5552_Offset_SelfCalibration(uint8_t conv_conf_idx) {
    uint8_t cmd;
    uint32_t raw_data = 0;
    int32_t adc_data = 0;

    if (conv_conf_idx > 0x03u) {
        return false;
    }

    cmd = (uint8_t)((1u << 7) | ((conv_conf_idx & 0x03u) << 4) | (CMD_MOD_OFFSET_CAL << 1));
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

    if (!CS5552_ReadConversionFrame(&raw_data)) {
        CS5552_CS_HIGH();
        return false;
    }
    CS5552_CS_HIGH();

    if (!CS5552_ParseConvData(raw_data, 2u, &adc_data, NULL)) {
        CS5552_RecordDataError();
        return false;
    }

    return true;
}

bool CS5552_StartContConv(uint8_t conv_conf_idx) {
    uint8_t cmd;

    if (conv_conf_idx > 0x03u) {
        return false;
    }

    cmd = (uint8_t)((1u << 7) | ((conv_conf_idx & 0x03u) << 4) | (CMD_MOD_CONT_CONV << 1));
    cmd = CS5552_CalcParity(cmd);
    CS5552_CS_LOW();
    CS5552_Delay_us(10);
    if (!SPI_SendByte(cmd)) {
        CS5552_CS_HIGH();
        printf("ContConv: cmd send failed\r\n");
        return false;
    }
    CS5552_Delay_us(20);
    CS5552_CS_HIGH();
    return true;
}

bool CS5552_ReadContData(int32_t *out_data) {
    if (out_data == NULL) {
        return false;
    }

    CS5552_CS_LOW();
    if (!CS5552_WaitSDO_Low(CS5552_SDO_TIMEOUT_MS)) {
        CS5552_CS_HIGH();
        return false;
    }
    if (!CS5552_ReadADC(out_data)) {
        CS5552_CS_HIGH();
        return false;
    }
    CS5552_CS_HIGH();
    return true;
}

bool CS5552_StartSingleConv(uint8_t conv_conf_idx) {
    uint8_t cmd;

    if (conv_conf_idx > 0x03u) {
        return false;
    }

    cmd = (uint8_t)((1u << 7) | ((conv_conf_idx & 0x03u) << 4) | (CMD_MOD_SINGLE_CONV << 1));
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
    uint32_t raw_data = 0;
    int32_t signed_data = 0;

    if (out_data == NULL) {
        return false;
    }

    if (!CS5552_ReadConversionFrame(&raw_data)) {
        return false;
    }
    if (!CS5552_ParseConvData(raw_data, 2u, &signed_data, NULL)) {
        CS5552_RecordDataError();
        return false;
    }

    *out_data = signed_data;
    return true;
}

bool CS5552_ParseConvData(uint32_t raw_data, uint8_t expected_channel,
                          int32_t *out_data, uint8_t *out_channel) {
    uint8_t channel;
    int32_t signed_data;

    if (out_data == NULL) {
        return false;
    }

    channel = (raw_data & CS5552_DATA_CHANNEL_BIT) ? 1u : 0u;
    if (out_channel != NULL) {
        *out_channel = channel;
    }

    if ((expected_channel <= 1u) && (channel != expected_channel)) {
        printf("ADC Data: channel mismatch exp=%u got=%u raw=0x%08lX\r\n",
               expected_channel, channel, (unsigned long)raw_data);
        return false;
    }

    signed_data = (int32_t)(raw_data & 0xFFFFFFFEu);
    signed_data >>= 1;
    *out_data = signed_data;

    if ((signed_data > CS5552_DATA_MAX_VALID) || (signed_data < CS5552_DATA_MIN_VALID)) {
        printf("ADC Data: out of range val=%ld raw=0x%08lX\r\n",
               (long)signed_data, (unsigned long)raw_data);
        return false;
    }

    return true;
}

bool CS5552_SingleChannelRead(uint8_t conv_conf_idx, int32_t *out_data, uint8_t *out_channel) {
    uint32_t raw_data = 0;
    uint8_t expected_channel;

    if ((out_data == NULL) || (conv_conf_idx > 0x03u)) {
        return false;
    }

    if (!CS5552_StartSingleConv(conv_conf_idx)) {
        return false;
    }
    if (!CS5552_WaitSDO_Low(CS5552_SDO_TIMEOUT_MS)) {
        CS5552_CS_HIGH();
        return false;
    }
    if (!CS5552_ReadConversionFrame(&raw_data)) {
        CS5552_CS_HIGH();
        return false;
    }
    CS5552_CS_HIGH();

    expected_channel = (conv_conf_idx == CS5552_CONFIG_IDX_CH1) ? 1u : 0u;
    if (!CS5552_ParseConvData(raw_data, expected_channel, out_data, out_channel)) {
        CS5552_RecordDataError();
        return false;
    }

    return true;
}

float CS5552_ConvertToVoltage(int32_t raw_code, uint8_t pga_gain) {
    float full_scale_mv;

    if (pga_gain == 0u) {
        return 0.0f;
    }

    full_scale_mv = (float)CS5552_VREF_MV / (float)pga_gain;
    return ((float)raw_code * full_scale_mv) / (float)(1L << 23);
}

void LED_disp(uint8_t led) {
    HAL_GPIO_WritePin(P_LEDWK0_GPIO_Port, P_LEDWK0_Pin, (led & 0x01u) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(P_LEDWK1_GPIO_Port, P_LEDWK1_Pin, (led & 0x02u) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static uint32_t led_tick = 0;

void LED_proc(void) {
    if (HAL_GetTick() - led_tick < 200u) {
        return;
    }

    led_tick = HAL_GetTick();
    HAL_GPIO_TogglePin(P_LEDWK0_GPIO_Port, P_LEDWK0_Pin);
    HAL_GPIO_TogglePin(P_LEDWK1_GPIO_Port, P_LEDWK1_Pin);
}
