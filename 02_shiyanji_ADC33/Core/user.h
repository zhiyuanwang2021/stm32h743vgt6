#ifndef __USER_H__
#define __USER_H__

#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#define CS5552_HSPI  (&hspi3)

/* 多芯片片选索引 */
#define CS5552_CHIP_0    0   /* P_ADCCS2 (PD3) */
#define CS5552_CHIP_1    1   /* P_ADCCS3 (PD0) */

void CS5552_SelectChip(uint8_t chip);
void CS5552_CS_LOW(void);
void CS5552_CS_HIGH(void);

#define CS5552_SDO_READ() HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11)

#define CS5552_SPI_TIMEOUT_MS       1000U
#define CS5552_SDO_TIMEOUT_MS       5000U
#define CS5552_CALIB_TIMEOUT_MS     2000U

#ifndef CS5552_VREF_MV
#define CS5552_VREF_MV              2500
#endif

#define REG_OS_CH0          0x00
#define REG_GAIN_CH0        0x01
#define REG_OS_CH1          0x02
#define REG_GAIN_CH1        0x03
#define REG_CONV_CONF0      0x04
#define REG_CONV_CONF1      0x05
#define REG_SYS_CONF0       0x06
#define REG_SYS_CONF1       0x07
#define REG_SYS_CONF2       0x08
#define REG_D_TARG          0x09
#define REG_CONV_DATA       0x0A

#define REG_MAX_ADDR        REG_CONV_DATA

#define SYS_CONF0_RESET         (1UL << 31)
#define SYS_CONF0_CSHIGH_MODE   (1UL << 16)
#define SYS_CONF0_ERR_CKS       (1 << 2)
#define SYS_CONF0_ERR_C         (1 << 1)

#define CONV_CONF0_PGA_SHIFT    5
#define CONV_CONF0_PGA_MASK     (0x07U << CONV_CONF0_PGA_SHIFT)
#define CONV_CONF0_PGA_1        (0x00U << CONV_CONF0_PGA_SHIFT)
#define CONV_CONF0_PGA_2        (0x01U << CONV_CONF0_PGA_SHIFT)
#define CONV_CONF0_PGA_4        (0x02U << CONV_CONF0_PGA_SHIFT)
#define CONV_CONF0_PGA_8        (0x03U << CONV_CONF0_PGA_SHIFT)
#define CONV_CONF0_PGA_16       (0x04U << CONV_CONF0_PGA_SHIFT)
#define CONV_CONF0_PGA_32       (0x05U << CONV_CONF0_PGA_SHIFT)
#define CONV_CONF0_PGA_64       (0x06U << CONV_CONF0_PGA_SHIFT)
#define CONV_CONF0_PGA_128      (0x07U << CONV_CONF0_PGA_SHIFT)

#define CONV_CONF0_RATE_SHIFT   0
#define CONV_CONF0_RATE_MASK    (0x1FU << CONV_CONF0_RATE_SHIFT)
#define CONV_CONF0_RATE_12_5HZ  (0x01U << CONV_CONF0_RATE_SHIFT)

#define SYS_CONF1_ADCPDN        (1UL << 16)
#define SYS_CONF1_ADITA_10      (2U << 8)
#define SYS_CONF1_CKMODE0       (1U << 4)

#define CS5552_SYS_CONF1_TYPICAL    0x00010210UL
#define CS5552_SYS_CONF2_TYPICAL    0x00020000UL

#define CMD_MOD_SINGLE_CONV  0x00
#define CMD_MOD_CONT_CONV    0x01
#define CMD_MOD_OFFSET_CAL   0x02
#define CMD_MOD_GAIN_CAL     0x06


/* --- SYS_CONF0 (0x06) --- */
#define SYS_CONF0_RESET         (1UL << 31)     /* 全局软件复位 */
#define SYS_CONF0_ERR_CKS       (1 << 2)        /* SPI校验错误标志 */
#define SYS_CONF0_ERR_C         (1 << 1)        /* 转换错误标志 */

/* --- CONV_CONFx (0x04/0x05) 位域（根据数据手册） ---
 * bit[7:4] DR  : 数据输出码率 (FR_SEL=0/HBF_EN=0 下默认)
 * bit[3]   保留 : 写 0
 * bit[2:0] GA  : 模拟增益
 */

/* --- DR<3:0> 数据率字段 @ bit[7:4] --- */
#define CONV_CONF_DR_SHIFT      4
#define CONV_CONF_DR_MASK       (0x0FU << CONV_CONF_DR_SHIFT)
#define CONV_CONF_DR_51200HZ    (0x0U << CONV_CONF_DR_SHIFT)
#define CONV_CONF_DR_25600HZ    (0x1U << CONV_CONF_DR_SHIFT)
#define CONV_CONF_DR_12800HZ    (0x2U << CONV_CONF_DR_SHIFT)
#define CONV_CONF_DR_6400HZ     (0x3U << CONV_CONF_DR_SHIFT)
#define CONV_CONF_DR_3200HZ     (0x4U << CONV_CONF_DR_SHIFT)
#define CONV_CONF_DR_1600HZ     (0x5U << CONV_CONF_DR_SHIFT)
#define CONV_CONF_DR_800HZ      (0x6U << CONV_CONF_DR_SHIFT)
#define CONV_CONF_DR_400HZ      (0x7U << CONV_CONF_DR_SHIFT)
#define CONV_CONF_DR_200HZ      (0x8U << CONV_CONF_DR_SHIFT)
#define CONV_CONF_DR_100HZ      (0x9U << CONV_CONF_DR_SHIFT)
#define CONV_CONF_DR_50HZ       (0xAU << CONV_CONF_DR_SHIFT)
#define CONV_CONF_DR_25HZ       (0xBU << CONV_CONF_DR_SHIFT)
#define CONV_CONF_DR_12_5HZ     (0xCU << CONV_CONF_DR_SHIFT)   /* 当前选用 */
#define CONV_CONF_DR_6_25HZ     (0xDU << CONV_CONF_DR_SHIFT)

/* --- GA<2:0> 增益字段 @ bit[2:0]（注意：编码非线性） --- */
#define CONV_CONF_GA_SHIFT      0
#define CONV_CONF_GA_MASK       (0x07U << CONV_CONF_GA_SHIFT)
#define CONV_CONF_GA_64X        (0x0U << CONV_CONF_GA_SHIFT)   /* ×64 (默认) */
#define CONV_CONF_GA_128X       (0x1U << CONV_CONF_GA_SHIFT)   /* ×128  当前选用 */
#define CONV_CONF_GA_16X        (0x2U << CONV_CONF_GA_SHIFT)   /* ×16 */
#define CONV_CONF_GA_32X        (0x3U << CONV_CONF_GA_SHIFT)   /* ×32 */
#define CONV_CONF_GA_4X         (0x4U << CONV_CONF_GA_SHIFT)   /* ×4 */
#define CONV_CONF_GA_8X         (0x5U << CONV_CONF_GA_SHIFT)   /* ×8 */
#define CONV_CONF_GA_1X         (0x6U << CONV_CONF_GA_SHIFT)   /* ×1 */
#define CONV_CONF_GA_2X         (0x7U << CONV_CONF_GA_SHIFT)   /* ×2 */

/* --- CONV_CONFx 通道选择位 CHN @ bit12 --- */
#define CONV_CONF_CHN_SHIFT     12
#define CONV_CONF_CHN_MASK      (1U << CONV_CONF_CHN_SHIFT)
#define CONV_CONF_CHN_CH0       (0U << CONV_CONF_CHN_SHIFT)   /* 选择通道 0 */
#define CONV_CONF_CHN_CH1       (1U << CONV_CONF_CHN_SHIFT)   /* 选择通道 1 */

/* 双通道推荐配置集索引 */
#define CS5552_CONFIG_IDX_CH0   0     /* CONV_CONF0 专用于 CH0 */
#define CS5552_CONFIG_IDX_CH1   2     /* CONV_CONF1 专用于 CH1 */

/* 转换数据帧 bit[0] 为通道标识 */
#define CS5552_DATA_CHANNEL_BIT 0x01u  /* bit0=0→CH0, bit0=1→CH1 */

/* CONV_DATA 数据格式验证限值 (参考 Table 5-11) */
#define CS5552_RAW_MAX_POSITIVE  0x3FFFFFFFu  /* +1.00 FS (含CHL=1) */
#define CS5552_RAW_MIN_NEGATIVE  0xC0000000u   /* -1.00 FS (含CHL=0) */
#define CS5552_DATA_MAX_VALID    536870911      /* 处理后的 +1.00 FS */
#define CS5552_DATA_MIN_VALID    (-536870912)   /* 处理后的 -1.00 FS */

/* --- SYS_CONF1 (0x07) 位定义（按数据手册 Table 5-8） --- */
#define SYS_CONF1_ADCPDN        (1UL << 16)     /* ADC 模拟模块使能，必须置1 */
#define SYS_CONF1_ADITA_10      (2U << 8)       /* ADITA<1:0>=10 正常模式(+50%电流)，必须置10 */
#define SYS_CONF1_CKMODE0       (1U << 4)       /* CKMODE<0>，必须置1 */

/* 官方推荐典型配置值（电桥传感器，DR<400Hz，无内部基准） */
#define CS5552_SYS_CONF1_TYPICAL    0x00010210UL//0x00110210
#define CS5552_SYS_CONF2_TYPICAL    0x00020000UL//0x00020000 

/* --- 转换命令模式 --- */
#define CMD_MOD_SINGLE_CONV  0x00
#define CMD_MOD_CONT_CONV    0x01
#define CMD_MOD_OFFSET_CAL   0x02
#define CMD_MOD_GAIN_CAL     0x06

extern uint32_t raw;
extern bool cs5552_ready;
void CS5552_Delay_Init(void);
void CS5552_Delay_ms(uint32_t ms);
void CS5552_Delay_us(uint32_t us);

bool SPI_SendByte(uint8_t data);
uint8_t SPI_ReceiveByte(void);

bool CS5552_Init(void);
void CS5552_Reset_SPI(void);
bool CS5552_WriteReg(uint8_t reg_addr, uint32_t data);
bool CS5552_ReadReg(uint8_t reg_addr, uint32_t *out_data);
bool CS5552_ReadADC_Single(uint8_t conv_conf_idx, int32_t *out_data);
bool CS5552_Offset_SelfCalibration(uint8_t conv_conf_idx);

bool CS5552_StartContConv(uint8_t conv_conf_idx);
bool CS5552_StartSingleConv(uint8_t conv_conf_idx);
bool CS5552_ReadADC(int32_t *out_data);
bool CS5552_ReadContData(int32_t *out_data);
bool CS5552_SingleChannelRead(uint8_t conv_conf_idx, int32_t *out_data, uint8_t *out_channel);
bool CS5552_ParseConvData(uint32_t raw_data, uint8_t expected_channel, int32_t *out_data, uint8_t *out_channel);

float CS5552_ConvertToVoltage(int32_t raw_code, uint8_t pga_gain);

void LED_disp(uint8_t led);
void LED_proc(void);
void ADC_proc(void);

#endif