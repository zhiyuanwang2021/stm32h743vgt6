/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "user.h"
#include "filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static SlidingAvgFilter voltage_filter_ch0;
static SlidingAvgFilter voltage_filter_ch1;
static volatile float g_voltage_filtered_ch0 = 0.0f;
static volatile uint8_t g_voltage_filtered_valid_ch0 = 0u;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

#include <stdio.h>
struct __FILE
{
  int handle;
};
FILE __stdout;
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 50);
  return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool cs5552_ready = false;
uint32_t ch0_valid_sample_count = 0;
uint32_t ch0_valid_sample_tick = 0;
uint32_t ch0_invalid_sample_count = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_TIM15_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_SPI1_Init();
  MX_TIM14_Init();
  MX_TIM13_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  printf("System Boot\r\n");

  cs5552_ready = false;

  CS5552_SelectChip(CS5552_CHIP_0);
  if (CS5552_Init()) {
    printf("CS5552-0 Init OK\r\n");
    CS5552_StartContConv(CS5552_CONFIG_IDX_CH1);
    cs5552_ready = true;
  } else {
    printf("CS5552-0 Init Fail!\r\n");
  }

  CS5552_SelectChip(CS5552_CHIP_1);
  if (CS5552_Init()) {
    printf("CS5552-1 Init OK\r\n");
    CS5552_StartContConv(CS5552_CONFIG_IDX_CH0);
  } else {
    printf("CS5552-1 Init Fail!\r\n");
  }

  Filter_Init(&voltage_filter_ch0);
  Filter_Init(&voltage_filter_ch1);

  if (HAL_TIM_Base_Start_IT(&htim17) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t voltage_print_tick = 0;

    if (g_voltage_filtered_valid_ch0 && (HAL_GetTick() - voltage_print_tick >= 10u)) {
      voltage_print_tick = HAL_GetTick();
      printf("%.6f\r\n", g_voltage_filtered_ch0);
    }

    if (HAL_GetTick() - ch0_valid_sample_tick >= 1000u) {
      printf("rate is %lu invalid %lu-----------------------\r\n\r\n\r\n\r\n",
             ch0_valid_sample_count, ch0_invalid_sample_count);
      
      ch0_valid_sample_count = 0;
      ch0_invalid_sample_count = 0;
      ch0_valid_sample_tick = HAL_GetTick();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  else if (htim->Instance == TIM17)
  {
    uint32_t adc_raw;
    int32_t adc_code;

    if (!cs5552_ready) {
      return;
    }

    CS5552_SelectChip(CS5552_CHIP_0);
    if (CS5552_ReadReg(REG_CONV_DATA, &adc_raw)) {
      uint32_t top3 = (adc_raw >> 29) & 0x7u;
      if (CS5552_ParseConvData(adc_raw, 1u, &adc_code, NULL) &&
          !(top3 == 0x2u || top3 == 0x3u || top3 == 0x4u || top3 == 0x5u ||
            adc_raw == 0xFFFFFFFFu || adc_raw == 0xFFFFFFFEu || adc_raw == 0x0u)) {
        ch0_valid_sample_count++;
        {
          int32_t adc_24bit = adc_code >> 6;
          float voltage_mv = CS5552_ConvertToVoltage(adc_24bit, 64);
          float voltage_filtered = Filter_Update(&voltage_filter_ch0, voltage_mv);
          g_voltage_filtered_ch0 = voltage_filtered * 1000;
          g_voltage_filtered_valid_ch0 = 1u;
        }
      } else {
        ch0_invalid_sample_count++;
        // printf("CH0 data Parse Error!\r\n");
      }
    } else {
      // printf("CH0 Read Error!\r\n");
    }
  }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  HAL_MPU_Disable();
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
