/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RS485.h"
#include "DAC8831.h"
#include "ETHw5500.h"
#include "Servo_Driver.h"
#include "myFifo.h"
#include "Encoder.h"
#include "gParameter.h"
#include "Eeprom_manage.h"
#include "mram_manage.h"
#include "td.h"
#include "pid.h"
#include <elog.h>
#include "MRAM.h"
#include "sensor.h"
#include "adrc.h"
#include "modbus_slave.h"
#include "filter.h"

// #include "delay.h"   // Retired software-SPI delay helper from the old ADC path.
// #include "io_spi.h"  // Retired software-SPI GPIO helper from the old ADC path.
#include "CS5552.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIRMWAREVERSION "1.16.7"
#define TIM6_PRINT_1S_COUNT 300U
#define TIM6_SEM_RELEASE_10MS_COUNT 10U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t test_check = 0;
static SlidingAvgFilter voltage_filter_ch0;
volatile float g_voltage_filtered_ch0 = 0.0f;
volatile uint8_t g_voltage_filtered_valid_ch0 = 0u;
volatile uint32_t ch0_valid_sample_count = 0u;
volatile uint32_t ch0_invalid_sample_count = 0u;
static uint8_t tim6SemReleaseCounter = 0u;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
extern void DIFuncMap_init(void);
extern osSemaphoreId cs5552SemHandle;
extern volatile uint32_t tim6SignalCounter;
extern volatile uint8_t tim6PrintFlag;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Fun_111 */
void hardwareInit(void){
  //HAL_TIM_PWM_Start_IT(&htim8,TIM_CHANNEL_1);
  //SPI2 CS init
  DAC8831_CS_HIGH();
  MRAM_CS_HIGH();
  //Servo init
  modFclkNumberPULSEInit(1000);
	Servo_PWM_Disable(PULSE);
#if (ServoPr007 == 0 || ServoPr007 == 1 || ServoPr007 == 2)
	Servo_PWM_Enable(DIR);
#else
	Servo_DIR_GPIO_Init();
#endif
  //DAC init
  DAC8831_Write(0);
  //485 init
  modbusInit();
  RS485_init(&huart2);
  // ADC init
  CS5552_CompatInit(&cs5552_compat_ctx);
  Filter_Init(&voltage_filter_ch0);
  //Eth fifo init
	fifo_init(&rev_fifo);
  //Encoder init
	Encoder_Init();
}

/* Fun_112 */
void softwareInit(void){
  //sensor calibrate function init
  sensorCalibrateFuncInit();
  //Work State and Mode init
  workStateDefaultInit(&ws);
  workModeDefaultInit(&wm);
  //para init
  parameter_default_init();
	sendata_default_init();
  storageParaDefaultInit();
  mramInit();
  servoParamCalcu(&servoParam);
  //Eth init
	ETHw5500_init();
  //sensor read when system init
  eepromSensorReadInit(&sensorCheck);
  encoderCountInit(pose.code,0,0,&encoder);
  //while(1);
	//sync application layer sensor data 
  senDateSync(SenData,&AL);
  //senor zero code init
  sensorZeroCodeInit();
  //init td filter
  create_TD(&load_td,10000);
  //pos init
	pidInitPos();
  pidInitLoad();
  pidInitExt();
  //di fucntion map
  DIFuncMap_init();
  //update the adrc variable
  poseCalcu();
  //update the pose once
  adrcInit(&leso,pose.orig);
  //manualBox init
  manualBoxInit();
  //update work state initState
  ws.init = initScomplete;
  //timer counter init
	HAL_TIM_Base_Start_IT(&htim6); 
  HAL_TIM_Base_Start_IT(&htim16);
}

/* Fun_113 */
void firmwareVersionShown(void){
  printf("**********  Firmware Version:%s  **********\r\n",FIRMWAREVERSION);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	// uint8_t err=0;
	// uint8_t send_buf_MRAM[10]={7,1,0x0A,8,6,0,8,1,0x0F,9};
	// uint8_t recv_buf_MRAM[10];
	// uint8_t i=0;
  // uint32_t counterUs = 0;
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
  hardwareInit();
  softwareInit();
  HAL_TIM_Base_Start(&htim13);
  //firmwareVersionShown();

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // delay_us(100000);
    // printf("delay_us!\r\n");
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
}
 uint16_t tim6us=0;

uint16_t pwmFinishcounter = 0;
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM8)
	{
    ++hal_output.pwmIntCounter;
    ++test.nr;
    // if(hal_output.pwmIntCounter >= hal_output.pwmCounter){
    //   hal_output.pwmIntCounter = 0;
    // }

  //   if(hal_output.pwmIntCounter >= hal_output.pwmCounter){
  //     HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
  //  }
	}
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{

  /* Disables the MPU */
  HAL_MPU_Disable();

  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if(htim->Instance==htim6.Instance)
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

    // if (++tim6SemReleaseCounter >= TIM6_SEM_RELEASE_10MS_COUNT)
    // {
    //   tim6SemReleaseCounter = 0u;
    //   if((cs5552SemHandle != NULL) && (osKernelRunning() == 1))
    //   {
    //       osSemaphoreRelease(cs5552SemHandle);
    //   }
    // }
	}
  //2kHz for encoder collecting
  if(htim->Instance == TIM16){
    Encoder_Get(Encoder0, &encoder);
    Encoder_Get(Encoder1, &encoder);
    Encoder_Get(Encoder2, &encoder);
    poseCodeCalculate_Int(&pose,encoder.count0);
    poseCodeCalculate_Int(&bigDeformationLower,encoder.count1);
    poseCodeCalculate_Int(&bigDeformationUpper,encoder.count2);
    poseSpeedFilter_Int(&filter_Int,&pose,&speedPose,10);
    AL.utcTime +=0.0005f; 
  }else if(htim->Instance == TIM17){
    FreeRTOSRunTimeTicks++;
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
