/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "ch423.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/************************* 宏定义 *************************/
// CH423S I2C配置
#define CH423_I2C_ADDR  0x80    // CH423S 7位地址（标准库使用7位地址）
#define CH423_CMD_SYS   0x48    // 系统配置命令
#define CH423_CMD_OC_L  0x44    // 写OC口低8位命令
#define CH423_CMD_IO    0x60    // 写IO口命令
#define CH423_SYS_PARAM 0x00    // 系统配置参数：使能IO输出

// 共阳数码管段码表（0~9，段a~g对应IO0~IO6，低电平亮）
const uint8_t SEG_TABLE[10] = {
    0xC0, // 0
    0xF9, // 1
    0xA4, // 2
    0xB0, // 3
    0x99, // 4
    0x92, // 5
    0x82, // 6
    0xF8, // 7
    0x80, // 8
    0x90  // 9
};

/************************* 全局变量 *************************/
ADC_HandleTypeDef hadc;
TIM_HandleTypeDef htim6;
I2C_HandleTypeDef hi2c1;

uint16_t adc_value = 0;           // ADC原始值
float voltage = 0.0f;             // 电压值
uint8_t disp_buf[3] = {0,0,0};    // 显示缓存
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
//static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/************************* CH423S初始化 *************************/
//void CH423_Init(void)
//{
//  uint8_t sys_cmd[2] = {CH423_CMD_SYS, CH423_SYS_PARAM};
//  HAL_StatusTypeDef rs = HAL_I2C_Master_Transmit(&hi2c1, CH423_I2C_ADDR, sys_cmd, 2, 100);
//  if (rs != HAL_OK){
//	  return;
//  }
//  HAL_Delay(1);
//}

//
///* 向CH423S写入命令和数据 */
//void CH423_Write(uint8_t cmd, uint8_t data)
//{
//  uint8_t tx_data[2] = {cmd, data};
//  HAL_I2C_Master_Transmit(&hi2c1, CH423_I2C_ADDR, tx_data, 2, 100);
//}

/************************* 数码管显示 *************************/
void Disp_Voltage(void)
{
  // 显示百位
  if (HAL_I2C_Master_Transmit(&hi2c1, CH423_I2C_ADDR, (uint8_t[]){CH423_CMD_OC_L, 0x01}, 2, 100) != HAL_OK){
	  return;
  }
  if (HAL_I2C_Master_Transmit(&hi2c1, CH423_I2C_ADDR, (uint8_t[]){CH423_CMD_IO, SEG_TABLE[disp_buf[0]]}, 2, 100) != HAL_OK){
	  return;
  }
  HAL_Delay(1);

  // 显示十位
  HAL_I2C_Master_Transmit(&hi2c1, CH423_I2C_ADDR, (uint8_t[]){CH423_CMD_OC_L, 0x02}, 2, 100);
  HAL_I2C_Master_Transmit(&hi2c1, CH423_I2C_ADDR, (uint8_t[]){CH423_CMD_IO, SEG_TABLE[disp_buf[1]]}, 2, 100);
  HAL_Delay(1);

  // 显示个位（带小数点）
  HAL_I2C_Master_Transmit(&hi2c1, CH423_I2C_ADDR, (uint8_t[]){CH423_CMD_OC_L, 0x04}, 2, 100);
  HAL_I2C_Master_Transmit(&hi2c1, CH423_I2C_ADDR, (uint8_t[]){CH423_CMD_IO, SEG_TABLE[disp_buf[2]] & 0x7F}, 2, 100);
  HAL_Delay(1);

  // 关闭位选
  HAL_I2C_Master_Transmit(&hi2c1, CH423_I2C_ADDR, (uint8_t[]){CH423_CMD_OC_L, 0x00}, 2, 100);
}


/************************* TIM6中断服务函数 *************************/
void TIM6_DAC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim6);

  // 采集ADC
  HAL_ADC_Start(&hadc);
  if (HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK)
  {
    adc_value = HAL_ADC_GetValue(&hadc);
  }
  HAL_ADC_Stop(&hadc);

  // 计算电压
  voltage = (adc_value * 3.3f) / 4096.0f;
  uint16_t volt_int = (uint16_t)(voltage * 100);

  // 分解显示
  disp_buf[0] = volt_int / 100;
  disp_buf[1] = (volt_int % 100) / 10;
  disp_buf[2] = volt_int % 10;

  // 刷新显示
  Disp_Voltage();
}


int main(void)
{

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC_Init();
  //MX_I2C1_Init();
  MX_TIM6_Init();

  //CH423_Init();
  CH423_WriteByte(CH423_SYS_CMD);
  CH423_WriteByte(CH423_SET_IO_CMD|BIT_IO0_DAT); //Set GPIO_0 HIGH
  CH423_WriteByte(CH423_SET_IO_CMD|BIT_IO1_DAT); //Set GPIO_0 HIGH
  CH423_WriteByte(CH423_OC_L_CMD|BIT_OC0_L_DAT);//Set GPO_0 HIGH
  CH423_WriteByte(CH423_OC_L_CMD|BIT_OC1_L_DAT);//Set GPO_8 HIGH

  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
	 Error_Handler();
  }

  while (1){}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_I2C1_Init(void)
//{
//
//  hi2c1.Instance = I2C1;
//  hi2c1.Init.Timing = 0x00201D2B;
//  hi2c1.Init.OwnAddress1 = 0;
//  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//  hi2c1.Init.OwnAddress2 = 0;
//  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
//  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure Analogue filter
//  */
//  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure Digital filter
//  */
//  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN I2C1_Init 2 */
//
//  /* USER CODE END I2C1_Init 2 */
//
//}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */
	__HAL_RCC_TIM6_CLK_ENABLE();
  /* USER CODE END TIM6_Init 0 */

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	// I2C1引脚（PB6=SCL, PB7=SDA）：复用开漏上拉
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// PA0（ADC输入）：模拟模式
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
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
