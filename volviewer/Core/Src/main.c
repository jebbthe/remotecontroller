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

#define TEST_LED
#define MODE_STATIC 0
#define MODE_DYNAMIC 1
#define TEST_MODE MODE_STATIC

ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim6;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM6_Init(void);

// 共阳数码管段码表（0~9，段a~g对应IO0~IO6，低电平亮）
const uint8_t SEG_TABLE_S[10] = {
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

// 共阴数码管段码表（0~9，段a~g对应IO0~IO6，高电平亮）
const uint8_t SEG_TABLE_D[10] = {
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


uint16_t adc_value = 0;           // ADC原始值
float voltage = 0.0f;             // 电压值
uint8_t disp_buf[3] = {0,0,0};    // 显示缓存

/************************* 数码管显示 *************************/
void Disp_Voltage(void)
{
	CH423_WriteByte(CH423_SET_IO0_CMD | (SEG_TABLE_D[disp_buf[0]] & 0x7F));
	CH423_WriteByte(CH423_SET_IO1_CMD | SEG_TABLE_D[disp_buf[1]]);
	CH423_WriteByte(CH423_SET_IO2_CMD | SEG_TABLE_D[disp_buf[2]]);
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

static void testStaticLED(){
	//共阳级静态驱动，无法满足诉求
	CH423_WriteByte(CH423_SYS_CMD | BIT_IO_OE);
	HAL_Delay(5);
	//初始化
	CH423_WriteByte(CH423_SET_IO_CMD | (0x00));
	CH423_WriteByte(CH423_SET_IO_CMD | (0x00));
	CH423_WriteByte(CH423_SET_IO_CMD | (0x00));
	CH423_WriteByte(CH423_OC_L_CMD | BIT_OC2_L_DAT | BIT_OC1_L_DAT | BIT_OC0_L_DAT);
	HAL_Delay(5);
	//测试代码
	while(1){
		CH423_WriteByte(CH423_OC_L_CMD | BIT_OC0_L_DAT);
		CH423_WriteByte(CH423_SET_IO_CMD | (SEG_TABLE_S[1] & 0x7F));
		CH423_WriteByte(CH423_OC_L_CMD | BIT_OC1_L_DAT);
		CH423_WriteByte(CH423_SET_IO_CMD | (SEG_TABLE_S[2]));
		CH423_WriteByte(CH423_OC_L_CMD | BIT_OC2_L_DAT);
		CH423_WriteByte(CH423_SET_IO_CMD | (SEG_TABLE_S[3]));
	}
}

static void testDynamicLED(){
	//共阳级静态驱动，无法满足诉求
	CH423_WriteByte(CH423_SYS_CMD | BIT_IO_OE | BIT_DEC_L);
	HAL_Delay(5);

	while(1){
		CH423_WriteByte(CH423_SET_IO0_CMD | (SEG_TABLE_D[1] & 0x7F));
		CH423_WriteByte(CH423_SET_IO1_CMD | (SEG_TABLE_D[2]));
		CH423_WriteByte(CH423_SET_IO2_CMD | (SEG_TABLE_D[3]));
	}
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC_Init();

#ifdef TEST_LED
	#if TEST_MODE == 0
	  testStaticLED();
	#elif
	  testDynamicLED();
	#endif
#endif

  MX_TIM6_Init();
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
	  Error_Handler();
  }

  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */
	__HAL_RCC_TIM6_CLK_ENABLE();
  /* USER CODE END TIM6_Init 0 */

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  //初始化LED引脚
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
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
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
