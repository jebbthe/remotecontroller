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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// LED引脚定义
#define LED_POWER_PIN      GPIO_PIN_0
#define LED_CHIP_COMM_PIN  GPIO_PIN_1
#define LED_FC_COMM_PIN    GPIO_PIN_2

// RTC6705 SPI引脚定义
#define SPI_CS_PIN        GPIO_PIN_4   // PA4 - 片选/锁存使能
#define SPI_CLK_PIN        GPIO_PIN_5  // PA5 - 时钟
#define SPI_DATA_PIN       GPIO_PIN_7  // PA7 - 数据

// RTC6705寄存器地址
#define RTC6705_REG_SYNTH_A    0x00
#define RTC6705_REG_SYNTH_B    0x01
#define RTC6705_REG_VCO_DFC_CTL 0x03
#define RTC6705_REG_PA         0x07
#define RTC6705_REG_STATE      0x0F
#define RTC6705_REG_VCO_5G     0x04  // 5G VCO控制寄存器
#define RTC6705_REG_VCO_2G     0x05  // 2G VCO控制寄存器

// PA寄存器位定义
#define RTC6705_PA_PD_Q5G_BIT  0x40  // 位6: 5G预驱动掉电控制 (1=掉电, 0=使能)

// RTC6705状态寄存器值
#define RTC6705_STATE_RESET    0x00  // 复位状态
#define RTC6705_STATE_PWRON_CAL 0x01 // 上电校准状态
#define RTC6705_STATE_STBY     0x02  // 待机状态
#define RTC6705_STATE_VCO_CAL  0x03  // VCO校准状态

// RTC6705晶振频率 (8MHz)
#define RTC6705_XTAL_FREQ_KHZ  8000

// 合成器寄存器A默认值（参考OpenVTx）
#define RTC6705_SYNTH_REG_A_DEFAULT  0x00190  // 包含PLL使能位

#define RTC6705_SYNTH_REG_B_DEFAULT  0x04781  // 对应N=2291, A=1，默认频率5865MHz

#define SPI_DATA_BITS 25  // 25位通信格式：4位地址+1位写控制+20位数据

#define SPI_WRITE_CTRL 1  // 写控制位：0=写，1=读

#define RTC6705_POWER_AMP_ON_OPENVTX   0x9F7E0
#define RTC6705_POWER_AMP_ON_DEFAULT  0x04FBD
#define RTC6705_POWER_AMP_ON   RTC6705_POWER_AMP_ON_DEFAULT


// PLL锁定等待时间（毫秒）
#define RTC6705_PLL_SETTLE_TIME_MS  500

// 当前频率 (MHz)
static uint16_t current_frequency_mhz = 5809;

// PLL锁定后开启PA的定时器（参考OpenVTx）
static uint32_t powerUpAfterSettleTime = 0;

// UART接收缓冲区
#define UART_RX_BUFFER_SIZE 32
static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static uint8_t uart_rx_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
// RTC6705函数
static void RTC6705_SPI_Init(void);
static void RTC6705_SPI_Write(uint8_t reg_addr, uint32_t data);
static void RTC6705_ResetState(void);
static void RTC6705_ResetSynthRegA(void);
static void RTC6705_PowerAmpOn(void);
static void RTC6705_PowerAmpOff(void);
static void RTC6705_SetFrequency(uint16_t frequency_mhz);
static void RTC6705_PowerUpAfterPLLSettleTime(void);
static void RTC6705_Init(void);

// LED控制函数
static void LED_Power_On(void);
static void LED_Power_Off(void);
static void LED_ChipComm_On(void);
static void LED_ChipComm_Off(void);
static void LED_FCComm_On(void);
static void LED_FCComm_Off(void);

// IRC Tramp协议函数
static void IRCTramp_ProcessCommand(uint8_t *buffer, uint8_t length);
static uint16_t IRCTramp_GetFrequencyFromChannel(uint8_t band, uint8_t channel);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // 初始化LED
  LED_Power_On();
  HAL_Delay(100);
  
  // 初始化RTC6705 SPI接口
  RTC6705_SPI_Init();
  HAL_Delay(10);
  
  // 初始化RTC6705芯片
  LED_ChipComm_On();
  RTC6705_Init();
  HAL_Delay(50);
  LED_ChipComm_Off();
  
  // 启动UART接收中断
  HAL_UART_Receive_IT(&huart1, uart_rx_buffer, 1);
  
  LED_FCComm_On();
  HAL_Delay(100);
  LED_FCComm_Off();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 主循环 - 保持电源LED常亮
    LED_Power_On();
    
    // 检查PLL锁定时间，如果到了就开启PA（参考OpenVTx）
    RTC6705_PowerUpAfterPLLSettleTime();
    
    HAL_Delay(10);  // 减少延时，提高响应速度
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA4
                           PA5 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/**
  * @brief  初始化RTC6705软件SPI接口
  * @retval None
  * @note   重要：RTC6705引脚1(SPI_SE)必须设置为高电平才能使用SPI模式
  *         如果SPI_SE为低电平，芯片会使用引脚模式（通过CS0/CS1/CS2选择通道），SPI通信将无效
  *         请检查硬件连接，确保SPI_SE引脚连接到高电平（3.3V）或通过GPIO控制
  *         如果VT电压始终为0V，很可能是SPI_SE引脚没有正确设置为高电平
  */
static void RTC6705_SPI_Init(void)
{
  // 初始化SPI引脚状态
  HAL_GPIO_WritePin(GPIOA, SPI_CS_PIN, GPIO_PIN_SET);    // CS/LE置高(空闲状态)
  HAL_GPIO_WritePin(GPIOA, SPI_CLK_PIN, GPIO_PIN_RESET); // CLK置低
  HAL_GPIO_WritePin(GPIOA, SPI_DATA_PIN, GPIO_PIN_RESET); // DATA置低
  
  // 注意：如果SPI_SE引脚由MCU控制，需要在这里设置为高电平
  // 例如：HAL_GPIO_WritePin(GPIOA, SPI_SE_PIN, GPIO_PIN_SET);
  // 但当前代码假设SPI_SE由硬件上拉或外部电路控制
}

static void RTC6705_SPI_Write(uint8_t reg_addr, uint32_t data)
{
  uint32_t spi_word = 0;
  uint8_t i;

  spi_word |= ((uint32_t) (reg_addr & 0x0F));
  spi_word |= ((uint32_t) SPI_WRITE_CTRL << 4);
  spi_word |= ((uint32_t)(data & 0x000FFFFF) << 5);

  HAL_GPIO_WritePin(GPIOA, SPI_CS_PIN, GPIO_PIN_RESET);
  HAL_Delay(1);

  // LSB先发送（核心修正）
  for (i = 0; i < 25; i++)
  {
	HAL_GPIO_WritePin(GPIOA, SPI_CLK_PIN, GPIO_PIN_RESET);
	HAL_Delay(1);

    HAL_GPIO_WritePin(GPIOA, SPI_DATA_PIN, (spi_word & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_Delay(1);

    HAL_GPIO_WritePin(GPIOA, SPI_CLK_PIN, GPIO_PIN_SET);
    HAL_Delay(1);

    spi_word >>= 1;  // 右移，准备下一位（LSB优先）
  }

  HAL_GPIO_WritePin(GPIOA, SPI_CS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, SPI_CLK_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, SPI_DATA_PIN, GPIO_PIN_RESET);
  HAL_Delay(1);
}


/**
  * @brief  复位合成器寄存器A（参考OpenVTx实现）
  * @retval None
  * @note   将合成器寄存器A设置为默认值，包含PLL使能位
  */
static void RTC6705_ResetSynthRegA(void)
{
  // 写入合成器寄存器A，使用默认值（包含PLL使能位）
  RTC6705_SPI_Write(RTC6705_REG_SYNTH_A, RTC6705_SYNTH_REG_A_DEFAULT);
  HAL_Delay(10);
}

/**
  * @brief  关闭RTC6705功放和预驱动（参考OpenVTx实现）
  * @retval None
  * @note   在频率切换时调用，避免在非目标频段发射信号
  */
static void RTC6705_PowerAmpOff(void)
{
  // 写入PA寄存器，数据为0（所有位清零，包括PD_Q5G=1，预驱动掉电）
  RTC6705_SPI_Write(RTC6705_REG_PA, 0x00000000);
  HAL_Delay(10);
}

/**
  * @brief  使能RTC6705功放和预驱动，配置为最大功率模式（参考OpenVTx实现）
  * @retval None
  * @note   在频率锁定后调用，开启预驱动和PA输出
  *         配置为最大功率模式：PA5G_PW=11, PA5G_BS=111, PD_Q5G=0
  *         输出功率：+13dBm（最大功率）
  */
static void RTC6705_PowerAmpOn(void)
{
  RTC6705_PowerAmpOff();  // 先掉电复位
  HAL_Delay(5);
  RTC6705_SPI_Write(RTC6705_REG_PA, RTC6705_POWER_AMP_ON);
  HAL_Delay(10);
}

/**
  * @brief  重置RTC6705状态寄存器
  * @retval None
  * @note   向状态寄存器写入复位值，使芯片进入初始状态
  */
static void RTC6705_ResetState(void) {
  // 向状态寄存器写入复位值（0x00）
  // 注意：RTC6705的SPI通信为25位格式（4位地址+1位写控制+20位数据）
  RTC6705_SPI_Write(RTC6705_REG_STATE, RTC6705_STATE_RESET);

  // 等待复位完成（根据芯片 datasheet 调整延迟时间）
  HAL_Delay(5);
}

static void RTC6705_SetFrequency(uint16_t frequency_mhz)
{
  uint32_t synth_reg_a, synth_reg_b;
  uint32_t N, A;
  
  frequency_mhz = (frequency_mhz < 5645) ? 5645 : (frequency_mhz > 5945) ? 5945 : frequency_mhz;
  uint32_t freq = frequency_mhz * 1000U;
  freq /= 40;
  
  N = freq / 64;
  A = freq % 64;
  
  // 配置寄存器B（先写B）：R=400 + N高位 + A
  synth_reg_b = 0;
  synth_reg_b = A | (N << 7);

  RTC6705_SPI_Write(RTC6705_REG_SYNTH_B, synth_reg_b);
  //RTC6705_SPI_Write(RTC6705_REG_SYNTH_B, 0x47981);
  HAL_Delay(10);

  RTC6705_SPI_Write(RTC6705_REG_VCO_DFC_CTL, 0x0FFD7);

  // 更新频率和定时器
  current_frequency_mhz = frequency_mhz;
  powerUpAfterSettleTime = HAL_GetTick() + RTC6705_PLL_SETTLE_TIME_MS;
  
  LED_ChipComm_On();
  HAL_Delay(50);
  LED_ChipComm_Off();
}


/**
  * @brief  PLL锁定后开启PA（参考OpenVTx实现）
  * @retval None
  * @note   在主循环中调用，检查PLL锁定时间后开启PA
  */
static void RTC6705_PowerUpAfterPLLSettleTime(void)
{
  if (powerUpAfterSettleTime == 0)
    return;  // 定时器未设置，直接返回
  
  if (HAL_GetTick() >= powerUpAfterSettleTime)
  {
    // PLL锁定时间已到，开启PA
    RTC6705_PowerAmpOn();
    powerUpAfterSettleTime = 0;  // 清除定时器
  }
}

static uint8_t RTC6705_ReadState(void) {
    // 实现SPI读寄存器（原代码仅写，需补充读逻辑）
    uint32_t state_data = 0;
    // 读寄存器流程：4位地址+1位读控制（1）+20位数据（LSB先读）
    // 补充SPI读函数后，读取0x0F寄存器，返回STATE[2:0]位
    return (state_data >> 0) & 0x07;  // 假设STATE位是低3位
}

/**
  * @brief  初始化RTC6705芯片（参考OpenVTx实现优化）
  * @retval None
  * @note   初始化顺序: 等待稳定 -> 复位状态 -> 配置VCO -> 写入频率 -> PLL锁定后开启PA
  *         重要提示：
  *         1. RTC6705引脚1(SPI_SE)必须设置为高电平才能使用SPI模式
  *         2. 如果SPI_SE为低电平，芯片会使用引脚模式（通过CS0/CS1/CS2选择通道）
  *         3. 采用OpenVTx的初始化流程，更可靠
  *         4. PA开启由定时器机制控制，在主循环中检查
  */
static void RTC6705_Init(void)
{
	// 等待芯片上电稳定（给足够时间让芯片完成内部初始化）
	HAL_Delay(2000);

	while (RTC6705_ReadState() == RTC6705_STATE_PWRON_CAL) {
	  HAL_Delay(10);
	}

	//double light
	LED_ChipComm_On();
	HAL_Delay(50);
	LED_ChipComm_Off();
	LED_ChipComm_On();
	HAL_Delay(50);
	LED_ChipComm_Off();

	RTC6705_ResetState();
	HAL_Delay(2000);

	//double light
	LED_ChipComm_On();
	HAL_Delay(50);
	LED_ChipComm_Off();
	LED_ChipComm_On();
	HAL_Delay(50);
	LED_ChipComm_Off();

	// 初始化合成器寄存器B（默认值），确保R分频器正确
	//RTC6705_SPI_Write(RTC6705_REG_SYNTH_B, RTC6705_SYNTH_REG_B_DEFAULT);
//	HAL_Delay(2000);
//	LED_ChipComm_On();
//	HAL_Delay(50);
//	LED_ChipComm_Off();

	//设置频率
	RTC6705_SetFrequency(5865);
	HAL_Delay(2000);
	LED_ChipComm_On();
	HAL_Delay(50);
	LED_ChipComm_Off();

}

/**
  * @brief  打开电源LED
  * @retval None
  */
static void LED_Power_On(void)
{
  HAL_GPIO_WritePin(GPIOA, LED_POWER_PIN, GPIO_PIN_SET);
}

/**
  * @brief  关闭电源LED
  * @retval None
  */
static void LED_Power_Off(void)
{
  HAL_GPIO_WritePin(GPIOA, LED_POWER_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  打开芯片通信LED
  * @retval None
  */
static void LED_ChipComm_On(void)
{
  HAL_GPIO_WritePin(GPIOA, LED_CHIP_COMM_PIN, GPIO_PIN_SET);
}

/**
  * @brief  关闭芯片通信LED
  * @retval None
  */
static void LED_ChipComm_Off(void)
{
  HAL_GPIO_WritePin(GPIOA, LED_CHIP_COMM_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  打开飞控通信LED
  * @retval None
  */
static void LED_FCComm_On(void)
{
  HAL_GPIO_WritePin(GPIOA, LED_FC_COMM_PIN, GPIO_PIN_SET);
}

/**
  * @brief  关闭飞控通信LED
  * @retval None
  */
static void LED_FCComm_Off(void)
{
  HAL_GPIO_WritePin(GPIOA, LED_FC_COMM_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  根据IRC Tramp波段和通道获取频率
  * @param  band: 波段索引 (0-7)
  * @param  channel: 通道索引 (0-7)
  * @retval 频率值 (MHz)
  */
static uint16_t IRCTramp_GetFrequencyFromChannel(uint8_t band, uint8_t channel)
{
  // IRC Tramp频率表 (5.8GHz频段)
  // 波段0: Raceband (5658-5917 MHz, 8通道)
  // 波段1: Band A (5865-5905 MHz, 8通道)
  // 波段2: Band B (5733-5769 MHz, 8通道)
  // 波段3: Band E (5705-5843 MHz, 8通道)
  // 波段4: Band F (5740-5925 MHz, 8通道)
  // 波段5: Band D (5705-5885 MHz, 8通道)
  // 波段6: Band C (5658-5925 MHz, 8通道)
  // 波段7: 自定义/用户定义
  
  const uint16_t band_base[8] = {5658, 5865, 5733, 5705, 5740, 5705, 5658, 5809};
  const uint16_t band_step[8] = {37, 5, 5, 5, 5, 5, 37, 7};
  
  if (band >= 8) band = 0;
  if (channel >= 8) channel = 0;
  
  return band_base[band] + (channel * band_step[band]);
}

/**
  * @brief  处理IRC Tramp协议命令
  * @param  buffer: 命令缓冲区
  * @param  length: 命令长度
  * @retval None
  */
static void IRCTramp_ProcessCommand(uint8_t *buffer, uint8_t length)
{
  if (length < 1) return;
  
  // IRC Tramp协议命令格式: <命令><数据>
  // 'F' + 频率(2字节,小端序) - 直接设置频率(MHz)
  // 'C' + 波段(1字节) + 通道(1字节) - 设置通道
  // 'P' + 功率(1字节) - 设置功率等级
  // 'S' - 查询状态
  
  switch (buffer[0])
  {
    case 'F':  // 直接设置频率
      if (length >= 3)
      {
        uint16_t freq = buffer[1] | (buffer[2] << 8);
        if (freq >= 5645 && freq <= 5945)
        {
          RTC6705_SetFrequency(freq);
          LED_FCComm_On();
          HAL_Delay(20);
          LED_FCComm_Off();
        }
      }
      break;
      
    case 'C':  // 设置通道
      if (length >= 3)
      {
        uint8_t band = buffer[1];
        uint8_t channel = buffer[2];
        uint16_t freq = IRCTramp_GetFrequencyFromChannel(band, channel);
        RTC6705_SetFrequency(freq);
        LED_FCComm_On();
        HAL_Delay(20);
        LED_FCComm_Off();
      }
      break;
      
    case 'P':  // 设置功率等级
      if (length >= 2)
      {
        uint8_t power = buffer[1];
        // 先关闭PA
        RTC6705_PowerAmpOff();
        HAL_Delay(10);
        
        // 根据功率等级设置（简化版，实际可以更精细控制）
        if (power == 0) 
        {
          // 低功率：可以调整PA寄存器相关位
          RTC6705_SPI_Write(RTC6705_REG_PA, 0x00000F3F);
        }
        else 
        {
          // 中高功率：使用最大功率
          RTC6705_PowerAmpOn();
        }
        
        LED_FCComm_On();
        HAL_Delay(20);
        LED_FCComm_Off();
      }
      break;
      
    case 'S':  // 查询状态
      {
        uint8_t response[3];
        response[0] = 'F';
        response[1] = current_frequency_mhz & 0xFF;
        response[2] = (current_frequency_mhz >> 8) & 0xFF;
        HAL_UART_Transmit(&huart1, response, 3, 100);
        LED_FCComm_On();
        HAL_Delay(20);
        LED_FCComm_Off();
      }
      break;
      
    default:
      break;
  }
}

/* USER CODE END 4 */

/**
  * @brief  UART接收完成回调函数
  * @param  huart: UART句柄
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uint8_t received_byte = uart_rx_buffer[uart_rx_index];
    
    // 检查是否接收到完整命令
    // IRC Tramp命令通常以换行符结束或固定长度
    if (received_byte == '\n' || received_byte == '\r')
    {
      // 处理完整命令
      if (uart_rx_index > 0)
      {
        IRCTramp_ProcessCommand(uart_rx_buffer, uart_rx_index);
      }
      uart_rx_index = 0;
    }
    else if (uart_rx_index < (UART_RX_BUFFER_SIZE - 1))
    {
      uart_rx_index++;
    }
    else
    {
      // 缓冲区溢出，重置
      uart_rx_index = 0;
    }
    
    // 继续接收
    HAL_UART_Receive_IT(&huart1, &uart_rx_buffer[uart_rx_index], 1);
  }
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
