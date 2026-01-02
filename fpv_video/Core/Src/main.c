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
#define RTC6705_REG_PA         0x07
#define RTC6705_REG_STATE      0x0F

// PA寄存器位定义
#define RTC6705_PA_PD_Q5G_BIT  0x40  // 位6: 5G预驱动掉电控制 (1=掉电, 0=使能)

// RTC6705状态寄存器值
#define RTC6705_STATE_RESET    0x00  // 复位状态
#define RTC6705_STATE_PWRON_CAL 0x01 // 上电校准状态
#define RTC6705_STATE_STBY     0x02  // 待机状态
#define RTC6705_STATE_VCO_CAL  0x03  // VCO校准状态

// RTC6705晶振频率 (8MHz)
#define RTC6705_XTAL_FREQ_KHZ  8000

// 当前频率 (MHz)
static uint16_t current_frequency_mhz = 5809;

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
static void RTC6705_SetFrequency(uint16_t frequency_mhz);
static void RTC6705_EnablePA(void);
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
    HAL_Delay(100);
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
  * @note   RTC6705引脚1(SPI_SE)必须设置为高电平以启用SPI模式，通常由硬件或独立GPIO控制
  */
static void RTC6705_SPI_Init(void)
{
  HAL_GPIO_WritePin(GPIOA, SPI_CS_PIN, GPIO_PIN_SET);    // CS置高(空闲状态)
  HAL_GPIO_WritePin(GPIOA, SPI_CLK_PIN, GPIO_PIN_RESET); // CLK置低
  HAL_GPIO_WritePin(GPIOA, SPI_DATA_PIN, GPIO_PIN_RESET); // DATA置低
}

/**
  * @brief  通过SPI向RTC6705寄存器写入数据
  * @param  reg_addr: 寄存器地址 (0x00-0x0F)
  * @param  data: 20位数据
  * @retval None
  */
static void RTC6705_SPI_Write(uint8_t reg_addr, uint32_t data)
{
  uint32_t spi_word;
  uint8_t i;
  
  // 数据格式: [3位地址][20位数据]
  spi_word = ((reg_addr & 0x07) << 20) | (data & 0x000FFFFF);
  
  // CS拉低开始传输
  HAL_GPIO_WritePin(GPIOA, SPI_CS_PIN, GPIO_PIN_RESET);
  HAL_Delay(1);
  
  // 发送24位数据 (3位地址 + 20位数据 + 1位填充)
  for (i = 0; i < 24; i++)
  {
    // 设置数据位
    if (spi_word & 0x00800000)
    {
      HAL_GPIO_WritePin(GPIOA, SPI_DATA_PIN, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(GPIOA, SPI_DATA_PIN, GPIO_PIN_RESET);
    }
    
    // 数据建立时间延时
    for (volatile uint8_t d = 0; d < 2; d++);
    
    // 时钟上升沿(数据在上升沿采样)
    HAL_GPIO_WritePin(GPIOA, SPI_CLK_PIN, GPIO_PIN_SET);
    for (volatile uint8_t d = 0; d < 2; d++);
    
    // 时钟下降沿
    HAL_GPIO_WritePin(GPIOA, SPI_CLK_PIN, GPIO_PIN_RESET);
    for (volatile uint8_t d = 0; d < 2; d++);
    
    // 数据左移
    spi_word <<= 1;
  }
  
  // CS拉高锁存数据
  HAL_GPIO_WritePin(GPIOA, SPI_CS_PIN, GPIO_PIN_SET);
  HAL_Delay(1);
}

/**
  * @brief  设置RTC6705发射频率
  * @param  frequency_mhz: 频率值 (MHz, 例如: 5809)
  * @retval None
  */
static void RTC6705_SetFrequency(uint16_t frequency_mhz)
{
  uint32_t synth_reg_a, synth_reg_b;
  uint32_t n_div;
  uint32_t freq_khz;
  
  // RTC6705使用PLL锁相环，参考频率8MHz
  // 公式: RF = (N * REF) / R
  // 5.8GHz频段范围: 5645-5945 MHz
  
  // 限制频率在有效范围内
  if (frequency_mhz < 5645) frequency_mhz = 5645;
  if (frequency_mhz > 5945) frequency_mhz = 5945;
  
  // 计算N分频器值
  // 使用8MHz参考频率，R=1，N = (频率_MHz * 1000) / 8000
  freq_khz = (uint32_t)frequency_mhz * 1000;
  n_div = freq_khz / RTC6705_XTAL_FREQ_KHZ;
  
  // 配置合成器寄存器A (0x00)
  // 位[10:0]: N分频器低11位
  // 位[11]: PLL使能
  synth_reg_a = 0x00000000;
  synth_reg_a |= (n_div & 0x7FF) << 0;
  synth_reg_a |= 0x00000800;  // 使能PLL
  
  // 配置合成器寄存器B (0x01)
  // 位[8:0]: N分频器高9位
  // 位[13:9]: R分频器 (设置为1)
  synth_reg_b = 0x00000000;
  synth_reg_b |= ((n_div >> 11) & 0x1FF) << 0;
  synth_reg_b |= (1 & 0x1F) << 9;
  
  // 写入寄存器
  RTC6705_SPI_Write(RTC6705_REG_SYNTH_A, synth_reg_a);
  HAL_Delay(10);
  RTC6705_SPI_Write(RTC6705_REG_SYNTH_B, synth_reg_b);
  HAL_Delay(10);
  
  // 更新当前频率
  current_frequency_mhz = frequency_mhz;
  
  // 闪烁芯片通信LED指示频率变化
  LED_ChipComm_On();
  HAL_Delay(50);
  LED_ChipComm_Off();
}

/**
  * @brief  使能RTC6705功放和预驱动
  * @retval None
  * @note   在频率锁定后调用，开启预驱动和PA输出
  */
static void RTC6705_EnablePA(void)
{
  // 配置PA寄存器(0x07) - 使能功放和预驱动，输出功率+13dBm
  // PA5G_PW = 11(最大功率), PA5G_BS = 111(最大增益)
  // PD_Q5G = 0 (使能预驱动)
  RTC6705_SPI_Write(RTC6705_REG_PA, 0x00000F7F);
  HAL_Delay(10);
}

/**
  * @brief  初始化RTC6705芯片
  * @retval None
  * @note   初始化顺序: 唤醒芯片 -> 配置频率 -> 等待PLL锁定 -> 开启PA
  *         使用PD_Q5G位在频率锁定前关闭预驱动，避免在非目标频段乱发射
  */
static void RTC6705_Init(void)
{
  // 等待芯片上电稳定
  HAL_Delay(10);
  
  // 唤醒芯片: 从RESET状态(0x00)切换到PWRON_CAL状态(0x01)
  // 关键步骤: 芯片默认处于RESET状态，不唤醒无法发射
  RTC6705_SPI_Write(RTC6705_REG_STATE, RTC6705_STATE_PWRON_CAL);
  HAL_Delay(20);  // 等待芯片进入PWRON_CAL状态并完成校准
  
  // 配置PA寄存器，但保持预驱动掉电(PD_Q5G=1)
  // 这样在频率锁定前不会在非目标频段发射信号
  // PA5G_PW = 11(最大功率), PA5G_BS = 111(最大增益), PD_Q5G = 1(预驱动掉电)
  RTC6705_SPI_Write(RTC6705_REG_PA, 0x00000F7F | RTC6705_PA_PD_Q5G_BIT);
  HAL_Delay(10);
  
  // 先设置频率为5809MHz
  RTC6705_SetFrequency(5809);
  
  // 等待PLL锁定(约50-100ms)
  // 在频率稳定前保持预驱动关闭，避免乱发射
  HAL_Delay(80);
  
  // 频率锁定后，使能预驱动和PA
  RTC6705_EnablePA();
  HAL_Delay(10);
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
        uint32_t pa_reg = 0x00000F7F;  // 默认最大功率
        if (power == 0) pa_reg = 0x00000F3F;      // 低功率
        else if (power == 1) pa_reg = 0x00000F5F; // 中低功率
        else if (power == 2) pa_reg = 0x00000F6F; // 中功率
        // power == 3 使用最大功率(默认)
        RTC6705_SPI_Write(RTC6705_REG_PA, pa_reg);
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
