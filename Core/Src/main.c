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
#include "lcd.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LASER_PIN   GPIO_PIN_4
#define LASER_PORT  GPIOC

#define WIFI_SSID   "ESP_2F0F28"
#define WIFI_PASS   "thereisnospoon"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart3;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
uint32_t last_k2_press_tick = 0;
uint8_t  k2_has_been_pressed = 0;
GPIO_PinState last_k2_state  = GPIO_PIN_SET;

char esp_rx[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
uint32_t read_adc1(void);
uint32_t read_adc2(void);
void laser_fire_1s(void);
void ESP_Send(char *cmd);
void ESP_ReadResponse(uint32_t timeout);
void ESP_ConnectWiFi(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t read_adc1(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    uint32_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return val;
}

uint32_t read_adc2(void)
{
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 100);
    uint32_t val = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);
    return val;
}

void laser_fire_1s(void)
{
    HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
}

void ESP_Send(char *cmd)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)cmd, strlen(cmd), 1000);
}

void ESP_ReadResponse(uint32_t timeout)
{
    memset(esp_rx, 0, sizeof(esp_rx));
    HAL_UART_Receive(&huart3, (uint8_t *)esp_rx, sizeof(esp_rx) - 1, timeout);
}

void ESP_ConnectWiFi(void)
{
    char cmd[96];

    LCD_DrawString(10, 60, "WiFi: testing AT... ");
    ESP_Send("AT\r\n");
    ESP_ReadResponse(2000);
    if (strstr(esp_rx, "OK"))
        LCD_DrawString(10, 60, "WiFi: AT OK         ");
    else
        LCD_DrawString(10, 60, "WiFi: no response   ");
    HAL_Delay(1000);

    LCD_DrawString(10, 60, "WiFi: set STA mode  ");
    ESP_Send("AT+CWMODE=1\r\n");
    ESP_ReadResponse(2000);
    HAL_Delay(1000);

    LCD_DrawString(10, 60, "WiFi: connecting... ");
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PASS);
    ESP_Send(cmd);
    ESP_ReadResponse(15000);

    if (strstr(esp_rx, "WIFI CONNECTED") || strstr(esp_rx, "GOT IP"))
        LCD_DrawString(10, 60, "WiFi: CONNECTED     ");
    else
        LCD_DrawString(10, 60, "WiFi: FAILED        ");
    HAL_Delay(1000);

    LCD_DrawString(10, 60, "WiFi: getting IP... ");
    ESP_Send("AT+CIFSR\r\n");
    ESP_ReadResponse(3000);

    char ip_line[32];
    char *p = strstr(esp_rx, "STAIP,");
    if (p)
    {
        snprintf(ip_line, sizeof(ip_line), "%.24s", p);
        LCD_DrawString(10, 60, ip_line);
    }
    else
    {
        LCD_DrawString(10, 60, "WiFi: no IP found   ");
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_FSMC_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc2);

  HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);

  LCD_INIT();
  LCD_Clear(0, 0, 240, 320, WHITE);
  LCD_DrawString(80, 20, "Lai Man To");

  ESP_ConnectWiFi();

  LCD_DrawString(10, 50, "Joystick + Laser");
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
      char buf[40];

      uint32_t x_raw = read_adc1();
      uint32_t y_raw = read_adc2();
      GPIO_PinState k2_now = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

      if ((last_k2_state == GPIO_PIN_SET) && (k2_now == GPIO_PIN_RESET))
      {
          HAL_Delay(20);
          if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
          {
              last_k2_press_tick = HAL_GetTick();
              k2_has_been_pressed = 1;

              LCD_DrawString(10, 270, "Laser: firing...     ");
              laser_fire_1s();
              LCD_DrawString(10, 270, "Laser: done          ");
          }
      }

      last_k2_state = k2_now;

      uint32_t x_cV = (x_raw * 330U) / 4095U;
      uint32_t y_cV = (y_raw * 330U) / 4095U;

      LCD_Clear(0, 80, 240, 180, WHITE);

      sprintf(buf, "X raw: %4lu", x_raw);
      LCD_DrawString(10, 90, buf);

      sprintf(buf, "Y raw: %4lu", y_raw);
      LCD_DrawString(10, 120, buf);

      sprintf(buf, "X: %lu.%02lu V", x_cV / 100U, x_cV % 100U);
      LCD_DrawString(10, 150, buf);

      sprintf(buf, "Y: %lu.%02lu V", y_cV / 100U, y_cV % 100U);
      LCD_DrawString(10, 180, buf);

      if (k2_now == GPIO_PIN_RESET)
          sprintf(buf, "%-22s", "Button: pushed");
      else
          sprintf(buf, "%-22s", "Button: not pushed");
      LCD_DrawString(10, 210, buf);

      if (k2_has_been_pressed)
      {
          uint32_t elapsed_ms = HAL_GetTick() - last_k2_press_tick;
          uint32_t sec  = elapsed_ms / 1000U;
          uint32_t frac = (elapsed_ms % 1000U) / 10U;
          sprintf(buf, "Last: %3lu.%02lu s ago ", sec, frac);
          LCD_DrawString(10, 240, buf);
      }
      else
      {
          sprintf(buf, "%-22s", "Last: never");
          LCD_DrawString(10, 240, buf);
      }

      HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief ADC1 Initialization Function
  */
static void MX_ADC1_Init(void)
{
  /* USER CODE BEGIN ADC1_Init 0 */
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }

  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */
}

/**
  * @brief ADC2 Initialization Function
  */
static void MX_ADC2_Init(void)
{
  /* USER CODE BEGIN ADC2_Init 0 */
  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */
  /* USER CODE END ADC2_Init 1 */

  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK) { Error_Handler(); }

  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) { Error_Handler(); }

  /* USER CODE BEGIN ADC2_Init 2 */
  /* USER CODE END ADC2_Init 2 */
}

/**
  * @brief I2C2 Initialization Function
  */
static void MX_I2C2_Init(void)
{
  /* USER CODE BEGIN I2C2_Init 0 */
  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */
  /* USER CODE END I2C2_Init 1 */

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) { Error_Handler(); }

  /* USER CODE BEGIN I2C2_Init 2 */
  /* USER CODE END I2C2_Init 2 */
}

/**
  * @brief USART3 Initialization Function
  */
static void MX_USART3_UART_Init(void)
{
  /* USER CODE BEGIN USART3_Init 0 */
  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */
  /* USER CODE END USART3_Init 1 */

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK) { Error_Handler(); }

  /* USER CODE BEGIN USART3_Init 2 */
  /* USER CODE END USART3_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);

  /* PC13: K2 button */
  GPIO_InitStruct.Pin  = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* PC4: laser output */
  GPIO_InitStruct.Pin   = LASER_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LASER_PORT, &GPIO_InitStruct);

  /* PD12: output */
  GPIO_InitStruct.Pin   = GPIO_PIN_12;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* PE1: output */
  GPIO_InitStruct.Pin   = GPIO_PIN_1;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}
/* FSMC initialization function */
static void MX_FSMC_Init(void)
{
  /* USER CODE BEGIN FSMC_Init 0 */
  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */
  /* USER CODE END FSMC_Init 1 */

  hsram1.Instance    = FSMC_NORSRAM_DEVICE;
  hsram1.Extended    = FSMC_NORSRAM_EXTENDED_DEVICE;
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux        = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType            = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth       = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode       = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity    = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode              = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive      = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation        = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal            = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode          = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait      = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst            = FSMC_WRITE_BURST_DISABLE;

  Timing.AddressSetupTime      = 15;
  Timing.AddressHoldTime       = 15;
  Timing.DataSetupTime         = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision           = 16;
  Timing.DataLatency           = 17;
  Timing.AccessMode            = FSMC_ACCESS_MODE_A;

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK) { Error_Handler(); }

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */
  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif
