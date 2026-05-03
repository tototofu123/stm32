/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Master UI Handheld Implementation
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include "lcd.h"
#include <stdio.h>
#include <string.h>

// Modules
#include "touch.h"
#include "seven_seg.h"
#include "alerts.h"
#include "peripherals.h"
#include "game_logic.h"
#include "ui.h"
#include "mode_2.h"
#include "mode_3.h"
/* USER CODE END Includes */

#define BTN_DEBOUNCE_MS     120U

#define K1_PIN              GPIO_PIN_0
#define K1_PORT             GPIOA

#define CAP_TOUCH_PIN       GPIO_PIN_1
#define CAP_TOUCH_PORT      GPIOA

#define K2_PIN              GPIO_PIN_13
#define K2_PORT             GPIOC

#define JOY_SW_PIN          GPIO_PIN_2
#define JOY_SW_PORT         GPIOC

#define BTN1_PIN            GPIO_PIN_2
#define BTN1_PORT           GPIOA
#define BTN2_PIN            GPIO_PIN_3
#define BTN2_PORT           GPIOA

ADC_HandleTypeDef  hadc1;
ADC_HandleTypeDef  hadc2;
I2C_HandleTypeDef  hi2c2;
UART_HandleTypeDef huart3;
SRAM_HandleTypeDef hsram1;

// Button states
GPIO_PinState last_k1_state  = GPIO_PIN_RESET;
GPIO_PinState last_k2_state  = GPIO_PIN_RESET;
GPIO_PinState last_jsw_state = GPIO_PIN_SET;
GPIO_PinState last_touch_state = GPIO_PIN_SET;

uint32_t last_k1_event_tick = 0U;
uint32_t last_k2_event_tick = 0U;

GPIO_PinState last_fire_input_state = GPIO_PIN_SET;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_FSMC_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_I2C2_Init();
    MX_USART3_UART_Init();

    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADCEx_Calibration_Start(&hadc2);

    HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
    Buzzer_Set(0);
    RGB_Set(1, 1, 1);
    SEG_AllOff();
    SEG_ShowPair(0, 0, 0);

    LCD_INIT();
    
    // Proactive drawing: ensure Home screen is visible immediately
    LCD_DrawHome();
    last_drawn_state = APP_HOME;

    WifiSetUp();
    HAL_UART_Receive_IT(&huart3, &esp_rx_byte, 1);

    while (1)
    {
        uint32_t now = HAL_GetTick();
        uint32_t x_raw = read_adc1();
        uint32_t y_raw = read_adc2();
        GPIO_PinState k1_now = HAL_GPIO_ReadPin(K1_PORT, K1_PIN);
        GPIO_PinState k2_now = HAL_GPIO_ReadPin(K2_PORT, K2_PIN);

        // -- Read Screen Touch --
        uint8_t ts_pressed = TouchPressed();
        uint8_t ts_click = 0;
        uint16_t px = 0, py = 0;
        static uint8_t last_ts_state_mem = 0;
        static uint32_t last_ts_event_tick = 0;

        if (ts_pressed)
        {
            uint16_t tx = TouchReadXRaw();
            uint16_t ty = TouchReadYRaw();
            uint16_t curr_px = map_u16(tx, TS_X_MIN, TS_X_MAX, 0, 239);
            uint16_t curr_py = map_u16(ty, TS_Y_MIN, TS_Y_MAX, 0, 319);
            curr_py = 319 - curr_py;

            if ((last_ts_state_mem == 0) && ((now - last_ts_event_tick) >= 250U)) {
                ts_click = 1;
                px = curr_px;
                py = curr_py;
                last_ts_event_tick = now;
            }
            last_ts_state_mem = 1;
        }
        else { last_ts_state_mem = 0; }

        uint8_t k1_click = 0U, k2_click = 0U;
        if ((k1_now == GPIO_PIN_SET) && (last_k1_state == GPIO_PIN_RESET) && ((now - last_k1_event_tick) >= BTN_DEBOUNCE_MS))
        { k1_click = 1U; last_k1_event_tick = now; }
        last_k1_state = k1_now;

        if ((k2_now == GPIO_PIN_SET) && (last_k2_state == GPIO_PIN_RESET) && ((now - last_k2_event_tick) >= BTN_DEBOUNCE_MS))
        { k2_click = 1U; last_k2_event_tick = now; }
        last_k2_state = k2_now;

        // --- Master UI Controller ---
        if (app_state != last_drawn_state)
        {
            if (app_state == APP_HOME)          LCD_DrawHome();
            else if (app_state == APP_SETTINGS) LCD_DrawSettings();
            else if (app_state == APP_MODE_SELECT) LCD_DrawModeSelect();
            else if (app_state == APP_MODE_CONFIRM) LCD_DrawModeConfirm();
            else if (app_state == APP_CAR_SELECT)  LCD_DrawCarSelect();
            else if (app_state == APP_CAR_CONFIRM) LCD_DrawCarConfirm();
            else if (app_state == APP_GAME) {
                if (selected_mode == GAME_MODE_2) Mode2_Init();
                else if (selected_mode == GAME_MODE_3) Mode3_Init();
                else LCD_DrawGameLayout();
            }
            last_drawn_state = app_state;
        }

        if (app_state == APP_HOME) {
            if (ts_click) {
                if (px >= 20 && px <= 220) {
                    if (py >= 100 && py <= 160) { app_state = APP_MODE_SELECT; Buzzer_BeepLong(); }
                    else if (py >= 180 && py <= 240) { app_state = APP_SETTINGS; Buzzer_BeepShort(); }
                }
            }
        }
        else if (app_state == APP_SETTINGS) {
            if (k1_click) app_state = APP_HOME;
        }
        else if (app_state == APP_MODE_SELECT) {
            if (k1_click) selected_mode = (game_mode_t)(((uint8_t)selected_mode + 1U) % 3U);
            if (k2_click) app_state = APP_MODE_CONFIRM;
            if (ts_click) {
                if (px >= 20 && px <= 220) {
                    if (py >= 70 && py <= 110) selected_mode = GAME_MODE_1;
                    else if (py >= 125 && py <= 165) selected_mode = GAME_MODE_2;
                    else if (py >= 180 && py <= 220) selected_mode = GAME_MODE_3;
                    app_state = APP_MODE_CONFIRM;
                }
            }
            if (selected_mode != last_drawn_mode) { LCD_UpdateModeSelection(); last_drawn_mode = selected_mode; }
        }
        else if (app_state == APP_MODE_CONFIRM) {
            if (k1_click) app_state = APP_MODE_SELECT;
            if (k2_click) {
                if (selected_mode == GAME_MODE_2 || selected_mode == GAME_MODE_3) app_state = APP_GAME;
                else app_state = APP_CAR_SELECT;
            }
        }
        else if (app_state == APP_CAR_SELECT) {
            if (k1_click) selected_car = (car_type_t)(((uint8_t)selected_car + 1U) % 7U);
            if (k2_click) app_state = APP_CAR_CONFIRM;
        }
        else if (app_state == APP_CAR_CONFIRM) {
            if (k1_click) app_state = APP_CAR_SELECT;
            if (k2_click) app_state = APP_GAME;
        }
        else if (app_state == APP_GAME) {
            if (selected_mode == GAME_MODE_2) {
                Mode2_Run(x_raw, y_raw, k1_click, k2_click, 0, ts_pressed, ts_click, px, py);
            }
            else if (selected_mode == GAME_MODE_3) {
                Mode3_Run(x_raw, y_raw, k1_click, k2_click, 0, ts_pressed, ts_click, px, py);
            }
            else {
                Game_Router_Task(x_raw, y_raw, k1_click, k2_click, 0);
            }
        }

        Buzzer_Task();
        SEG_Task();
        HAL_Delay(10);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef o = {0};
    RCC_ClkInitTypeDef c = {0};
    RCC_PeriphCLKInitTypeDef p = {0};
    o.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    o.HSEState = RCC_HSE_ON;
    o.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    o.HSIState = RCC_HSI_ON;
    o.PLL.PLLState = RCC_PLL_ON;
    o.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    o.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&o) != HAL_OK) Error_Handler();
    c.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    c.AHBCLKDivider = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV2;
    c.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
    p.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    p.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&p) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef s = {0};
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
    s.Channel = ADC_CHANNEL_10;
    s.Rank = ADC_REGULAR_RANK_1;
    s.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &s) != HAL_OK) Error_Handler();
}

static void MX_ADC2_Init(void)
{
    ADC_ChannelConfTypeDef s = {0};
    hadc2.Instance = ADC2;
    hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) Error_Handler();
    s.Channel = ADC_CHANNEL_11;
    s.Rank = ADC_REGULAR_RANK_1;
    s.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc2, &s) != HAL_OK) Error_Handler();
}

static void MX_I2C2_Init(void)
{
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) Error_Handler();
}

static void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RGB_R_PORT, RGB_R_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RGB_G_PORT, RGB_G_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RGB_B_PORT, RGB_B_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BEEP_PORT, BEEP_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(T_CS_PORT, T_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(T_CLK_PORT, T_CLK_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(T_DIN_PORT, T_DIN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    g.Pin = GPIO_PIN_12; g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOD, &g);
    g.Pin = T_CS_PIN; g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(T_CS_PORT, &g);
    g.Pin = GPIO_PIN_1 | GPIO_PIN_7; HAL_GPIO_Init(GPIOE, &g);
    g.Pin = T_CLK_PIN | T_DIN_PIN; g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &g);
    g.Pin = T_DOUT_PIN | T_IRQ_PIN; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &g);
    g.Pin = LASER_PIN; g.Speed = GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(LASER_PORT, &g);
    g.Pin = RGB_R_PIN | RGB_G_PIN | RGB_B_PIN | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);
    g.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | BEEP_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = CAP_TOUCH_PIN; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CAP_TOUCH_PORT, &g);
    g.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);
    g.Pin = JOY_SW_PIN; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(JOY_SW_PORT, &g);
    g.Pin = BTN1_PIN | BTN2_PIN; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BTN1_PORT, &g);
    g.Pin = K1_PIN; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(K1_PORT, &g);
    g.Pin = K2_PIN; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(K2_PORT, &g);
    g.Pin = DS18B20_PIN; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DS18B20_PORT, &g);
}

static void MX_FSMC_Init(void)
{
    FSMC_NORSRAM_TimingTypeDef t = {0};
    hsram1.Instance = FSMC_NORSRAM_DEVICE;
    hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
    hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
    hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
    hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
    hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
    hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
    hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
    hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
    hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
    hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
    hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
    hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
    hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
    hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
    t.AddressSetupTime = 15;
    t.AddressHoldTime = 15;
    t.DataSetupTime = 255;
    t.BusTurnAroundDuration = 15;
    t.CLKDivision = 16;
    t.DataLatency = 17;
    t.AccessMode = FSMC_ACCESS_MODE_A;
    if (HAL_SRAM_Init(&hsram1, &t, NULL) != HAL_OK) Error_Handler();
    __HAL_AFIO_FSMCNADV_DISCONNECTED();
}

void Error_Handler(void) { __disable_irq(); while (1) {} }
