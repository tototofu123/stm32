#include "peripherals.h"
#include <stdio.h>
#include <string.h>

// WiFi state variables
char wifi_line1[32] = "idle";
char wifi_line2[32] = "none";

// UART ESP8266 buffer variables
char    esp_rx[256];
uint8_t esp_rx_byte;
volatile uint16_t esp_rx_index = 0;
volatile uint8_t  esp_rx_done  = 0;

// --- ADC Functions ---
uint32_t read_adc1(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t v = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return v;
}

uint32_t read_adc2(void)
{
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 10);
    uint32_t v = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);
    return v;
}

// --- DS18B20 Temperature Sensor ---
static void ds_delay_us(uint16_t us)
{
    uint32_t n = (uint32_t)us * 18U;
    while (n--) { __NOP(); }
}

static void ds_pin_out(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin   = DS18B20_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DS18B20_PORT, &g);
}

static void ds_pin_in(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin  = DS18B20_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DS18B20_PORT, &g);
}

static uint8_t ds_start(void)
{
    uint8_t present;
    ds_pin_out();
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
    ds_delay_us(500);
    ds_pin_in();
    ds_delay_us(70);
    present = (HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
    ds_delay_us(430);
    return present;
}

static void ds_write(uint8_t data)
{
    uint8_t i;
    for (i = 0; i < 8U; i++)
    {
        ds_pin_out();
        HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
        ds_delay_us(2);
        if (data & 0x01U) ds_pin_in();
        ds_delay_us(60);
        ds_pin_in();
        ds_delay_us(2);
        data >>= 1U;
    }
}

static uint8_t ds_read_byte(void)
{
    uint8_t i, val = 0U;
    for (i = 0; i < 8U; i++)
    {
        ds_pin_out();
        HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
        ds_delay_us(2);
        ds_pin_in();
        ds_delay_us(10);
        if (HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN) == GPIO_PIN_SET)
            val |= (uint8_t)(1U << i);
        ds_delay_us(55);
    }
    return val;
}

int32_t DS18B20_ReadRaw(void)
{
    uint8_t lo, hi;
    int16_t raw;

    if (!ds_start()) return -2032;
    ds_write(0xCCU);
    ds_write(0x44U);
    HAL_Delay(750);

    if (!ds_start()) return -2032;
    ds_write(0xCCU);
    ds_write(0xBEU);

    lo = ds_read_byte();
    hi = ds_read_byte();

    raw = (int16_t)(((uint16_t)hi << 8) | lo);
    if (raw == 0x0550) return -2032; // Error reading

    return (int32_t)raw;
}

// --- WiFi / ESP8266 UART Functions ---
void sendAT(const char *cmd)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
    HAL_Delay(20);
}

void readResponse(void)
{
    char buffer[128] = {0};
    HAL_UART_Receive(&huart3, (uint8_t *)buffer, sizeof(buffer) - 1, 1000);

    if (strstr(buffer, "Hello from ESP01s client!") != NULL)
    {
        snprintf(wifi_line1, sizeof(wifi_line1), "connect");
    }
}

void WifiSetUp(void)
{
    sendAT("AT");
    sendAT("AT+CWMODE=2");
    sendAT("AT+CWSAP=\"ESP8266_AP_01\",\"12345678\",5,3");
    sendAT("AT+CIFSR");
    sendAT("AT+CIPMUX=1");
    sendAT("AT+CIPSERVER=1,80");
    readResponse();
}

// Global UART Interrupt Callback (moved here from main.c)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        char c = (char)esp_rx_byte;

        if (esp_rx_index < sizeof(esp_rx) - 1)
        {
            esp_rx[esp_rx_index++] = c;
            esp_rx[esp_rx_index]   = '\0';
        }

        esp_rx_done = 1;
        HAL_UART_Receive_IT(&huart3, &esp_rx_byte, 1);
    }
}