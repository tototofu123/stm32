#include "peripherals.h"
#include "lcd.h"
#include "ui.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

char wifi_line1[32] = "WiFi:Init...";
char wifi_line2[32] = "";
uint8_t esp_rx_byte;
char esp_rx_buffer[1024]; 
uint16_t esp_rx_idx = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (esp_rx_idx < sizeof(esp_rx_buffer) - 1)
        {
            esp_rx_buffer[esp_rx_idx++] = esp_rx_byte;
        }
        HAL_UART_Receive_IT(&huart3, &esp_rx_byte, 1);
    }
}

void sendAT(const char *cmd)
{
    char buf[128];
    snprintf(buf, sizeof(buf), "%s\r\n", cmd);
    HAL_UART_Transmit(&huart3, (uint8_t *)buf, strlen(buf), 100);
}

uint8_t waitForResponse(const char* target, uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms) {
        if (strstr(esp_rx_buffer, target)) return 1;
        HAL_Delay(5); // Small delay to yield
    }
    return 0;
}

void WifiSetUp(void)
{
    // NO BLOCKING RESET: Using faster commands and fewer delays
    sendAT("AT+CWMODE=3");
    HAL_Delay(100); // Minimum delay for state switch
    
    sendAT("AT+CWSAP=\"TANK_CAR\",\"password\",1,3");
    HAL_Delay(100);
    
    sendAT("AT+CIPMUX=1");
    HAL_Delay(100);

    strcpy(wifi_line2, "192.168.4.1"); 
    strcpy(wifi_line1, "AP+STA ACTIVE");
}

void WifiScan(void)
{
    wifi_count = 0;
    esp_rx_idx = 0;
    memset(esp_rx_buffer, 0, sizeof(esp_rx_buffer));
    
    sendAT("AT+CWLAP");
    
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 3000) { // Reduced scan window
        if (strstr(esp_rx_buffer, "OK") || strstr(esp_rx_buffer, "ERROR")) break;
        HAL_GPIO_TogglePin(RGB_B_PORT, RGB_B_PIN);
        HAL_Delay(50);
    }
    HAL_GPIO_WritePin(RGB_B_PORT, RGB_B_PIN, GPIO_PIN_SET);
    
    char *ptr = esp_rx_buffer;
    while (wifi_count < MAX_WIFI_NETWORKS) {
        ptr = strstr(ptr, "+CWLAP:(");
        if (!ptr) break;
        ptr = strchr(ptr, '"');
        if (!ptr) break;
        ptr++;
        char *end = strchr(ptr, '"');
        if (!end) break;
        uint16_t len = end - ptr;
        if (len > 32) len = 32;
        strncpy(wifi_ssids[wifi_count], ptr, len);
        wifi_ssids[wifi_count][len] = '\0';
        wifi_count++;
        ptr = end + 1;
    }
}

void WifiJoin(const char* ssid, const char* pass)
{
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", ssid, pass);
    sendAT(cmd);
    strcpy(wifi_line1, "Connecting...");
}

uint32_t read_adc1(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return val;
}

uint32_t read_adc2(void)
{
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 10);
    uint32_t val = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);
    return val;
}

uint16_t map_u16(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
    if (x <= in_min) return (uint16_t)out_min;
    if (x >= in_max) return (uint16_t)out_max;
    return (uint16_t)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

int32_t DS18B20_ReadRaw(void) { return -2032; }
