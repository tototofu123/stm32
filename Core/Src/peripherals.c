#include "peripherals.h"
#include "lcd.h"
#include "ui.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

char wifi_line1[32] = "WiFi:Init...";
char wifi_line2[32] = "";
uint8_t esp_rx_byte;
char esp_rx_buffer[512];
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

void WifiSetUp(void)
{
    sendAT("AT+RST");
    HAL_Delay(1000);
    sendAT("AT+CWMODE=1");
    HAL_Delay(500);
    sendAT("AT+CIPMUX=0");
    HAL_Delay(500);
    strcpy(wifi_line1, "WiFi:READY");
}

void WifiScan(void)
{
    wifi_count = 0;
    esp_rx_idx = 0;
    memset(esp_rx_buffer, 0, sizeof(esp_rx_buffer));
    
    sendAT("AT+CWLAP");
    
    // Wait for scan to complete (approx 3-5 seconds)
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 5000) {
        if (strstr(esp_rx_buffer, "OK") || strstr(esp_rx_buffer, "ERROR")) break;
        HAL_Delay(100);
    }
    
    // Simple parser for +CWLAP:(x,"SSID",y,...)
    char *ptr = esp_rx_buffer;
    while (wifi_count < MAX_WIFI_NETWORKS) {
        ptr = strstr(ptr, "+CWLAP:(");
        if (!ptr) break;
        
        ptr = strchr(ptr, '"'); // First quote
        if (!ptr) break;
        ptr++;
        
        char *end = strchr(ptr, '"'); // Second quote
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
    HAL_Delay(5000); // Wait for connection
    
    // Try to get IP
    esp_rx_idx = 0;
    memset(esp_rx_buffer, 0, sizeof(esp_rx_buffer));
    sendAT("AT+CIFSR");
    HAL_Delay(1000);
    
    char *ptr = strstr(esp_rx_buffer, "STAIP,\"");
    if (ptr) {
        ptr += 7;
        char *end = strchr(ptr, '"');
        if (end) {
            uint16_t len = end - ptr;
            if (len > 31) len = 31;
            strncpy(wifi_line2, ptr, len);
            wifi_line2[len] = '\0';
            strcpy(wifi_line1, "CONNECTED");
            return;
        }
    }
    strcpy(wifi_line1, "FAIL");
}

void RGB_Set(uint8_t r, uint8_t g, uint8_t b)
{
    HAL_GPIO_WritePin(RGB_R_PORT, RGB_R_PIN, r ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(RGB_G_PORT, RGB_G_PIN, g ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(RGB_B_PORT, RGB_B_PIN, b ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

uint32_t read_adc1(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    return HAL_ADC_GetValue(&hadc1);
}

uint32_t read_adc2(void)
{
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 10);
    return HAL_ADC_GetValue(&hadc2);
}

uint16_t map_u16(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
    if (x <= in_min) return (uint16_t)out_min;
    if (x >= in_max) return (uint16_t)out_max;
    return (uint16_t)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

int32_t DS18B20_ReadRaw(void) { return -2032; }
