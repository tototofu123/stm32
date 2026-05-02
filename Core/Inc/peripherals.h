#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include "main.h"
#include <stdint.h>

// DS18B20 Pins
#define DS18B20_PIN         GPIO_PIN_8
#define DS18B20_PORT        GPIOC

// External Handles (Defined in main.c by CubeMX)
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern UART_HandleTypeDef huart3;

// External WiFi & UART Variables (Exposed for the UI to read)
extern char wifi_line1[32];
extern char wifi_line2[32];
extern char esp_rx[256];
extern uint8_t esp_rx_byte;
extern volatile uint16_t esp_rx_index;
extern volatile uint8_t  esp_rx_done;

// Function Prototypes
uint32_t read_adc1(void);
uint32_t read_adc2(void);

int32_t  DS18B20_ReadRaw(void);

void sendAT(const char *cmd);
void readResponse(void);
void WifiSetUp(void);

#endif // PERIPHERALS_H