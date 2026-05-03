#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include "main.h"
#include "alerts.h" // Get shared RGB definitions
#include <stdint.h>

// WiFi (ESP8266)
extern char wifi_line1[32];
extern char wifi_line2[32];
extern uint8_t esp_rx_byte;

void sendAT(const char *cmd);
void WifiSetUp(void);
void WifiScan(void);
void WifiJoin(const char* ssid, const char* pass);

// Sensors
uint32_t read_adc1(void);
uint32_t read_adc2(void);
uint16_t map_u16(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
int32_t DS18B20_ReadRaw(void);

#define DS18B20_PIN         GPIO_PIN_11
#define DS18B20_PORT        GPIOC

#endif // PERIPHERALS_H
