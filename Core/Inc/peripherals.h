/*
 * =============================================================================
 * PERIPHERALS.H - HARDWARE INTERFACE AND SENSOR API
 * =============================================================================
 *
 * This header declares low-level hardware helpers: WiFi/ESP AT command interface,
 * ADC sensor reading, and utility functions for the embedded game system.
 *
 * Responsibility (peripherals.c implementation):
 * - Initialize and communicate with ESP8266 WiFi module via UART3.
 * - Provide AT command sending and response buffering.
 * - Implement WiFi scan and connection management.
 * - Read analog joystick inputs from ADC1 (X-axis) and ADC2 (Y-axis).
 * - Provide range mapping utility to convert raw ADC to normalized values.
 * - Interface with DS18B20 temperature sensor (Dallas 1-Wire protocol).
 * - Handle UART interrupt for ESP RX byte reception.
 *
 * Global variables exported:
 * - esp_rx_byte: Latest byte received from ESP module via UART interrupt.
 * - wifi_line1, wifi_line2: Status strings for WiFi display.
 *
 * Data flow:
 * - main.c calls WifiSetUp() during startup to initialize ESP.
 * - main.c reads ADC values each frame for joystick input.
 * - game_logic.c or main.c sends motor/fire commands via sendAT().
 * - UI code optionally displays WiFi status strings.
 * - Temperature sensor (DS18B20) interface currently not actively used.
 *
 * No C++ classes; pure C hardware drivers.
 * =============================================================================
 */

#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include "main.h"
#include "alerts.h" // Get shared RGB definitions
#include <stdint.h>

/*
 * =============================================================================
 * WIFI/ESP8266 MODULE INTERFACE
 * =============================================================================
 * Serial communication with ESP8266 WiFi module via UART3 AT commands.
 */
/* WiFi status display strings (shown in UI) */
extern char wifi_line1[32];  /* First line of WiFi status (e.g., "WiFi: Connected") */
extern char wifi_line2[32];  /* Second line of WiFi status (e.g., IP address) */

/* UART RX buffer for ESP module responses */
extern uint8_t esp_rx_byte;  /* Latest byte received from ESP via UART interrupt (ISR) */

/*
 * sendAT:
 * Sends an AT command to the ESP8266 module via UART3.
 * Input: cmd (null-terminated AT command string, e.g., "AT+GMR").
 * Response is captured into esp_rx_byte and global response buffers.
 */
void sendAT(const char *cmd);

/*
 * WifiSetUp:
 * Initializes the ESP8266 module during system startup.
 * Sends "AT" probe, resets module if needed, and prepares for WiFi operations.
 * Should be called once after UART3 is initialized.
 */
void WifiSetUp(void);

/*
 * WifiScan:
 * Scans for available WiFi networks and populates the SSID list.
 * Results are accessed through ui.c (wifi_ssids[], wifi_count).
 */
void WifiScan(void);

/*
 * WifiJoin:
 * Attempts to connect to a WiFi network with given SSID and password.
 * Input: ssid (network name), pass (network password).
 * Currently not actively used; can be integrated for WiFi config feature.
 */
void WifiJoin(const char* ssid, const char* pass);

/*
 * =============================================================================
 * ANALOG SENSOR INTERFACE
 * =============================================================================
 * ADC reading functions for joystick inputs and other analog sensors.
 */
/*
 * read_adc1:
 * Reads the analog value from ADC1 (joystick X-axis).
 * Output: 12-bit ADC value (0-4095, higher = further right).
 */
uint32_t read_adc1(void);

/*
 * read_adc2:
 * Reads the analog value from ADC2 (joystick Y-axis).
 * Output: 12-bit ADC value (0-4095, higher = further back/down).
 */
uint32_t read_adc2(void);

/*
 * map_u16:
 * Maps a raw sensor value from one range to another.
 * Utility function used to convert ADC readings to normalized coordinates or percentages.
 * Input: x (value to map), in_min/in_max (input range), out_min/out_max (output range).
 * Output: Mapped value in output range (clamped).
 */
uint16_t map_u16(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);

/*
 * =============================================================================
 * TEMPERATURE SENSOR INTERFACE
 * =============================================================================
 * DS18B20 1-Wire temperature sensor (currently not actively used in gameplay).
 */
#define DS18B20_PIN         GPIO_PIN_11  /* 1-Wire data pin */
#define DS18B20_PORT        GPIOC        /* GPIO port for 1-Wire line */

/*
 * DS18B20_ReadRaw:
 * Reads the raw temperature value from the DS18B20 sensor.
 * Output: Raw 12-bit temperature reading (lower 12 bits of 16-bit value).
 * Note: This is a stub implementation; currently not integrated into gameplay.
 */
int32_t DS18B20_ReadRaw(void);

#endif  /* PERIPHERALS_H */
