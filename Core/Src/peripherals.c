/*
 * peripherals.c contains the shared helper functions for external hardware and
 * simple data acquisition. It covers UART communication with the ESP module,
 * WiFi setup, and joystick ADC reading helpers.
 *
 * Functions in this file:
 * - HAL_UART_RxCpltCallback: stores received UART bytes into a receive buffer.
 * - sendAT: sends an AT command through USART3.
 * - waitForResponse: waits for a specific response string from the ESP buffer.
 * - WifiSetUp: sends the startup WiFi/AP configuration sequence.
 * - WifiScan: queries nearby WiFi networks and stores SSIDs.
 * - WifiJoin: sends a join command for a selected WiFi network.
 * - read_adc1: reads the first joystick ADC channel.
 * - read_adc2: reads the second joystick ADC channel.
 * - map_u16: maps a raw integer from one range into another.
 * - DS18B20_ReadRaw: returns a placeholder temperature raw value.
 *
 * Global variables used here include wifi_line1, wifi_line2, esp_rx_byte,
 * esp_rx_buffer, esp_rx_idx, wifi_ssids, wifi_count, and the ADC/UART handles.
 * No classes are used in this C file.
 */
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

/* HAL_UART_RxCpltCallback stores each received UART byte and immediately arms
 * the next receive interrupt so incoming ESP data keeps flowing.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (esp_rx_idx < sizeof(esp_rx_buffer) - 1)
        {
            esp_rx_buffer[esp_rx_idx++] = esp_rx_byte;
        }
        HAL_UART_Receive_IT(&huart3, &esp_rx_byte, 1);

        if(esp_rx_idx >= 1000)  {memset(esp_rx_buffer, 0, sizeof(esp_rx_buffer));
        esp_rx_idx = 0;}
    }
}



/* sendAT formats and transmits an AT command plus CRLF over USART3.
 */
void sendAT(const char *cmd)
{
    char buf[128];
    snprintf(buf, sizeof(buf), "%s\r\n", cmd);
    HAL_UART_Transmit(&huart3, (uint8_t *)buf, strlen(buf), 100);
}

/* waitForResponse polls the ESP receive buffer until the expected response
 * string arrives or the timeout expires.
 */
uint8_t waitForResponse(const char* target, uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms) {
        if (strstr(esp_rx_buffer, target)) {
        	memset(esp_rx_buffer, 0, sizeof(esp_rx_buffer));
        	esp_rx_idx = 0;
        	return 1;}
        HAL_Delay(5); // Small delay to yield
    }
    return 0;
}


void Wifi_ProcessIncoming(void) {
    // Check for the +IPD (Inbound Packet Data) header from ESP8266
    char *ipd_ptr = strstr(esp_rx_buffer, "+IPD,");
    if (ipd_ptr != NULL) {
        // Find the colon ':' that separates the header from the actual data
        char *msg_ptr = strchr(ipd_ptr, ':');

        if (msg_ptr != NULL) {
            msg_ptr++; // Move pointer to the start of the actual message

            // Check if the received message contains "connect"
            if (strstr(msg_ptr, "connect") != NULL) {
                strcpy(wifi_line2, "Connected");
               // Buzzer_BeepShort(); // Optional audio feedback
            }

            // Clear the buffer and index once processed to prevent re-triggering[cite: 1]

        }
    }
}

/*void Wifi_Update_Polling(void) {
    uint8_t ch;

    // Check if a byte is waiting in the UART3 data register
    // Timeout is set to 0 so it doesn't block the game
    if (HAL_UART_Receive(&huart3, &ch, 1, 0) == HAL_OK) {

        // Add the byte to our search buffer
        if (esp_rx_idx < sizeof(esp_rx_buffer) - 1) {
            esp_rx_buffer[esp_rx_idx++] = (char)ch;
            esp_rx_buffer[esp_rx_idx] = '\0'; // Keep string null-terminated
        } else {
            // Buffer safety: reset if full
            esp_rx_idx = 0;
            memset(esp_rx_buffer, 0, sizeof(esp_rx_buffer));
        }

        // Check for the "connect" message within the +IPD wrapper
        char *ipd_ptr = strstr(esp_rx_buffer, "+IPD,");
        if (ipd_ptr) {
            char *msg_start = strchr(ipd_ptr, ':');
            if (msg_start && strstr(msg_start, "connect")) {
                strcpy(wifi_line2, "Connected"); // Trigger your UI change

                // Clear buffer after successful match
                esp_rx_idx = 0;
                memset(esp_rx_buffer, 0, sizeof(esp_rx_buffer));
            }
        }
    }
}
*/


/* WifiSetUp sends the startup access-point configuration to the ESP module and
 * stores the local address text for the UI.
 */
void WifiSetUp(void)
{
    // NO BLOCKING RESET: Using faster commands and fewer delays
    sendAT("AT+CWMODE=3");
    HAL_Delay(100); // Minimum delay for state switch
    
    sendAT("AT+CWSAP=\"TANK_CAR_02\",\"password\",2,3");
    HAL_Delay(100);
    
    sendAT("AT+CIPMUX=1");
    HAL_Delay(100);

    sendAT("AT+CIPSERVER=1,80");
    HAL_Delay(500);

    strcpy(wifi_line1, "AP+STA ACTIVE");

    //strcpy(wifi_line2, "192.168.4.1");


}

void Wifi_SendToClient(int link_id, char* msg) {
    char cmd[64];

    char terminated_msg[128];

        // Add the \r terminator the client is looking for
        snprintf(terminated_msg, sizeof(terminated_msg), "%s\r", msg);
        int len = strlen(terminated_msg);

    // 1. Format the command: AT+CIPSEND=ID,LENGTH
    sprintf(cmd, "AT+CIPSEND=%d,%d", link_id, len);
    sendAT(cmd); // Your existing function that adds \r\n
    HAL_Delay(100);
    // 2. Wait for the '>' prompt from the ESP8266
    // We use your waitForResponse to ensure the modem is ready
        // 3. Transmit the raw message without \r\n (unless you want them in the msg)
        HAL_UART_Transmit(&huart3, (uint8_t *)terminated_msg, len, 250);

}

/* WifiScan clears the receive buffer, requests a scan, and parses returned SSID
 * entries into the local list.
 */
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

/* WifiJoin sends a join request for the chosen WiFi network and updates the UI
 * text to show that a connection attempt is in progress.
 */
void WifiJoin(const char* ssid, const char* pass)
{
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", ssid, pass);
    sendAT(cmd);
    strcpy(wifi_line1, "Connecting...");
}

/* read_adc1 samples the first ADC channel and returns the current joystick X
 * reading.
 */
uint32_t read_adc1(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return val;
}

/* read_adc2 samples the second ADC channel and returns the current joystick Y
 * reading.
 */
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
