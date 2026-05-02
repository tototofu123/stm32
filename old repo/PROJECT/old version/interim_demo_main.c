/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include "lcd.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
typedef enum
{
    WIFI_STATE_IDLE = 0,
    WIFI_STATE_SEND_AT,
    WIFI_STATE_WAIT_AT,
    WIFI_STATE_SEND_CWMODE,
    WIFI_STATE_WAIT_CWMODE,
    WIFI_STATE_SEND_CWJAP,
    WIFI_STATE_WAIT_CWJAP,
    WIFI_STATE_SEND_CIFSR,
    WIFI_STATE_WAIT_CIFSR,
    WIFI_STATE_DONE,
    WIFI_STATE_FAIL
} wifi_state_t;

typedef enum
{
    MOTOR_FWD = 0,
    MOTOR_STOP1,
    MOTOR_BACK,
    MOTOR_STOP2
} motor_state_t;
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
#define LASER_PIN       GPIO_PIN_4
#define LASER_PORT      GPIOC

#define WIFI_SSID       "ESP_2F0F28"
#define WIFI_PASS       "thereisnospoon"

#define LED_LEFT_PIN    GPIO_PIN_2
#define LED_LEFT_PORT   GPIOA
#define FIRE_SIG_PIN    GPIO_PIN_3
#define FIRE_SIG_PORT   GPIOA
#define LED_RIGHT_PIN   GPIO_PIN_4
#define LED_RIGHT_PORT  GPIOA
#define LED_FRONT_PIN   GPIO_PIN_5
#define LED_FRONT_PORT  GPIOA

#define DS18B20_PIN     GPIO_PIN_8
#define DS18B20_PORT    GPIOC

#define K1_PIN          GPIO_PIN_13
#define K1_PORT         GPIOC

#define MOT_L_IN1_PIN   GPIO_PIN_6
#define MOT_L_IN1_PORT  GPIOB
#define MOT_L_IN2_PIN   GPIO_PIN_7
#define MOT_L_IN2_PORT  GPIOB
#define MOT_R_IN3_PIN   GPIO_PIN_8
#define MOT_R_IN3_PORT  GPIOB
#define MOT_R_IN4_PIN   GPIO_PIN_9
#define MOT_R_IN4_PORT  GPIOB
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart3;
SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
uint32_t last_k2_press_tick  = 0;
uint8_t  k2_has_been_pressed = 0;
GPIO_PinState last_k2_state  = GPIO_PIN_SET;

char     esp_rx[256];
uint8_t  esp_rx_byte;
volatile uint16_t esp_rx_index = 0;
volatile uint8_t  esp_rx_done  = 0;

wifi_state_t wifi_state      = WIFI_STATE_IDLE;
uint32_t     wifi_state_tick = 0;
char wifi_line1[32] = "WiFi: idle";
char wifi_line2[32] = "IP: none";
char laser_line[32] = "Laser: ready";
char last_ip[32]    = "IP: none";

char prev_wifi_line1[32] = "";
char prev_wifi_line2[32] = "";
char prev_x_value[24]    = "";
char prev_y_value[24]    = "";
char prev_btn_value[24]  = "";
char prev_last_value[24] = "";
char prev_laser_value[24]= "";
char prev_temp_value[24] = "";
char prev_motor_value[16]= "";

uint8_t  laser_active     = 0;
uint32_t laser_end_tick   = 0;

int32_t  ds18b20_raw       = -2032;
uint32_t ds18b20_last_tick = 0;

motor_state_t motor_state      = MOTOR_FWD;
uint32_t      motor_state_tick = 0;
char          motor_label[16]  = "FWD";
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
void laser_start_1s(void);
void laser_update(void);
void ESP_Send(char *cmd);
void ESP_StartReceiveIT(void);
void ESP_ClearBuffer(void);
void WiFi_Start(void);
void WiFi_Task(void);
void Motor_Task(void);
void LCD_UpdateUI(uint32_t x_raw, uint32_t y_raw, GPIO_PinState k2_now,
                  int32_t temp_raw);
static void    ds_pin_out(void);
static void    ds_pin_in(void);
static void    ds_delay_us(uint16_t us);
static uint8_t ds_start(void);
static void    ds_write(uint8_t data);
static uint8_t ds_read(void);
int32_t        DS18B20_ReadRaw(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#define MOTOR_L_FWD() do { \
    HAL_GPIO_WritePin(MOT_L_IN1_PORT, MOT_L_IN1_PIN, GPIO_PIN_SET);   \
    HAL_GPIO_WritePin(MOT_L_IN2_PORT, MOT_L_IN2_PIN, GPIO_PIN_RESET); \
} while(0)

#define MOTOR_L_REV() do { \
    HAL_GPIO_WritePin(MOT_L_IN1_PORT, MOT_L_IN1_PIN, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(MOT_L_IN2_PORT, MOT_L_IN2_PIN, GPIO_PIN_SET);   \
} while(0)

#define MOTOR_L_OFF() do { \
    HAL_GPIO_WritePin(MOT_L_IN1_PORT, MOT_L_IN1_PIN, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(MOT_L_IN2_PORT, MOT_L_IN2_PIN, GPIO_PIN_RESET); \
} while(0)

#define MOTOR_R_FWD() do { \
    HAL_GPIO_WritePin(MOT_R_IN3_PORT, MOT_R_IN3_PIN, GPIO_PIN_SET);   \
    HAL_GPIO_WritePin(MOT_R_IN4_PORT, MOT_R_IN4_PIN, GPIO_PIN_RESET); \
} while(0)

#define MOTOR_R_REV() do { \
    HAL_GPIO_WritePin(MOT_R_IN3_PORT, MOT_R_IN3_PIN, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(MOT_R_IN4_PORT, MOT_R_IN4_PIN, GPIO_PIN_SET);   \
} while(0)

#define MOTOR_R_OFF() do { \
    HAL_GPIO_WritePin(MOT_R_IN3_PORT, MOT_R_IN3_PIN, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(MOT_R_IN4_PORT, MOT_R_IN4_PIN, GPIO_PIN_RESET); \
} while(0)

static void ds_delay_us(uint16_t us)
{
    uint32_t count = (uint32_t)us * 18U;
    while (count--) { __NOP(); }
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
    uint8_t presence;
    ds_pin_out();
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
    ds_delay_us(500);
    ds_pin_in();
    ds_delay_us(70);
    presence = (HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
    ds_delay_us(430);
    return presence;
}

static void ds_write(uint8_t data)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        ds_pin_out();
        HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
        ds_delay_us(2);
        if (data & 0x01) ds_pin_in();
        ds_delay_us(60);
        ds_pin_in();
        ds_delay_us(2);
        data >>= 1;
    }
}

static uint8_t ds_read(void)
{
    uint8_t i, value = 0;
    for (i = 0; i < 8; i++)
    {
        ds_pin_out();
        HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
        ds_delay_us(2);
        ds_pin_in();
        ds_delay_us(10);
        if (HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN) == GPIO_PIN_SET)
            value |= (1U << i);
        ds_delay_us(55);
    }
    return value;
}

int32_t DS18B20_ReadRaw(void)
{
    uint8_t lo, hi;
    int16_t raw;
    if (!ds_start()) return -2032;
    ds_write(0xCC);
    ds_write(0x44);
    HAL_Delay(750);
    if (!ds_start()) return -2032;
    ds_write(0xCC);
    ds_write(0xBE);
    lo  = ds_read();
    hi  = ds_read();
    raw = (int16_t)(((uint16_t)hi << 8) | lo);
    if (raw == 0x0550) return -2032;
    return (int32_t)raw;
}

void Motor_Task(void)
{
    uint32_t now = HAL_GetTick();

    switch (motor_state)
    {
    case MOTOR_FWD:
        MOTOR_L_FWD();
        MOTOR_R_FWD();
        strcpy(motor_label, "FWD");
        if (now - motor_state_tick >= 2000U)
        {
            motor_state = MOTOR_STOP1;
            motor_state_tick = now;
        }
        break;

    case MOTOR_STOP1:
        MOTOR_L_OFF();
        MOTOR_R_OFF();
        strcpy(motor_label, "STOP");
        if (now - motor_state_tick >= 1000U)
        {
            motor_state = MOTOR_BACK;
            motor_state_tick = now;
        }
        break;

    case MOTOR_BACK:
        MOTOR_L_REV();
        MOTOR_R_REV();
        strcpy(motor_label, "BACK");
        if (now - motor_state_tick >= 2000U)
        {
            motor_state = MOTOR_STOP2;
            motor_state_tick = now;
        }
        break;

    case MOTOR_STOP2:
        MOTOR_L_OFF();
        MOTOR_R_OFF();
        strcpy(motor_label, "STOP");
        if (now - motor_state_tick >= 1000U)
        {
            motor_state = MOTOR_FWD;
            motor_state_tick = now;
        }
        break;

    default:
        motor_state = MOTOR_FWD;
        motor_state_tick = now;
        break;
    }
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

void laser_start_1s(void)
{
    HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_SET);
    laser_active   = 1;
    laser_end_tick = HAL_GetTick() + 1000U;
    snprintf(laser_line, sizeof(laser_line), "Laser: firing");
}

void laser_update(void)
{
    if (laser_active && HAL_GetTick() >= laser_end_tick)
    {
        HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
        laser_active = 0;
        snprintf(laser_line, sizeof(laser_line), "Laser: done");
    }
}

void ESP_Send(char *cmd)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)cmd, strlen(cmd), 1000);
}

void ESP_ClearBuffer(void)
{
    memset(esp_rx, 0, sizeof(esp_rx));
    esp_rx_index = 0;
    esp_rx_done  = 0;
}

void ESP_StartReceiveIT(void)
{
    HAL_UART_Receive_IT(&huart3, &esp_rx_byte, 1);
}

void WiFi_Start(void)
{
    wifi_state      = WIFI_STATE_SEND_AT;
    wifi_state_tick = HAL_GetTick();
    snprintf(wifi_line1, sizeof(wifi_line1), "WiFi: starting");
    snprintf(wifi_line2, sizeof(wifi_line2), "IP: waiting");
}

void WiFi_Task(void)
{
    char  cmd[96];
    char *p;

    switch (wifi_state)
    {
    case WIFI_STATE_IDLE: break;

    case WIFI_STATE_SEND_AT:
        ESP_ClearBuffer();
        ESP_Send("AT\r\n");
        wifi_state      = WIFI_STATE_WAIT_AT;
        wifi_state_tick = HAL_GetTick();
        snprintf(wifi_line1, sizeof(wifi_line1), "WiFi: testing AT");
        break;

    case WIFI_STATE_WAIT_AT:
        if (strstr(esp_rx, "OK"))
            wifi_state = WIFI_STATE_SEND_CWMODE;
        else if (HAL_GetTick() - wifi_state_tick > 2000U)
        {
            wifi_state = WIFI_STATE_FAIL;
            snprintf(wifi_line1, sizeof(wifi_line1), "WiFi: AT timeout");
        }
        break;

    case WIFI_STATE_SEND_CWMODE:
        ESP_ClearBuffer();
        ESP_Send("AT+CWMODE=1\r\n");
        wifi_state      = WIFI_STATE_WAIT_CWMODE;
        wifi_state_tick = HAL_GetTick();
        snprintf(wifi_line1, sizeof(wifi_line1), "WiFi: set STA");
        break;

    case WIFI_STATE_WAIT_CWMODE:
        if (strstr(esp_rx, "OK") || strstr(esp_rx, "no change"))
            wifi_state = WIFI_STATE_SEND_CWJAP;
        else if (HAL_GetTick() - wifi_state_tick > 3000U)
        {
            wifi_state = WIFI_STATE_FAIL;
            snprintf(wifi_line1, sizeof(wifi_line1), "WiFi: mode fail");
        }
        break;

    case WIFI_STATE_SEND_CWJAP:
        ESP_ClearBuffer();
        snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PASS);
        ESP_Send(cmd);
        wifi_state      = WIFI_STATE_WAIT_CWJAP;
        wifi_state_tick = HAL_GetTick();
        snprintf(wifi_line1, sizeof(wifi_line1), "WiFi: connecting");
        snprintf(wifi_line2, sizeof(wifi_line2), "SSID: %s", WIFI_SSID);
        break;

    case WIFI_STATE_WAIT_CWJAP:
        if (strstr(esp_rx, "WIFI CONNECTED") || strstr(esp_rx, "OK") || strstr(esp_rx, "GOT IP"))
            wifi_state = WIFI_STATE_SEND_CIFSR;
        else if (strstr(esp_rx, "FAIL") || strstr(esp_rx, "ERROR"))
        {
            wifi_state = WIFI_STATE_FAIL;
            snprintf(wifi_line1, sizeof(wifi_line1), "WiFi: join fail");
        }
        else if (HAL_GetTick() - wifi_state_tick > 20000U)
        {
            wifi_state = WIFI_STATE_FAIL;
            snprintf(wifi_line1, sizeof(wifi_line1), "WiFi: join timeout");
        }
        break;

    case WIFI_STATE_SEND_CIFSR:
        ESP_ClearBuffer();
        ESP_Send("AT+CIFSR\r\n");
        wifi_state      = WIFI_STATE_WAIT_CIFSR;
        wifi_state_tick = HAL_GetTick();
        snprintf(wifi_line1, sizeof(wifi_line1), "WiFi: connected");
        snprintf(wifi_line2, sizeof(wifi_line2), "IP: reading");
        break;

    case WIFI_STATE_WAIT_CIFSR:
        p = strstr(esp_rx, "STAIP,");
        if (p != NULL)
        {
            char ip_only[24] = {0};
            int  i = 0;
            p = strchr(p, '"');
            if (p != NULL)
            {
                p++;
                while (*p && *p != '"' && i < (int)(sizeof(ip_only) - 1))
                    ip_only[i++] = *p++;
                ip_only[i] = '\0';
                snprintf(last_ip,    sizeof(last_ip),    "IP: %s", ip_only);
                snprintf(wifi_line1, sizeof(wifi_line1), "WiFi: connected");
                snprintf(wifi_line2, sizeof(wifi_line2), "%s", last_ip);
                wifi_state = WIFI_STATE_DONE;
            }
        }
        else if (HAL_GetTick() - wifi_state_tick > 5000U)
        {
            snprintf(wifi_line1, sizeof(wifi_line1), "WiFi: connected");
            snprintf(wifi_line2, sizeof(wifi_line2), "IP: unavailable");
            wifi_state = WIFI_STATE_DONE;
        }
        break;

    case WIFI_STATE_DONE: break;

    case WIFI_STATE_FAIL:
        if (strlen(wifi_line2) == 0)
            snprintf(wifi_line2, sizeof(wifi_line2), "IP: none");
        break;

    default:
        wifi_state = WIFI_STATE_FAIL;
        snprintf(wifi_line1, sizeof(wifi_line1), "WiFi: state error");
        snprintf(wifi_line2, sizeof(wifi_line2), "IP: none");
        break;
    }
}

void LCD_UpdateUI(uint32_t x_raw, uint32_t y_raw, GPIO_PinState k2_now,
                  int32_t temp_raw)
{
    char x_value[32], y_value[32], btn_value[8];
    char last_value[24], laser_value[24];
    char temp_value[24];

    uint32_t x_cV = (x_raw * 330U) / 4095U;
    uint32_t y_cV = (y_raw * 330U) / 4095U;

    snprintf(x_value, sizeof(x_value), "%4lu  %lu.%02luV", x_raw, x_cV/100U, x_cV%100U);
    snprintf(y_value, sizeof(y_value), "%4lu  %lu.%02luV", y_raw, y_cV/100U, y_cV%100U);

    strcpy(btn_value, (k2_now == GPIO_PIN_RESET) ? "ON " : "OFF");

    if (k2_has_been_pressed)
    {
        uint32_t e = HAL_GetTick() - last_k2_press_tick;
        snprintf(last_value, sizeof(last_value), "%lu.%02lus", e/1000U, (e%1000U)/10U);
    }
    else strcpy(last_value, "never");

    if (strncmp(laser_line, "Laser: ", 7) == 0)
        snprintf(laser_value, sizeof(laser_value), "%s", laser_line + 7);
    else
        snprintf(laser_value, sizeof(laser_value), "%s", laser_line);

    if (temp_raw <= -2032)
    {
        strcpy(temp_value, "NO SENSOR");
    }
    else
    {
        int32_t t = temp_raw, neg = 0;
        if (t < 0) { neg = 1; t = -t; }
        int32_t whole = t / 16;
        int32_t frac  = ((t % 16) * 100) / 16;
        if (neg) snprintf(temp_value, sizeof(temp_value), "-%ld.%02ldC", whole, frac);
        else     snprintf(temp_value, sizeof(temp_value),  "%ld.%02ldC", whole, frac);
    }

    if (strcmp(wifi_line1, prev_wifi_line1) != 0)
    {
        LCD_Clear(60, 40, 239, 60, WHITE);
        LCD_DrawString(60, 40, wifi_line1 + 6);
        strcpy(prev_wifi_line1, wifi_line1);
    }
    if (strcmp(wifi_line2, prev_wifi_line2) != 0)
    {
        LCD_Clear(35, 65, 239, 85, WHITE);
        LCD_DrawString(35, 65, (strncmp(wifi_line2,"IP:",3)==0) ? wifi_line2+3 : wifi_line2);
        strcpy(prev_wifi_line2, wifi_line2);
    }
    if (strcmp(x_value, prev_x_value) != 0)
    {
        LCD_Clear(35, 105, 239, 125, WHITE);
        LCD_DrawString(35, 105, x_value);
        strcpy(prev_x_value, x_value);
    }
    if (strcmp(y_value, prev_y_value) != 0)
    {
        LCD_Clear(35, 135, 239, 155, WHITE);
        LCD_DrawString(35, 135, y_value);
        strcpy(prev_y_value, y_value);
    }
    if (strcmp(btn_value, prev_btn_value) != 0)
    {
        LCD_Clear(55, 165, 95, 185, WHITE);
        LCD_DrawString(55, 165, btn_value);
        strcpy(prev_btn_value, btn_value);
    }
    if (strcmp(last_value, prev_last_value) != 0)
    {
        LCD_Clear(140, 165, 239, 185, WHITE);
        LCD_DrawString(140, 165, last_value);
        strcpy(prev_last_value, last_value);
    }
    if (strcmp(laser_value, prev_laser_value) != 0)
    {
        LCD_Clear(75, 195, 239, 215, WHITE);
        LCD_DrawString(75, 195, laser_value);
        strcpy(prev_laser_value, laser_value);
    }
    if (strcmp(temp_value, prev_temp_value) != 0)
    {
        LCD_Clear(75, 225, 239, 245, WHITE);
        LCD_DrawString(75, 225, temp_value);
        strcpy(prev_temp_value, temp_value);
    }
    if (strcmp(motor_label, prev_motor_value) != 0)
    {
        LCD_Clear(75, 253, 239, 274, WHITE);
        LCD_DrawString(75, 253, motor_label);
        strcpy(prev_motor_value, motor_label);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (esp_rx_index < sizeof(esp_rx) - 1)
        {
            esp_rx[esp_rx_index++] = esp_rx_byte;
            esp_rx[esp_rx_index]   = '\0';
        }
        esp_rx_done = 1;
        HAL_UART_Receive_IT(&huart3, &esp_rx_byte, 1);
    }
}
/* USER CODE END 0 */

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

  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc2);

  HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
  MOTOR_L_OFF();
  MOTOR_R_OFF();

  LCD_INIT();
  LCD_Clear(0, 0, 240, 320, WHITE);

  LCD_DrawString(70,  10, "Lai Man To");
  LCD_DrawString(10,  40, "WiFi:");
  LCD_DrawString(10,  65, "IP:");
  LCD_DrawString(10, 105, "X:");
  LCD_DrawString(10, 135, "Y:");
  LCD_DrawString(10, 165, "Btn:");
  LCD_DrawString(100,165, "Last:");
  LCD_DrawString(10, 195, "Laser:");
  LCD_DrawString(10, 225, "Temp:");
  LCD_DrawString(10, 253, "Motor:");

  LCD_DrawString(75, 225, "NO SENSOR");
  strcpy(prev_temp_value, "NO SENSOR");
  LCD_DrawString(75, 253, "FWD");
  strcpy(prev_motor_value, "FWD");

  prev_wifi_line1[0] = '\0';
  prev_wifi_line2[0] = '\0';
  prev_x_value[0]    = '\0';
  prev_y_value[0]    = '\0';
  prev_btn_value[0]  = '\0';
  prev_last_value[0] = '\0';
  prev_laser_value[0]= '\0';

  motor_state_tick = HAL_GetTick();

  ESP_ClearBuffer();
  ESP_StartReceiveIT();
  WiFi_Start();

  snprintf(laser_line, sizeof(laser_line), "Laser: ready");
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint32_t      x_raw  = read_adc1();
      uint32_t      y_raw  = read_adc2();
      GPIO_PinState k2_now = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

      if (HAL_GetTick() - ds18b20_last_tick >= 2000U)
      {
          ds18b20_raw       = DS18B20_ReadRaw();
          ds18b20_last_tick = HAL_GetTick();
      }

      if ((last_k2_state == GPIO_PIN_SET) && (k2_now == GPIO_PIN_RESET))
      {
          HAL_Delay(20);
          if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
          {
              last_k2_press_tick  = HAL_GetTick();
              k2_has_been_pressed = 1;
              laser_start_1s();
          }
      }
      last_k2_state = k2_now;

      laser_update();
      Motor_Task();

      HAL_GPIO_WritePin(LED_LEFT_PORT,  LED_LEFT_PIN,  GPIO_PIN_RESET);
      HAL_GPIO_WritePin(FIRE_SIG_PORT,  FIRE_SIG_PIN,  laser_active ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_RIGHT_PORT, LED_RIGHT_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_FRONT_PORT, LED_FRONT_PIN, GPIO_PIN_RESET);

      WiFi_Task();
      LCD_UpdateUI(x_raw, y_raw, k2_now, ds18b20_raw);

      HAL_Delay(10);
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct       = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct       = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit     = {0};

  RCC_OscInitStruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState        = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState        = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState    = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL      = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                   |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance                   = ADC1;
  hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion       = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }
  sConfig.Channel      = ADC_CHANNEL_10;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc2.Instance                   = ADC2;
  hadc2.Init.ScanConvMode          = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode    = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion       = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK) { Error_Handler(); }
  sConfig.Channel      = ADC_CHANNEL_11;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_I2C2_Init(void)
{
  hi2c2.Instance             = I2C2;
  hi2c2.Init.ClockSpeed      = 100000;
  hi2c2.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1     = 0;
  hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2     = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) { Error_Handler(); }
}

static void MX_USART3_UART_Init(void)
{
  huart3.Instance          = USART3;
  huart3.Init.BaudRate     = 115200;
  huart3.Init.WordLength   = UART_WORDLENGTH_8B;
  huart3.Init.StopBits     = UART_STOPBITS_1;
  huart3.Init.Parity       = UART_PARITY_NONE;
  huart3.Init.Mode         = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,  GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,   GPIO_PIN_SET);
  HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin  = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = LASER_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LASER_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = DS18B20_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DS18B20_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = GPIO_PIN_1;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = GPIO_PIN_12;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB,
      MOT_L_IN1_PIN | MOT_L_IN2_PIN | MOT_R_IN3_PIN | MOT_R_IN4_PIN,
      GPIO_PIN_RESET);
  GPIO_InitStruct.Pin   = MOT_L_IN1_PIN | MOT_L_IN2_PIN |
                          MOT_R_IN3_PIN | MOT_R_IN4_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOA,
      LED_LEFT_PIN | FIRE_SIG_PIN | LED_RIGHT_PIN | LED_FRONT_PIN,
      GPIO_PIN_RESET);
  GPIO_InitStruct.Pin   = LED_LEFT_PIN | FIRE_SIG_PIN |
                          LED_RIGHT_PIN | LED_FRONT_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_FSMC_Init(void)
{
  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  hsram1.Instance  = FSMC_NORSRAM_DEVICE;
  hsram1.Extended  = FSMC_NORSRAM_EXTENDED_DEVICE;
  hsram1.Init.NSBank             = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux     = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType         = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth    = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode    = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode           = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive   = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation     = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal         = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode       = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait   = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst         = FSMC_WRITE_BURST_DISABLE;
  Timing.AddressSetupTime      = 15;
  Timing.AddressHoldTime       = 15;
  Timing.DataSetupTime         = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision           = 16;
  Timing.DataLatency           = 17;
  Timing.AccessMode            = FSMC_ACCESS_MODE_A;

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK) { Error_Handler(); }
  __HAL_AFIO_FSMCNADV_DISCONNECTED();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
