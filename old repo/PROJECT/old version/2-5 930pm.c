/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Joystick + laser + RGB LED + LCD + UART motor commands
  *                   Added boot menu flow:
  *                   MODE SELECT -> CAR SELECT -> GAME
  *                   Mode 1 = current gameplay
  *                   Mode 2/3 = placeholder
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
typedef enum {
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

typedef enum {
    LASER_IDLE = 0,
    LASER_ARMED,
    LASER_PRIMING,
    LASER_FIRING,
    LASER_COOLDOWN
} laser_state_t;

typedef enum {
    SEG_IDLE = 0,
    SEG_K1_COUNT,
    SEG_K2_SHOW88,
    SEG_JSW_CD,
    SEG_ZERO_HOLD
} seg_mode_t;

typedef enum {
    APP_MODE_SELECT = 0,
    APP_CAR_SELECT,
    APP_GAME
} app_state_t;

typedef enum {
    GAME_MODE_1 = 0,
    GAME_MODE_2,
    GAME_MODE_3
} game_mode_t;

typedef enum {
    CAR_V0 = 0,
    CAR_V1,
    CAR_V2,
    CAR_V3,
    CAR_V4,
    CAR_V5,
    CAR_V6
} car_type_t;
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
#define WIFI_SSID           "ESP_2F0F28"
#define WIFI_PASS           "thereisnospoon"

#define LASER_PIN           GPIO_PIN_4
#define LASER_PORT          GPIOC
#define LASER_PRIME_MS      1000U
#define LASER_FIRE_MS       1000U
#define LASER_COOLDOWN_MS   3000U

#define K1_PIN              GPIO_PIN_0
#define K1_PORT             GPIOA

#define K2_PIN              GPIO_PIN_13
#define K2_PORT             GPIOC

#define JOY_SW_PIN          GPIO_PIN_2
#define JOY_SW_PORT         GPIOC

#define BTN1_PIN            GPIO_PIN_2
#define BTN1_PORT           GPIOA
#define BTN2_PIN            GPIO_PIN_3
#define BTN2_PORT           GPIOA

#define RGB_R_PIN           GPIO_PIN_5
#define RGB_R_PORT          GPIOB
#define RGB_G_PIN           GPIO_PIN_0
#define RGB_G_PORT          GPIOB
#define RGB_B_PIN           GPIO_PIN_1
#define RGB_B_PORT          GPIOB

#define DS18B20_PIN         GPIO_PIN_8
#define DS18B20_PORT        GPIOC

#define X_LEFT_THRESH_ADC   2234U
#define X_RIGHT_THRESH_ADC  3474U
#define Y_FWD_THRESH_ADC    1200U

#define ADC_MIN             0U
#define ADC_MAX             4095U

#define MOTOR_CMD_INTERVAL  80U
#define LCD_FAST_UPDATE_MS  120U
#define LCD_SLOW_UPDATE_MS  350U

#define UI_BG               WHITE
#define UI_HEAD             CYAN
#define UI_BOX_SEL          GREEN
#define UI_BOX_NSEL         YELLOW
#define UI_BOTTOM           MAGENTA
#define UI_PLACEHOLDER      RED

#define LSEG_A_PIN          GPIO_PIN_5
#define LSEG_A_PORT         GPIOA
#define LSEG_B_PIN          GPIO_PIN_6
#define LSEG_B_PORT         GPIOA
#define LSEG_C_PIN          GPIO_PIN_4
#define LSEG_C_PORT         GPIOC
#define LSEG_D_PIN          GPIO_PIN_4
#define LSEG_D_PORT         GPIOA
#define LSEG_E_PIN          GPIO_PIN_7
#define LSEG_E_PORT         GPIOA
#define LSEG_F_PIN          GPIO_PIN_7
#define LSEG_F_PORT         GPIOB
#define LSEG_G_PIN          GPIO_PIN_6
#define LSEG_G_PORT         GPIOB
#define LSEG_DP_PIN         GPIO_PIN_7
#define LSEG_DP_PORT        GPIOE

#define RSEG_A_PIN          GPIO_PIN_14
#define RSEG_A_PORT         GPIOB
#define RSEG_B_PIN          GPIO_PIN_15
#define RSEG_B_PORT         GPIOB
#define RSEG_C_PIN          GPIO_PIN_5
#define RSEG_C_PORT         GPIOC
#define RSEG_D_PIN          GPIO_PIN_7
#define RSEG_D_PORT         GPIOC
#define RSEG_E_PIN          GPIO_PIN_6
#define RSEG_E_PORT         GPIOC
#define RSEG_F_PIN          GPIO_PIN_13
#define RSEG_F_PORT         GPIOB
#define RSEG_G_PIN          GPIO_PIN_12
#define RSEG_G_PORT         GPIOB
/* USER CODE END PD */

#define LCD_TEXT(x, y, s)   LCD_DrawString((x), (y), (unsigned char *)(s))

ADC_HandleTypeDef  hadc1;
ADC_HandleTypeDef  hadc2;
I2C_HandleTypeDef  hi2c2;
UART_HandleTypeDef huart3;
SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
GPIO_PinState last_k1_state  = GPIO_PIN_RESET;
GPIO_PinState last_k2_state  = GPIO_PIN_RESET;
GPIO_PinState last_jsw_state = GPIO_PIN_SET;

app_state_t app_state = APP_MODE_SELECT;
app_state_t last_drawn_state = (app_state_t)255;

game_mode_t selected_mode = GAME_MODE_1;
game_mode_t last_drawn_mode = (game_mode_t)255;

car_type_t selected_car = CAR_V0;
car_type_t last_drawn_car = (car_type_t)255;

GPIO_PinState last_fire_input_state = GPIO_PIN_SET;
char current_dir_cmd = 'S';
uint32_t last_direction_change_tick = 0U;

uint32_t last_jsw_press_tick  = 0;
uint8_t  jsw_has_been_pressed = 0;

laser_state_t laser_state = LASER_IDLE;
uint32_t      laser_tick  = 0;
char          laser_line[32] = "READY";

char     motion_line[24] = "STOP 0%";
char     last_motor_cmd  = 'S';
uint8_t  last_motor_speed = 0;
uint32_t motor_cmd_tick  = 0;

char    esp_rx[256];
uint8_t esp_rx_byte;
volatile uint16_t esp_rx_index = 0;
volatile uint8_t  esp_rx_done  = 0;

char    esp_cmd_rx[8] = "S000";
uint8_t fire_cmd_priority = 0U;

wifi_state_t wifi_state      = WIFI_STATE_IDLE;
uint32_t     wifi_state_tick = 0;
char wifi_line1[32] = "idle";
char wifi_line2[32] = "none";

int32_t  ds18b20_raw       = -2032;
uint32_t ds18b20_last_tick = 0;

seg_mode_t seg_mode = SEG_IDLE;
uint32_t seg_tick = 0;
int seg_tenths = 0;
uint8_t seg_left = 0;
uint8_t seg_right = 0;
uint8_t seg_dp = 0;

uint32_t lcd_fast_tick = 0;
uint32_t lcd_slow_tick = 0;
uint32_t last_k1_event_tick = 0U;
uint32_t last_k2_event_tick = 0U;
uint8_t turn_delay_active = 0U;
char turn_target_cmd = 'S';
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);

static uint32_t read_adc1(void);
static uint32_t read_adc2(void);
static void     RGB_Set(uint8_t r, uint8_t g, uint8_t b);
static void     Motor_SendCmd(char cmd, uint8_t speed);
static void     Fire_SendCmd(uint8_t fire_on);
static uint8_t  map_range_percent(uint32_t value, uint32_t start, uint32_t end);
static void     Drive_Task(uint32_t x_raw, uint32_t y_raw);
static void     laser_on_press(void);
static void     laser_on_release(void);
static void     laser_update(void);
static void     RGB_Update_From_State(void);
static void     LCD_DrawModeSelect(void);
static void     LCD_DrawCarSelect(void);
static void     LCD_DrawGameLayout(void);
static void     LCD_UpdateModeSelection(void);
static void     LCD_UpdateCarSelection(void);
static void     LCD_UpdateGameFast(uint32_t x_raw, uint32_t y_raw);
static void     LCD_UpdateGameSlow(uint8_t fire_pressed);
static void     LCD_ClearTextField(uint16_t x, uint16_t y, uint16_t chars, uint16_t bg);
static const char *MODE_Name(game_mode_t mode)
{
    switch (mode)
    {
        case GAME_MODE_1: return "MODE 1";
        case GAME_MODE_2: return "MODE 2";
        case GAME_MODE_3: return "MODE 3";
        default:          return "MODE 1";
    }
}

static const char *CAR_Code(car_type_t car)
{
    switch (car)
    {
        case CAR_V0: return "V0";
        case CAR_V1: return "V1";
        case CAR_V2: return "V2";
        case CAR_V3: return "V3";
        case CAR_V4: return "V4";
        case CAR_V5: return "V5";
        case CAR_V6: return "V6";
        default:     return "V0";
    }
}

static const char *CAR_Label(car_type_t car)
{
    switch (car)
    {
        case CAR_V0: return "STANDARD";
        case CAR_V1: return "AUTO FIRE";
        case CAR_V2: return "RAPID SHOT";
        case CAR_V3: return "MOVING CAST";
        case CAR_V4: return "FORWARD SPEED";
        case CAR_V5: return "LONG BEAM";
        case CAR_V6: return "GUN PLATFORM";
        default:     return "STANDARD";
    }
}

static uint32_t car_prime_ms(void)
{
    switch (selected_car)
    {
        case CAR_V4: return 800U;
        case CAR_V5: return 1200U;
        case CAR_V6: return 400U;
        default:     return LASER_PRIME_MS;
    }
}

static uint32_t car_fire_ms(void)
{
    switch (selected_car)
    {
        case CAR_V2: return 500U;
        case CAR_V4: return 400U;
        case CAR_V5: return 1800U;
        case CAR_V6: return 400U;
        default:     return LASER_FIRE_MS;
    }
}

static uint32_t car_cooldown_ms(void)
{
    switch (selected_car)
    {
        case CAR_V2: return 1500U;
        case CAR_V3: return 6000U;
        case CAR_V4: return 4000U;
        case CAR_V5: return 5500U;
        case CAR_V6: return 1800U;
        default:     return LASER_COOLDOWN_MS;
    }
}

static uint8_t car_apply_speed_cap(char cmd, uint8_t speed_percent)
{
    uint16_t max_speed = 100U;
    uint16_t scaled;

    switch (selected_car)
    {
        case CAR_V1:
            max_speed = 70U;
            break;

        case CAR_V4:
            if ((cmd == 'F') || (cmd == 'L') || (cmd == 'R'))
                max_speed = 200U;
            break;

        case CAR_V6:
            max_speed = 60U;
            break;

        default:
            max_speed = 100U;
            break;
    }

    scaled = ((uint16_t)speed_percent * max_speed) / 100U;
    if (scaled > 255U) scaled = 255U;
    return (uint8_t)scaled;
}

static uint8_t car_allows_move_while_firing(void)
{
    return (selected_car == CAR_V3) ? 1U : 0U;
}

static uint8_t car_uses_k2_fire(void)
{
    return (selected_car == CAR_V3) ? 1U : 0U;
}

static uint8_t car_auto_fire_enabled(void)
{
    return (selected_car == CAR_V1) ? 1U : 0U;
}

static uint32_t car_turn_delay_ms(void)
{
    switch (selected_car)
    {
        case CAR_V2: return 500U;
        default:     return 200U;
    }
}

static uint32_t read_adc1(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t v = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return v;
}

static uint32_t read_adc2(void)
{
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 10);
    uint32_t v = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);
    return v;
}

static void RGB_Set(uint8_t r, uint8_t g, uint8_t b)
{
    HAL_GPIO_WritePin(RGB_R_PORT, RGB_R_PIN, r ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(RGB_G_PORT, RGB_G_PIN, g ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(RGB_B_PORT, RGB_B_PIN, b ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static uint8_t map_range_percent(uint32_t value, uint32_t start, uint32_t end)
{
    uint32_t num, den, pct;

    if (end == start) return 0;

    if (end > start)
    {
        if (value <= start) return 0;
        if (value >= end)   return 100;
        num = value - start;
        den = end - start;
    }
    else
    {
        if (value >= start) return 0;
        if (value <= end)   return 100;
        num = start - value;
        den = start - end;
    }

    pct = (num * 100U) / den;
    if (pct > 100U) pct = 100U;
    return (uint8_t)pct;
}

static void SEG_WritePin(GPIO_TypeDef *port, uint16_t pin, uint8_t on)
{
    HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void SEG_AllOff(void)
{
    SEG_WritePin(LSEG_A_PORT, LSEG_A_PIN, 0);
    SEG_WritePin(LSEG_B_PORT, LSEG_B_PIN, 0);
    SEG_WritePin(LSEG_C_PORT, LSEG_C_PIN, 0);
    SEG_WritePin(LSEG_D_PORT, LSEG_D_PIN, 0);
    SEG_WritePin(LSEG_E_PORT, LSEG_E_PIN, 0);
    SEG_WritePin(LSEG_F_PORT, LSEG_F_PIN, 0);
    SEG_WritePin(LSEG_G_PORT, LSEG_G_PIN, 0);
    SEG_WritePin(LSEG_DP_PORT, LSEG_DP_PIN, 0);

    SEG_WritePin(RSEG_A_PORT, RSEG_A_PIN, 0);
    SEG_WritePin(RSEG_B_PORT, RSEG_B_PIN, 0);
    SEG_WritePin(RSEG_C_PORT, RSEG_C_PIN, 0);
    SEG_WritePin(RSEG_D_PORT, RSEG_D_PIN, 0);
    SEG_WritePin(RSEG_E_PORT, RSEG_E_PIN, 0);
    SEG_WritePin(RSEG_F_PORT, RSEG_F_PIN, 0);
    SEG_WritePin(RSEG_G_PORT, RSEG_G_PIN, 0);
}

static void SEG_ShowLeft(uint8_t d, uint8_t dp)
{
    static const uint8_t lut[10][7] = {
        {1,1,1,1,1,1,0},
        {0,1,1,0,0,0,0},
        {1,1,0,1,1,0,1},
        {1,1,1,1,0,0,1},
        {0,1,1,0,0,1,1},
        {1,0,1,1,0,1,1},
        {1,0,1,1,1,1,1},
        {1,1,1,0,0,0,0},
        {1,1,1,1,1,1,1},
        {1,1,1,1,0,1,1}
    };

    if (d > 9) d = 0;

    SEG_WritePin(LSEG_A_PORT, LSEG_A_PIN, lut[d][0]);
    SEG_WritePin(LSEG_B_PORT, LSEG_B_PIN, lut[d][1]);
    SEG_WritePin(LSEG_C_PORT, LSEG_C_PIN, lut[d][2]);
    SEG_WritePin(LSEG_D_PORT, LSEG_D_PIN, lut[d][3]);
    SEG_WritePin(LSEG_E_PORT, LSEG_E_PIN, lut[d][4]);
    SEG_WritePin(LSEG_F_PORT, LSEG_F_PIN, lut[d][5]);
    SEG_WritePin(LSEG_G_PORT, LSEG_G_PIN, lut[d][6]);
    SEG_WritePin(LSEG_DP_PORT, LSEG_DP_PIN, dp ? 1 : 0);
}

static void SEG_ShowRight(uint8_t d)
{
    static const uint8_t lut[10][7] = {
        {1,1,1,1,1,1,0},
        {0,1,1,0,0,0,0},
        {1,1,0,1,1,0,1},
        {1,1,1,1,0,0,1},
        {0,1,1,0,0,1,1},
        {1,0,1,1,0,1,1},
        {1,0,1,1,1,1,1},
        {1,1,1,0,0,0,0},
        {1,1,1,1,1,1,1},
        {1,1,1,1,0,1,1}
    };

    if (d > 9) d = 0;

    SEG_WritePin(RSEG_A_PORT, RSEG_A_PIN, lut[d][0]);
    SEG_WritePin(RSEG_B_PORT, RSEG_B_PIN, lut[d][1]);
    SEG_WritePin(RSEG_C_PORT, RSEG_C_PIN, lut[d][2]);
    SEG_WritePin(RSEG_D_PORT, RSEG_D_PIN, lut[d][3]);
    SEG_WritePin(RSEG_E_PORT, RSEG_E_PIN, lut[d][4]);
    SEG_WritePin(RSEG_F_PORT, RSEG_F_PIN, lut[d][5]);
    SEG_WritePin(RSEG_G_PORT, RSEG_G_PIN, lut[d][6]);
}

static void SEG_ShowPair(uint8_t left, uint8_t right, uint8_t dp)
{
    seg_left = left;
    seg_right = right;
    seg_dp = dp;
    SEG_ShowLeft(left, dp);
    SEG_ShowRight(right);
}

static void SEG_ShowTenths(int t)
{
    if (t < 0) t = 0;
    if (t > 99) t = 99;
    SEG_ShowPair((uint8_t)(t / 10), (uint8_t)(t % 10), 1);
}

static void SEG_StartCooldownCountdown(uint32_t cooldown_ms)
{
    seg_mode = SEG_JSW_CD;
    seg_tenths = (int)(cooldown_ms / 100U);
    if (seg_tenths < 0) seg_tenths = 0;
    if (seg_tenths > 99) seg_tenths = 99;
    seg_tick = HAL_GetTick();
    SEG_ShowTenths(seg_tenths);
}

static void SEG_Task(void)
{
    uint32_t now = HAL_GetTick();

    switch (seg_mode)
    {
    case SEG_JSW_CD:
        while ((now - seg_tick) >= 100U)
        {
            seg_tick += 100U;
            if (seg_tenths > 0)
            {
                seg_tenths--;
                SEG_ShowTenths(seg_tenths);
            }
            else
            {
                SEG_ShowTenths(0);
                seg_mode = SEG_ZERO_HOLD;
                seg_tick = now;
                break;
            }
        }
        break;

    case SEG_ZERO_HOLD:
        if ((now - seg_tick) >= 1000U)
        {
            SEG_ShowPair(0, 0, 0);
            seg_mode = SEG_IDLE;
        }
        break;

    case SEG_IDLE:
    default:
        break;
    }
}

static void Motor_SendCmd(char cmd, uint8_t speed)
{
    uint32_t now = HAL_GetTick();
    char tx[8];

    if (cmd == last_motor_cmd &&
        speed == last_motor_speed &&
        (now - motor_cmd_tick) < MOTOR_CMD_INTERVAL)
    {
        return;
    }

    snprintf(tx, sizeof(tx), "%c%03u", cmd, speed);

    if (!fire_cmd_priority)
    {
        snprintf(esp_cmd_rx, sizeof(esp_cmd_rx), "%s", tx);
    }

    sendAT(tx);

    last_motor_cmd = cmd;
    last_motor_speed = speed;
    motor_cmd_tick = now;
}

static void Fire_SendCmd(uint8_t fire_on)
{
    char tx[8];

    fire_cmd_priority = fire_on ? 1U : 0U;
    snprintf(tx, sizeof(tx), "T%03u", fire_on ? 1U : 0U);
    snprintf(esp_cmd_rx, sizeof(esp_cmd_rx), "%s", tx);
    sendAT(tx);
}

static void laser_on_press(void)
{
    if (laser_state != LASER_IDLE) return;
    laser_state = LASER_ARMED;
    strcpy(laser_line, "ARMED");
}

static void laser_on_release(void)
{
    if (laser_state != LASER_ARMED) return;

    laser_state = LASER_PRIMING;
    laser_tick = HAL_GetTick();
    strcpy(laser_line, "CHARGING");
}

static void laser_update(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - laser_tick;
    uint32_t prime_ms = car_prime_ms();
    uint32_t fire_ms = car_fire_ms();
    uint32_t cooldown_ms = car_cooldown_ms();

    switch (laser_state)
    {
        case LASER_IDLE:
            strcpy(laser_line, "READY");
            fire_cmd_priority = 0U;
            break;

        case LASER_ARMED:
            strcpy(laser_line, "ARMED");
            break;

        case LASER_PRIMING:
            if (elapsed >= prime_ms)
            {
                HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_SET);
                laser_state = LASER_FIRING;
                laser_tick = now;
                strcpy(laser_line, "FIRING");
                Fire_SendCmd(1);
            }
            else
            {
                uint32_t rem = prime_ms - elapsed;
                snprintf(laser_line, sizeof(laser_line), "CHG:%lums", rem);
            }
            break;

        case LASER_FIRING:
            if (elapsed >= fire_ms)
            {
                HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
                Fire_SendCmd(0);
                laser_state = LASER_COOLDOWN;
                laser_tick = now;
                strcpy(laser_line, "COOLDOWN");
                SEG_StartCooldownCountdown(cooldown_ms);
            }
            else
            {
                uint32_t rem = fire_ms - elapsed;
                snprintf(laser_line, sizeof(laser_line), "FIRE:%lums", rem);
            }
            break;

        case LASER_COOLDOWN:
            if (elapsed >= cooldown_ms)
            {
                laser_state = LASER_IDLE;
                strcpy(laser_line, "READY");
                fire_cmd_priority = 0U;
            }
            else
            {
                uint32_t rem = cooldown_ms - elapsed;
                snprintf(laser_line, sizeof(laser_line), "CD:%lu.%lus",
                         rem / 1000U, (rem % 1000U) / 100U);
            }
            break;

        default:
            laser_state = LASER_IDLE;
            fire_cmd_priority = 0U;
            break;
    }

    if ((app_state == APP_GAME) &&
        (selected_mode == GAME_MODE_1) &&
        car_auto_fire_enabled() &&
        (laser_state == LASER_IDLE))
    {
        laser_state = LASER_PRIMING;
        laser_tick = now;
        strcpy(laser_line, "AUTO CHARGE");
    }
}

static void RGB_Update_From_State(void)
{
    switch (laser_state)
    {
        case LASER_ARMED:
            RGB_Set(0, 0, 1);
            break;

        case LASER_PRIMING:
            RGB_Set(1, 1, 0);
            break;

        case LASER_FIRING:
            RGB_Set(0, 1, 0);
            break;

        case LASER_COOLDOWN:
            RGB_Set(1, 0, 0);
            break;

        case LASER_IDLE:
        default:
            RGB_Set(1, 1, 1);
            break;
    }
}

static void Drive_Task(uint32_t x_raw, uint32_t y_raw)
{
    char cmd = 'S';
    uint8_t speed_percent = 0U;
    uint8_t speed_cmd = 0U;
    uint32_t now = HAL_GetTick();
    uint32_t turn_delay = car_turn_delay_ms();

    if (!car_allows_move_while_firing() &&
        ((laser_state == LASER_PRIMING) || (laser_state == LASER_FIRING)))
    {
        snprintf(motion_line, sizeof(motion_line), "LOCK %3u%%", 0U);
        Motor_SendCmd('S', 0);
        current_dir_cmd = 'S';
        turn_delay_active = 0U;
        turn_target_cmd = 'S';
        return;
    }

    if (y_raw < Y_FWD_THRESH_ADC)
    {
        cmd = 'F';
        speed_percent = map_range_percent(y_raw, Y_FWD_THRESH_ADC, ADC_MIN);
    }
    else if (x_raw < X_LEFT_THRESH_ADC)
    {
        cmd = 'L';
        speed_percent = map_range_percent(x_raw, X_LEFT_THRESH_ADC, ADC_MIN);
    }
    else if (x_raw > X_RIGHT_THRESH_ADC)
    {
        cmd = 'R';
        speed_percent = map_range_percent(x_raw, X_RIGHT_THRESH_ADC, ADC_MAX);
    }
    else
    {
        cmd = 'S';
        speed_percent = 0U;
    }

    speed_cmd = car_apply_speed_cap(cmd, speed_percent);

    if ((current_dir_cmd != 'S') && (cmd != 'S') && (cmd != current_dir_cmd))
    {
        if (!turn_delay_active)
        {
            turn_delay_active = 1U;
            turn_target_cmd = cmd;
            last_direction_change_tick = now;
        }
    }

    if (turn_delay_active)
    {
        if ((now - last_direction_change_tick) < turn_delay)
        {
            snprintf(motion_line, sizeof(motion_line), "DELAY %3lums", turn_delay);
            Motor_SendCmd('S', 0);
            return;
        }
        else
        {
            turn_delay_active = 0U;
            cmd = turn_target_cmd;
            speed_cmd = car_apply_speed_cap(cmd, speed_percent);
        }
    }

    if (cmd == 'F')
        snprintf(motion_line, sizeof(motion_line), "FRONT %3u", speed_cmd);
    else if (cmd == 'L')
        snprintf(motion_line, sizeof(motion_line), "LEFT  %3u", speed_cmd);
    else if (cmd == 'R')
        snprintf(motion_line, sizeof(motion_line), "RIGHT %3u", speed_cmd);
    else
        snprintf(motion_line, sizeof(motion_line), "STOP  %3u", 0U);

    current_dir_cmd = cmd;
    Motor_SendCmd(cmd, speed_cmd);
}

static void LCD_ClearTextField(uint16_t x, uint16_t y, uint16_t chars, uint16_t bg)
{
    LCD_Clear(x, y, chars * WIDTH_EN_CHAR, HEIGHT_EN_CHAR, bg);
}

static void LCD_DrawModeSelect(void)
{
    LCD_Clear(0, 0, 240, 320, UI_BG);

    LCD_Clear(0, 0, 240, 6, UI_HEAD);
    LCD_TEXT(10, 12, "SELECT MODE");
    LCD_TEXT(10, 32, "K1:NEXT   K2:CONFIRM");

    LCD_Clear(20, 70, 200, 40, (selected_mode == GAME_MODE_1) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(80, 82, "MODE 1");

    LCD_Clear(20, 125, 200, 40, (selected_mode == GAME_MODE_2) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(80, 137, "MODE 2");

    LCD_Clear(20, 180, 200, 40, (selected_mode == GAME_MODE_3) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(80, 192, "MODE 3");

    LCD_Clear(0, 250, 240, 70, UI_BOTTOM);
    LCD_TEXT(10, 260, "MODE 1 = PLAY NOW");
    LCD_TEXT(10, 280, "MODE 2/3 = PLACEHOLDER");
}

static void LCD_UpdateModeSelection(void)
{
    LCD_Clear(20, 70, 200, 40, (selected_mode == GAME_MODE_1) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(80, 82, "MODE 1");

    LCD_Clear(20, 125, 200, 40, (selected_mode == GAME_MODE_2) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(80, 137, "MODE 2");

    LCD_Clear(20, 180, 200, 40, (selected_mode == GAME_MODE_3) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(80, 192, "MODE 3");
}

static void LCD_DrawCarSelect(void)
{
    char line[32];

    LCD_Clear(0, 0, 240, 320, UI_BG);

    LCD_Clear(0, 0, 240, 6, UI_HEAD);
    LCD_TEXT(10, 10, "SELECT CAR");
    LCD_TEXT(10, 28, "K1:NEXT K2:START");

    snprintf(line, sizeof(line), "MODE:%s", MODE_Name(selected_mode));
    LCD_TEXT(10, 46, line);

    LCD_Clear(14,  64, 212, 20, (selected_car == CAR_V0) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 68, "V0 STANDARD");

    LCD_Clear(14,  88, 212, 20, (selected_car == CAR_V1) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 92, "V1 AUTO FIRE");

    LCD_Clear(14, 112, 212, 20, (selected_car == CAR_V2) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 116, "V2 RAPID SHOT");

    LCD_Clear(14, 136, 212, 20, (selected_car == CAR_V3) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 140, "V3 MOVING CAST");

    LCD_Clear(14, 160, 212, 20, (selected_car == CAR_V4) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 164, "V4 FORWARD SPD");

    LCD_Clear(14, 184, 212, 20, (selected_car == CAR_V5) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 188, "V5 LONG BEAM");

    LCD_Clear(14, 208, 212, 20, (selected_car == CAR_V6) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 212, "V6 GUN PLATFORM");

    LCD_Clear(0, 246, 240, 74, UI_BOTTOM);
    LCD_TEXT(10, 256, "CAR:");
    LCD_TEXT(60, 256, (char *)CAR_Code(selected_car));
    LCD_TEXT(10, 278, "TYPE:");
    LCD_TEXT(60, 278, (char *)CAR_Label(selected_car));
}

static void LCD_UpdateCarSelection(void)
{
    LCD_Clear(14,  64, 212, 20, (selected_car == CAR_V0) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 68, "V0 STANDARD");

    LCD_Clear(14,  88, 212, 20, (selected_car == CAR_V1) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 92, "V1 AUTO FIRE");

    LCD_Clear(14, 112, 212, 20, (selected_car == CAR_V2) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 116, "V2 RAPID SHOT");

    LCD_Clear(14, 136, 212, 20, (selected_car == CAR_V3) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 140, "V3 MOVING CAST");

    LCD_Clear(14, 160, 212, 20, (selected_car == CAR_V4) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 164, "V4 FORWARD SPD");

    LCD_Clear(14, 184, 212, 20, (selected_car == CAR_V5) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 188, "V5 LONG BEAM");

    LCD_Clear(14, 208, 212, 20, (selected_car == CAR_V6) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 212, "V6 GUN PLATFORM");

    LCD_ClearTextField(60, 256, 8, UI_BOTTOM);
    LCD_TEXT(60, 256, (char *)CAR_Code(selected_car));

    LCD_ClearTextField(60, 278, 20, UI_BOTTOM);
    LCD_TEXT(60, 278, (char *)CAR_Label(selected_car));
}

static void LCD_DrawGameLayout(void)
{
    LCD_Clear(0, 0, 240, 320, UI_BG);

    LCD_Clear(0, 0, 240, 6, UI_HEAD);

    LCD_TEXT(10, 10,  "WiFi:");
    LCD_TEXT(10, 30,  "IP:");
    LCD_TEXT(10, 50,  "Mode:");
    LCD_TEXT(10, 70,  "Direction:");
    LCD_TEXT(10, 90,  "Speed:");
    LCD_TEXT(10, 110, "Button:");
    LCD_TEXT(10, 130, "Laser:");
    LCD_TEXT(10, 150, "Motion:");
    LCD_TEXT(10, 170, "ESP:");

    LCD_Clear(0, 215, 240, 105, UI_BOTTOM);
    LCD_TEXT(10, 225, "Car:");
    LCD_TEXT(10, 245, "Car Type:");
}

static void LCD_UpdateGameFast(uint32_t x_raw, uint32_t y_raw)
{
    char dir_str[16];
    char spd_str[16];

    uint8_t speed = 0;

    if (selected_mode != GAME_MODE_1)
    {
        strcpy(dir_str, "PLACEHOLDER");
        strcpy(spd_str, "---");
    }
    else
    {
        if (y_raw < Y_FWD_THRESH_ADC)
        {
            strcpy(dir_str, "FORWARD");
            speed = map_range_percent(y_raw, Y_FWD_THRESH_ADC, ADC_MIN);
        }
        else if (x_raw < X_LEFT_THRESH_ADC)
        {
            strcpy(dir_str, "LEFT");
            speed = map_range_percent(x_raw, X_LEFT_THRESH_ADC, ADC_MIN);
        }
        else if (x_raw > X_RIGHT_THRESH_ADC)
        {
            strcpy(dir_str, "RIGHT");
            speed = map_range_percent(x_raw, X_RIGHT_THRESH_ADC, ADC_MAX);
        }
        else
        {
            strcpy(dir_str, "STOP");
            speed = 0;
        }

        speed = car_apply_speed_cap(
            (dir_str[0] == 'F') ? 'F' :
            (dir_str[0] == 'L') ? 'L' :
            (dir_str[0] == 'R') ? 'R' : 'S',
            speed
        );
        snprintf(spd_str, sizeof(spd_str), "%3u", speed);
    }

    LCD_ClearTextField(60, 10, 20, UI_BG);
    LCD_TEXT(60, 10, wifi_line1);

    LCD_ClearTextField(40, 30, 24, UI_BG);
    LCD_TEXT(40, 30, wifi_line2);

    LCD_ClearTextField(50, 50, 12, UI_BG);
    LCD_TEXT(50, 50, (char *)MODE_Name(selected_mode));

    LCD_ClearTextField(90, 70, 14, UI_BG);
    LCD_TEXT(90, 70, dir_str);

    LCD_ClearTextField(70, 90, 10, UI_BG);
    LCD_TEXT(70, 90, spd_str);
}

static void LCD_UpdateGameSlow(uint8_t fire_pressed)
{
    char btn_str[16];
    char laser_disp[32];
    char motion_disp[24];
    char esp_disp[8];

    if (fire_pressed) strcpy(btn_str, "PRESSED");
    else              strcpy(btn_str, "RELEASE");

    snprintf(laser_disp, sizeof(laser_disp), "%s", laser_line);
    snprintf(motion_disp, sizeof(motion_disp), "%s", motion_line);
    snprintf(esp_disp, sizeof(esp_disp), "%s", esp_cmd_rx);

    LCD_ClearTextField(70, 110, 12, UI_BG);
    LCD_TEXT(70, 110, btn_str);

    LCD_ClearTextField(60, 130, 20, UI_BG);
    LCD_TEXT(60, 130, laser_disp);

    LCD_ClearTextField(70, 150, 20, UI_BG);
    LCD_TEXT(70, 150, motion_disp);

    LCD_ClearTextField(50, 170, 8, UI_BG);
    LCD_TEXT(50, 170, esp_disp);

    LCD_ClearTextField(50, 225, 8, UI_BOTTOM);
    LCD_TEXT(50, 225, (char *)CAR_Code(selected_car));

    LCD_ClearTextField(80, 245, 20, UI_BOTTOM);
    LCD_TEXT(80, 245, (char *)CAR_Label(selected_car));
}

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

static int32_t DS18B20_ReadRaw(void)
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
    if (raw == 0x0550) return -2032;

    return (int32_t)raw;
}

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

    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADCEx_Calibration_Start(&hadc2);

    HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
    RGB_Set(1, 1, 1);
    SEG_AllOff();
    SEG_ShowPair(0, 0, 0);

    LCD_INIT();
    WifiSetUp();
    HAL_UART_Receive_IT(&huart3, &esp_rx_byte, 1);

    while (1)
    {
        uint32_t      now      = HAL_GetTick();
        uint32_t      x_raw    = read_adc1();
        uint32_t      y_raw    = read_adc2();
        GPIO_PinState k1_now   = HAL_GPIO_ReadPin(K1_PORT, K1_PIN);
        GPIO_PinState k2_now   = HAL_GPIO_ReadPin(K2_PORT, K2_PIN);
        GPIO_PinState jsw_now  = HAL_GPIO_ReadPin(JOY_SW_PORT, JOY_SW_PIN);
        GPIO_PinState btn1_now = HAL_GPIO_ReadPin(BTN1_PORT, BTN1_PIN);
        GPIO_PinState btn2_now = HAL_GPIO_ReadPin(BTN2_PORT, BTN2_PIN);
        uint8_t       k1_click = 0U;
        uint8_t       k2_click = 0U;
        uint8_t       fire_pressed = 0U;

        (void)btn1_now;
        (void)btn2_now;

        if ((last_k1_state == GPIO_PIN_RESET) && (k1_now == GPIO_PIN_SET))
        {
            HAL_Delay(20);
            if (HAL_GPIO_ReadPin(K1_PORT, K1_PIN) == GPIO_PIN_SET)
                k1_click = 1U;
        }
        last_k1_state = k1_now;

        if ((last_k2_state == GPIO_PIN_RESET) && (k2_now == GPIO_PIN_SET))
        {
            HAL_Delay(20);
            if (HAL_GPIO_ReadPin(K2_PORT, K2_PIN) == GPIO_PIN_SET)
                k2_click = 1U;
        }
        last_k2_state = k2_now;

        if (app_state != last_drawn_state)
        {
            if (app_state == APP_MODE_SELECT)
            {
                LCD_DrawModeSelect();
                last_drawn_mode = (game_mode_t)255;
            }
            else if (app_state == APP_CAR_SELECT)
            {
                LCD_DrawCarSelect();
                last_drawn_car = (car_type_t)255;
            }
            else
            {
                LCD_DrawGameLayout();
                lcd_fast_tick = 0U;
                lcd_slow_tick = 0U;
                current_dir_cmd = 'S';
                last_direction_change_tick = 0U;
                last_fire_input_state = car_uses_k2_fire() ? GPIO_PIN_RESET : GPIO_PIN_SET;
            }

            last_drawn_state = app_state;
        }

        if (app_state == APP_MODE_SELECT)
        {
            HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
            laser_state = LASER_IDLE;
            fire_cmd_priority = 0U;
            strcpy(laser_line, "READY");
            strcpy(motion_line, "STOP  0%");
            Motor_SendCmd('S', 0);
            RGB_Set(1, 1, 1);

            if (selected_mode != last_drawn_mode)
            {
                LCD_UpdateModeSelection();
                last_drawn_mode = selected_mode;
            }

            if (k1_click)
            {
                selected_mode = (game_mode_t)(((uint8_t)selected_mode + 1U) % 3U);
            }

            if (k2_click)
            {
                selected_car = CAR_V0;
                app_state = APP_CAR_SELECT;
            }
        }
        else if (app_state == APP_CAR_SELECT)
        {
            HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
            laser_state = LASER_IDLE;
            fire_cmd_priority = 0U;
            strcpy(laser_line, "READY");
            strcpy(motion_line, "STOP  0%");
            Motor_SendCmd('S', 0);
            RGB_Set(1, 1, 1);

            if (selected_car != last_drawn_car)
            {
                LCD_UpdateCarSelection();
                last_drawn_car = selected_car;
            }

            if (k1_click)
            {
                selected_car = (car_type_t)(((uint8_t)selected_car + 1U) % 7U);
            }

            if (k2_click)
            {
                app_state = APP_GAME;
            }
        }
        else
        {
            if (selected_mode == GAME_MODE_1)
            {
                if (car_uses_k2_fire())
                {
                    if ((last_fire_input_state == GPIO_PIN_RESET) && (k2_now == GPIO_PIN_SET))
                    {
                        HAL_Delay(20);
                        if (HAL_GPIO_ReadPin(K2_PORT, K2_PIN) == GPIO_PIN_SET)
                        {
                            laser_on_press();
                        }
                    }

                    if ((last_fire_input_state == GPIO_PIN_SET) && (k2_now == GPIO_PIN_RESET))
                    {
                        HAL_Delay(20);
                        if (HAL_GPIO_ReadPin(K2_PORT, K2_PIN) == GPIO_PIN_RESET)
                        {
                            laser_on_release();
                        }
                    }

                    last_fire_input_state = k2_now;
                    fire_pressed = (k2_now == GPIO_PIN_SET) ? 1U : 0U;
                }
                else
                {
                    if ((last_fire_input_state == GPIO_PIN_SET) && (jsw_now == GPIO_PIN_RESET))
                    {
                        HAL_Delay(20);
                        if (HAL_GPIO_ReadPin(JOY_SW_PORT, JOY_SW_PIN) == GPIO_PIN_RESET)
                        {
                            last_jsw_press_tick  = HAL_GetTick();
                            jsw_has_been_pressed = 1U;
                            laser_on_press();
                        }
                    }

                    if ((last_fire_input_state == GPIO_PIN_RESET) && (jsw_now == GPIO_PIN_SET))
                    {
                        HAL_Delay(20);
                        if (HAL_GPIO_ReadPin(JOY_SW_PORT, JOY_SW_PIN) == GPIO_PIN_SET)
                        {
                            laser_on_release();
                        }
                    }

                    last_fire_input_state = jsw_now;
                    fire_pressed = (jsw_now == GPIO_PIN_RESET) ? 1U : 0U;
                }

                if (HAL_GetTick() - ds18b20_last_tick >= 2000U)
                {
                    ds18b20_raw       = DS18B20_ReadRaw();
                    ds18b20_last_tick = HAL_GetTick();
                }

                laser_update();
                RGB_Update_From_State();
                SEG_Task();
                Drive_Task(x_raw, y_raw);

                if ((now - lcd_fast_tick) >= LCD_FAST_UPDATE_MS)
                {
                    lcd_fast_tick = now;
                    LCD_UpdateGameFast(x_raw, y_raw);
                }

                if ((now - lcd_slow_tick) >= LCD_SLOW_UPDATE_MS)
                {
                    lcd_slow_tick = now;
                    LCD_UpdateGameSlow(fire_pressed);
                }
            }
            else
            {
                HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
                laser_state = LASER_IDLE;
                fire_cmd_priority = 0U;
                strcpy(laser_line, "MODE TODO");
                snprintf(motion_line, sizeof(motion_line), "%s WAIT", MODE_Name(selected_mode));
                Motor_SendCmd('S', 0);
                RGB_Set(1, 1, 0);
                SEG_AllOff();

                if ((now - lcd_fast_tick) >= LCD_FAST_UPDATE_MS)
                {
                    lcd_fast_tick = now;
                    LCD_UpdateGameFast(x_raw, y_raw);
                }

                if ((now - lcd_slow_tick) >= LCD_SLOW_UPDATE_MS)
                {
                    lcd_slow_tick = now;
                    LCD_UpdateGameSlow(0U);
                }
            }
        }

        HAL_Delay(10);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef       o = {0};
    RCC_ClkInitTypeDef       c = {0};
    RCC_PeriphCLKInitTypeDef p = {0};

    o.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    o.HSEState       = RCC_HSE_ON;
    o.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    o.HSIState       = RCC_HSI_ON;
    o.PLL.PLLState   = RCC_PLL_ON;
    o.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    o.PLL.PLLMUL     = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&o) != HAL_OK) Error_Handler();

    c.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    c.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV2;
    c.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_2) != HAL_OK) Error_Handler();

    p.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    p.AdcClockSelection    = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&p) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef s = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    s.Channel      = ADC_CHANNEL_10;
    s.Rank         = ADC_REGULAR_RANK_1;
    s.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &s) != HAL_OK) Error_Handler();
}

static void MX_ADC2_Init(void)
{
    ADC_ChannelConfTypeDef s = {0};

    hadc2.Instance                   = ADC2;
    hadc2.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc2.Init.ContinuousConvMode    = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion       = 1;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) Error_Handler();

    s.Channel      = ADC_CHANNEL_11;
    s.Rank         = ADC_REGULAR_RANK_1;
    s.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc2, &s) != HAL_OK) Error_Handler();
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
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) Error_Handler();
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

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);

    g.Pin = GPIO_PIN_12;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOD, &g);

    g.Pin = GPIO_PIN_1 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOE, &g);

    g.Pin = LASER_PIN;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LASER_PORT, &g);

    g.Pin = RGB_R_PIN | RGB_G_PIN | RGB_B_PIN | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    g.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &g);

    g.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_13;
    HAL_GPIO_Init(GPIOC, &g);

    g.Pin  = JOY_SW_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(JOY_SW_PORT, &g);

    g.Pin  = BTN1_PIN | BTN2_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BTN1_PORT, &g);

    g.Pin  = K1_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(K1_PORT, &g);

    g.Pin  = K2_PIN;
    HAL_GPIO_Init(K2_PORT, &g);

    g.Pin  = DS18B20_PIN;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DS18B20_PORT, &g);
}

static void MX_FSMC_Init(void)
{
    FSMC_NORSRAM_TimingTypeDef t = {0};

    hsram1.Instance = FSMC_NORSRAM_DEVICE;
    hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
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

    t.AddressSetupTime      = 15;
    t.AddressHoldTime       = 15;
    t.DataSetupTime         = 255;
    t.BusTurnAroundDuration = 15;
    t.CLKDivision           = 16;
    t.DataLatency           = 17;
    t.AccessMode            = FSMC_ACCESS_MODE_A;

    if (HAL_SRAM_Init(&hsram1, &t, NULL) != HAL_OK) Error_Handler();

    __HAL_AFIO_FSMCNADV_DISCONNECTED();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
