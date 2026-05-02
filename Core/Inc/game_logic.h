#ifndef GAME_LOGIC_H
#define GAME_LOGIC_H

#include "main.h"
#include <stdint.h>

// Enums
typedef enum {
    APP_MODE_SELECT = 0,
    APP_MODE_CONFIRM,
    APP_CAR_SELECT,
    APP_CAR_CONFIRM,
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

typedef enum {
    LASER_IDLE = 0,
    LASER_ARMED,
    LASER_PRIMING,
    LASER_FIRING,
    LASER_COOLDOWN
} laser_state_t;

// Tuning Defines
#define LASER_PIN           GPIO_PIN_4
#define LASER_PORT          GPIOC
#define LASER_PRIME_MS      1000U
#define LASER_FIRE_MS       1000U
#define LASER_COOLDOWN_MS   3000U

#define X_LEFT_THRESH_ADC   2234U
#define X_RIGHT_THRESH_ADC  3474U
#define Y_FWD_THRESH_ADC    1200U
#define ADC_MIN             0U
#define ADC_MAX             4095U
#define MOTOR_CMD_INTERVAL  80U

// Extern Variables
extern app_state_t   app_state;
extern game_mode_t   selected_mode;
extern car_type_t    selected_car;
extern laser_state_t laser_state;
extern uint8_t       touch_ability_shots_needed;

extern char          laser_line[32];
extern char          motion_line[24];
extern char          esp_cmd_rx[8];
extern uint8_t       fire_cmd_priority;

extern char          current_dir_cmd;
extern uint8_t       turn_delay_active;
extern char          turn_target_cmd;
extern uint32_t      last_direction_change_tick;

// Public Functions
const char *MODE_Name(game_mode_t mode);
const char *CAR_Code(car_type_t car);
const char *CAR_Label(car_type_t car);
uint8_t map_range_percent(uint32_t value, uint32_t start, uint32_t end);
uint8_t car_apply_speed_cap(char cmd, uint8_t speed_percent);
uint8_t car_uses_k2_fire(void);
uint8_t car_auto_fire_enabled(void);

void Motor_SendCmd(char cmd, uint8_t speed);
void Fire_SendCmd(uint8_t fire_on);
void laser_on_press(void);
void laser_on_release(void);
void laser_update(void);
void RGB_Update_From_State(void);

// Central Game Router
void Game_Router_Task(uint32_t x_raw, uint32_t y_raw, uint8_t k1_click, uint8_t k2_click, uint8_t fire_pressed);

#endif // GAME_LOGIC_H