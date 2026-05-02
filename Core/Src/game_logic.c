#include "game_logic.h"
#include "peripherals.h"
#include "alerts.h"
#include "seven_seg.h"
#include <stdio.h>
#include <string.h>

// Global State Variables
app_state_t   app_state = APP_MODE_SELECT;
game_mode_t   selected_mode = GAME_MODE_1;
car_type_t    selected_car = CAR_V0;
laser_state_t laser_state = LASER_IDLE;
uint8_t       touch_ability_shots_needed = 0U;

char laser_line[32] = "READY";
char motion_line[24] = "STOP 0%";
char esp_cmd_rx[8] = "S000";

uint8_t  fire_cmd_priority = 0U;
char     current_dir_cmd = 'S';
uint32_t last_direction_change_tick = 0U;
uint8_t  turn_delay_active = 0U;
char     turn_target_cmd = 'S';

uint32_t laser_tick = 0;
char     last_motor_cmd = 'S';
uint8_t  last_motor_speed = 0;
uint32_t motor_cmd_tick = 0;

const char *MODE_Name(game_mode_t mode)
{
    switch (mode)
    {
        case GAME_MODE_1: return "MODE 1";
        case GAME_MODE_2: return "MODE 2";
        case GAME_MODE_3: return "MODE 3";
        default:          return "MODE 1";
    }
}

const char *CAR_Code(car_type_t car)
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

const char *CAR_Label(car_type_t car)
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

uint8_t car_apply_speed_cap(char cmd, uint8_t speed_percent)
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

uint8_t car_uses_k2_fire(void)
{
    return (selected_car == CAR_V3) ? 1U : 0U;
}

uint8_t car_auto_fire_enabled(void)
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

uint8_t map_range_percent(uint32_t value, uint32_t start, uint32_t end)
{
    uint32_t num, den, pct;
    if (end == start) return 0;
    if (end > start) {
        if (value <= start) return 0;
        if (value >= end)   return 100;
        num = value - start;
        den = end - start;
    } else {
        if (value >= start) return 0;
        if (value <= end)   return 100;
        num = start - value;
        den = start - end;
    }
    pct = (num * 100U) / den;
    if (pct > 100U) pct = 100U;
    return (uint8_t)pct;
}

void Motor_SendCmd(char cmd, uint8_t speed)
{
    uint32_t now = HAL_GetTick();
    char tx[8];

    if (cmd == last_motor_cmd && speed == last_motor_speed && (now - motor_cmd_tick) < MOTOR_CMD_INTERVAL)
        return;

    snprintf(tx, sizeof(tx), "%c%03u", cmd, speed);
    if (!fire_cmd_priority)
        snprintf(esp_cmd_rx, sizeof(esp_cmd_rx), "%s", tx);

    sendAT(tx);
    last_motor_cmd = cmd;
    last_motor_speed = speed;
    motor_cmd_tick = now;
}

void Fire_SendCmd(uint8_t fire_on)
{
    char tx[8];
    fire_cmd_priority = fire_on ? 1U : 0U;
    snprintf(tx, sizeof(tx), "T%03u", fire_on ? 1U : 0U);
    snprintf(esp_cmd_rx, sizeof(esp_cmd_rx), "%s", tx);
    sendAT(tx);
}

void laser_on_press(void)
{
    if (laser_state != LASER_IDLE) return;
    laser_state = LASER_ARMED;
    strcpy(laser_line, "ARMED");
}

void laser_on_release(void)
{
    if (laser_state != LASER_ARMED) return;
    laser_state = LASER_PRIMING;
    laser_tick = HAL_GetTick();
    strcpy(laser_line, "CHARGING");
}

void laser_update(void)
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
            if (elapsed >= prime_ms) {
                HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_SET);
                laser_state = LASER_FIRING;
                laser_tick = now;
                strcpy(laser_line, "FIRING");
                Fire_SendCmd(1);
            } else {
                uint32_t rem = prime_ms - elapsed;
                snprintf(laser_line, sizeof(laser_line), "CHG:%lums", rem);
            }
            break;

        case LASER_FIRING:
            if (elapsed >= fire_ms) {
                HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
                Fire_SendCmd(0);
                laser_state = LASER_COOLDOWN;
                laser_tick = now;
                strcpy(laser_line, "COOLDOWN");
                SEG_StartCooldownCountdown(cooldown_ms);

                if (touch_ability_shots_needed > 0)
                    touch_ability_shots_needed--;
            } else {
                uint32_t rem = fire_ms - elapsed;
                snprintf(laser_line, sizeof(laser_line), "FIRE:%lums", rem);
            }
            break;

        case LASER_COOLDOWN:
            if (elapsed >= cooldown_ms) {
                laser_state = LASER_IDLE;
                strcpy(laser_line, "READY");
                fire_cmd_priority = 0U;
            } else {
                uint32_t rem = cooldown_ms - elapsed;
                snprintf(laser_line, sizeof(laser_line), "CD:%lu.%lus", rem / 1000U, (rem % 1000U) / 100U);
            }
            break;

        default:
            laser_state = LASER_IDLE;
            fire_cmd_priority = 0U;
            break;
    }

    if ((app_state == APP_GAME) && (selected_mode == GAME_MODE_1) && car_auto_fire_enabled() && (laser_state == LASER_IDLE))
    {
        laser_state = LASER_PRIMING;
        laser_tick = now;
        strcpy(laser_line, "AUTO CHARGE");
    }
}

void RGB_Update_From_State(void)
{
    switch (laser_state)
    {
        case LASER_ARMED:    RGB_Set(0, 0, 1); break; // blue
        case LASER_PRIMING:  RGB_Set(1, 1, 0); break; // yellow
        case LASER_FIRING:   RGB_Set(0, 1, 0); break; // green
        case LASER_COOLDOWN: RGB_Set(1, 0, 0); break; // red
        case LASER_IDLE:
        default:             RGB_Set(1, 1, 1); break; // white
    }
}

void Drive_Task(uint32_t x_raw, uint32_t y_raw)
{
    char cmd = 'S';
    uint8_t speed_percent = 0U;
    uint8_t speed_cmd = 0U;
    uint32_t now = HAL_GetTick();
    uint32_t turn_delay = car_turn_delay_ms();

    if (!car_allows_move_while_firing() && ((laser_state == LASER_PRIMING) || (laser_state == LASER_FIRING)))
    {
        snprintf(motion_line, sizeof(motion_line), "LOCK %3u%%", 0U);
        Motor_SendCmd('S', 0);
        current_dir_cmd = 'S';
        turn_delay_active = 0U;
        turn_target_cmd = 'S';
        return;
    }

    if (y_raw < Y_FWD_THRESH_ADC) {
        cmd = 'F';
        speed_percent = map_range_percent(y_raw, Y_FWD_THRESH_ADC, ADC_MIN);
    } else if (x_raw < X_LEFT_THRESH_ADC) {
        cmd = 'L';
        speed_percent = map_range_percent(x_raw, X_LEFT_THRESH_ADC, ADC_MIN);
    } else if (x_raw > X_RIGHT_THRESH_ADC) {
        cmd = 'R';
        speed_percent = map_range_percent(x_raw, X_RIGHT_THRESH_ADC, ADC_MAX);
    } else {
        cmd = 'S';
        speed_percent = 0U;
    }

    speed_cmd = car_apply_speed_cap(cmd, speed_percent);

    if ((current_dir_cmd != 'S') && (cmd != 'S') && (cmd != current_dir_cmd)) {
        if (!turn_delay_active) {
            turn_delay_active = 1U;
            turn_target_cmd = cmd;
            last_direction_change_tick = now;
        }
    }

    if (turn_delay_active) {
        if ((now - last_direction_change_tick) < turn_delay) {
            snprintf(motion_line, sizeof(motion_line), "DELAY %3lums", turn_delay);
            Motor_SendCmd('S', 0);
            return;
        } else {
            turn_delay_active = 0U;
            cmd = turn_target_cmd;
            speed_cmd = car_apply_speed_cap(cmd, speed_percent);
        }
    }

    if (cmd == 'F')      snprintf(motion_line, sizeof(motion_line), "FRONT %3u", speed_cmd);
    else if (cmd == 'L') snprintf(motion_line, sizeof(motion_line), "LEFT  %3u", speed_cmd);
    else if (cmd == 'R') snprintf(motion_line, sizeof(motion_line), "RIGHT %3u", speed_cmd);
    else                 snprintf(motion_line, sizeof(motion_line), "STOP  %3u", 0U);

    current_dir_cmd = cmd;
    Motor_SendCmd(cmd, speed_cmd);
}