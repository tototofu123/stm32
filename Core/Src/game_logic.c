/*
 * game_logic.c owns the shared combat rules and runtime state for the tank
 * game. It decides car timing, motion limits, laser timing, and the shared
 * values that the UI and other game modes display.
 *
 * Functions in this file:
 * - MODE_Name: returns a readable mode name.
 * - CAR_Code: returns the compact car code.
 * - CAR_Label: returns the full car label.
 * - car_prime_ms: returns the charge time for the active car.
 * - car_fire_ms: returns the active fire duration.
 * - car_cooldown_ms: returns the cooldown time after firing.
 * - car_apply_speed_cap: limits speed by car version.
 * - car_allows_move_while_firing: reports whether movement is allowed while firing.
 * - car_uses_k2_fire: reports whether K2 triggers firing.
 * - car_auto_fire_enabled: reports whether the car auto-fires.
 * - car_turn_delay_ms: returns the turn delay for the active car.
 * - map_range_percent: maps a raw value into a percentage.
 * - Motor_SendCmd: sends a motor command to the ESP bridge.
 * - Fire_SendCmd: sends the fire command to the ESP bridge.
 * - laser_on_press: arms the laser when the trigger is pressed.
 * - laser_on_release: starts laser charging when the trigger is released.
 *
 * Global variables used here include app_state, selected_mode, selected_car,
 * laser_state, laser_line, motion_line, esp_cmd_rx, and motor/laser timing
 * trackers. No classes are used in this C file.
 */
#include "game_logic.h"
#include "peripherals.h"
#include "alerts.h"
#include "seven_seg.h"
#include "mode_2.h"
#include "mode_3.h"
#include "ui.h"
#include <stdio.h>
#include <string.h>

/*
 * =============================================================================
 * GLOBAL STATE VARIABLES - RUNTIME GAME STATE
 * =============================================================================
 * These variables track the current state of the application and gameplay.
 * They are read and written by various modules (main.c, ui.c, mode_*.c, etc.).
 */

/* Navigation and selection state */
app_state_t   app_state = APP_HOME;        // Current app screen context.
game_mode_t   selected_mode = GAME_MODE_1; // User-selected game mode.
car_type_t    selected_car = CAR_V0;       // User-selected vehicle type.

/* Gameplay state during active Mode 1 */
laser_state_t laser_state = LASER_IDLE;    // Laser firing state machine.
uint8_t       touch_ability_shots_needed = 0U;  // Counter for special ability activation.

/* HUD status display strings (updated by this module, read by ui.c) */
char laser_line[32] = "READY";   // Firing status (e.g., "READY", "FIRING", "COOLDOWN").
char motion_line[24] = "STOP 0%"; // Movement status (e.g., "FWD 75%", "TURN").
char esp_cmd_rx[8] = "S000";      // Last response received from ESP module.

/* Movement command state tracking */
uint8_t  fire_cmd_priority = 0U;        // Priority flag for firing queue.
char     current_dir_cmd = 'S';         // Current direction command ('S'=stop, 'F'=forward, 'B'=back, 'L'=left, 'R'=right).
uint32_t last_direction_change_tick = 0U;  // Timestamp of last direction command sent.
uint8_t  turn_delay_active = 0U;        // Flag: car turn delay is active (car type dependent).
char     turn_target_cmd = 'S';         // Target direction after turn delay expires.

/* Timing trackers for motor and laser updates */
uint32_t laser_tick = 0;            // Timestamp of last laser state update.
char     last_motor_cmd = 'S';      // Last direction command sent to ESP (for avoiding redundant sends).
uint8_t  last_motor_speed = 0;      // Last speed value sent to ESP.
uint32_t motor_cmd_tick = 0;        // Timestamp of last motor command transmission.

/*
 * MODE_Name:
 * Converts a game_mode_t enum into a user-readable string for display.
 * Used by menu screens and HUD to label the selected mode.
 */
const char *MODE_Name(game_mode_t mode)
{
    switch (mode)
    {
        case GAME_MODE_1: return "MODE 1";  // Single-player tank duel.
        case GAME_MODE_2: return "MODE 2";  // Drawing challenge.
        case GAME_MODE_3: return "MODE 3";  // Arena multiplayer.
        default:          return "MODE 1";  // Fallback to Mode 1.
    }
}

/*
 * CAR_Code:
 * Converts a car_type_t enum into a compact two-character code (e.g., "V0", "V3").
 * Used in status displays and logs where space is limited.
 */
const char *CAR_Code(car_type_t car)
{
    switch (car)
    {
        case CAR_V0: return "V0";  // Standard car.
        case CAR_V1: return "V1";  // Auto-fire variant.
        case CAR_V2: return "V2";  // Rapid-shot variant.
        case CAR_V3: return "V3";  // Moving-cast variant.
        case CAR_V4: return "V4";  // Forward-speed variant.
        case CAR_V5: return "V5";  // Long-beam variant.
        case CAR_V6: return "V6";  // Gun-platform variant.
        default:     return "V0";  // Fallback to standard.
    }
}

/*
 * CAR_Label:
 * Converts a car_type_t enum into a full descriptive name for UI display.
 * Shown in car-select preview panels and HUD during gameplay.
 */
const char *CAR_Label(car_type_t car)
{
    switch (car)
    {
        case CAR_V0: return "STANDARD";       // Balanced baseline.
        case CAR_V1: return "AUTO FIRE";     // Auto-fires when charged.
        case CAR_V2: return "RAPID SHOT";    // Short fire, fast cooldown.
        case CAR_V3: return "MOVING CAST";   // Can move while firing.
        case CAR_V4: return "FORWARD SPEED"; // Higher forward velocity.
        case CAR_V5: return "LONG BEAM";     // Extended laser range.
        case CAR_V6: return "GUN PLATFORM";  // Stationary turret mode.
        default:     return "STANDARD";      // Fallback to baseline.
    }
}

/*
 * car_prime_ms:
 * Returns the charge-up time (in milliseconds) the user must hold the trigger
 * before the laser beam begins firing. Varies by car type.
 * Car V6 (Gun Platform) has fastest prime, V5 (Long Beam) has slowest.
 */
static uint32_t car_prime_ms(void)
{
    switch (selected_car)
    {
        case CAR_V4: return 800U;     // Forward Speed: 800 ms charge.
        case CAR_V5: return 1200U;    // Long Beam: 1200 ms charge (longest).
        case CAR_V6: return 400U;     // Gun Platform: 400 ms charge (fastest).
        default:     return LASER_PRIME_MS;  // Standard: uses base constant.
    }
}

/*
 * car_fire_ms:
 * Returns the active firing duration (in milliseconds) - how long the laser
 * beam stays on once firing begins. Car V5 has the longest sustained beam.
 */
static uint32_t car_fire_ms(void)
{
    switch (selected_car)
    {
        case CAR_V2: return 500U;     // Rapid Shot: 500 ms fire (shortest).
        case CAR_V4: return 400U;     // Forward Speed: 400 ms fire.
        case CAR_V5: return 1800U;    // Long Beam: 1800 ms fire (longest).
        case CAR_V6: return 400U;     // Gun Platform: 400 ms fire.
        default:     return LASER_FIRE_MS;  // Standard: uses base constant.
    }
}

/*
 * car_cooldown_ms:
 * Returns the cooldown delay (in milliseconds) after a shot fires before the
 * next shot can begin charging. Car V3 (Moving Cast) has longest cooldown
 * as balance for its unique move-while-firing ability.
 */
static uint32_t car_cooldown_ms(void)
{
    switch (selected_car)
    {
        case CAR_V2: return 1500U;    // Rapid Shot: 1500 ms cooldown (short for rapid fire).
        case CAR_V3: return 6000U;    // Moving Cast: 6000 ms cooldown (longest, balances move-while-fire).
        case CAR_V4: return 4000U;    // Forward Speed: 4000 ms cooldown.
        case CAR_V5: return 5500U;    // Long Beam: 5500 ms cooldown.
        case CAR_V6: return 1800U;    // Gun Platform: 1800 ms cooldown.
        default:     return LASER_COOLDOWN_MS;  // Standard: uses base constant.
    }
}

/*
 * car_apply_speed_cap:
 * Constrains movement speed based on car type. Each variant has different
 * acceleration/top-speed characteristics to balance gameplay.
 * Input: cmd (direction), speed_percent (0-100 normalized).
 * Output: capped speed value (0-255) to send to motor controller.
 */
uint8_t car_apply_speed_cap(char cmd, uint8_t speed_percent)
{
    uint16_t max_speed = 100U;  // Default max speed (baseline).
    uint16_t scaled;
    
    // Apply car-specific speed limits.
    switch (selected_car)
    {
        case CAR_V1:
            // Auto Fire: slower movement to balance rapid-fire advantage.
            max_speed = 70U;
            break;
        case CAR_V4:
            // Forward Speed: bonus speed only in forward/diagonal directions.
            if ((cmd == 'F') || (cmd == 'L') || (cmd == 'R'))
                max_speed = 200U;  // Double speed for forward motion.
            break;
        case CAR_V6:
            // Gun Platform: stationary turret, very slow movement.
            max_speed = 60U;
            break;
        default:
            // Standard and all other cars: baseline speed.
            max_speed = 100U;
            break;
    }
    
    // Scale input percentage by max_speed limit, clamping result to 8-bit range.
    scaled = ((uint16_t)speed_percent * max_speed) / 100U;
    if (scaled > 255U) scaled = 255U;  // Clamp to max motor command value.
    return (uint8_t)scaled;
}

/*
 * car_allows_move_while_firing:
 * Checks if the selected car has the special ability to move while its laser
 * is actively firing. Only Car V3 (Moving Cast) has this capability.
 */
static uint8_t car_allows_move_while_firing(void)
{
    // Only Car V3 (Moving Cast) can move while firing.
    return (selected_car == CAR_V3) ? 1U : 0U;
}

/*
 * car_uses_k2_fire:
 * Checks if the car uses K2 button for firing instead of joystick press.
 * Car V3 (Moving Cast) uses K2 to allow joystick for simultaneous movement.
 */
uint8_t car_uses_k2_fire(void)
{
    // Car V3 uses K2 button to enable free joystick movement during firing.
    return (selected_car == CAR_V3) ? 1U : 0U;
}

/*
 * car_auto_fire_enabled:
 * Checks if the car should automatically fire when fully charged.
 * Car V1 (Auto Fire) fires by itself without user releasing trigger.
 */
uint8_t car_auto_fire_enabled(void)
{
    // Only Car V1 (Auto Fire) automatically fires when trigger is held and charge completes.
    return (selected_car == CAR_V1) ? 1U : 0U;
}

/*
 * car_turn_delay_ms:
 * Returns the inertia/turn-delay time (in milliseconds) when changing direction.
 * Some cars turn slower to balance their speed advantages.
 */
static uint32_t car_turn_delay_ms(void)
{
    switch (selected_car)
    {
        case CAR_V2:
            // Rapid Shot: slower turn (500 ms) to balance fast firing.
            return 500U;
        default:
            // Standard and all others: baseline turn delay (200 ms).
            return 200U;
    }
}

/*
 * map_range_percent:
 * Maps a raw analog value (ADC, joystick, etc.) into 0-100 percentage.
 * Handles both forward and reverse ranges (start < end or start > end).
 * Input: value (raw sensor reading), start/end (calibration range).
 * Output: percentage 0-100, clamped.
 */
uint8_t map_range_percent(uint32_t value, uint32_t start, uint32_t end)
{
    uint32_t num, den, pct;
    
    // Guard: invalid range (start == end).
    if (end == start) return 0;
    
    if (end > start) {
        // Normal range: start is minimum, end is maximum.
        if (value <= start) return 0;    // Below minimum: 0%.
        if (value >= end)   return 100;  // Above maximum: 100%.
        // Interpolate in-range value.
        num = value - start;
        den = end - start;
    } else {
        // Inverted range: start is maximum, end is minimum (reversed axis).
        if (value >= start) return 0;    // Above maximum: 0%.
        if (value <= end)   return 100;  // Below minimum: 100%.
        // Interpolate in-range value (reversed).
        num = start - value;
        den = start - end;
    }
    
    // Calculate percentage and clamp to 100.
    pct = (num * 100U) / den;
    if (pct > 100U) pct = 100U;
    return (uint8_t)pct;
}

/*
 * Motor_SendCmd:
 * Transmits a motion command to the ESP motor controller via UART.
 * Includes debouncing: suppresses repeated sends of the same command
 * within MOTOR_CMD_INTERVAL (80 ms) to reduce bus traffic.
 * Input: cmd ('S'=stop, 'F'=forward, 'B'=back, 'L'=left, 'R'=right),
 *        speed (0-255 PWM duty cycle).
 */
void Motor_SendCmd(char cmd, uint8_t speed)
{
    uint32_t now = HAL_GetTick();
    char tx[8];

    // Skip redundant sends: same command within interval.
    if (cmd == last_motor_cmd && speed == last_motor_speed && (now - motor_cmd_tick) < MOTOR_CMD_INTERVAL)
        return;

    // Format command: direction char + 3-digit speed (e.g., "F128").
    snprintf(tx, sizeof(tx), "%c%03u", cmd, speed);
    
    // Only send if not blocked by fire command priority.
    if (!fire_cmd_priority)
        snprintf(esp_cmd_rx, sizeof(esp_cmd_rx), "%s", tx);

    Wifi_SendToClient(0,tx);                    // Transmit via UART to ESP module.
    last_motor_cmd = cmd;          // Remember this command for debouncing.
    last_motor_speed = speed;
    motor_cmd_tick = now;          // Update timestamp for next interval check.
}

/*
 * Fire_SendCmd:
 * Transmits the laser fire control signal (on/off) to the hardware via ESP.
 * Sets fire_cmd_priority flag to prevent motion commands from interrupting
 * a critical fire state change.
 * Input: fire_on (1=fire, 0=stop).
 */
void Fire_SendCmd(uint8_t fire_on)
{
    char tx[8];
    // Mark fire state change as high-priority so motion doesn't override.
    fire_cmd_priority = fire_on ? 1U : 0U;
    // Format fire command: 'T' + 0/1 (e.g., "T001" for fire, "T000" for stop).
    snprintf(tx, sizeof(tx), "T%03u", fire_on ? 1U : 0U);
    snprintf(esp_cmd_rx, sizeof(esp_cmd_rx), "%s", tx);
    Wifi_SendToClient(0,tx);  // Transmit via UART to ESP module.
}

/*
 * laser_on_press:
 * Called when the user presses the fire trigger.
 * Transitions laser from IDLE to ARMED state.
 * Guard: only valid if currently idle (prevents double-presses).
 */
void laser_on_press(void)
{
    // Only transition if laser is currently idle.
    if (laser_state != LASER_IDLE) return;
    laser_state = LASER_ARMED;     // Transition to armed state.
    strcpy(laser_line, "ARMED");   // Update HUD display.
}

/*
 * laser_on_release:
 * Called when the user releases the fire trigger after holding it.
 * Transitions from ARMED to PRIMING state (charging the shot).
 * Records timing reference for charge-time calculation.
 * Guard: only valid if currently armed.
 */
void laser_on_release(void)
{
    // Only transition if laser is currently armed.
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

                // Recharge Special Ability every time we finish a firing cycle
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

void SpecialAbility_ResetCooldown(void)
{
    if (touch_ability_shots_needed == 0) {
        // Can be used during COOLDOWN, PRIMING, or FIRING to reset
        if (laser_state == LASER_COOLDOWN || laser_state == LASER_PRIMING || laser_state == LASER_FIRING) {
            HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
            Fire_SendCmd(0);
            laser_state = LASER_IDLE;
            strcpy(laser_line, "READY");
            fire_cmd_priority = 0U;
            
            // Abort the 7-Segment countdown
            seg_mode = SEG_IDLE;

            // Set cooldown for 10 rounds
            touch_ability_shots_needed = 10;
            Buzzer_BeepLong(); // Sound feedback for ability use
        }
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

static void Drive_Task_Mode1(uint32_t x_raw, uint32_t y_raw)
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

void Game_Router_Task(uint32_t x_raw, uint32_t y_raw, uint8_t k1_click, uint8_t k2_click, uint8_t fire_pressed)
{
    if (app_state == APP_GAME) {
        if (selected_mode == GAME_MODE_1) {
            if (fire_pressed) laser_on_press();
            else laser_on_release();
            
            laser_update();
            RGB_Update_From_State();
            Drive_Task_Mode1(x_raw, y_raw);
        }
        else {
            Motor_SendCmd('S', 0);
        }
    } else {
        Motor_SendCmd('S', 0);
    }
}
