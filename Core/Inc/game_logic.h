#ifndef GAME_LOGIC_H
#define GAME_LOGIC_H

/*
 * =============================================================================
 * GAME_LOGIC.H - SHARED GAME STATE AND CORE GAMEPLAY RULES
 * =============================================================================
 *
 * This header declares all enums, global state, and public functions that
 * game_logic.c owns and that other modules depend on.
 *
 * Responsibility (game_logic.c implementation):
 * - Define global game state: which mode/car is selected, laser state, etc.
 * - Implement timing and damage rules that depend on car type.
 * - Provide car lookup functions (speed, fire duration, cooldown, etc.).
 * - Handle ESP/motor command generation for driving control.
 * - Manage laser firing state machine (armed, priming, firing, cooldown).
 * - Route gameplay logic for Mode 1 (Game_Router_Task).
 *
 * Key enums defined here:
 * - app_state_t: Navigation states (HOME, SETTINGS, MODE_SELECT, CAR_SELECT, GAME, etc.).
 * - game_mode_t: Gameplay type (MODE_1, MODE_2, MODE_3).
 * - car_type_t: Vehicle variant (V0-V6, each with different laser characteristics).
 * - laser_state_t: Firing state machine (IDLE, ARMED, PRIMING, FIRING, COOLDOWN).
 *
 * Global variables exported:
 * - app_state: current navigation/app state.
 * - selected_mode, selected_car: user choices.
 * - laser_state, laser_line, motion_line: shared gameplay HUD/status strings.
 * - esp_cmd_rx: UART response buffer from ESP module.
 * - current_dir_cmd, turn_delay_active: movement state tracking.
 *
 * Data flow:
 * - main.c drives the main loop and uses game_logic.c to execute gameplay.
 * - ui.c queries game_logic state to draw screens and HUD.
 * - mode_2.c and mode_3.c import these enums and may query some globals.
 * - peripherals.c sends motor/fire commands via Motor_SendCmd/Fire_SendCmd.
 *
 * No C++ classes; pure C state and functions.
 * =============================================================================
 */

#include "main.h"
#include <stdint.h>

/*
 * =============================================================================
 * APPLICATION STATE MACHINE ENUMERATION
 * =============================================================================
 * Each state corresponds to one screen or gameplay context.
 */
typedef enum {
    APP_HOME = 0,
    APP_SETTINGS,
    APP_WIFI_SETTINGS,
    APP_WIFI_KEYBOARD,
    APP_MODE_SELECT,
    APP_MODE_CONFIRM,
    APP_CAR_SELECT,
    APP_CAR_CONFIRM,
    APP_GAME,
    APP_MODE_3  // Virtual Arena / Tank Game (reserved/unused currently)
} app_state_t;
/*
 * app_state_t values:
 *   APP_HOME: title/landing screen, shows START BATTLE and PREFERENCES buttons.
 *   APP_SETTINGS: 6-option settings menu (theme, audio, colorblind, LED, 7-seg, font).
 *   APP_WIFI_SETTINGS/KEYBOARD: WiFi configuration (currently disabled).
 *   APP_MODE_SELECT: choose mode 1, 2, or 3.
 *   APP_MODE_CONFIRM: confirmation popup after mode selection.
 *   APP_CAR_SELECT: choose vehicle V0-V6 (Mode 1 only).
 *   APP_CAR_CONFIRM: confirmation popup after car selection.
 *   APP_GAME: active gameplay (branches to Mode1/2/3 logic).
 */

/*
 * =============================================================================
 * GAMEPLAY MODE ENUMERATION
 * =============================================================================
 */
typedef enum {
    GAME_MODE_1 = 0,  // Single-player tank duel (user vs ESP-controlled opponent).
    GAME_MODE_2,      // Drawing challenge (trace path with joystick or touch).
    GAME_MODE_3       // Arena multiplayer (configurable map, bots, obstacles, power-ups).
} game_mode_t;

/*
 * =============================================================================
 * VEHICLE TYPE ENUMERATION
 * =============================================================================
 * Each car has unique laser timing, speed caps, and special traits.
 */
typedef enum {
    CAR_V0 = 0,  // V0 STANDARD: balanced baseline car.
    CAR_V1,      // V1 AUTO FIRE: automatically fires when charged.
    CAR_V2,      // V2 RAPID SHOT: shorter fire duration, faster cooldown.
    CAR_V3,      // V3 MOVING CAST: can move while firing.
    CAR_V4,      // V4 FORWARD SPEED: higher forward velocity.
    CAR_V5,      // V5 LONG BEAM: longer laser beam range.
    CAR_V6       // V6 GUN PLATFORM: stationary turret mode.
} car_type_t;

/*
 * =============================================================================
 * LASER STATE MACHINE ENUMERATION
 * =============================================================================
 * Describes the firing cycle: idle -> armed (trigger held) -> priming (charge-up)
 * -> firing (beam active) -> cooldown (wait before next shot).
 */
typedef enum {
    LASER_IDLE = 0,     // Weapon ready, no input.
    LASER_ARMED,        // Trigger held, awaiting charge-up delay.
    LASER_PRIMING,      // Charging, visual feedback active.
    LASER_FIRING,       // Beam active, dealing damage.
    LASER_COOLDOWN      // Post-fire delay before next shot allowed.
} laser_state_t;

/*
 * =============================================================================
 * HARDWARE AND GAMEPLAY CONSTANTS
 * =============================================================================
 */
/* Laser hardware wiring */
#define LASER_PIN           GPIO_PIN_4   // Laser control line on GPIO Port C.
#define LASER_PORT          GPIOC

/* Default laser timing (car-specific overrides exist in game_logic.c) */
#define LASER_PRIME_MS      1000U        // Charge-up delay before firing starts.
#define LASER_FIRE_MS       1000U        // Active beam duration.
#define LASER_COOLDOWN_MS   3000U        // Minimum time between consecutive shots.

/*
 * Joystick calibration thresholds (auto-set at boot in main.c).
 * These define the boundary values for detecting left/right/forward/backward.
 */
extern uint32_t x_left_thresh;   // X-axis threshold for leftward movement.
extern uint32_t x_right_thresh;  // X-axis threshold for rightward movement.
extern uint32_t y_fwd_thresh;    // Y-axis threshold for forward movement.
extern uint32_t y_back_thresh;   // Y-axis threshold for backward movement.
extern uint32_t adc_center_x;    // Joystick center X value (for calibration).
extern uint32_t adc_center_y;    // Joystick center Y value (for calibration).

/* Macro shortcuts for threshold access */
#define X_LEFT_THRESH_ADC   x_left_thresh
#define X_RIGHT_THRESH_ADC  x_right_thresh
#define Y_FWD_THRESH_ADC    y_fwd_thresh
#define Y_BACK_THRESH_ADC   y_back_thresh
#define ADC_MIN             0U           // ADC minimum raw value.
#define ADC_MAX             4095U        // ADC maximum raw value (12-bit).
#define MOTOR_CMD_INTERVAL  80U          // Milliseconds between motor command updates.

/*
 * =============================================================================
 * GLOBAL STATE VARIABLES
 * =============================================================================
 * These are the "facts" that describe the current game situation.
 */

/* Application navigation state */
extern app_state_t   app_state;       // Current app screen/context.
extern game_mode_t   selected_mode;   // User-chosen game mode.
extern car_type_t    selected_car;    // User-chosen vehicle.

/* Gameplay state */
extern laser_state_t laser_state;     // Firing state machine.
extern uint8_t       touch_ability_shots_needed;  // Special ability counter.

/* Shared HUD status strings (updated by game_logic.c, displayed by ui.c) */
extern char          laser_line[32];  // Firing status display (e.g., "READY", "FIRING").
extern char          motion_line[24]; // Movement status display (e.g., "STOP 0%", "FWD 50%").
extern char          esp_cmd_rx[8];   // Latest response from ESP module.
extern uint8_t       fire_cmd_priority;  // Firing priority/queue flag.

/* Movement state tracking */
extern char          current_dir_cmd; // Current direction command sent to ESP ('S'=stop, 'F'=forward, etc.).
extern uint8_t       turn_delay_active;  // Flag: turn delay cooldown active.
extern char          turn_target_cmd; // Target direction after turn delay expires.
extern uint32_t      last_direction_change_tick;  // Timestamp of last direction change.

/*
 * =============================================================================
 * PUBLIC FUNCTION PROTOTYPES
 * =============================================================================
 */

/* Lookup/string functions for mode and car information */
const char *MODE_Name(game_mode_t mode);              // Returns mode string ("MODE 1", etc.).
const char *CAR_Code(car_type_t car);                 // Returns compact car code ("V0", "V1", etc.).
const char *CAR_Label(car_type_t car);                // Returns full car name ("STANDARD", "AUTO FIRE", etc.).

/* Input/ADC mapping utilities */
uint8_t map_range_percent(uint32_t value, uint32_t start, uint32_t end);  // Maps ADC range to percentage.
uint8_t car_apply_speed_cap(char cmd, uint8_t speed_percent);            // Limits movement speed by car type.

/* Car capability queries */
uint8_t car_uses_k2_fire(void);                       // Checks if car fires via K2 button.
uint8_t car_auto_fire_enabled(void);                  // Checks if car auto-fires when charged.

/* Hardware command transmission */
void Motor_SendCmd(char cmd, uint8_t speed);          // Send direction/speed command to ESP motor controller.
void Fire_SendCmd(uint8_t fire_on);                   // Send laser fire control signal to hardware.

/* Laser state machine control */
void laser_on_press(void);                            // Called when trigger pressed (arm laser).
void laser_on_release(void);                          // Called when trigger released (start charging).
void laser_update(void);                              // Updates laser state each frame (charging, firing, cooldown).

/* Special gameplay mechanics */
void SpecialAbility_ResetCooldown(void);              // Reset special ability cooldown (called by cap touch key).
void RGB_Update_From_State(void);                     // Update LED color based on current game state.

/* Central gameplay dispatcher for Mode 1 */
void Game_Router_Task(uint32_t x_raw, uint32_t y_raw, uint8_t k1_click, uint8_t k2_click, uint8_t fire_pressed);
// Processes Mode 1 gameplay: movement, firing

#endif // GAME_LOGIC_H
