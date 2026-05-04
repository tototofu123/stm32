/*
 * =============================================================================
 * SEVEN_SEG.H - DUAL 7-SEGMENT DISPLAY DRIVER AND TELEMETRY API
 * =============================================================================
 *
 * This header declares the 7-segment LED display driver for the dual-digit
 * display panel showing game mode, car selection, HP, timers, and status.
 *
 * Responsibility (seven_seg.c implementation):
 * - Control 14 GPIO pins (7 segments per digit + decimal points).
 * - Implement digit display functions (0-9, 'A', custom patterns).
 * - Manage display modes (idle, cooldown countdown, etc.).
 * - Provide telemetry updates: mode/car display, HP readout, timer display.
 * - Handle multiplexing and refresh timing (called from main.c frame loop).
 *
 * Hardware:
 * - Two 7-segment displays: left (LSEG_*) and right (RSEG_*).
 * - Each segment is a GPIO pin (A-G segments + decimal point per digit).
 * - Active-high control: GPIO=1 lights the segment LED.
 *
 * Global variables exported:
 * - seg_mode: Current display mode (IDLE, M2_TIMER, etc.).
 * - seg_tick, seg_tenths: Timing trackers for cooldown countdowns.
 * - seg_left, seg_right, seg_dp: Current display values (digits 0-9 or 10=A).
 *
 * Data flow:
 * - main.c sets seg_mode and calls SEG_ShowPair(left, right) to update display.
 * - main.c calls SEG_Task() each frame to refresh display and advance timers.
 * - Display shows: home (00), settings (88), game mode (1-3), car (0-6), HP (0-9/A).
 *
 * No C++ classes; pure C driver.
 * =============================================================================
 */

#ifndef SEVEN_SEG_H
#define SEVEN_SEG_H

#include "main.h"
#include <stdint.h>

/*
 * =============================================================================
 * LEFT 7-SEGMENT DISPLAY PIN CONFIGURATION
 * =============================================================================
 * GPIO assignments for the left digit (segments A-G and decimal point).
 */
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

/*
 * =============================================================================
 * RIGHT 7-SEGMENT DISPLAY PIN CONFIGURATION
 * =============================================================================
 * GPIO assignments for the right digit (segments A-G and decimal point).
 */
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

/*
 * =============================================================================
 * DISPLAY MODE ENUMERATION
 * =============================================================================
 * Defines different operational states for the 7-segment display driver.
 */
/*
 * Display mode states controlling what is shown and how it updates.
 */
typedef enum {
    SEG_IDLE = 0,           /* No special display mode; show static values */
    SEG_K1_COUNT,           /* K1 button press counter (unused currently) */
    SEG_K2_SHOW88,          /* Show "88" (all segments on) for visibility test */
    SEG_JSW_CD,             /* Joystick button cooldown countdown */
    SEG_ZERO_HOLD,          /* Hold "00" on display */
    SEG_MODE2_CMD,          /* Mode 2 command history display */
    SEG_M2_TIMER,           /* Mode 2 challenge timer countdown */
    SEG_M3_OBSTACLES        /* Mode 3 obstacle setup display */
} seg_mode_t;

/*
 * =============================================================================
 * GLOBAL DISPLAY STATE VARIABLES
 * =============================================================================
 */
/* Current display mode and timing */
extern seg_mode_t seg_mode;       /* Active display mode (controls refresh behavior) */
extern uint32_t   seg_tick;       /* Timestamp for timer/countdown tracking */
extern int        seg_tenths;     /* Tenths-of-second counter for cooldown countdown */

/* Current digit and decimal point values */
extern uint8_t    seg_left;       /* Left digit value (0-9 or 10 for 'A') */
extern uint8_t    seg_right;      /* Right digit value (0-9 or 10 for 'A') */
extern uint8_t    seg_dp;         /* Decimal point flags (bit 0=left, bit 1=right) */

/*
 * =============================================================================
 * 7-SEGMENT DISPLAY INTERFACE FUNCTIONS
 * =============================================================================
 */
/*
 * SEG_WritePin:
 * Low-level GPIO control: sets or clears an individual segment LED.
 * Input: port (GPIO port), pin (GPIO pin), on (1=light, 0=off).
 */
void SEG_WritePin(GPIO_TypeDef *port, uint16_t pin, uint8_t on);

/*
 * SEG_AllOff:
 * Turns off all segments on both left and right digits.
 */
void SEG_AllOff(void);

/*
 * SEG_ShowLeft:
 * Displays a single digit on the left 7-segment display.
 * Input: d (digit 0-9, 10='A'), dp (decimal point: 1=on, 0=off).
 */
void SEG_ShowLeft(uint8_t d, uint8_t dp);

/*
 * SEG_ShowRight:
 * Displays a single digit on the right 7-segment display.
 * Input: d (digit 0-9, 10='A').
 */
void SEG_ShowRight(uint8_t d);

/*
 * SEG_ShowPair:
 * Displays two digits simultaneously on left and right displays.
 * Input: left (left digit 0-9/10), right (right digit 0-9/10), dp (decimal flags).
 * Used to show mode/car (1/0), HP display (9/A), etc.
 */
void SEG_ShowPair(uint8_t left, uint8_t right, uint8_t dp);

/*
 * SEG_ShowTenths:
 * Displays a countdown timer value (in tenths of seconds).
 * Input: t (time value in 0.1s units).
 */
void SEG_ShowTenths(int t);

/*
 * SEG_ShowCmd:
 * Displays a movement command character (F/B/L/R/S) as visual feedback.
 * Input: cmd (command character).
 */
void SEG_ShowCmd(char cmd);

/*
 * SEG_ShowCustom:
 * Displays raw 7-segment bit patterns (advanced control).
 * Input: left_bits, right_bits (7-bit pattern per digit).
 */
void SEG_ShowCustom(uint8_t left_bits, uint8_t right_bits);

/*
 * SEG_StartCooldownCountdown:
 * Starts a cooldown timer display that counts down from duration_ms to 0.
 * Input: cooldown_ms (duration in milliseconds).
 */
void SEG_StartCooldownCountdown(uint32_t cooldown_ms);

/*
 * SEG_Task:
 * Updates display state each frame (refresh, advance timers, mode changes).
 * Should be called once per frame from main.c's game loop.
 */
void SEG_Task(void);

#endif  /* SEVEN_SEG_H */
