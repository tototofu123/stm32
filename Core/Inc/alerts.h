/*
 * =============================================================================
 * ALERTS.H - AUDIO AND RGB LED FEEDBACK API
 * =============================================================================
 *
 * This header declares the buzzer and RGB LED drivers for user feedback during
 * gameplay and menu navigation.
 *
 * Responsibility (alerts.c implementation):
 * - Control the buzzer speaker for beep/alert sounds.
 * - Manage audio mute state (respects audio_enabled setting).
 * - Control RGB LED for game state visualization.
 * - Provide timed beep functions (short/long durations).
 * - Update buzzer state each frame (handle timing, decay).
 *
 * Hardware:
 * - Buzzer: GPIO PA8 (active-high digital output).
 * - RGB LED: GPIO PB5 (red), PB0 (green), PB1 (blue) (active-high).
 * - All outputs are digital GPIO (on/off), not PWM.
 *
 * Global variables exported: None (all state is internal to alerts.c).
 *
 * Data flow:
 * - main.c calls Buzzer_BeepShort/Long to queue beeps on user interaction.
 * - main.c calls RGB_Set to change LED color based on game state.
 * - main.c calls Buzzer_Task each frame to advance beep timers.
 * - audio_enabled setting (from ui.c) controls mute behavior.
 *
 * No C++ classes; pure C driver.
 * =============================================================================
 */

#ifndef ALERTS_H
#define ALERTS_H

#include "main.h"
#include <stdint.h>

/*
 * =============================================================================
 * BUZZER HARDWARE CONFIGURATION
 * =============================================================================
 * GPIO assignments and timing constants for the piezo speaker.
 */
#define BEEP_PIN            GPIO_PIN_8
#define BEEP_PORT           GPIOA
#define BEEP_SHORT_MS       80U
#define BEEP_LONG_MS        500U

/*
 * =============================================================================
 * RGB LED HARDWARE CONFIGURATION
 * =============================================================================
 * GPIO assignments for tri-color LED feedback (red, green, blue channels).
 */
#define RGB_R_PIN           GPIO_PIN_5
#define RGB_R_PORT          GPIOB
#define RGB_G_PIN           GPIO_PIN_0
#define RGB_G_PORT          GPIOB
#define RGB_B_PIN           GPIO_PIN_1
#define RGB_B_PORT          GPIOB

/*
 * =============================================================================
 * AUDIO AND LED FEEDBACK INTERFACE FUNCTIONS
 * =============================================================================
 */
/*
 * RGB_Set:
 * Sets the RGB LED to a specific color by controlling each channel.
 * Input: r, g, b (0=off, 1=on for each color channel).
 * Example: RGB_Set(1, 0, 0) = red, RGB_Set(1, 1, 1) = white.
 */
void RGB_Set(uint8_t r, uint8_t g, uint8_t b);

/*
 * Buzzer_Set:
 * Directly controls the buzzer output (low-level).
 * Input: on (1=buzzer on, 0=buzzer off).
 * Usually called via Buzzer_BeepShort/Long; rarely used directly.
 */
void Buzzer_Set(uint8_t on);

/*
 * Buzzer_SetMute:
 * Enables or disables global buzzer mute (overrides individual beeps).
 * Input: mute (1=mute all sounds, 0=allow sounds based on audio_enabled).
 * Called by settings menu when audio_enabled toggle changes.
 */
void Buzzer_SetMute(uint8_t mute);

/*
 * Buzzer_BeepShort:
 * Queues a short beep (~80 ms) as user feedback for button presses, menu changes.
 * Respects the audio_enabled setting and mute state.
 */
void Buzzer_BeepShort(void);

/*
 * Buzzer_BeepLong:
 * Queues a long beep (~500 ms) for more attention-grabbing alerts.
 * Respects the audio_enabled setting and mute state.
 */
void Buzzer_BeepLong(void);

/*
 * Buzzer_Task:
 * Updates buzzer state each frame (decrement timer, turn off when expired).
 * Should be called once per frame from main.c's game loop.
 * Handles the timing decay of queued beeps.
 */
void Buzzer_Task(void);

#endif  /* ALERTS_H */
