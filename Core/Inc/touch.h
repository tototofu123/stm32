/*
 * =============================================================================
 * TOUCH.H - CAPACITIVE TOUCHSCREEN CONTROLLER API
 * =============================================================================
 *
 * This header declares the XPT2046 resistive touchscreen controller interface,
 * providing coordinate reading and calibration for menu/gameplay interaction.
 *
 * Responsibility (touch.c implementation):
 * - Initialize XPT2046 touch controller (SPI-like protocol).
 * - Read X and Y raw ADC values from touchscreen.
 * - Detect touch/press state (IRQ line monitoring).
 * - Apply calibration mapping to convert raw coordinates to screen pixels.
 * - Handle SPI communication (bit-banging on GPIO).
 *
 * Hardware:
 * - XPT2046 4-wire resistive touch panel controller.
 * - SPI interface: CS (chip select), CLK (clock), DIN (data in), DOUT (data out).
 * - IRQ pin: interrupt signal when touch detected.
 * - Calibration bounds: TS_X_MIN/MAX, TS_Y_MIN/MAX define raw ADC range.
 *
 * Global variables exported: None (all state is local to touch.c).
 *
 * Data flow:
 * - main.c calls TouchPressed() every frame to detect new touch events.
 * - main.c calls TouchReadXRaw/Y to get raw coordinate values.
 * - Coordinates are mapped in main.c to screen pixel range (0-239 x, 0-319 y).
 * - Touch coordinates used for menu navigation, car selection, Mode 2 canvas, etc.
 *
 * No C++ classes; pure C driver.
 * =============================================================================
 */

#ifndef TOUCH_H
#define TOUCH_H

#include "main.h"
#include <stdint.h>

/*
 * =============================================================================
 * XPT2046 TOUCHSCREEN CONTROLLER PIN CONFIGURATION
 * =============================================================================
 * GPIO assignments for bit-banged SPI communication with the touch controller.
 */
#define T_CS_PIN            GPIO_PIN_13
#define T_CS_PORT           GPIOD
#define T_CLK_PIN           GPIO_PIN_0
#define T_CLK_PORT          GPIOE
#define T_DIN_PIN           GPIO_PIN_2
#define T_DIN_PORT          GPIOE
#define T_DOUT_PIN          GPIO_PIN_3
#define T_DOUT_PORT         GPIOE
#define T_IRQ_PIN           GPIO_PIN_4
#define T_IRQ_PORT          GPIOE

/*
 * =============================================================================
 * XPT2046 COMMAND CODES
 * =============================================================================
 * SPI command bytes to read X or Y coordinate from the touch controller.
 */
#define XPT_CMD_Y           0x90  /* Command to read Y-axis coordinate */
#define XPT_CMD_X           0xD0  /* Command to read X-axis coordinate */

/*
 * =============================================================================
 * TOUCHSCREEN CALIBRATION CONSTANTS
 * =============================================================================
 * Raw ADC value ranges for the touch panel, determined empirically.
 * Used to map raw coordinates to screen pixel positions (0-239 x, 0-319 y).
 */
#define TS_X_MIN            220
#define TS_X_MAX            3850
#define TS_Y_MIN            260
#define TS_Y_MAX            3780

/*
 * =============================================================================
 * TOUCHSCREEN INTERFACE FUNCTIONS
 * =============================================================================
 */
/*
 * TouchPressed:
 * Detects if the touchscreen is currently being touched.
 * Output: 1 if touch detected (IRQ asserted), 0 if no touch.
 * Used to gate coordinate reading and debounce touch events.
 */
uint8_t  TouchPressed(void);

/*
 * TouchReadXRaw:
 * Reads the raw X-axis (horizontal) coordinate from the touch controller.
 * Output: 12-bit ADC value (raw, uncalibrated; TS_X_MIN to TS_X_MAX range).
 * Must be mapped to screen pixels (0-239) using calibration bounds.
 */
uint16_t TouchReadXRaw(void);

/*
 * TouchReadYRaw:
 * Reads the raw Y-axis (vertical) coordinate from the touch controller.
 * Output: 12-bit ADC value (raw, uncalibrated; TS_Y_MIN to TS_Y_MAX range).
 * Must be mapped to screen pixels (0-319) using calibration bounds.
 */
uint16_t TouchReadYRaw(void);

#endif  /* TOUCH_H */
