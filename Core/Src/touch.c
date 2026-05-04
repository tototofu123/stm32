/*
 * touch.c handles the capacitive touch controller interface. It bit-bangs the
 * serial protocol, reads the raw coordinates, and exposes a simple pressed
 * state to the rest of the application.
 *
 * Functions in this file:
 * - T_Delay: provides a tiny bit delay for the touch serial lines.
 * - T_WriteBit: writes one bit to the touch controller.
 * - T_ReadBit: reads one bit from the touch controller.
 * - T_WriteByte: sends one byte to the controller.
 * - XPT2046_Read12: reads a 12-bit coordinate value from the controller.
 * - TouchPressed: reports whether the touch IRQ line is active.
 * - TouchReadXRaw: reads the raw X coordinate.
 * - TouchReadYRaw: reads the raw Y coordinate.
 *
 * Global variables used here are the touch GPIO pin macros from touch.h and
 * the HAL GPIO layer. No classes are used in this C file.
 */
#include "touch.h"

// --- Private Helper Functions ---
/* T_Delay inserts a tiny line-settle delay so the touch controller can sample
 * correctly.
 */
static void T_Delay(void)
{
    for (volatile int i = 0; i < 12; i++) __NOP();
}

/* T_WriteBit sends one serial bit to the touch controller using the clock and
 * data pins.
 */
static void T_WriteBit(uint8_t b)
{
    HAL_GPIO_WritePin(T_DIN_PORT, T_DIN_PIN, b ? GPIO_PIN_SET : GPIO_PIN_RESET);
    T_Delay();
    HAL_GPIO_WritePin(T_CLK_PORT, T_CLK_PIN, GPIO_PIN_SET);
    T_Delay();
    HAL_GPIO_WritePin(T_CLK_PORT, T_CLK_PIN, GPIO_PIN_RESET);
}

/* T_ReadBit reads one serial bit from the touch controller using the clock and
 * data pins.
 */
static uint8_t T_ReadBit(void)
{
    HAL_GPIO_WritePin(T_CLK_PORT, T_CLK_PIN, GPIO_PIN_SET);
    T_Delay();
    HAL_GPIO_WritePin(T_CLK_PORT, T_CLK_PIN, GPIO_PIN_RESET);
    T_Delay();
    return (HAL_GPIO_ReadPin(T_DOUT_PORT, T_DOUT_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

/* T_WriteByte sends one full byte to the touch controller, most-significant bit
 * first.
 */
static void T_WriteByte(uint8_t data)
{
    for (int i = 7; i >= 0; i--) T_WriteBit((data >> i) & 1U);
}

/* XPT2046_Read12 issues a command and reads back a 12-bit coordinate or touch
 * value from the controller.
 */
static uint16_t XPT2046_Read12(uint8_t cmd)
{
    uint16_t v = 0;
    HAL_GPIO_WritePin(T_CS_PORT, T_CS_PIN, GPIO_PIN_RESET);
    T_WriteByte(cmd);
    for (int i = 0; i < 16; i++)
    {
        v <<= 1;
        v |= T_ReadBit();
    }
    HAL_GPIO_WritePin(T_CS_PORT, T_CS_PIN, GPIO_PIN_SET);
    return (v >> 4) & 0x0FFF;
}

// --- Public Functions ---
/* TouchPressed reports whether the touch IRQ line indicates a valid touch.
 */
uint8_t TouchPressed(void)
{
    return (HAL_GPIO_ReadPin(T_IRQ_PORT, T_IRQ_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
}

/* TouchReadXRaw returns the raw X measurement from the touch controller.
 */
uint16_t TouchReadXRaw(void)
{
    return XPT2046_Read12(XPT_CMD_X);
}

/* TouchReadYRaw returns the raw Y measurement from the touch controller.
 */
uint16_t TouchReadYRaw(void)
{
    return XPT2046_Read12(XPT_CMD_Y);
}
