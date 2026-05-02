#ifndef TOUCH_H
#define TOUCH_H

#include "main.h"
#include <stdint.h>

// XPT2046 LCD Touch Screen Pins
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

// XPT2046 Commands
#define XPT_CMD_Y           0x90
#define XPT_CMD_X           0xD0

// Screen Calibration Bounds
#define TS_X_MIN            220
#define TS_X_MAX            3850
#define TS_Y_MIN            260
#define TS_Y_MAX            3780

// Function Prototypes
uint8_t  TouchPressed(void);
uint16_t TouchReadXRaw(void);
uint16_t TouchReadYRaw(void);
uint16_t map_u16(uint16_t v, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);

#endif // TOUCH_H