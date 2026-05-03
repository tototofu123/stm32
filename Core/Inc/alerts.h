#ifndef ALERTS_H
#define ALERTS_H

#include "main.h"
#include <stdint.h>

// Buzzer Pins
#define BEEP_PIN            GPIO_PIN_8
#define BEEP_PORT           GPIOA
#define BEEP_SHORT_MS       80U
#define BEEP_LONG_MS        500U

// Shared RGB LED Pins
#define RGB_R_PIN           GPIO_PIN_5
#define RGB_R_PORT          GPIOB
#define RGB_G_PIN           GPIO_PIN_0
#define RGB_G_PORT          GPIOB
#define RGB_B_PIN           GPIO_PIN_1
#define RGB_B_PORT          GPIOB

// Function Prototypes
void RGB_Set(uint8_t r, uint8_t g, uint8_t b);
void Buzzer_Set(uint8_t on);
void Buzzer_SetMute(uint8_t mute);
void Buzzer_BeepShort(void);
void Buzzer_BeepLong(void);
void Buzzer_Task(void);

#endif // ALERTS_H
