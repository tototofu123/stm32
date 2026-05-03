// mode_2.h
#ifndef MODE_2_H
#define MODE_2_H

#include "main.h"
#include <stdint.h>

typedef enum {
    M2_STATE_INPUT_SELECT = 0,  // Choose joystick or touch
    M2_STATE_DRAWING,
    M2_STATE_RESET_CONFIRM,     // Confirmation for clearing canvas
    M2_STATE_MOVING,
    M2_STATE_SHOOTING,
    M2_STATE_FINISHED
} mode2_state_t;

typedef enum {
    M2_INPUT_JOYSTICK = 0,
    M2_INPUT_TOUCH
} mode2_input_method_t;

extern mode2_state_t m2_state;
extern mode2_input_method_t m2_input_method;
extern mode2_input_method_t selected_input;
extern char m2_cmd_history[11];  // Last 10 commands + null terminator

void Mode2_Init(void);
void Mode2_ResetCanvas(void);
void Mode2_Run(uint32_t joy_x, uint32_t joy_y, uint8_t k1_click, uint8_t k2_click, uint8_t fire_pressed, uint8_t ts_pressed, uint8_t ts_click, uint16_t ts_x, uint16_t ts_y);

#endif // MODE_2_H