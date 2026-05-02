// mode_2.h
#ifndef MODE_2_H
#define MODE_2_H

#include "main.h"
#include <stdint.h>

typedef enum {
    M2_STATE_DRAWING = 0,
    M2_STATE_MOVING,
    M2_STATE_SHOOTING,
    M2_STATE_FINISHED
} mode2_state_t;

extern mode2_state_t m2_state;

void Mode2_Init(void);
void Mode2_Run(uint32_t joy_x, uint32_t joy_y, uint8_t k1_click, uint8_t k2_click, uint8_t fire_pressed);

#endif // MODE_2_H