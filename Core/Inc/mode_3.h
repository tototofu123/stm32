#ifndef MODE_3_H
#define MODE_3_H

#include "main.h"
#include <stdint.h>

// Mode 3 States
typedef enum {
    M3_STATE_INIT = 0,
    M3_STATE_WAIT_FOR_PEER,
    M3_STATE_BATTLE,
    M3_STATE_GAME_OVER
} mode3_state_t;

// Extern Globals
extern mode3_state_t m3_state;
extern uint32_t      m3_world_seed;

// Function Prototypes
void Mode3_Init(void);
void Mode3_Run(uint32_t joy_x, uint32_t joy_y, uint8_t k1_click, uint8_t k2_click, 
               uint8_t fire_pressed, uint8_t ts_pressed, uint8_t ts_click, uint16_t ts_x, uint16_t ts_y);

#endif // MODE_3_H
