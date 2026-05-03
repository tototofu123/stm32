#ifndef MODE_3_H
#define MODE_3_H

#include "main.h"
#include <stdint.h>

// Arena Sizes
typedef enum {
    M3_SIZE_DEFAULT = 0, // 240x320
    M3_SIZE_NORMAL,      // 480x640 (4x)
    M3_SIZE_LARGE        // 720x960 (9x)
} mode3_size_t;

// Mode 3 States
typedef enum {
    M3_STATE_SETUP = 0,
    M3_STATE_INIT_ARENA,
    M3_STATE_BATTLE,
    M3_STATE_GAME_OVER
} mode3_state_t;

// Tile Types
#define TILE_EMPTY  0
#define TILE_WALL   1 // Block movement
#define TILE_RIVER  2 // Slow movement

// Arena Config
extern mode3_size_t m3_arena_size;
extern uint8_t      m3_use_walls;
extern uint8_t      m3_use_rivers;
extern uint32_t     m3_world_seed;

// Function Prototypes
void Mode3_Init(void);
void Mode3_Run(uint32_t joy_x, uint32_t joy_y, uint8_t k1_click, uint8_t k2_click, 
               uint8_t fire_pressed, uint8_t ts_pressed, uint8_t ts_click, uint16_t ts_x, uint16_t ts_y);

// UI Helpers
void LCD_DrawMode3Setup(void);
void LCD_UpdateMode3Setup(void);

#endif // MODE_3_H
