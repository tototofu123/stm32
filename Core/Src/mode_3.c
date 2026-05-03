#include "mode_3.h"
#include "lcd.h"
#include "ui.h"
#include "alerts.h"
#include "peripherals.h"
#include <stdio.h>
#include <stdlib.h>

mode3_state_t m3_state = M3_STATE_INIT;
uint32_t      m3_world_seed = 0;

void Mode3_Init(void) {
    m3_state = M3_STATE_INIT;
    
    // MANDATORY: Mode 3 must be silent
    Buzzer_SetMute(1); 
    
    // Generate random seed based on tick
    m3_world_seed = HAL_GetTick();
    srand(m3_world_seed);
    
    LCD_Clear(0, 0, 240, 320, UI_BG);
    LCD_DrawMode3Placeholder();
}

void Mode3_Run(uint32_t joy_x, uint32_t joy_y, uint8_t k1_click, uint8_t k2_click, 
               uint8_t fire_pressed, uint8_t ts_pressed, uint8_t ts_click, uint16_t ts_x, uint16_t ts_y) {
    
    if (m3_state == M3_STATE_INIT) {
        // Placeholder logic: wait for K2 to "Simulate" peer connection
        if (k2_click) {
            m3_state = M3_STATE_BATTLE;
            LCD_Clear(0, 20, 240, 300, UI_BG);
            LCD_SetColors(GREEN, UI_BG);
            LCD_TEXT(10, 100, "BATTLE START!");
            LCD_SetColors(BLUE, WHITE);
        }
    }
    else if (m3_state == M3_STATE_BATTLE) {
        // Future: Handle Procedural Rendering and WiFi position updates
        if (k1_click) {
            // Test hit detection logic placeholder
        }
    }
}
