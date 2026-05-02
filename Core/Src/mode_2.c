// mode_2.c
#include "mode_2.h"
#include "game_logic.h"
#include "lcd.h"
#include "ui.h"
#include "alerts.h"
#include <stdlib.h>

#define MAX_POINTS 100
#define M2_JOY_DEADZONE_L 1800
#define M2_JOY_DEADZONE_H 2300

uint16_t m2_path_x[MAX_POINTS];
uint16_t m2_path_y[MAX_POINTS];
uint16_t m2_path_count = 0;

uint16_t m2_cursor_x = 120;
uint16_t m2_cursor_y = 160;

mode2_state_t m2_state = M2_STATE_DRAWING;

static uint16_t target_pt_idx = 0;
static uint32_t move_timer = 0;

void Mode2_Init(void) {
    m2_path_count = 0;
    m2_cursor_x = 120;
    m2_cursor_y = 160;
    m2_state = M2_STATE_DRAWING;
    target_pt_idx = 0;
    LCD_Clear(0, 0, 240, 320, UI_BG);
    LCD_DrawMode2Canvas();
}

void Mode2_Run(uint32_t joy_x, uint32_t joy_y, uint8_t k1_click, uint8_t k2_click, uint8_t fire_pressed) {
    uint32_t now = HAL_GetTick();

    if (m2_state == M2_STATE_DRAWING) {
        uint16_t old_x = m2_cursor_x;
        uint16_t old_y = m2_cursor_y;

        // Joystick moves cursor
        if (joy_x < M2_JOY_DEADZONE_L && m2_cursor_x > 0) m2_cursor_x -= 2;
        if (joy_x > M2_JOY_DEADZONE_H && m2_cursor_x < 239) m2_cursor_x += 2;
        if (joy_y < M2_JOY_DEADZONE_L && m2_cursor_y < 319) m2_cursor_y += 2; 
        if (joy_y > M2_JOY_DEADZONE_H && m2_cursor_y > 80) m2_cursor_y -= 2; // Prevent drawing over text

        // Check if cursor moved
        if (old_x != m2_cursor_x || old_y != m2_cursor_y) {
            LCD_DrawDot(m2_cursor_x, m2_cursor_y, RED);
            
            // Save point if moved at least 15 pixels from the last point
            if (m2_path_count == 0 || (abs(m2_cursor_x - m2_path_x[m2_path_count-1]) > 15 || abs(m2_cursor_y - m2_path_y[m2_path_count-1]) > 15)) {
                if (m2_path_count < MAX_POINTS) {
                    m2_path_x[m2_path_count] = m2_cursor_x;
                    m2_path_y[m2_path_count] = m2_cursor_y;
                    if (m2_path_count > 0) {
                        LCD_DrawLine(m2_path_x[m2_path_count-1], m2_path_y[m2_path_count-1], m2_cursor_x, m2_cursor_y, BLUE);
                    }
                    m2_path_count++;
                }
            }
        }

        if (k2_click && m2_path_count > 0) {
            Buzzer_BeepShort();
            LCD_ClearTextField(10, 50, 20, UI_BG);
            LCD_TEXT(10, 50, "MOVING...");
            m2_state = M2_STATE_MOVING;
            target_pt_idx = 0;
            move_timer = now;
        }
    } 
    else if (m2_state == M2_STATE_MOVING) {
        if (target_pt_idx >= m2_path_count - 1) {
            Motor_SendCmd('S', 0);
            LCD_ClearTextField(10, 50, 20, UI_BG);
            LCD_TEXT(10, 50, "PRESS K1 TO FIRE!");
            m2_state = M2_STATE_SHOOTING;
            return;
        }

        if (now >= move_timer) {
            // Translate coordinates to movement
            int dx = m2_path_x[target_pt_idx + 1] - m2_path_x[target_pt_idx];
            int dy = m2_path_y[target_pt_idx + 1] - m2_path_y[target_pt_idx];
            
            char cmd = 'S';
            uint32_t duration = 0;

            if (abs(dx) > abs(dy)) {
                cmd = (dx > 0) ? 'R' : 'L';
                duration = abs(dx) * 12; // Tuning: 12ms per pixel
            } else {
                cmd = (dy > 0) ? 'B' : 'F'; // Assuming Y+ is visually 'down/back'
                duration = abs(dy) * 12;
            }

            Motor_SendCmd(cmd, 60); // Move at 60% speed
            move_timer = now + duration;
            target_pt_idx++;
        }
    } 
    else if (m2_state == M2_STATE_SHOOTING) {
        Motor_SendCmd('S', 0);
        if (k1_click || fire_pressed) {
            laser_on_press();
        } else {
            laser_on_release();
        }
        
        if (laser_state == LASER_COOLDOWN) {
            LCD_ClearTextField(10, 50, 20, UI_BG);
            LCD_TEXT(10, 50, "K2 TO RESET CANVAS");
            m2_state = M2_STATE_FINISHED;
        }
    }
    else if (m2_state == M2_STATE_FINISHED) {
        if (k2_click) {
            Buzzer_BeepShort();
            Mode2_Init(); // Clear canvas and restart
        }
    }
}