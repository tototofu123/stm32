// mode_2.c
#include "mode_2.h"
#include "game_logic.h"
#include "lcd.h"
#include "ui.h"
#include "alerts.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define MAX_POINTS 100
#define CANVAS_X_MIN 10
#define CANVAS_X_MAX 230
#define CANVAS_Y_MIN 90
#define CANVAS_Y_MAX 280 // Leave space for history at bottom

uint16_t m2_path_x[MAX_POINTS];
uint16_t m2_path_y[MAX_POINTS];
uint16_t m2_path_count = 0;

uint16_t m2_cursor_x = 120;
uint16_t m2_cursor_y = 190;

char m2_cmd_history[11] = {0};  
static uint16_t m2_cmd_idx = 0;
static uint8_t history_slot = 0;
static char last_history_cmd = ' ';

mode2_state_t m2_state = M2_STATE_INPUT_SELECT;
mode2_input_method_t m2_input_method = M2_INPUT_JOYSTICK;

static uint16_t target_pt_idx = 0;
static uint32_t move_timer = 0;
static uint16_t m2_visit_x[MAX_POINTS];
static uint16_t m2_visit_y[MAX_POINTS];
static uint16_t m2_visit_count = 0;
mode2_input_method_t selected_input = M2_INPUT_JOYSTICK;

void Mode2_Init(void) {
    m2_state = M2_STATE_INPUT_SELECT;
    selected_input = M2_INPUT_JOYSTICK;
    Buzzer_SetMute(1);  
    LCD_Clear(0, 0, 240, 320, UI_BG);
    LCD_DrawMode2InputSelect();
}

void Mode2_ResetCanvas(void) {
    m2_path_count = 0;
    m2_cursor_x = 120;
    m2_cursor_y = 190;
    m2_visit_count = 0;
    target_pt_idx = 0;
    m2_cmd_idx = 0;
    history_slot = 0;
    last_history_cmd = ' ';
    memset(m2_cmd_history, 0, sizeof(m2_cmd_history));
    strcpy(laser_line, "READY");
    strcpy(motion_line, "READY");
    m2_state = M2_STATE_DRAWING;
    LCD_DrawMode2Canvas();
}

void Mode2_Run(uint32_t joy_x, uint32_t joy_y, uint8_t k1_click, uint8_t k2_click, 
               uint8_t fire_pressed, uint8_t ts_pressed, uint8_t ts_click, uint16_t ts_x, uint16_t ts_y) {
    uint32_t now = HAL_GetTick();

    if (m2_state == M2_STATE_INPUT_SELECT) {
        if (k1_click) {
            selected_input = (selected_input == M2_INPUT_JOYSTICK) ? M2_INPUT_TOUCH : M2_INPUT_JOYSTICK;
            LCD_UpdateMode2InputSelect(); // Partial refresh
            HAL_Delay(50);
        }

        if (ts_click) {
            mode2_input_method_t touched_input = selected_input;
            uint8_t hit = 0;
            if (ts_y >= 70 && ts_y <= 150) { touched_input = M2_INPUT_JOYSTICK; hit = 1; }
            else if (ts_y >= 170 && ts_y <= 250) { touched_input = M2_INPUT_TOUCH; hit = 1; }

            if (hit) {
                if (touched_input == selected_input) {
                    m2_input_method = selected_input;
                    Mode2_ResetCanvas();
                    return;
                } else {
                    selected_input = touched_input;
                    LCD_UpdateMode2InputSelect();
                    HAL_Delay(50);
                }
            }
        }
        
        if (k2_click) {
            m2_input_method = selected_input;
            Mode2_ResetCanvas();
        }
    }
    else if (m2_state == M2_STATE_DRAWING) {
        uint16_t old_x = m2_cursor_x;
        uint16_t old_y = m2_cursor_y;

        if (m2_input_method == M2_INPUT_JOYSTICK) {
            if (joy_x < X_LEFT_THRESH_ADC && m2_cursor_x > CANVAS_X_MIN) m2_cursor_x -= 3;
            if (joy_x > X_RIGHT_THRESH_ADC && m2_cursor_x < CANVAS_X_MAX) m2_cursor_x += 3;
            if (joy_y < Y_FWD_THRESH_ADC && m2_cursor_y > CANVAS_Y_MIN) m2_cursor_y -= 3;
            if (joy_y > 3000 && m2_cursor_y < CANVAS_Y_MAX) m2_cursor_y += 3;
        } 
        else {  
            if (ts_pressed && ts_x >= CANVAS_X_MIN && ts_x <= CANVAS_X_MAX && 
                ts_y >= CANVAS_Y_MIN && ts_y <= CANVAS_Y_MAX) {
                m2_cursor_x = ts_x;
                m2_cursor_y = ts_y;
            }
        }

        if (m2_cursor_x < CANVAS_X_MIN) m2_cursor_x = CANVAS_X_MIN;
        if (m2_cursor_x > CANVAS_X_MAX) m2_cursor_x = CANVAS_X_MAX;
        if (m2_cursor_y < CANVAS_Y_MIN) m2_cursor_y = CANVAS_Y_MIN;
        if (m2_cursor_y > CANVAS_Y_MAX) m2_cursor_y = CANVAS_Y_MAX;

        if (old_x != m2_cursor_x || old_y != m2_cursor_y) {
            // Draw path line
            LCD_DrawLine(old_x, old_y, m2_cursor_x, m2_cursor_y, BLUE);
            // Draw bigger red cursor
            LCD_Clear(m2_cursor_x - 2, m2_cursor_y - 2, 5, 5, RED);
        }

        if (old_x != m2_cursor_x || old_y != m2_cursor_y) {
            if (m2_path_count == 0 || (abs(m2_cursor_x - m2_path_x[m2_path_count-1]) > 8 || 
                                       abs(m2_cursor_y - m2_path_y[m2_path_count-1]) > 8)) {
                if (m2_path_count < MAX_POINTS) {
                    m2_path_x[m2_path_count] = m2_cursor_x;
                    m2_path_y[m2_path_count] = m2_cursor_y;
                    m2_path_count++;
                }
            }
        }

        if (k1_click) {
            LCD_DrawMode2ResetConfirm();
            m2_state = M2_STATE_RESET_CONFIRM;
        }

        if (k2_click && m2_path_count > 0) {
            LCD_ClearTextField(10, 30, 24, UI_BG);
            LCD_ClearTextField(10, 48, 24, UI_BG);
            LCD_ClearTextField(10, 65, 24, UI_BG);
            LCD_TEXT(10, 40, "MOVING...");
            LCD_Clear(0, 280, 240, 40, UI_BOTTOM); // Clear for history
            m2_state = M2_STATE_MOVING;
            target_pt_idx = 0;
            m2_visit_count = 0;
            history_slot = 0;
            last_history_cmd = ' ';
            move_timer = now + 500; 
        }
    }
    else if (m2_state == M2_STATE_RESET_CONFIRM) {
        if (k1_click) { 
            LCD_DrawMode2Canvas();
            if (m2_path_count > 0) {
                for(int i=0; i<m2_path_count-1; i++) {
                    LCD_DrawLine(m2_path_x[i], m2_path_y[i], m2_path_x[i+1], m2_path_y[i+1], BLUE);
                }
            }
            m2_state = M2_STATE_DRAWING;
        }
        if (k2_click) { Mode2_ResetCanvas(); }
    }
    else if (m2_state == M2_STATE_MOVING) {
        if (target_pt_idx < m2_path_count) {
            uint16_t curr_x = m2_path_x[target_pt_idx];
            uint16_t curr_y = m2_path_y[target_pt_idx];
            LCD_Clear(curr_x - 2, curr_y - 2, 5, 5, RED);
            if (m2_visit_count > 0) {
                LCD_DrawLine(m2_visit_x[m2_visit_count - 1], m2_visit_y[m2_visit_count - 1], curr_x, curr_y, RED);
            }
        }

        if (target_pt_idx >= m2_path_count - 1) {
            Motor_SendCmd('S', 0);
            LCD_ClearTextField(10, 40, 24, UI_BG);
            LCD_TEXT(10, 40, "READY TO FIRE! (K1)");
            m2_state = M2_STATE_SHOOTING;
            return;
        }

        if (now >= move_timer) {
            if (m2_visit_count < MAX_POINTS) {
                m2_visit_x[m2_visit_count] = m2_path_x[target_pt_idx];
                m2_visit_y[m2_visit_count] = m2_path_y[target_pt_idx];
                m2_visit_count++;
            }

            int dx = m2_path_x[target_pt_idx + 1] - m2_path_x[target_pt_idx];
            int dy = m2_path_y[target_pt_idx + 1] - m2_path_y[target_pt_idx];
            char cmd = 'S';
            uint32_t duration = 0;

            if (abs(dx) >= abs(dy)) { cmd = (dx > 0) ? 'R' : 'L'; duration = abs(dx) * 15; }
            else { cmd = (dy > 0) ? 'B' : 'F'; duration = abs(dy) * 15; }

            if (duration > 0) {
                Motor_SendCmd(cmd, 60);
                
                // History display logic
                uint8_t prev_slot = (history_slot == 0) ? 9 : (history_slot - 1);
                if (last_history_cmd != ' ') {
                    LCD_DrawMode2CommandHistory(last_history_cmd, prev_slot, 0); // "Blue-ify" previous
                }
                LCD_DrawMode2CommandHistory(cmd, history_slot, 1); // "Green-ify" new
                last_history_cmd = cmd;
                history_slot = (history_slot + 1) % 10;
                
                move_timer = now + duration;
                target_pt_idx++;
            } else { target_pt_idx++; }
        }
    } 
    else if (m2_state == M2_STATE_SHOOTING) {
        Motor_SendCmd('S', 0);
        if (k1_click || fire_pressed) {
            LCD_ClearTextField(10, 40, 24, UI_BG);
            LCD_TEXT(10, 40, "!!! FIRING !!!");
            laser_on_press();
            HAL_Delay(500);
            laser_on_release();
            LCD_ClearTextField(10, 40, 24, UI_BG);
            LCD_TEXT(10, 40, "ROUND COMPLETE");
            HAL_Delay(500);
            LCD_TEXT(10, 60, "K2: RESET CANVAS");
            m2_state = M2_STATE_FINISHED;
        }
    }
    else if (m2_state == M2_STATE_FINISHED) {
        if (k2_click) { Mode2_ResetCanvas(); }
    }
}