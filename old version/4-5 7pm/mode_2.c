// mode_2.c
#include "mode_2.h"
#include "game_logic.h"
#include "lcd.h"
#include "ui.h"
#include "alerts.h"
#include "seven_seg.h"
#include "peripherals.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#define MAX_POINTS 1000 
#define CANVAS_X_MIN 10
#define CANVAS_X_MAX 230
#define CANVAS_Y_MIN 80
#define CANVAS_Y_MAX 310 

uint16_t m2_path_x[MAX_POINTS];
uint16_t m2_path_y[MAX_POINTS];
uint16_t m2_path_count = 0;

uint16_t m2_cursor_x = 120;
uint16_t m2_cursor_y = 190;

static uint8_t history_slot = 0;
static char last_history_cmd = ' ';
static uint16_t overlap_counter = 0;
const uint16_t rainbow_colors[6] = {RED, YELLOW, GREEN, CYAN, BLUE, MAGENTA};

mode2_state_t m2_state = M2_STATE_INPUT_SELECT;
mode2_input_method_t m2_input_method = M2_INPUT_JOYSTICK;

static uint16_t target_pt_idx = 0;
static uint32_t move_timer = 0;
static uint16_t m2_visit_x[MAX_POINTS];
static uint16_t m2_visit_y[MAX_POINTS];
static uint16_t m2_visit_count = 0;

static uint32_t total_path_distance = 0;
static uint32_t covered_distance = 0;
static uint32_t remaining_time_ms = 0;

mode2_input_method_t selected_input = M2_INPUT_JOYSTICK;

void Mode2_Init(void) {
    m2_state = M2_STATE_INPUT_SELECT;
    selected_input = M2_INPUT_JOYSTICK;
    Buzzer_SetMute(1);  
    LCD_Clear(0, 0, 240, 320, UI_BG);
    LCD_DrawMode2InputSelect();
    SEG_ShowPair(0, 1, 0); 
}

void Mode2_ResetCanvas(void) {
    // Standardize: First point is the start position
    m2_path_x[0] = 120;
    m2_path_y[0] = 190;
    m2_path_count = 1;
    
    m2_cursor_x = 120;
    m2_cursor_y = 190;
    m2_visit_count = 0;
    target_pt_idx = 0;
    history_slot = 0;
    last_history_cmd = ' ';
    total_path_distance = 0;
    covered_distance = 0;
    remaining_time_ms = 0;
    overlap_counter = 0;
    strcpy(laser_line, "READY");
    strcpy(motion_line, "READY");
    m2_state = M2_STATE_DRAWING;
    LCD_DrawMode2Canvas();
    seg_mode = SEG_M2_TIMER; seg_tenths = 0;
}

static void DrawCanvasArrow(uint16_t x, uint16_t y, int dx, int dy, uint16_t color)
{
    if (dx == 0 && dy == 0) return;
    const int s = 5; 
    int adx = abs(dx);
    int ady = abs(dy);
    uint8_t is_diag = (adx * 10 >= ady * 4) && (ady * 10 >= adx * 4);

    if (!is_diag) {
        if (adx > ady) { // Horizontal
            if (dx > 0) { // Right
                LCD_DrawLine(x + s, y, x, y - s, color); LCD_DrawLine(x + s, y, x, y + s, color); LCD_DrawLine(x + s, y, x - s, y, color);
            } else { // Left
                LCD_DrawLine(x - s, y, x, y - s, color); LCD_DrawLine(x - s, y, x, y + s, color); LCD_DrawLine(x - s, y, x + s, y, color);
            }
        } else { // Vertical
            if (dy > 0) { // Down
                LCD_DrawLine(x, y + s, x - s, y, color); LCD_DrawLine(x, y + s, x + s, y, color); LCD_DrawLine(x, y + s, x, y - s, color);
            } else { // Up
                LCD_DrawLine(x, y - s, x - s, y, color); LCD_DrawLine(x, y - s, x + s, y, color); LCD_DrawLine(x, y - s, x, y + s, color);
            }
        }
    } else { // Diagonal
        if (dx > 0 && dy < 0) { // NE
            LCD_DrawLine(x + s, y - s, x + s - 6, y - s, color); LCD_DrawLine(x + s, y - s, x + s, y - s + 6, color); LCD_DrawLine(x + s, y - s, x - s, y + s, color);
        } else if (dx > 0 && dy > 0) { // SE
            LCD_DrawLine(x + s, y + s, x + s - 6, y + s, color); LCD_DrawLine(x + s, y + s, x + s, y + s - 6, color); LCD_DrawLine(x + s, y + s, x - s, y - s, color);
        } else if (dx < 0 && dy > 0) { // SW
            LCD_DrawLine(x - s, y + s, x - s + 6, y + s, color); LCD_DrawLine(x - s, y + s, x - s, y + s - 6, color); LCD_DrawLine(x - s, y + s, x + s, y - s, color);
        } else if (dx < 0 && dy < 0) { // NW
            LCD_DrawLine(x - s, y - s, x - s + 6, y - s, color); LCD_DrawLine(x - s, y - s, x - s, y - s + 6, color); LCD_DrawLine(x - s, y - s, x + s, y + s, color);
        }
    }
}

static uint16_t GetTimeColor(uint32_t ms) {
    uint32_t s = ms / 1000;
    if (s < 5)  return BLACK;
    if (s < 15) return GREEN;
    if (s < 25) return BLUE;
    if (s < 35) return RED;
    return GREY;
}

void Mode2_Run(uint32_t joy_x, uint32_t joy_y, uint8_t k1_click, uint8_t k2_click, 
               uint8_t fire_pressed, uint8_t ts_pressed, uint8_t ts_click, uint16_t ts_x, uint16_t ts_y,
               uint8_t joy_up, uint8_t joy_down, uint8_t joy_left, uint8_t joy_right) {
    uint32_t now = HAL_GetTick();

    if (m2_state == M2_STATE_INPUT_SELECT) {
        if (k1_click || joy_up || joy_down) { selected_input = (selected_input == M2_INPUT_JOYSTICK) ? M2_INPUT_TOUCH : M2_INPUT_JOYSTICK; LCD_UpdateMode2InputSelect(); SEG_ShowPair(0, selected_input + 1, 0); Buzzer_BeepShort(); HAL_Delay(50); }
        if (ts_click) {
            mode2_input_method_t touched_input = selected_input; uint8_t hit = 0;
            if (ts_y >= 70 && ts_y <= 150) { touched_input = M2_INPUT_JOYSTICK; hit = 1; }
            else if (ts_y >= 170 && ts_y <= 250) { touched_input = M2_INPUT_TOUCH; hit = 1; }
            if (hit) {
                if (touched_input == selected_input) { m2_input_method = selected_input; Buzzer_BeepShort(); Mode2_ResetCanvas(); return; }
                else { selected_input = touched_input; LCD_UpdateMode2InputSelect(); SEG_ShowPair(0, selected_input + 1, 0); Buzzer_BeepShort(); HAL_Delay(50); }
            }
        }
        if (k2_click) { m2_input_method = selected_input; Buzzer_BeepShort(); Mode2_ResetCanvas(); }
    }
    else if (m2_state == M2_STATE_DRAWING) {
        uint16_t old_x = m2_cursor_x; uint16_t old_y = m2_cursor_y;

        if (m2_input_method == M2_INPUT_JOYSTICK) {
            if (joy_x < X_LEFT_THRESH_ADC && m2_cursor_x > CANVAS_X_MIN) m2_cursor_x -= 3;
            if (joy_x > X_RIGHT_THRESH_ADC && m2_cursor_x < CANVAS_X_MAX) m2_cursor_x += 3;
            if (joy_y < Y_FWD_THRESH_ADC && m2_cursor_y > CANVAS_Y_MIN) m2_cursor_y -= 3;
            if (joy_y > Y_BACK_THRESH_ADC && m2_cursor_y < CANVAS_Y_MAX) m2_cursor_y += 3;
        } 
        else {  
            if (ts_pressed && ts_x >= CANVAS_X_MIN && ts_x <= CANVAS_X_MAX && ts_y >= CANVAS_Y_MIN && ts_y <= CANVAS_Y_MAX) {
                m2_cursor_x = ts_x; m2_cursor_y = ts_y;
            }
        }

        if (m2_cursor_x < CANVAS_X_MIN) m2_cursor_x = CANVAS_X_MIN;
        if (m2_cursor_x > CANVAS_X_MAX) m2_cursor_x = CANVAS_X_MAX;
        if (m2_cursor_y < CANVAS_Y_MIN) m2_cursor_y = CANVAS_Y_MIN;
        if (m2_cursor_y > CANVAS_Y_MAX) m2_cursor_y = CANVAS_Y_MAX;

        if (old_x != m2_cursor_x || old_y != m2_cursor_y) {
            uint8_t is_overlap = 0;
            if (m2_path_count > 10) {
                for(int i=0; i<m2_path_count-10; i++) {
                    if (abs(m2_cursor_x - m2_path_x[i]) <= 12 && abs(m2_cursor_y - m2_path_y[i]) <= 12) { is_overlap = 1; break; }
                }
            }
            uint16_t line_color = BLUE;
            if (is_overlap) { line_color = rainbow_colors[overlap_counter % 6]; overlap_counter++; }
            LCD_Clear(old_x - 5, old_y - 5, 11, 11, line_color);
            LCD_DrawLine(old_x, old_y, m2_cursor_x, m2_cursor_y, line_color);
            uint16_t current_color = GetTimeColor(remaining_time_ms);
            LCD_Clear(m2_cursor_x - 5, m2_cursor_y - 5, 11, 11, current_color);
            LCD_Clear(m2_cursor_x - 2, m2_cursor_y - 2, 5, 5, (current_color == BLACK) ? WHITE : BLACK); 
            LCD_DrawMode2Stats(m2_path_count, MAX_POINTS, total_path_distance, remaining_time_ms / 1000);
            seg_tenths = remaining_time_ms / 100;
        }

        if (old_x != m2_cursor_x || old_y != m2_cursor_y) {
            // Path Normalization: Break long segments into 8-pixel intervals
            int dx_raw = (int)m2_cursor_x - (int)m2_path_x[m2_path_count-1];
            int dy_raw = (int)m2_cursor_y - (int)m2_path_y[m2_path_count-1];
            int dist = (int)sqrt(dx_raw*dx_raw + dy_raw*dy_raw);
            
            while (dist >= 8 && m2_path_count < MAX_POINTS) {
                int dx_cur = (int)m2_cursor_x - (int)m2_path_x[m2_path_count-1];
                int dy_cur = (int)m2_cursor_y - (int)m2_path_y[m2_path_count-1];
                float angle = atan2(dy_cur, dx_cur);
                
                m2_path_x[m2_path_count] = m2_path_x[m2_path_count-1] + (int)(8 * cos(angle));
                m2_path_y[m2_path_count] = m2_path_y[m2_path_count-1] + (int)(8 * sin(angle));
                
                total_path_distance += 8;
                remaining_time_ms += 8 * 12;
                m2_path_count++;
                
                dist = (int)sqrt((m2_cursor_x - m2_path_x[m2_path_count-1])*(m2_cursor_x - m2_path_x[m2_path_count-1]) + 
                                 (m2_cursor_y - m2_path_y[m2_path_count-1])*(m2_cursor_y - m2_path_y[m2_path_count-1]));
            }
            seg_tenths = remaining_time_ms / 100;
        }

        if (k1_click) { LCD_DrawMode2ResetConfirm(); m2_state = M2_STATE_RESET_CONFIRM; }
        if (k2_click && m2_path_count > 1) {
            LCD_Clear(0, 22, 240, 52, UI_BG);
            m2_state = M2_STATE_MOVING;
            target_pt_idx = 0; m2_visit_count = 0; history_slot = 0; last_history_cmd = ' ';
            covered_distance = 0;
            Motor_SendCmd('S', 0); move_timer = now + 400; 
        }
    }
    else if (m2_state == M2_STATE_RESET_CONFIRM) {
        if (k1_click) { 
            LCD_DrawMode2Canvas(); 
            if (m2_path_count > 1) { 
                for(int i=0; i<m2_path_count-1; i++) LCD_DrawLine(m2_path_x[i], m2_path_y[i], m2_path_x[i+1], m2_path_y[i+1], BLUE); 
            } 
            m2_state = M2_STATE_DRAWING; 
        }
        if (k2_click) { Mode2_ResetCanvas(); }
    }
    else if (m2_state == M2_STATE_MOVING) {
        if (target_pt_idx < m2_path_count) {
            uint16_t curr_x = m2_path_x[target_pt_idx];
            uint16_t curr_y = m2_path_y[target_pt_idx];
            int dx = 0, dy = 0;
            if (target_pt_idx < m2_path_count - 1) {
                dx = (int)m2_path_x[target_pt_idx + 1] - (int)m2_path_x[target_pt_idx];
                dy = (int)m2_path_y[target_pt_idx + 1] - (int)m2_path_y[target_pt_idx];
            } else if (target_pt_idx > 0) {
                dx = (int)m2_path_x[target_pt_idx] - (int)m2_path_x[target_pt_idx-1];
                dy = (int)m2_path_y[target_pt_idx] - (int)m2_path_y[target_pt_idx-1];
            }
            DrawCanvasArrow(curr_x, curr_y, dx, dy, RED);
            if (m2_visit_count > 0) LCD_DrawLine(m2_visit_x[m2_visit_count - 1], m2_visit_y[m2_visit_count - 1], curr_x, curr_y, RED);
        }

        LCD_DrawMode2Stats(covered_distance, total_path_distance, total_path_distance, remaining_time_ms / 1000);
        seg_tenths = remaining_time_ms / 100;

        if (target_pt_idx >= m2_path_count) {
            Motor_SendCmd('S', 0); SEG_ShowPair(0, 0, 0); seg_mode = SEG_IDLE;
            LCD_SetColors(RED, UI_BG); 
            LCD_ClearTextField(150, 22, 11, UI_BG);
            LCD_TEXT(150, 22, "READY! (K1)"); 
            LCD_SetColors(BLUE, UI_BG);
            m2_state = M2_STATE_SHOOTING; return;
        }

        if (now >= move_timer) {
            if (m2_visit_count < MAX_POINTS) { m2_visit_x[m2_visit_count] = m2_path_x[target_pt_idx]; m2_visit_y[m2_visit_count] = m2_path_y[target_pt_idx]; m2_visit_count++; }
            if (target_pt_idx == m2_path_count - 1) { target_pt_idx++; move_timer = now + 50; return; }
            int dx = (int)m2_path_x[target_pt_idx + 1] - (int)m2_path_x[target_pt_idx];
            int dy = (int)m2_path_y[target_pt_idx + 1] - (int)m2_path_y[target_pt_idx];
            char cmd = 'F'; 
            if (abs(dx) >= abs(dy)) { cmd = (dx > 0) ? 'R' : 'L'; } else { cmd = 'F'; }
            uint32_t seg_dist = (uint32_t)sqrt(dx*dx + dy*dy);
            uint32_t duration = seg_dist * 12; 
            if (duration > 0) {
                Motor_SendCmd(cmd, 60);
                uint8_t prev_slot = (history_slot == 0) ? 9 : (history_slot - 1);
                if (last_history_cmd != ' ') LCD_DrawMode2CommandHistory(last_history_cmd, prev_slot, 0); 
                LCD_DrawMode2CommandHistory(cmd, history_slot, 1); 
                last_history_cmd = cmd; history_slot = (history_slot + 1) % 10;
                covered_distance += seg_dist;
                if (remaining_time_ms > duration) remaining_time_ms -= duration; else remaining_time_ms = 0;
                move_timer = now + duration; target_pt_idx++;
            } else { target_pt_idx++; }
            seg_tenths = remaining_time_ms / 100;
        }
    } 
    else if (m2_state == M2_STATE_SHOOTING) {
        Motor_SendCmd('S', 0);
        if (k1_click || fire_pressed) {
            LCD_ClearTextField(150, 22, 11, UI_BG);
            LCD_SetColors(RED, UI_BG); 
            LCD_TEXT(150, 22, "FIRING!!!");
            HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_SET);
            RGB_Set(0, 1, 0); HAL_Delay(500);
            HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
            RGB_Set(1, 1, 1);
            LCD_ClearTextField(150, 22, 11, UI_BG);
            LCD_TEXT(150, 22, "FINISHED!");
            LCD_TEXT(150, 38, "K2:RESET"); 
            LCD_SetColors(BLUE, UI_BG); 
            m2_state = M2_STATE_FINISHED;
        }
    }
    else if (m2_state == M2_STATE_FINISHED) { if (k2_click) { Mode2_ResetCanvas(); } }
}
