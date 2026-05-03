#include "ui.h"
#include "lcd.h"
#include "peripherals.h"
#include "mode_2.h"
#include "mode_3.h"
#include "alerts.h"
#include <stdio.h>
#include <string.h>

app_state_t last_drawn_state = (app_state_t)255;
game_mode_t last_drawn_mode = (game_mode_t)255;
car_type_t  last_drawn_car = (car_type_t)255;
uint32_t    lcd_fast_tick = 0;
uint32_t    lcd_slow_tick = 0;
uint8_t     touch_display_flag = 0U;

// WiFi List Variables
char wifi_ssids[MAX_WIFI_NETWORKS][33];
uint8_t wifi_count = 0;
int8_t selected_wifi_idx = -1;

// Keyboard Variables
char keyboard_buffer[33] = "";
uint8_t kb_shift = 0;

void LCD_ClearTextField(uint16_t x, uint16_t y, uint16_t chars, uint16_t bg)
{
    LCD_Clear(x, y, chars * 8, 16, bg); 
}

void LCD_DrawStatusBar(void)
{
    LCD_Clear(0, 0, 240, 20, UI_HEAD);
    LCD_SetColors(BLUE, UI_HEAD);
    
    char ip_line[40];
    if (strlen(wifi_line2) > 5) {
        snprintf(ip_line, sizeof(ip_line), "IP:%s", wifi_line2);
    } else {
        strcpy(ip_line, "WiFi:DISCONNECTED");
    }
    LCD_TEXT(5, 2, ip_line);
    LCD_SetColors(BLUE, WHITE);
}

void LCD_DrawHome(void)
{
    LCD_Clear(0, 0, 240, 320, UI_BG);
    LCD_DrawStatusBar();
    
    LCD_SetColors(BLACK, UI_BG);
    LCD_TEXT(55, 60, "MASTER CONTROL");
    
    LCD_Clear(20, 100, 200, 60, BLUE);
    LCD_Clear(22, 102, 196, 56, WHITE);
    LCD_SetColors(BLUE, WHITE);
    LCD_TEXT(80, 122, ">> BATTLE <<");
    
    LCD_Clear(20, 180, 200, 60, MY_GRAY);
    LCD_Clear(22, 182, 196, 56, WHITE);
    LCD_SetColors(MY_GRAY, WHITE);
    LCD_TEXT(75, 202, ">> SETTINGS <<");
    
    LCD_Clear(0, 280, 240, 40, UI_BOTTOM);
    LCD_SetColors(WHITE, UI_BOTTOM);
    LCD_TEXT(40, 292, "TOUCH SCREEN TO START");
    LCD_SetColors(BLUE, WHITE);
}

void LCD_DrawSettings(void)
{
    LCD_Clear(0, 0, 240, 320, UI_BG);
    LCD_DrawStatusBar();
    
    LCD_SetColors(BLACK, UI_BG);
    LCD_TEXT(10, 30, "NETWORK SETTINGS");
    
    LCD_DrawWiFiList();
    
    LCD_Clear(0, 280, 240, 40, UI_BOTTOM);
    LCD_SetColors(WHITE, UI_BOTTOM);
    LCD_TEXT(10, 292, "K1:BACK   K2:SCAN");
    LCD_SetColors(BLUE, WHITE);
}

void LCD_DrawWiFiList(void)
{
    LCD_Clear(10, 60, 220, 160, WHITE);
    LCD_DrawRectangle(10, 60, 220, 160, BLACK);
    
    if (wifi_count == 0) {
        LCD_SetColors(BLACK, WHITE);
        LCD_TEXT(20, 80, "No Networks Found");
        LCD_TEXT(20, 100, "Press K2 to Scan");
    } else {
        for (int i = 0; i < wifi_count && i < 7; i++) {
            uint16_t y = 65 + (i * 22);
            if (i == selected_wifi_idx) {
                LCD_Clear(12, y-2, 216, 20, UI_BOX_SEL);
                LCD_SetColors(WHITE, UI_BOX_SEL);
            } else {
                LCD_SetColors(BLACK, WHITE);
            }
            LCD_TEXT(15, y, wifi_ssids[i]);
        }
    }
    LCD_SetColors(BLUE, WHITE);
}

void LCD_DrawKeyboard(const char* current_input)
{
    LCD_Clear(0, 0, 240, 320, UI_BG);
    LCD_DrawStatusBar();
    
    LCD_SetColors(BLACK, UI_BG);
    LCD_TEXT(10, 25, "ENTER PASSWORD:");
    
    // Input Box
    LCD_Clear(10, 45, 220, 30, WHITE);
    LCD_DrawRectangle(10, 45, 220, 30, BLACK);
    LCD_SetColors(BLACK, WHITE);
    LCD_TEXT(15, 52, current_input);
    
    // Keyboard Grid
    const char* keys = kb_shift ? "ABCDEF GHIJKL MNOPQR STUVWX YZ0123 456789" : "abcdef ghijkl mnopqr stuvwx yz.,-_ !?@#$%";
    
    for (int r = 0; r < 6; r++) {
        for (int c = 0; c < 6; c++) {
            int idx = (r * 6) + c;
            if (idx >= (int)strlen(keys)) break;
            
            uint16_t x = 10 + (c * 38);
            uint16_t y = 90 + (r * 32);
            
            LCD_Clear(x, y, 35, 28, UI_BOX_NSEL);
            LCD_SetColors(BLACK, UI_BOX_NSEL);
            char key_str[2] = {keys[idx], '\0'};
            LCD_TEXT(x + 12, y + 6, key_str);
        }
    }
    
    // Special Keys
    LCD_Clear(10, 282, 70, 35, kb_shift ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 292, "SHIFT");
    
    LCD_Clear(85, 282, 70, 35, RED);
    LCD_TEXT(95, 292, "BACK");
    
    LCD_Clear(160, 282, 70, 35, GREEN);
    LCD_TEXT(170, 292, "ENTER");
    
    LCD_SetColors(BLUE, WHITE);
}

void LCD_DrawMode3Placeholder(void)
{
    LCD_Clear(0, 0, 240, 320, UI_BG);
    LCD_DrawStatusBar();
    
    LCD_SetColors(RED, UI_BG);
    LCD_TEXT(60, 60, "MODE 3: TANK ARENA");
    
    LCD_SetColors(BLACK, UI_BG);
    LCD_TEXT(20, 100, "Initializing Seed...");
    
    char seed_str[32];
    snprintf(seed_str, sizeof(seed_str), "Seed: %lu", m3_world_seed);
    LCD_TEXT(20, 125, seed_str);
    
    LCD_SetColors(BLUE, UI_BG);
    LCD_TEXT(20, 160, "Waiting for Phone...");
    LCD_TEXT(20, 185, "(K2 to Sim Connect)");
    
    LCD_Clear(0, 280, 240, 40, UI_BOTTOM);
    LCD_SetColors(WHITE, UI_BOTTOM);
    LCD_TEXT(60, 292, "[ SILENT MODE ]");
    LCD_SetColors(BLUE, WHITE);
}

void LCD_DrawModeSelect(void)
{
    LCD_Clear(0, 0, 240, 320, UI_BG);
    LCD_DrawStatusBar();
    LCD_TEXT(10, 32, "K1:NEXT   K2:CONFIRM");

    LCD_Clear(20, 70, 200, 40, (selected_mode == GAME_MODE_1) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(80, 82, "MODE 1");
    LCD_Clear(20, 125, 200, 40, (selected_mode == GAME_MODE_2) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(80, 137, "MODE 2");
    LCD_Clear(20, 180, 200, 40, (selected_mode == GAME_MODE_3) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(80, 192, "MODE 3");

    LCD_Clear(0, 250, 240, 70, UI_BOTTOM);
    LCD_TEXT(10, 260, "MODE 1 = PLAY NOW");
    LCD_TEXT(10, 280, "MODE 2 = DRAW FIGHT");
}

void LCD_UpdateModeSelection(void)
{
    LCD_Clear(20, 70, 200, 40, (selected_mode == GAME_MODE_1) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(80, 82, "MODE 1");
    LCD_Clear(20, 125, 200, 40, (selected_mode == GAME_MODE_2) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(80, 137, "MODE 2");
    LCD_Clear(20, 180, 200, 40, (selected_mode == GAME_MODE_3) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(80, 192, "MODE 3");
}

void LCD_DrawModeConfirm(void)
{
    LCD_Clear(18, 78, 204, 164, MY_BLACK);  
    LCD_Clear(20, 80, 200, 160, WHITE);     
    
    LCD_TEXT(30, 95, "You choosed:");
    LCD_TEXT(30, 115, (char *)MODE_Name(selected_mode));
    LCD_TEXT(30, 135, "Are you sure?");
    LCD_TEXT(30, 155, "Once confirmed");
    LCD_TEXT(30, 175, "cannot change!");

    LCD_Clear(30, 195, 80, 35, MY_GRAY);
    LCD_TEXT(42, 200, "REGRET");
    LCD_TEXT(50, 215, "(K1)");
    
    LCD_Clear(130, 195, 80, 35, MY_GREEN);
    LCD_TEXT(138, 200, "CONFIRM");
    LCD_TEXT(150, 215, "(K2)");
}

void LCD_DrawCarSelect(void)
{
    char line[32];
    LCD_Clear(0, 0, 240, 320, UI_BG);
    LCD_DrawStatusBar();
    LCD_TEXT(10, 28, "K1:NEXT K2:START");

    snprintf(line, sizeof(line), "MODE:%s", MODE_Name(selected_mode));
    LCD_TEXT(10, 46, line);

    LCD_Clear(14,  64, 212, 20, (selected_car == CAR_V0) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 68, "V0 STANDARD");
    LCD_Clear(14,  88, 212, 20, (selected_car == CAR_V1) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 92, "V1 AUTO FIRE");
    LCD_Clear(14, 112, 212, 20, (selected_car == CAR_V2) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 116, "V2 RAPID SHOT");
    LCD_Clear(14, 136, 212, 20, (selected_car == CAR_V3) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 140, "V3 MOVING CAST");
    LCD_Clear(14, 160, 212, 20, (selected_car == CAR_V4) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 164, "V4 FORWARD SPD");
    LCD_Clear(14, 184, 212, 20, (selected_car == CAR_V5) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 188, "V5 LONG BEAM");
    LCD_Clear(14, 208, 212, 20, (selected_car == CAR_V6) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 212, "V6 GUN PLATFORM");

    LCD_Clear(0, 246, 240, 74, UI_BOTTOM);
    LCD_TEXT(10, 256, "CAR:");
    LCD_TEXT(60, 256, (char *)CAR_Code(selected_car));
    LCD_TEXT(10, 278, "TYPE:");
    LCD_TEXT(60, 278, (char *)CAR_Label(selected_car));
}

void LCD_UpdateCarSelection(void)
{
    LCD_Clear(14,  64, 212, 20, (selected_car == CAR_V0) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 68, "V0 STANDARD");
    LCD_Clear(14,  88, 212, 20, (selected_car == CAR_V1) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 92, "V1 AUTO FIRE");
    LCD_Clear(14, 112, 212, 20, (selected_car == CAR_V2) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 116, "V2 RAPID SHOT");
    LCD_Clear(14, 136, 212, 20, (selected_car == CAR_V3) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 140, "V3 MOVING CAST");
    LCD_Clear(14, 160, 212, 20, (selected_car == CAR_V4) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 164, "V4 FORWARD SPD");
    LCD_Clear(14, 184, 212, 20, (selected_car == CAR_V5) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 188, "V5 LONG BEAM");
    LCD_Clear(14, 208, 212, 20, (selected_car == CAR_V6) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 212, "V6 GUN PLATFORM");

    LCD_ClearTextField(60, 256, 8, UI_BOTTOM);
    LCD_TEXT(60, 256, (char *)CAR_Code(selected_car));
    LCD_ClearTextField(60, 278, 20, UI_BOTTOM);
    LCD_TEXT(60, 278, (char *)CAR_Label(selected_car));
}

void LCD_DrawCarConfirm(void)
{
    LCD_Clear(18, 78, 204, 164, MY_BLACK);  
    LCD_Clear(20, 80, 200, 160, WHITE);     
    
    LCD_TEXT(30, 95, "You choosed:");
    LCD_TEXT(30, 115, (char *)CAR_Label(selected_car));
    LCD_TEXT(30, 135, "Are you sure?");
    LCD_TEXT(30, 155, "Once confirmed");
    LCD_TEXT(30, 175, "cannot change!");

    LCD_Clear(30, 195, 80, 35, MY_GRAY);
    LCD_TEXT(42, 200, "REGRET");
    LCD_TEXT(50, 215, "(K1)");
    
    LCD_Clear(130, 195, 80, 35, MY_GREEN);
    LCD_TEXT(138, 200, "CONFIRM");
    LCD_TEXT(150, 215, "(K2)");
}

void LCD_DrawGameLayout(void)
{
    LCD_Clear(0, 0, 240, 320, UI_BG);
    LCD_DrawStatusBar();

    LCD_TEXT(10, 50,  "Mode:");
    LCD_TEXT(10, 70,  "Direction:");
    LCD_TEXT(10, 90,  "Speed:");
    LCD_TEXT(10, 110, "Button:");
    LCD_TEXT(10, 130, "Laser:");
    LCD_TEXT(10, 150, "Motion:");
    LCD_TEXT(10, 170, "ESP:");
    LCD_TEXT(10, 190, "Touch:");

    LCD_Clear(0, 215, 240, 105, UI_BOTTOM);
    LCD_TEXT(10, 225, "Car:");
    LCD_TEXT(10, 245, "Car Type:");
}

void LCD_DrawMode2InputSelect(void) {
    LCD_Clear(0, 0, 240, 320, UI_BG);
    LCD_DrawStatusBar();
    LCD_TEXT(40, 30, "SELECT CONTROL");
    
    if (selected_input == M2_INPUT_JOYSTICK) {
        LCD_Clear(10, 70, 220, 80, BLUE);
        LCD_Clear(12, 72, 216, 76, WHITE);
        LCD_SetColors(BLUE, WHITE);
        LCD_TEXT(40, 102, ">> [JOYSTICK] <<");
    } else {
        LCD_Clear(10, 70, 220, 80, GREY);
        LCD_Clear(12, 72, 216, 76, WHITE);
        LCD_SetColors(BLACK, WHITE);
        LCD_TEXT(55, 102, "[JOYSTICK]");
    }
    
    if (selected_input == M2_INPUT_TOUCH) {
        LCD_Clear(10, 170, 220, 80, MAGENTA);
        LCD_Clear(12, 172, 216, 76, WHITE);
        LCD_SetColors(MAGENTA, WHITE);
        LCD_TEXT(25, 202, ">> [TOUCH SCREEN] <<");
    } else {
        LCD_Clear(10, 170, 220, 80, GREY);
        LCD_Clear(12, 172, 216, 76, WHITE);
        LCD_SetColors(BLACK, WHITE);
        LCD_TEXT(45, 202, "[TOUCH SCREEN]");
    }
    
    LCD_Clear(0, 280, 240, 40, UI_BOTTOM);
    LCD_TEXT(20, 292, "K1:Switch  K2:Confirm");
    LCD_SetColors(BLUE, WHITE);
}

void LCD_UpdateMode2InputSelect(void) {
    // Redraw with standard clear blocks to ensure NO artifacts
    LCD_Clear(10, 70, 220, 80, (selected_input == M2_INPUT_JOYSTICK) ? BLUE : GREY);
    LCD_Clear(12, 72, 216, 76, WHITE);
    if (selected_input == M2_INPUT_JOYSTICK) {
        LCD_SetColors(BLUE, WHITE); LCD_TEXT(40, 102, ">> [JOYSTICK] <<");
    } else {
        LCD_SetColors(BLACK, WHITE); LCD_TEXT(55, 102, "[JOYSTICK]");
    }

    LCD_Clear(10, 170, 220, 80, (selected_input == M2_INPUT_TOUCH) ? MAGENTA : GREY);
    LCD_Clear(12, 172, 216, 76, WHITE);
    if (selected_input == M2_INPUT_TOUCH) {
        LCD_SetColors(MAGENTA, WHITE); LCD_TEXT(25, 202, ">> [TOUCH SCREEN] <<");
    } else {
        LCD_SetColors(BLACK, WHITE); LCD_TEXT(45, 202, "[TOUCH SCREEN]");
    }
    LCD_SetColors(BLUE, WHITE);
}

void LCD_DrawMode2CommandHistory(char cmd, uint8_t slot, uint8_t is_new)
{
    uint16_t x = 5 + (slot * 23);
    uint16_t y = 55; 
    
    if (cmd == '>') cmd = 'R';
    if (cmd == '<') cmd = 'L';
    if (cmd == 'B') cmd = 'F'; 
    
    if (is_new) {
        LCD_SetColors(GREEN, UI_BG);
    } else {
        LCD_SetColors(BLUE, UI_BG);
    }
    
    LCD_DrawChar(x, y, cmd);
    LCD_SetColors(BLUE, WHITE);
}

void LCD_DrawMode2Stats(uint16_t current_move, uint16_t total_moves, uint32_t distance, uint32_t seconds_left)
{
    char line1[32];
    char line2[32];
    
    snprintf(line1, sizeof(line1), "Steps: %u/%u", current_move, total_moves);
    snprintf(line2, sizeof(line2), "D:%lu T:%lus", distance, seconds_left);
    
    LCD_SetColors(BLACK, UI_BG);
    LCD_ClearTextField(10, 22, 18, UI_BG);
    LCD_TEXT(10, 22, line1);
    LCD_ClearTextField(10, 38, 22, UI_BG);
    LCD_TEXT(10, 38, line2);
    LCD_SetColors(BLUE, WHITE);
}

void LCD_DrawMode2Canvas(void) 
{
    LCD_Clear(0, 0, 240, 320, UI_BG);
    LCD_DrawStatusBar();
    LCD_SetColors(BLUE, UI_HEAD);
    LCD_TEXT(10, 2, "DRAW FIGHT"); 
    
    LCD_SetColors(BLUE, UI_BG);
    if (m2_state == M2_STATE_DRAWING) {
        LCD_TEXT(170, 22, "K1:RST");
        LCD_TEXT(170, 38, "K2:OK");
        
        if (m2_input_method == M2_INPUT_TOUCH) {
            LCD_TEXT(10, 55, "Click LCD to Draw");
        } else {
            LCD_TEXT(10, 55, "Use Joy to Draw");
        }
    }
    
    LCD_DrawLine(0, 75, 240, 75, MY_BLACK);
}

void LCD_DrawMode2ResetConfirm(void)
{
    LCD_Clear(20, 100, 200, 120, MY_BLACK);
    LCD_Clear(22, 102, 196, 116, WHITE);
    
    LCD_SetColors(BLUE, WHITE);
    LCD_TEXT(40, 115, "RESET CANVAS?");
    LCD_TEXT(40, 140, "K1: REGRET (NO)");
    LCD_TEXT(40, 165, "K2: CONFIRM (YES)");
}

void LCD_UpdateGameFast(uint32_t x_raw, uint32_t y_raw)
{
    char dir_str[16];
    char spd_str[16];
    uint8_t speed = 0;

    if (selected_mode != GAME_MODE_1) {
        strcpy(dir_str, "PLACEHOLDER");
        strcpy(spd_str, "---");
    } else {
        if (y_raw < Y_FWD_THRESH_ADC) {
            strcpy(dir_str, "FORWARD");
            speed = map_range_percent(y_raw, Y_FWD_THRESH_ADC, ADC_MIN);
        } else if (x_raw < X_LEFT_THRESH_ADC) {
            strcpy(dir_str, "LEFT");
            speed = map_range_percent(x_raw, X_LEFT_THRESH_ADC, ADC_MIN);
        } else if (x_raw > X_RIGHT_THRESH_ADC) {
            strcpy(dir_str, "RIGHT");
            speed = map_range_percent(x_raw, X_RIGHT_THRESH_ADC, ADC_MAX);
        } else {
            strcpy(dir_str, "STOP");
            speed = 0;
        }

        speed = car_apply_speed_cap(dir_str[0], speed);
        snprintf(spd_str, sizeof(spd_str), "%3u%%", speed);
    }

    LCD_ClearTextField(50, 50, 12, UI_BG);
    LCD_TEXT(50, 50, (char *)MODE_Name(selected_mode));

    LCD_ClearTextField(90, 70, 14, UI_BG);
    LCD_TEXT(90, 70, dir_str);

    LCD_ClearTextField(70, 90, 10, UI_BG);
    LCD_TEXT(70, 90, spd_str);
}

void LCD_UpdateGameSlow(uint8_t fire_pressed)
{
    char btn_str[16];
    char laser_disp[32];
    char motion_disp[24];
    char esp_disp[8];
    char touch_str[16];

    if (fire_pressed) strcpy(btn_str, "PRESSED");
    else              strcpy(btn_str, "RELEASE");

    if (touch_ability_shots_needed == 0)
        strcpy(touch_str, "READY");
    else
        snprintf(touch_str, sizeof(touch_str), "WAIT %u SHT", touch_ability_shots_needed);

    snprintf(laser_disp, sizeof(laser_disp), "%s", laser_line);
    snprintf(motion_disp, sizeof(motion_disp), "%s", motion_line);
    snprintf(esp_disp, sizeof(esp_disp), "%s", esp_cmd_rx);

    LCD_ClearTextField(70, 110, 12, UI_BG);
    LCD_TEXT(70, 110, btn_str);

    LCD_ClearTextField(60, 130, 20, UI_BG);
    LCD_TEXT(60, 130, laser_disp);

    LCD_ClearTextField(70, 150, 20, UI_BG);
    LCD_TEXT(70, 150, motion_disp);

    LCD_ClearTextField(50, 170, 8, UI_BG);
    LCD_TEXT(50, 170, esp_disp);

    LCD_ClearTextField(70, 190, 16, UI_BG);
    LCD_TEXT(70, 190, touch_str);

    LCD_ClearTextField(50, 225, 8, UI_BOTTOM);
    LCD_TEXT(50, 225, (char *)CAR_Code(selected_car));

    LCD_ClearTextField(80, 245, 20, UI_BOTTOM);
    LCD_TEXT(80, 245, (char *)CAR_Label(selected_car));
}
