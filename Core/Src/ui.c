#include "ui.h"
#include "lcd.h"
#include "peripherals.h"
#include "mode_2.h"
#include <stdio.h>
#include <string.h>

app_state_t last_drawn_state = (app_state_t)255;
game_mode_t last_drawn_mode = (game_mode_t)255;
car_type_t  last_drawn_car = (car_type_t)255;
uint32_t    lcd_fast_tick = 0;
uint32_t    lcd_slow_tick = 0;
uint8_t     touch_display_flag = 0U;

void LCD_ClearTextField(uint16_t x, uint16_t y, uint16_t chars, uint16_t bg)
{
    // Width = 8, Height = 16 for standard font
    LCD_Clear(x, y, chars * 8, 16, bg); 
}

void LCD_DrawModeSelect(void)
{
    LCD_Clear(0, 0, 240, 320, UI_BG);
    LCD_Clear(0, 0, 240, 6, UI_HEAD);
    LCD_TEXT(10, 12, "SELECT MODE");
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
    LCD_Clear(0, 0, 240, 6, UI_HEAD);
    LCD_TEXT(10, 10, "SELECT CAR");
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
    LCD_Clear(0, 0, 240, 6, UI_HEAD);

    LCD_TEXT(10, 10,  "WiFi:");
    LCD_TEXT(10, 30,  "IP:");
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
    LCD_Clear(0, 0, 240, 40, UI_HEAD);
    LCD_TEXT(40, 10, "SELECT CONTROL");
    
    // Joystick Section
    if (selected_input == M2_INPUT_JOYSTICK) {
        LCD_Clear(10, 70, 220, 80, BLUE);
        LCD_Clear(12, 72, 216, 76, WHITE);
        LCD_TEXT(40, 102, ">> [JOYSTICK] <<");
    } else {
        LCD_Clear(30, 80, 180, 60, GREY);
        LCD_TEXT(55, 102, "[JOYSTICK]");
    }
    
    // Touch Section
    if (selected_input == M2_INPUT_TOUCH) {
        LCD_Clear(10, 170, 220, 80, MAGENTA);
        LCD_Clear(12, 172, 216, 76, WHITE);
        LCD_TEXT(25, 202, ">> [TOUCH SCREEN] <<");
    } else {
        LCD_Clear(30, 180, 180, 60, GREY);
        LCD_TEXT(45, 202, "[TOUCH SCREEN]");
    }
    
    LCD_Clear(0, 280, 240, 40, UI_BOTTOM);
    LCD_TEXT(20, 292, "K1:Switch  K2:Confirm");
}

void LCD_UpdateMode2InputSelect(void) {
    // Just refresh the two control boxes
    // Joystick Section
    if (selected_input == M2_INPUT_JOYSTICK) {
        LCD_Clear(10, 70, 220, 80, BLUE);
        LCD_Clear(12, 72, 216, 76, WHITE);
        LCD_SetColors(BLUE, WHITE);
        LCD_TEXT(40, 102, ">> [JOYSTICK] <<");
    } else {
        LCD_Clear(30, 80, 180, 60, GREY);
        LCD_SetColors(BLUE, GREY);
        LCD_TEXT(55, 102, "[JOYSTICK]");
    }
    
    // Touch Section
    if (selected_input == M2_INPUT_TOUCH) {
        LCD_Clear(10, 170, 220, 80, MAGENTA);
        LCD_Clear(12, 172, 216, 76, WHITE);
        LCD_SetColors(MAGENTA, WHITE);
        LCD_TEXT(25, 202, ">> [TOUCH SCREEN] <<");
    } else {
        LCD_Clear(30, 180, 180, 60, GREY);
        LCD_SetColors(BLUE, GREY);
        LCD_TEXT(45, 202, "[TOUCH SCREEN]");
    }
    LCD_SetColors(BLUE, WHITE); // Reset to default
}

void LCD_DrawMode2CommandHistory(char cmd, uint8_t slot, uint8_t is_new)
{
    uint16_t x = 5 + (slot * 23);
    uint16_t y = 292;
    
    if (is_new) {
        LCD_SetColors(GREEN, UI_BOTTOM);
    } else {
        LCD_SetColors(BLUE, UI_BOTTOM);
    }
    
    LCD_DrawChar(x, y, cmd);
    LCD_SetColors(BLUE, WHITE); // Reset
}

void LCD_DrawMode2Canvas(void) 
{
    LCD_Clear(0, 0, 240, 320, UI_BG);
    LCD_Clear(0, 0, 240, 20, UI_HEAD);
    LCD_SetColors(BLUE, UI_HEAD);
    LCD_TEXT(10, 5, "DRAW FIGHT");
    
    LCD_SetColors(BLUE, UI_BG);
    if (m2_input_method == M2_INPUT_TOUCH) {
        LCD_TEXT(10, 30, "Click on LCD to start");
        LCD_TEXT(10, 48, "the route drawing");
    } else {
        LCD_TEXT(10, 30, "Use Joy to Draw");
    }
    
    LCD_TEXT(10, 65, "K1:Clear K2:Confirm");
    
    // Draw boundary line
    LCD_DrawLine(0, 85, 240, 85, MY_BLACK);
}

void LCD_DrawMode2ResetConfirm(void)
{
    // Semi-transparent look is hard with simple LCD, so just draw a box
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

    LCD_ClearTextField(60, 10, 20, UI_BG);
    LCD_TEXT(60, 10, wifi_line1);

    LCD_ClearTextField(40, 30, 24, UI_BG);
    LCD_TEXT(40, 30, wifi_line2);

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
