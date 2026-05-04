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

// Global Settings Definitions
ui_theme_t current_theme = THEME_DEFAULT;
uint8_t    audio_enabled = 1;
uint8_t    colorblind_mode = 0;
uint8_t    led_enabled = 1;
uint8_t    seg_enabled = 1;
ui_font_t  current_font = FONT_DEFAULT;

// Temporary Settings for UI
ui_theme_t temp_theme = THEME_DEFAULT;
uint8_t    temp_audio = 1;
uint8_t    temp_cb = 0;
uint8_t    temp_led = 1;
uint8_t    temp_seg = 1;
ui_font_t  temp_font = FONT_DEFAULT;
int8_t     settings_focus_idx = 0;
int8_t     home_focus_idx = 0;
int8_t     mode_focus_idx = 0;
int8_t     car_focus_idx = 0;

static const uint16_t car_palette[7] = {BLUE, CYAN, GREEN, YELLOW, MAGENTA, RED, 0xFD20};

static uint16_t CarColorByIndex(int8_t idx)
{
    if (idx < 0 || idx > 6) return UI_BOTTOM;
    return car_palette[idx];
}

static void DrawCarColorChip(uint16_t x, uint16_t y, uint16_t color, uint8_t selected)
{
    LCD_Clear(x, y, 16, 16, color);
    LCD_DrawRectangle(x, y, 16, 16, selected ? BLACK : WHITE);
}

static car_type_t CarPreviewIndex(void)
{
    if (car_focus_idx >= 0 && car_focus_idx <= 6) return (car_type_t)car_focus_idx;
    return selected_car;
}

static void LCD_DrawCarInfoPanel(car_type_t car, uint16_t bg)
{
    char line1[40];
    char line2[40];
    char line3[40];

    switch (car) {
        case CAR_V0:
            strcpy(line1, "V0 STANDARD");
            strcpy(line2, "SPD:100  CHG:1000");
            strcpy(line3, "FIRE:1000  CD:3000");
            break;
        case CAR_V1:
            strcpy(line1, "V1 AUTO FIRE");
            strcpy(line2, "SPD:70   CHG:1000");
            strcpy(line3, "FIRE:1000  CD:3000");
            break;
        case CAR_V2:
            strcpy(line1, "V2 RAPID SHOT");
            strcpy(line2, "SPD:100  CHG:1000");
            strcpy(line3, "FIRE:500   CD:1500");
            break;
        case CAR_V3:
            strcpy(line1, "V3 MOVING CAST");
            strcpy(line2, "SPD:100  CHG:1000");
            strcpy(line3, "FIRE:1000  CD:6000");
            break;
        case CAR_V4:
            strcpy(line1, "V4 FORWARD SPD");
            strcpy(line2, "SPD:200  CHG:800");
            strcpy(line3, "FIRE:400   CD:4000");
            break;
        case CAR_V5:
            strcpy(line1, "V5 LONG BEAM");
            strcpy(line2, "SPD:100  CHG:1200");
            strcpy(line3, "FIRE:1800  CD:5500");
            break;
        case CAR_V6:
            strcpy(line1, "V6 GUN PLATFORM");
            strcpy(line2, "SPD:60   CHG:400");
            strcpy(line3, "FIRE:400   CD:1800");
            break;
        default:
            strcpy(line1, "CAR");
            strcpy(line2, "SPD:100  CHG:1000");
            strcpy(line3, "FIRE:1000  CD:3000");
            break;
    }

    LCD_Clear(0, 246, 240, 74, bg);
    LCD_SetColors(BLACK, bg);
    LCD_TEXT(10, 252, line1);
    LCD_TEXT(10, 270, line2);
    LCD_TEXT(10, 288, line3);
}

// WiFi List Variables
char wifi_ssids[MAX_WIFI_NETWORKS][33];
uint8_t wifi_count = 0;
int8_t selected_wifi_idx = -1;

// Keyboard Variables
char keyboard_buffer[33] = "";
uint8_t kb_shift = 0;

void LCD_ClearTextField(uint16_t x, uint16_t y, uint16_t chars, uint16_t bg)
{
    LCD_Clear(x, y, chars * 10, 20, bg); 
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

void LCD_DrawBackButton(uint8_t selected)
{
    uint16_t bg = selected ? MY_GREEN : RED;
    uint16_t fg = selected ? BLACK : WHITE;
    LCD_Clear(5, 25, 45, 22, bg);
    LCD_SetColors(fg, bg);
    LCD_TEXT(10, 28, "BACK");
    LCD_SetColors(BLUE, WHITE);
}

void LCD_DrawHome(void)
{
    LCD_Clear(0, 0, 240, 320, UI_BG);
    LCD_DrawStatusBar();
    
    LCD_SetColors(BLACK, UI_BG);
    LCD_TEXT(55, 60, "TANK COMMANDER");
    
    // Modernized buttons with Focus Support
    uint16_t b1_color = (home_focus_idx == 0) ? UI_BOX_SEL : BLUE;
    LCD_Clear(20, 100, 200, 60, b1_color);
    LCD_DrawRectangle(20, 100, 200, 60, BLACK);
    LCD_SetColors((home_focus_idx == 0) ? BLACK : WHITE, b1_color);
    LCD_TEXT(80, 122, "START BATTLE");
    
    uint16_t b2_color = (home_focus_idx == 1) ? UI_BOX_SEL : DARK_GRAY;
    LCD_Clear(20, 180, 200, 60, b2_color);
    LCD_DrawRectangle(20, 180, 200, 60, BLACK);
    LCD_SetColors((home_focus_idx == 1) ? BLACK : WHITE, b2_color);
    LCD_TEXT(75, 202, "PREFERENCES");
    
    LCD_Clear(0, 280, 240, 40, DARK_GRAY);
    LCD_SetColors(WHITE, DARK_GRAY);
    LCD_TEXT(30, 292, "READY FOR ENGAGEMENT");
    LCD_SetColors(BLUE, WHITE);
}

void LCD_DrawSettings(void)
{
    LCD_Clear(0, 0, 240, 320, UI_BG);
    LCD_DrawStatusBar();
    LCD_DrawBackButton(settings_focus_idx == -1);
    
    LCD_SetColors(BLACK, UI_BG);
    LCD_TEXT(80, 25, "SETTINGS");
    
    for (uint8_t i = 0; i < 6; i++) {
        LCD_UpdateSettingsOption(i);
    }
    
    // Confirm / Back Area
    LCD_Clear(0, 280, 110, 40, RED);
    LCD_DrawRectangle(0, 280, 110, 40, BLACK);
    LCD_SetColors(WHITE, RED);
    LCD_TEXT(10, 292, "TAP: CANCEL");
    
    LCD_Clear(130, 280, 110, 40, MY_GREEN);
    LCD_DrawRectangle(130, 280, 110, 40, BLACK);
    LCD_SetColors(BLACK, MY_GREEN);
    LCD_TEXT(140, 292, "TAP: CONFIRM");
    
    LCD_SetColors(BLUE, WHITE);
}

void LCD_UpdateSettingsOption(uint8_t option_idx)
{
    uint16_t y = 45 + (option_idx * 33);
    uint16_t box_color = (settings_focus_idx == option_idx) ? UI_BOX_SEL : UI_BOX_NSEL;
    
    if (option_idx < 6) {
        LCD_Clear(20, y - 4, 200, 28, box_color);
        LCD_DrawRectangle(20, y - 4, 200, 28, BLACK);
        LCD_SetColors(BLACK, box_color);
        
        if (settings_focus_idx == option_idx) {
            LCD_TEXT(25, y, "->");
        } else {
            LCD_Clear(25, y, 15, 16, box_color);
        }
    }

    switch (option_idx) {
        case 0: // Theme
            {
                const char* themes[] = {"THEME: DEFAULT", "THEME: DARK", "THEME: LIGHT"};
                LCD_TEXT(45, y, themes[temp_theme]);
            }
            break;
        case 1: // Audio
            {
                char aud_buf[32]; snprintf(aud_buf, sizeof(aud_buf), "BUZZER: %s", temp_audio ? "ON" : "OFF");
                LCD_TEXT(45, y, aud_buf);
            }
            break;
        case 2: // Colorblind
            {
                char cb_buf[32]; snprintf(cb_buf, sizeof(cb_buf), "CONTRAST: %s", temp_cb ? "HIGH" : "NORMAL");
                LCD_TEXT(45, y, cb_buf);
            }
            break;
        case 3: // LED
            {
                char led_buf[32]; snprintf(led_buf, sizeof(led_buf), "RGB LED: %s", temp_led ? "ON" : "OFF");
                LCD_TEXT(45, y, led_buf);
            }
            break;
        case 4: // 7-SEG
            {
                char seg_buf[32]; snprintf(seg_buf, sizeof(seg_buf), "7-SEG: %s", temp_seg ? "ON" : "OFF");
                LCD_TEXT(45, y, seg_buf);
            }
            break;
        case 5: // Font
            {
                char font_buf[32]; snprintf(font_buf, sizeof(font_buf), "FONT: %s", temp_font == FONT_LARGE ? "LARGE" : "DEFAULT");
                LCD_TEXT(45, y, font_buf);
            }
            break;
    }
}

void LCD_DrawWiFiSettings(void)
{
    LCD_Clear(0, 0, 240, 320, UI_BG);
    LCD_DrawStatusBar();
    LCD_DrawBackButton(0);
    
    LCD_SetColors(BLACK, UI_BG);
    LCD_TEXT(60, 30, "WIFI CONFIG");
    
    LCD_DrawWiFiList();
    
    LCD_Clear(0, 280, 240, 40, DARK_GRAY);
    LCD_SetColors(WHITE, DARK_GRAY);
    LCD_TEXT(10, 292, "K1:BACK   K2:RE-SCAN");
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
                LCD_SetColors(BLACK, UI_BOX_SEL);
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
            LCD_DrawRectangle(x, y, 35, 28, BLACK);
            LCD_SetColors(BLACK, UI_BOX_NSEL);
            char key_str[2] = {keys[idx], '\0'};
            LCD_TEXT(x + 12, y + 6, key_str);
        }
    }
    
    // Special Keys
    LCD_Clear(10, 282, 70, 35, kb_shift ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_DrawRectangle(10, 282, 70, 35, BLACK);
    LCD_SetColors(BLACK, kb_shift ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(20, 292, "SHIFT");
    
    LCD_Clear(85, 282, 70, 35, RED);
    LCD_DrawRectangle(85, 282, 70, 35, BLACK);
    LCD_SetColors(WHITE, RED);
    LCD_TEXT(95, 292, "BACK");
    
    LCD_Clear(160, 282, 70, 35, GREEN);
    LCD_DrawRectangle(160, 282, 70, 35, BLACK);
    LCD_SetColors(BLACK, GREEN);
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
    LCD_DrawBackButton(mode_focus_idx == -1);
    LCD_SetColors(BLACK, UI_BG);
    LCD_TEXT(60, 32, "CHOOSE OPERATION");

    LCD_Clear(20, 70, 200, 40, (mode_focus_idx == 0) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_DrawRectangle(20, 70, 200, 40, BLACK);
    LCD_SetColors(BLACK, (mode_focus_idx == 0) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(60, 82, "MODE 1: BATTLE");
    
    LCD_Clear(20, 125, 200, 40, (mode_focus_idx == 1) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_DrawRectangle(20, 125, 200, 40, BLACK);
    LCD_SetColors(BLACK, (mode_focus_idx == 1) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(60, 137, "MODE 2: DRAW");
    
    LCD_Clear(20, 180, 200, 40, (mode_focus_idx == 2) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_DrawRectangle(20, 180, 200, 40, BLACK);
    LCD_SetColors(BLACK, (mode_focus_idx == 2) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(60, 192, "MODE 3: MAP");

    LCD_Clear(0, 250, 240, 70, DARK_GRAY);
    LCD_SetColors(WHITE, DARK_GRAY);
    LCD_TEXT(10, 260, "M1 = REAL-TIME CONTROL");
    LCD_TEXT(10, 280, "M2 = AUTONOMOUS PATH");
    LCD_TEXT(10, 300, "M3 = LOCAL BATTLE");
}

void LCD_UpdateModeSelection(void)
{
    LCD_Clear(20, 70, 200, 40, (mode_focus_idx == 0) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_DrawRectangle(20, 70, 200, 40, BLACK);
    LCD_SetColors(BLACK, (mode_focus_idx == 0) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(60, 82, "MODE 1: BATTLE");

    LCD_Clear(20, 125, 200, 40, (mode_focus_idx == 1) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_DrawRectangle(20, 125, 200, 40, BLACK);
    LCD_SetColors(BLACK, (mode_focus_idx == 1) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(60, 137, "MODE 2: DRAW");

    LCD_Clear(20, 180, 200, 40, (mode_focus_idx == 2) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_DrawRectangle(20, 180, 200, 40, BLACK);
    LCD_SetColors(BLACK, (mode_focus_idx == 2) ? UI_BOX_SEL : UI_BOX_NSEL);
    LCD_TEXT(60, 192, "MODE 3: MAP");

    LCD_DrawBackButton(mode_focus_idx == -1);
}

void LCD_DrawModeConfirm(void)
{
    LCD_Clear(5, 75, 230, 175, MY_BLACK);  
    LCD_Clear(8, 78, 224, 169, WHITE);     
    LCD_DrawRectangle(8, 78, 224, 169, BLACK);
    
    LCD_SetColors(BLACK, WHITE);
    LCD_TEXT(20, 90, "You choosed:");
    LCD_SetColors(BLUE, WHITE);
    LCD_TEXT(20, 110, (char *)MODE_Name(selected_mode));
    LCD_SetColors(BLACK, WHITE);
    LCD_TEXT(20, 130, "Are you sure?");
    LCD_TEXT(20, 150, "Once confirmed");
    LCD_TEXT(20, 170, "cannot change!");

    LCD_Clear(10, 190, 105, 45, DARK_GRAY);
    LCD_DrawRectangle(10, 190, 105, 45, BLACK);
    LCD_SetColors(WHITE, DARK_GRAY);
    LCD_TEXT(25, 205, "REGRET");
    
    LCD_Clear(125, 190, 105, 45, MY_GREEN);
    LCD_DrawRectangle(125, 190, 105, 45, BLACK);
    LCD_SetColors(BLACK, MY_GREEN);
    LCD_TEXT(135, 205, "CONFIRM");
}

void LCD_DrawCarSelect(void)
{
    char line[32];
    car_type_t preview_car = CarPreviewIndex();
    uint16_t preview_color = CarColorByIndex(preview_car);
    LCD_Clear(0, 0, 240, 320, UI_BG);
    LCD_DrawStatusBar();
    LCD_DrawBackButton(car_focus_idx == -1);
    LCD_SetColors(BLACK, UI_BG);
    LCD_TEXT(60, 28, "SELECT CHASSIS");

    snprintf(line, sizeof(line), "OP: %s", MODE_Name(selected_mode));
    LCD_TEXT(10, 46, line);

    for (int i = 0; i < 7; i++) {
        uint16_t y = 64 + (i * 26);
        uint16_t color = (car_focus_idx == i) ? UI_BOX_SEL : UI_BOX_NSEL;
        uint16_t chip_color = CarColorByIndex((int8_t)i);
        LCD_Clear(14, y, 212, 24, color);
        LCD_DrawRectangle(14, y, 212, 24, BLACK);
        LCD_SetColors(BLACK, color);
        const char* labels[] = {"V0 STANDARD", "V1 AUTO FIRE", "V2 RAPID SHOT", "V3 MOVING CAST", "V4 FORWARD SPD", "V5 LONG BEAM", "V6 GUN PLATFORM"};
        DrawCarColorChip(18, y + 4, chip_color, car_focus_idx == i);
        LCD_TEXT(40, y+2, labels[i]);
    }

    LCD_DrawCarInfoPanel(preview_car, preview_color);
}

void LCD_UpdateCarSelection(void)
{
    car_type_t preview_car = CarPreviewIndex();
    uint16_t preview_color = CarColorByIndex(preview_car);

    for (int i = 0; i < 7; i++) {
        uint16_t y = 64 + (i * 26);
        uint16_t color = (car_focus_idx == i) ? UI_BOX_SEL : UI_BOX_NSEL;
        uint16_t chip_color = CarColorByIndex((int8_t)i);
        LCD_Clear(14, y, 212, 24, color);
        LCD_DrawRectangle(14, y, 212, 24, BLACK);
        LCD_SetColors(BLACK, color);
        const char* labels[] = {"V0 STANDARD", "V1 AUTO FIRE", "V2 RAPID SHOT", "V3 MOVING CAST", "V4 FORWARD SPD", "V5 LONG BEAM", "V6 GUN PLATFORM"};
        DrawCarColorChip(18, y + 4, chip_color, car_focus_idx == i);
        LCD_TEXT(40, y+2, labels[i]);
    }

    LCD_DrawCarInfoPanel(preview_car, preview_color);
    LCD_DrawBackButton(car_focus_idx == -1);
}

void LCD_DrawCarConfirm(void)
{
    LCD_Clear(5, 75, 230, 175, MY_BLACK);  
    LCD_Clear(8, 78, 224, 169, WHITE);     
    LCD_DrawRectangle(8, 78, 224, 169, BLACK);
    
    LCD_SetColors(BLACK, WHITE);
    LCD_TEXT(20, 90, "You choosed:");
    LCD_SetColors(BLUE, WHITE);
    LCD_TEXT(20, 110, (char *)CAR_Label(selected_car));
    LCD_SetColors(BLACK, WHITE);
    LCD_TEXT(20, 130, "Are you sure?");
    LCD_TEXT(20, 150, "Once confirmed");
    LCD_TEXT(20, 170, "cannot change!");

    LCD_Clear(10, 190, 105, 45, DARK_GRAY);
    LCD_DrawRectangle(10, 190, 105, 45, BLACK);
    LCD_SetColors(WHITE, DARK_GRAY);
    LCD_TEXT(25, 205, "REGRET");
    
    LCD_Clear(125, 190, 105, 45, MY_GREEN);
    LCD_DrawRectangle(125, 190, 105, 45, BLACK);
    LCD_SetColors(BLACK, MY_GREEN);
    LCD_TEXT(135, 205, "CONFIRM");
}

void LCD_DrawGameLayout(void)
{
    LCD_Clear(0, 0, 240, 320, UI_BG);
    LCD_DrawStatusBar();

    LCD_SetColors(BLACK, UI_BG);
    LCD_TEXT(10, 50,  "Mode:");
    LCD_TEXT(10, 70,  "Direction:");
    LCD_TEXT(10, 90,  "Speed:");
    LCD_TEXT(10, 110, "Button:");
    LCD_TEXT(10, 130, "Laser:");
    LCD_TEXT(10, 150, "Motion:");
    LCD_TEXT(10, 170, "ESP:");
    LCD_TEXT(10, 190, "Touch:");

    uint16_t car_color = CarColorByIndex((int8_t)selected_car);
    LCD_Clear(0, 215, 240, 105, car_color);
    LCD_SetColors(BLACK, car_color);
    LCD_TEXT(10, 220, "Car:");
}

void LCD_DrawMode2InputSelect(void) {
    LCD_Clear(0, 0, 240, 320, UI_BG);
    LCD_DrawStatusBar();
    LCD_DrawBackButton(0);
    LCD_TEXT(60, 30, "SELECT CONTROL");
    
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

    LCD_ClearTextField(110, 50, 12, UI_BG);
    LCD_TEXT(110, 50, (char *)MODE_Name(selected_mode));

    LCD_ClearTextField(110, 70, 14, UI_BG);
    LCD_TEXT(110, 70, dir_str);

    LCD_ClearTextField(110, 90, 10, UI_BG);
    LCD_TEXT(110, 90, spd_str);
}

void LCD_UpdateGameSlow(uint8_t fire_pressed)
{
    char btn_str[16];
    char laser_disp[32];
    char motion_disp[24];
    char esp_disp[8];
    char touch_str[16];
    char car_line[40];
    char speed_line[40];
    char charge_line[40];
    char cooldown_line[40];

    if (fire_pressed) strcpy(btn_str, "PRESSED");
    else              strcpy(btn_str, "RELEASE");

    if (touch_ability_shots_needed == 0)
        strcpy(touch_str, "READY");
    else
        snprintf(touch_str, sizeof(touch_str), "WAIT %u SHT", touch_ability_shots_needed);

    snprintf(laser_disp, sizeof(laser_disp), "%s", laser_line);
    snprintf(motion_disp, sizeof(motion_disp), "%s", motion_line);
    snprintf(esp_disp, sizeof(esp_disp), "%s", esp_cmd_rx);

    switch (selected_car) {
        case CAR_V0:
            snprintf(car_line, sizeof(car_line), "V0 STANDARD");
            snprintf(speed_line, sizeof(speed_line), "Speed: 100");
            snprintf(charge_line, sizeof(charge_line), "Chg: 1000ms  Fire: 1000ms");
            snprintf(cooldown_line, sizeof(cooldown_line), "Cooldown: 3000ms");
            break;
        case CAR_V1:
            snprintf(car_line, sizeof(car_line), "V1 AUTO FIRE");
            snprintf(speed_line, sizeof(speed_line), "Speed: 70");
            snprintf(charge_line, sizeof(charge_line), "Chg: 1000ms  Fire: 1000ms");
            snprintf(cooldown_line, sizeof(cooldown_line), "Cooldown: 3000ms");
            break;
        case CAR_V2:
            snprintf(car_line, sizeof(car_line), "V2 RAPID SHOT");
            snprintf(speed_line, sizeof(speed_line), "Speed: 100");
            snprintf(charge_line, sizeof(charge_line), "Chg: 1000ms  Fire: 500ms");
            snprintf(cooldown_line, sizeof(cooldown_line), "Cooldown: 1500ms");
            break;
        case CAR_V3:
            snprintf(car_line, sizeof(car_line), "V3 MOVING CAST");
            snprintf(speed_line, sizeof(speed_line), "Speed: 100");
            snprintf(charge_line, sizeof(charge_line), "Chg: 1000ms  Fire: 1000ms");
            snprintf(cooldown_line, sizeof(cooldown_line), "Cooldown: 6000ms");
            break;
        case CAR_V4:
            snprintf(car_line, sizeof(car_line), "V4 FORWARD SPD");
            snprintf(speed_line, sizeof(speed_line), "Speed: 200");
            snprintf(charge_line, sizeof(charge_line), "Chg: 800ms  Fire: 400ms");
            snprintf(cooldown_line, sizeof(cooldown_line), "Cooldown: 4000ms");
            break;
        case CAR_V5:
            snprintf(car_line, sizeof(car_line), "V5 LONG BEAM");
            snprintf(speed_line, sizeof(speed_line), "Speed: 100");
            snprintf(charge_line, sizeof(charge_line), "Chg: 1200ms  Fire: 1800ms");
            snprintf(cooldown_line, sizeof(cooldown_line), "Cooldown: 5500ms");
            break;
        case CAR_V6:
            snprintf(car_line, sizeof(car_line), "V6 GUN PLATFORM");
            snprintf(speed_line, sizeof(speed_line), "Speed: 60");
            snprintf(charge_line, sizeof(charge_line), "Chg: 400ms  Fire: 400ms");
            snprintf(cooldown_line, sizeof(cooldown_line), "Cooldown: 1800ms");
            break;
        default:
            snprintf(car_line, sizeof(car_line), "UNKNOWN CAR");
            snprintf(speed_line, sizeof(speed_line), "Speed: 100");
            snprintf(charge_line, sizeof(charge_line), "Chg: 1000ms  Fire: 1000ms");
            snprintf(cooldown_line, sizeof(cooldown_line), "Cooldown: 3000ms");
            break;
    }

    LCD_SetColors(BLACK, UI_BG);
    LCD_ClearTextField(110, 110, 12, UI_BG);
    LCD_TEXT(110, 110, btn_str);

    LCD_ClearTextField(110, 130, 12, UI_BG);
    LCD_TEXT(110, 130, laser_disp);

    LCD_ClearTextField(110, 150, 12, UI_BG);
    LCD_TEXT(110, 150, motion_disp);

    LCD_ClearTextField(110, 170, 12, UI_BG);
    LCD_TEXT(110, 170, esp_disp);

    LCD_ClearTextField(110, 190, 12, UI_BG);
    LCD_TEXT(110, 190, touch_str);

    uint16_t car_color = CarColorByIndex((int8_t)selected_car);
    LCD_Clear(0, 215, 240, 105, car_color);
    LCD_SetColors(BLACK, car_color);
    
    LCD_ClearTextField(10, 220, 30, car_color);
    LCD_TEXT(10, 220, car_line);
    
    LCD_ClearTextField(10, 238, 30, car_color);
    LCD_TEXT(10, 238, speed_line);
    
    LCD_ClearTextField(10, 256, 40, car_color);
    LCD_TEXT(10, 256, charge_line);
    
    LCD_ClearTextField(10, 274, 30, car_color);
    LCD_TEXT(10, 274, cooldown_line);
}
