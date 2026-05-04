#ifndef UI_H
#define UI_H

#include "main.h"
#include <stdint.h>

// Global Settings (Added for Settings Expansion)
typedef enum { THEME_DEFAULT = 0, THEME_DARK, THEME_LIGHT } ui_theme_t;
typedef enum { FONT_DEFAULT = 0, FONT_LARGE } ui_font_t;

extern ui_theme_t current_theme;
extern uint8_t    audio_enabled;
extern uint8_t    colorblind_mode;
extern uint8_t    led_enabled;
extern uint8_t    seg_enabled;
extern ui_font_t  current_font;

extern ui_theme_t temp_theme;
extern uint8_t    temp_audio;
extern uint8_t    temp_cb;
extern uint8_t    temp_led;
extern uint8_t    temp_seg;
extern ui_font_t  temp_font;
extern uint8_t    settings_focus_idx;
extern int8_t     home_focus_idx;
extern int8_t     mode_focus_idx;
extern int8_t     car_focus_idx;

#include "game_logic.h"

// LCD UI Updates
#define LCD_FAST_UPDATE_MS  120U
#define LCD_SLOW_UPDATE_MS  350U

// Colors
#define UI_BG               WHITE
#define UI_HEAD             CYAN
#define UI_TOP              UI_HEAD
#define UI_BOX_SEL          GREEN
#define UI_BOX_NSEL         YELLOW
#define UI_BOTTOM           MAGENTA
#define UI_PLACEHOLDER      RED
#define DARK_GRAY           0x4208
#define LIGHT_BLUE          0x07FF
#define MY_BLACK            0x0000
#define MY_GREEN            0x07E0

#define LCD_TEXT(x, y, s)   LCD_DrawString((x), (y), (const char *)(s))

// Externs
extern app_state_t last_drawn_state;
extern game_mode_t last_drawn_mode;
extern car_type_t  last_drawn_car;
extern uint32_t    lcd_fast_tick;
extern uint32_t    lcd_slow_tick;
extern uint8_t     touch_display_flag;

// WiFi List Structure
#define MAX_WIFI_NETWORKS 10
extern char wifi_ssids[MAX_WIFI_NETWORKS][33];
extern uint8_t wifi_count;
extern int8_t selected_wifi_idx;

// Keyboard State
extern char keyboard_buffer[33];
extern uint8_t kb_shift;

// Function Prototypes
void LCD_ClearTextField(uint16_t x, uint16_t y, uint16_t chars, uint16_t bg);

// Master Screens
void LCD_DrawStatusBar(void);
void LCD_DrawHome(void);
void LCD_DrawSettings(void);
void LCD_DrawWiFiList(void);
void LCD_DrawKeyboard(const char* current_input);
void LCD_DrawBackButton(void);
void LCD_UpdateSettingsOption(uint8_t option_idx);
void LCD_DrawWiFiSettings(void);
void LCD_DrawMode3Placeholder(void);

// Existing Screens
void LCD_DrawModeSelect(void);
void LCD_UpdateModeSelection(void);
void LCD_DrawModeConfirm(void);
void LCD_DrawCarSelect(void);
void LCD_UpdateCarSelection(void);
void LCD_DrawCarConfirm(void);
void LCD_DrawGameLayout(void);
void LCD_DrawMode2InputSelect(void);
void LCD_UpdateMode2InputSelect(void);
void LCD_DrawMode2Stats(uint16_t current_move, uint16_t total_moves, uint32_t distance, uint32_t seconds_left);
void LCD_DrawMode2Canvas(void);
void LCD_DrawMode2ResetConfirm(void);
void LCD_DrawMode2CommandHistory(char cmd, uint8_t slot, uint8_t is_new);
void LCD_UpdateGameFast(uint32_t x_raw, uint32_t y_raw);
void LCD_UpdateGameSlow(uint8_t fire_pressed);

#endif // UI_H
