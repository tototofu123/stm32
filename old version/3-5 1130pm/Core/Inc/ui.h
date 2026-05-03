#ifndef UI_H
#define UI_H

#include "main.h"
#include "game_logic.h"

// LCD UI Updates
#define LCD_FAST_UPDATE_MS  120U
#define LCD_SLOW_UPDATE_MS  350U

// Colors
#define UI_BG               WHITE
#define UI_HEAD             CYAN
#define UI_BOX_SEL          GREEN
#define UI_BOX_NSEL         YELLOW
#define UI_BOTTOM           MAGENTA
#define UI_PLACEHOLDER      RED
#define MY_GRAY             0x8410
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
