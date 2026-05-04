#ifndef UI_H
#define UI_H

/*
 * =============================================================================
 * UI.H - USER INTERFACE STATE AND SCREEN DRAWING API
 * =============================================================================
 *
 * This header declares all UI-related enums, global state, and function
 * prototypes that ui.c implements and that main.c and other modules use.
 *
 * Responsibility (ui.c implementation):
 * - Define all UI colors, layout constants, and update-frequency timings.
 * - Manage UI state (focus indices, settings temp copies, keyboard buffer).
 * - Draw all screens (home, settings, mode select, car select, game HUD, etc.).
 * - Handle focus/selection highlighting (back button, menu rows, car cards).
 * - Update game HUD on fast (~120 ms) and slow (~350 ms) schedules.
 * - Draw Mode 2 canvas and input selection interface.
 *
 * Key enums defined here:
 * - ui_theme_t: Color scheme (DEFAULT, DARK, LIGHT).
 * - ui_font_t: Font size (DEFAULT, LARGE).
 *
 * Global variables exported:
 * - current_theme, audio_enabled, colorblind_mode, led_enabled, seg_enabled:
 *   Live settings that affect game behavior and appearance.
 * - temp_theme, temp_audio, temp_cb, temp_led, temp_seg, temp_font:
 *   Temporary copies for settings menu (committed on OK, discarded on CANCEL).
 * - settings_focus_idx, home_focus_idx, mode_focus_idx, car_focus_idx:
 *   Navigation focus states for each screen context (-1 for back button).
 * - lcd_fast_tick, lcd_slow_tick: timing trackers for HUD update frequency.
 * - wifi_ssids, wifi_count, selected_wifi_idx: WiFi network list state.
 * - keyboard_buffer, kb_shift: Soft keyboard input state (currently unused).
 *
 * Data flow:
 * - main.c calls LCD_Draw* when app_state changes and LCD_Update* during active
 *   gameplay to keep HUD in sync with joystick/fire state.
 * - ui.c reads game_logic globals (laser_line, motion_line, selected_car, etc.)
 *   to display live gameplay status.
 * - mode_2.c and mode_3.c call their own LCD_Draw functions for game-specific screens.
 * - peripherals.c provides WiFi SSID list that ui.c displays.
 *
 * No C++ classes; pure C state and functions.
 * =============================================================================
 */

#include "main.h"
#include <stdint.h>

/*
 * =============================================================================
 * UI CONFIGURATION ENUMS
 * =============================================================================
 */
/* Color theme selection */
typedef enum { THEME_DEFAULT = 0, THEME_DARK, THEME_LIGHT } ui_theme_t;
/* Font size selection */
typedef enum { FONT_DEFAULT = 0, FONT_LARGE } ui_font_t;

/*
 * Live application settings (persisted across sessions).
 */
extern ui_theme_t current_theme;    // Active color scheme.
extern uint8_t    audio_enabled;    // Buzzer/audio mute flag.
extern uint8_t    colorblind_mode;  // Enable colorblind-friendly palette.
extern uint8_t    led_enabled;      // RGB LED control flag.
extern uint8_t    seg_enabled;      // 7-segment display control flag.
extern ui_font_t  current_font;     // Active font size.

/*
 * Temporary settings copies used in settings menu.
 * User can preview changes; apply with OK or discard with CANCEL.
 */
extern ui_theme_t temp_theme;
extern uint8_t    temp_audio;
extern uint8_t    temp_cb;          // Colorblind temp copy.
extern uint8_t    temp_led;
extern uint8_t    temp_seg;
extern ui_font_t  temp_font;

/*
 * Navigation focus indices for each screen context.
 * Range: 0-N for menu items, -1 for back button.
 */
extern int8_t     settings_focus_idx;  // Settings menu focus (0-5 or -1=back).
extern int8_t     home_focus_idx;      // Home screen focus (unused currently).
extern int8_t     mode_focus_idx;      // Mode select focus (0-2 or -1=back).
extern int8_t     car_focus_idx;       // Car select focus (0-6 or -1=back).

#include "game_logic.h"

/*
 * =============================================================================
 * LCD UPDATE TIMING CONSTANTS
 * =============================================================================
 * Control how often the HUD is refreshed during gameplay.
 */
#define LCD_FAST_UPDATE_MS  120U   // Frequent updates for movement and immediate feedback.
#define LCD_SLOW_UPDATE_MS  350U   // Infrequent updates for status text and HUD panels.

/*
 * =============================================================================
 * COLOR PALETTE AND LAYOUT CONSTANTS
 * =============================================================================
 * Standard colors used for backgrounds, highlights, text, etc.
 */
/* Color definitions */
#define UI_BG               WHITE           // Background color (default white).
#define UI_HEAD             CYAN            // Header bar color.
#define UI_TOP              UI_HEAD         // Alias for header.
#define UI_BOX_SEL          GREEN           // Selected box/button color.
#define UI_BOX_NSEL         YELLOW          // Unselected box/button color.
#define UI_BOTTOM           MAGENTA         // Bottom panel color (HUD area).
#define UI_PLACEHOLDER      RED             // Placeholder/error color.
#define DARK_GRAY           0x4208          // Dark gray for borders.
#define LIGHT_BLUE          0x07FF          // Light blue for accents.
#define MY_BLACK            0x0000          // Pure black text.
#define MY_GREEN            0x07E0          // Pure green text.

/* Convenience macro for drawing text */
#define LCD_TEXT(x, y, s)   LCD_DrawString((x), (y), (const char *)(s))

/*
 * =============================================================================
 * UI STATE TRACKING VARIABLES
 * =============================================================================
 */
/* Last drawn state (to avoid redundant full redraws) */
extern app_state_t last_drawn_state;  // Previous app_state when last full draw occurred.
extern game_mode_t last_drawn_mode;   // Previous selected_mode when last draw occurred.
extern car_type_t  last_drawn_car;    // Previous selected_car when last draw occurred.

/* HUD update timing */
extern uint32_t    lcd_fast_tick;     // Timestamp of last fast HUD update.
extern uint32_t    lcd_slow_tick;     // Timestamp of last slow HUD update.
extern uint8_t     touch_display_flag;  // Flag for touch coordinate display (debug).

/*
 * WiFi network list (from peripherals.c ESP module scan result).
 */
#define MAX_WIFI_NETWORKS 10
extern char wifi_ssids[MAX_WIFI_NETWORKS][33];  // SSID strings scanned from ESP.
extern uint8_t wifi_count;                      // Number of networks found.
extern int8_t selected_wifi_idx;                // Currently focused network index.

/*
 * Soft keyboard input state (for WiFi password/login entry).
 * Currently unused; reserved for future WiFi credential entry.
 */
extern char keyboard_buffer[33];  // Input buffer for keyboard.
extern uint8_t kb_shift;          // Shift key state (for uppercase).

/*
 * =============================================================================
 * UI FUNCTION PROTOTYPES
 * =============================================================================
 */

/* Helper utilities */
/*
 * LCD_ClearTextField:
 * Clears a rectangular text area by overwriting it with background color.
 * Used to erase old status text before redrawing updated values.
 */
void LCD_ClearTextField(uint16_t x, uint16_t y, uint16_t chars, uint16_t bg);

/* Master screens (full-screen redraws) */
/*
 * LCD_DrawStatusBar: Draws the top status bar (WiFi indicator, etc.).
 */
void LCD_DrawStatusBar(void);

/*
 * LCD_DrawHome: Renders the home/landing screen with START BATTLE and
 * PREFERENCES buttons.
 */
void LCD_DrawHome(void);

/*
 * LCD_DrawSettings: Renders the settings menu screen (Theme, Audio, Colorblind,
 * LED, 7-Seg, Font) with OK/CANCEL buttons.
 */
void LCD_DrawSettings(void);

/*
 * LCD_DrawWiFiList: Renders the WiFi network list for selection.
 * (Currently unused; WiFi config disabled).
 */
void LCD_DrawWiFiList(void);

/*
 * LCD_DrawKeyboard: Renders a soft keyboard for text input.
 * (Currently unused; reserved for future WiFi credential entry).
 */
void LCD_DrawKeyboard(const char* current_input);

/*
 * LCD_DrawBackButton: Draws a back/navigation button with dynamic highlight.
 * Input: selected (1=highlighted green, 0=standard color).
 */
void LCD_DrawBackButton(uint8_t selected);

/*
 * LCD_UpdateSettingsOption: Redraws a single settings option row to reflect
 * current temp value (without full screen redraw).
 */
void LCD_UpdateSettingsOption(uint8_t option_idx);

/*
 * LCD_DrawWiFiSettings: Renders WiFi configuration screen.
 * (Currently unused; WiFi feature disabled).
 */
void LCD_DrawWiFiSettings(void);

/*
 * LCD_DrawMode3Placeholder: Placeholder screen for Mode 3 arena.
 */
void LCD_DrawMode3Placeholder(void);

/* Navigation screens */
/*
 * LCD_DrawModeSelect: Renders mode 1/2/3 selection screen.
 */
void LCD_DrawModeSelect(void);

/*
 * LCD_UpdateModeSelection: Incremental update for mode selection focus.
 */
void LCD_UpdateModeSelection(void);

/*
 * LCD_DrawModeConfirm: Renders confirmation popup for mode choice.
 */
void LCD_DrawModeConfirm(void);

/*
 * LCD_DrawCarSelect: Renders car V0-V6 selection screen with preview panel.
 */
void LCD_DrawCarSelect(void);

/*
 * LCD_UpdateCarSelection: Incremental update for car selection focus.
 */
void LCD_UpdateCarSelection(void);

/*
 * LCD_DrawCarConfirm: Renders confirmation popup for car choice.
 */
void LCD_DrawCarConfirm(void);

/* Gameplay screens */
/*
 * LCD_DrawGameLayout: Renders Mode 1 game screen layout (arena, HUD).
 */
void LCD_DrawGameLayout(void);

/*
 * LCD_DrawMode2InputSelect: Renders Mode 2 input method selection (joystick/touch).
 */
void LCD_DrawMode2InputSelect(void);

/*
 * LCD_UpdateMode2InputSelect: Incremental update for Mode 2 input selection.
 */
void LCD_UpdateMode2InputSelect(void);

/*
 * LCD_DrawMode2Stats: Renders Mode 2 challenge stats (moves, distance, time).
 */
void LCD_DrawMode2Stats(uint16_t current_move, uint16_t total_moves, uint32_t distance, uint32_t seconds_left);

/*
 * LCD_DrawMode2Canvas: Renders the Mode 2 drawing canvas.
 */
void LCD_DrawMode2Canvas(void);

/*
 * LCD_DrawMode2ResetConfirm: Renders reset confirmation popup for Mode 2.
 */
void LCD_DrawMode2ResetConfirm(void);

/*
 * LCD_DrawMode2CommandHistory: Draws a command movement history slot.
 */
void LCD_DrawMode2CommandHistory(char cmd, uint8_t slot, uint8_t is_new);

/* HUD updates (frequent, during gameplay) */
/*
 * LCD_UpdateGameFast: Fast HUD update called ~120 ms (movement display).
 */
void LCD_UpdateGameFast(uint32_t x_raw, uint32_t y_raw);

/*
 * LCD_UpdateGameSlow: Slow HUD update called ~350 ms (status, stats).
 */
void LCD_UpdateGameSlow(uint8_t fire_pressed);

/*
 * =============================================================================
 * USAGE PATTERN
 * =============================================================================
 * Typical usage flow:
 * 1. main.c detects app_state change (e.g., APP_HOME -> APP_SETTINGS).
 * 2. main.c calls LCD_DrawSettings() for full screen redraw.
 * 3. During gameplay, main.c calls LCD_UpdateGameFast/Slow on timer intervals.
 * 4. Touch/joystick input updates focus indices (settings_focus_idx, etc.).
 * 5. main.c calls LCD_UpdateSettingsOption(idx) for incremental highlight updates.
 * 6. On confirm or cancel, settings are applied or discarded by main.c.
 * =============================================================================
 */

#endif // UI_H
