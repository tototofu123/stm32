/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Handheld Mode Logic with Special Ability
  ******************************************************************************
  */
/*
 * main.c is the firmware entry point. It initializes HAL and all peripherals,
 * calibrates the joystick, handles every input source, routes the application
 * state machine, and keeps the UI, audio, 7-segment display, and game modes
 * synchronized during runtime.
 *
 * Functions in this file:
 * - SystemClock_Config: sets the MCU clock tree and prescalers.
 * - MX_GPIO_Init: configures GPIO pins used by buttons, LEDs, and control lines.
 * - MX_FSMC_Init: configures the LCD external memory interface.
 * - MX_ADC1_Init: configures the first joystick ADC channel.
 * - MX_ADC2_Init: configures the second joystick ADC channel.
 * - MX_I2C2_Init: configures the I2C bus used by external hardware.
 * - MX_USART3_UART_Init: configures the UART used by the ESP/WiFi module.
 * - main: performs startup initialization and runs the main polling loop.
 * - Error_Handler: halts the system if a fatal error occurs.
 *
 * Global variables used here include the HAL handles, debounce trackers,
 * joystick threshold values, and the shared application state imported from
 * the gameplay and UI modules. No classes are used in this C file.
 */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include "lcd.h"
#include <stdio.h>
#include <string.h>

// Modules
#include "touch.h"
#include "seven_seg.h"
#include "alerts.h"
#include "peripherals.h"
#include "game_logic.h"
#include "ui.h"
#include "mode_2.h"
#include "mode_3.h"
/* USER CODE END Includes */

#define BTN_DEBOUNCE_MS     120U

// --- Hardware Pin Definitions ---
#define K1_PIN              GPIO_PIN_0
#define K1_PORT             GPIOA
#define CAP_TOUCH_PIN       GPIO_PIN_1
#define CAP_TOUCH_PORT      GPIOA
#define K2_PIN              GPIO_PIN_13
#define K2_PORT             GPIOC
#define JOY_SW_PIN          GPIO_PIN_2
#define JOY_SW_PORT         GPIOC
#define BTN1_PIN            GPIO_PIN_2
#define BTN1_PORT           GPIOA
#define BTN2_PIN            GPIO_PIN_3
#define BTN2_PORT           GPIOA

ADC_HandleTypeDef  hadc1;
ADC_HandleTypeDef  hadc2;
//I2C_HandleTypeDef  hi2c2;
UART_HandleTypeDef huart3;
SRAM_HandleTypeDef hsram1;

// State trackers
GPIO_PinState last_k1_state  = GPIO_PIN_RESET;
GPIO_PinState last_k2_state  = GPIO_PIN_RESET;
GPIO_PinState last_cap_state = GPIO_PIN_SET;
uint32_t last_k1_event_tick = 0U;
uint32_t last_k2_event_tick = 0U;

// UART RX byte for ESP is defined in peripherals.c

uint32_t x_left_thresh = 1700U;
uint32_t x_right_thresh = 2700U;
uint32_t y_fwd_thresh = 1700U;
uint32_t y_back_thresh = 2700U;
uint32_t adc_center_x = 2200U;
uint32_t adc_center_y = 2200U;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);

int main(void)
{
    /* main performs all board startup work and then runs the forever loop that
     * polls inputs, updates the current app state, and dispatches gameplay logic.
     */

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_FSMC_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    //MX_I2C2_Init();
    MX_USART3_UART_Init();

    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADCEx_Calibration_Start(&hadc2);

    // Joystick auto-calibration (increased deadzone to 1000 for stability)
    // We read current joystick center values once at startup to adapt to
    // hardware variation between boards and analog stick tolerances.
    uint32_t cx = read_adc1();
    uint32_t cy = read_adc2();
    // Safety range check:
    // Only accept values that look physically valid for a centered joystick.
    // If the reading is too close to ADC rails, we treat it as noisy/bad boot
    // input and keep default thresholds.
    if(cx > 800 && cx < 3600) {
        // Store center and create left/right trigger thresholds around center.
        adc_center_x = cx;
        x_left_thresh = cx - 1000;
        x_right_thresh = cx + 1000;
    }
    if(cy > 800 && cy < 3600) {
        // Same logic for forward/backward axis threshold calculation.
        adc_center_y = cy;
        y_fwd_thresh = cy - 1000;
        y_back_thresh = cy + 1000;
    }

    HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
    Buzzer_Set(0);
    RGB_Set(1, 1, 1);
    SEG_AllOff();
    SEG_ShowPair(0, 0, 0);

    LCD_INIT();

    LCD_DrawHome();
    last_drawn_state = APP_HOME;

    WifiSetUp();
    HAL_UART_Receive_IT(&huart3, &esp_rx_byte, 1);

    while (1)
    {

    	//Wifi_ProcessIncoming();
        // ===== FRAME START =====
        // Every pass through this loop is one firmware "frame".
        // 1) Read all inputs
        // 2) Convert raw signals into events (clicks/pulses)
        // 3) Run state-machine logic for current screen/mode
        // 4) Draw/update outputs (LCD, buzzer, 7-seg, LEDs)
        // 5) Delay a little to keep timing stable
        uint32_t now = HAL_GetTick();
        uint32_t x_raw = read_adc1();
        uint32_t y_raw = read_adc2();

        // Joystick direction pulses
        // We convert continuous analog direction into single-step events.
        // Example: if the user keeps holding UP, we only emit one joy_up pulse
        // until they release and press UP again. This prevents menu overscroll.
        static uint8_t last_j_up = 0, last_j_down = 0, last_j_left = 0, last_j_right = 0;
        uint8_t joy_up = 0, joy_down = 0, joy_left = 0, joy_right = 0;
        if (y_raw < Y_FWD_THRESH_ADC) { if(!last_j_up) joy_up = 1; last_j_up = 1; } else last_j_up = 0;
        if (y_raw > Y_BACK_THRESH_ADC) { if(!last_j_down) joy_down = 1; last_j_down = 1; } else last_j_down = 0;
        if (x_raw < X_LEFT_THRESH_ADC) { if(!last_j_left) joy_left = 1; last_j_left = 1; } else last_j_left = 0;
        if (x_raw > X_RIGHT_THRESH_ADC) { if(!last_j_right) joy_right = 1; last_j_right = 1; } else last_j_right = 0;

        GPIO_PinState k1_now = HAL_GPIO_ReadPin(K1_PORT, K1_PIN);
        GPIO_PinState k2_now = HAL_GPIO_ReadPin(K2_PORT, K2_PIN);
        GPIO_PinState cap_now = HAL_GPIO_ReadPin(CAP_TOUCH_PORT, CAP_TOUCH_PIN);
        GPIO_PinState joy_sw_now = HAL_GPIO_ReadPin(JOY_SW_PORT, JOY_SW_PIN);

        // JOY_SW is wired active-low: RESET means physically pressed.
        uint8_t fire_pressed = (joy_sw_now == GPIO_PIN_RESET) ? 1 : 0;

        // Capacitive key edge detect (pressed now, not pressed in previous frame).
        // This avoids repeating the same action every frame while held.
        if (cap_now == GPIO_PIN_RESET && last_cap_state == GPIO_PIN_SET) {
            if (app_state == APP_MODE_SELECT) {
                // Quick-start behavior: start game immediately from mode select.
                selected_mode = (mode_focus_idx >= 0) ? (game_mode_t)mode_focus_idx : GAME_MODE_1;
                selected_car = CAR_V0;
                app_state = APP_GAME;
                Buzzer_BeepShort();
            } else {
                // In gameplay/non-mode-select context, cap key acts as ability reset.
                SpecialAbility_ResetCooldown();
            }
        }
        // Save current sampled state for next frame edge detection.
        last_cap_state = cap_now;

        uint8_t ts_pressed = TouchPressed();
        uint8_t ts_click = 0;
        uint16_t px = 0, py = 0;
        static uint8_t last_ts_state_mem = 0;
        static uint32_t last_ts_event_tick = 0;

        if (ts_pressed)
        {
            // Read raw touch ADC-like values.
            uint16_t tx = TouchReadXRaw();
            uint16_t ty = TouchReadYRaw();
            // Map raw touch range to LCD pixel coordinates.
            uint16_t curr_px = map_u16(tx, TS_X_MIN, TS_X_MAX, 0, 239);
            uint16_t curr_py = map_u16(ty, TS_Y_MIN, TS_Y_MAX, 0, 319);
            // Panel coordinate system is vertically mirrored relative to screen.
            curr_py = 319 - curr_py;

            // Touch click event generation with 200 ms debounce.
            // We only emit one click when finger first touches the panel.
            if ((last_ts_state_mem == 0) && ((now - last_ts_event_tick) >= 200U)) {
                ts_click = 1; px = curr_px; py = curr_py;
                last_ts_event_tick = now;
            }
            // Mark touch as currently active.
            last_ts_state_mem = 1;
        } else {
            // Finger released: allow next press to become a new click event.
            last_ts_state_mem = 0;
        }

        uint8_t k1_click = 0U, k2_click = 0U;
        // Rising-edge + debounce for K1.
        // Condition means: button just transitioned to pressed state AND enough
        // time passed since the last accepted event.
        if ((k1_now == GPIO_PIN_SET) && (last_k1_state == GPIO_PIN_RESET) && ((now - last_k1_event_tick) >= BTN_DEBOUNCE_MS))
        { k1_click = 1U; last_k1_event_tick = now; }
        last_k1_state = k1_now;

        // Same debounce policy for K2.
        if ((k2_now == GPIO_PIN_SET) && (last_k2_state == GPIO_PIN_RESET) && ((now - last_k2_event_tick) >= BTN_DEBOUNCE_MS))
        { k2_click = 1U; last_k2_event_tick = now; }
        last_k2_state = k2_now;

        if (app_state != last_drawn_state)
        {
            // Full-screen draw only when entering a new app state.
            // This reduces flicker and avoids expensive full redraws each frame.
            if (app_state == APP_HOME)          LCD_DrawHome();
            else if (app_state == APP_SETTINGS) LCD_DrawSettings();
            else if (app_state == APP_WIFI_KEYBOARD) LCD_DrawKeyboard(keyboard_buffer);
            else if (app_state == APP_MODE_SELECT) LCD_DrawModeSelect();
            else if (app_state == APP_MODE_CONFIRM) LCD_DrawModeConfirm();
            else if (app_state == APP_CAR_SELECT)  LCD_DrawCarSelect();
            else if (app_state == APP_CAR_CONFIRM) LCD_DrawCarConfirm();
            else if (app_state == APP_GAME) {
                if (selected_mode == GAME_MODE_2) Mode2_Init();
                else if (selected_mode == GAME_MODE_3) Mode3_Init();
                else LCD_DrawGameLayout();
            }
            last_drawn_state = app_state;
        }

        // --- Hardware Quick Start (Cap Touch) ---
        // Global quick-start path from menu contexts.
        // Note: this block requires a press edge, same as other edge detectors.
        if (cap_now == GPIO_PIN_RESET && last_cap_state == GPIO_PIN_SET) {
            if (app_state == APP_MODE_SELECT || app_state == APP_CAR_SELECT || app_state == APP_HOME) {
                selected_mode = GAME_MODE_1;
                selected_car = CAR_V0;
                app_state = APP_GAME;
                Buzzer_BeepShort();
            }
        }

        if (ts_click)
        {
            // Global Back Button Touch Check
            if (px < 60 && py < 50 && app_state != APP_HOME && app_state != APP_GAME) {
                if (app_state == APP_SETTINGS) app_state = APP_HOME;
                else if (app_state == APP_WIFI_KEYBOARD) app_state = APP_SETTINGS;
                else if (app_state == APP_MODE_SELECT) app_state = APP_HOME;
                else if (app_state == APP_MODE_CONFIRM) app_state = APP_MODE_SELECT;
                else if (app_state == APP_CAR_SELECT) app_state = APP_MODE_SELECT;
                else if (app_state == APP_CAR_CONFIRM) app_state = APP_CAR_SELECT;
                Buzzer_BeepShort();
                ts_click = 0; // Consume click
            }
        }

        // --- Navigation Logic (Touch & HW) ---
        // State machine branch: each block below handles one active screen.
        if (app_state == APP_HOME) {
            if (k1_click) {
                app_state = APP_MODE_SELECT;
                Buzzer_BeepShort();
            }
            if (k2_click) {
                app_state = APP_SETTINGS;
                temp_theme = current_theme;
                temp_audio = audio_enabled;
                temp_cb = colorblind_mode;
                temp_led = led_enabled;
                temp_seg = seg_enabled;
                temp_font = current_font;
                Buzzer_BeepShort();
            }
            if (ts_click) {
                if (px >= 20 && px <= 220) {
                    if (py >= 100 && py <= 160) {
                        app_state = APP_MODE_SELECT;
                        Buzzer_BeepShort();
                    }
                    else if (py >= 180 && py <= 240) {
                        app_state = APP_SETTINGS;
                        temp_theme = current_theme;
                        temp_audio = audio_enabled;
                        temp_cb = colorblind_mode;
                        temp_led = led_enabled;
                        temp_seg = seg_enabled;
                        temp_font = current_font;
                        Buzzer_BeepShort();
                    }
                }
            }
        }
        else if (app_state == APP_SETTINGS) {
            // Navigation in settings list:
            // focus index 0..5 = options, -1 = back button.
            if (joy_down || k1_click) {
                if (settings_focus_idx < 5) settings_focus_idx++;
                else settings_focus_idx = -1;
                LCD_DrawSettings();
                Buzzer_BeepShort();
            }
            if (joy_up) {
                // Wrap-around navigation when going backward past the beginning.
                if (settings_focus_idx == -1) settings_focus_idx = 5;
                else if (settings_focus_idx > 0) settings_focus_idx--;
                else settings_focus_idx = -1;
                LCD_DrawSettings();
                Buzzer_BeepShort();
            }
            // Confirm action (change setting or exit) with debounce.
            if ((fire_pressed || k2_click) && ((now - last_k2_event_tick) >= 300)) {
                last_k2_event_tick = now;
                if (settings_focus_idx == -1) {
                    app_state = APP_HOME;
                    Buzzer_BeepShort();
                } else {
                    if (settings_focus_idx == 0) temp_theme = (ui_theme_t)((temp_theme + 1) % 3);
                    else if (settings_focus_idx == 1) { temp_audio = !temp_audio; Buzzer_SetMute(!temp_audio); }
                    else if (settings_focus_idx == 2) temp_cb = !temp_cb;
                    else if (settings_focus_idx == 3) temp_led = !temp_led;
                    else if (settings_focus_idx == 4) temp_seg = !temp_seg;
                    else if (settings_focus_idx == 5) temp_font = (ui_font_t)((temp_font + 1) % 2);
                    LCD_UpdateSettingsOption((uint8_t)settings_focus_idx);
                    Buzzer_BeepShort();
                }
            }

            if (ts_click) {
                // Touch detection: map pixel Y-coordinate to settings row.
                // Each row is 33 pixels tall (41, 74, 107, 140, 173, 206, 239).
                uint8_t hit = 0;
                int8_t new_idx = settings_focus_idx;
                if (py >= 41 && py <= 74) { new_idx = 0; hit = 1; }
                else if (py >= 74 && py <= 107) { new_idx = 1; hit = 1; }
                else if (py >= 107 && py <= 140) { new_idx = 2; hit = 1; }
                else if (py >= 140 && py <= 173) { new_idx = 3; hit = 1; }
                else if (py >= 173 && py <= 206) { new_idx = 4; hit = 1; }
                else if (py >= 206 && py <= 239) { new_idx = 5; hit = 1; }

                if (hit) {
                    // Touch landed on a settings row.
                    if (new_idx == settings_focus_idx) {
                        // Double-tap same row: toggle/cycle the setting.
                        if (new_idx == 0) temp_theme = (ui_theme_t)((temp_theme + 1) % 3);
                        else if (new_idx == 1) { temp_audio = !temp_audio; Buzzer_SetMute(!temp_audio); }
                        else if (new_idx == 2) temp_cb = !temp_cb;
                        else if (new_idx == 3) temp_led = !temp_led;
                        else if (new_idx == 4) temp_seg = !temp_seg;
                        else if (new_idx == 5) temp_font = (ui_font_t)((temp_font + 1) % 2);
                        LCD_UpdateSettingsOption((uint8_t)new_idx);
                    } else {
                        // Single-tap different row: just move focus, do not change value.
                        int8_t old = settings_focus_idx;
                        settings_focus_idx = new_idx;
                        LCD_UpdateSettingsOption((uint8_t)old);
                        LCD_UpdateSettingsOption((uint8_t)new_idx);
                    }
                    Buzzer_BeepShort();
                }
                else if (py >= 280) {
                    // Touch in bottom confirmation area (OK/CANCEL buttons).
                    if (px >= 120) {
                        // OK button: commit all temp settings to live config.
                        current_theme = temp_theme; audio_enabled = temp_audio; colorblind_mode = temp_cb;
                        led_enabled = temp_led; seg_enabled = temp_seg; current_font = temp_font;
                        // Apply LED and 7-seg changes immediately.
                        if (!led_enabled) RGB_Set(0, 0, 0); else RGB_Update_From_State();
                        if (!seg_enabled) SEG_AllOff();
                        app_state = APP_HOME; Buzzer_BeepShort();
                    } else if (px <= 110) {
                        // CANCEL button: discard temp settings and return to home.
                        app_state = APP_HOME; Buzzer_SetMute(!audio_enabled); Buzzer_BeepShort();
                    }
                }
            }
        }
        else if (app_state == APP_MODE_SELECT) {
            // Mode selection screen navigation.
            // Three modes: 0=MODE1, 1=MODE2, 2=MODE3, or -1=back.
            if (joy_down || k1_click) {
                if (mode_focus_idx < 2) mode_focus_idx++;
                else mode_focus_idx = -1;
                LCD_UpdateModeSelection();
                Buzzer_BeepShort();
            }
            if (joy_up) {
                if (mode_focus_idx == -1) mode_focus_idx = 2;
                else if (mode_focus_idx > 0) mode_focus_idx--;
                else mode_focus_idx = -1;
                LCD_UpdateModeSelection();
                Buzzer_BeepShort();
            }
            // Confirm mode selection with 300ms debounce (prevent double-presses).
            if ((fire_pressed || k2_click) && ((now - last_k2_event_tick) >= 300)) {
                if (mode_focus_idx == -1) {
                    // Back button pressed: return to home.
                    app_state = APP_HOME;
                } else {
                    // Mode selected: store selection and show confirmation popup.
                    selected_mode = (game_mode_t)mode_focus_idx;
                    app_state = APP_MODE_CONFIRM;
                }
                last_k2_event_tick = now;
                Buzzer_BeepShort();
            }

            // Touch input for mode selection.
            if (ts_click) {
                if (px >= 20 && px <= 220) {
                    // Map Y-coordinate to mode row (3 modes on screen).
                    int8_t new_idx = -1;
                    if (py >= 70 && py <= 110) new_idx = 0;  // MODE 1 row
                    else if (py >= 125 && py <= 165) new_idx = 1;  // MODE 2 row
                    else if (py >= 180 && py <= 220) new_idx = 2;  // MODE 3 row

                    if (new_idx != -1) {
                        if (new_idx == mode_focus_idx) {
                            // Double-tap same mode: confirm selection.
                            selected_mode = (game_mode_t)new_idx;
                            app_state = APP_MODE_CONFIRM;
                        } else {
                            // Single-tap different mode: just move focus.
                            mode_focus_idx = (int8_t)new_idx;
                            LCD_UpdateModeSelection();
                        }
                        Buzzer_BeepShort();
                    }
                }
            }
        }
        else if (app_state == APP_MODE_CONFIRM) {
            // Confirmation popup for mode choice.
            if (k1_click) {
                // Cancel: go back to mode selection.
                app_state = APP_MODE_SELECT;
                Buzzer_BeepShort();
            }
            // Confirm with 300ms debounce.
            if ((fire_pressed || k2_click) && ((now - last_k2_event_tick) >= 300)) {
                last_k2_event_tick = now;
                // Mode 2 and 3 skip car selection and go straight to game.
                // Mode 1 requires car selection before starting.
                if (selected_mode == GAME_MODE_2 || selected_mode == GAME_MODE_3) {
                    app_state = APP_GAME;
                } else {
                    app_state = APP_CAR_SELECT;
                }
                Buzzer_BeepShort();
            }
            if (ts_click) {
                if (py >= 190 && py <= 235) {
                    if (px >= 10 && px <= 115) { app_state = APP_MODE_SELECT; Buzzer_BeepShort(); }
                    else if (px >= 125 && px <= 230) {
                        if (selected_mode == GAME_MODE_2 || selected_mode == GAME_MODE_3) { app_state = APP_GAME; }
                        else { app_state = APP_CAR_SELECT; }
                        Buzzer_BeepShort();
                    }
                }
            }
        }
        else if (app_state == APP_CAR_SELECT) {
            // Car selection screen: 7 cars (V0-V6) or back button (-1).
            if (joy_down || k1_click) {
                // Navigate down through car list.
                if (car_focus_idx < 6) car_focus_idx++;
                else car_focus_idx = -1;  // Wrap to back button.
                LCD_UpdateCarSelection();
                Buzzer_BeepShort();
            }
            if (joy_up) {
                // Navigate up with wrap-around.
                if (car_focus_idx == -1) car_focus_idx = 6;
                else if (car_focus_idx > 0) car_focus_idx--;
                else car_focus_idx = -1;
                LCD_UpdateCarSelection();
                Buzzer_BeepShort();
            }
            // Confirm car selection with 300ms debounce.
            if ((fire_pressed || k2_click) && ((now - last_k2_event_tick) >= 300)) {
                if (car_focus_idx == -1) {
                    // Back: return to mode selection.
                    app_state = APP_MODE_SELECT;
                } else {
                    // Car chosen: store and show confirmation popup.
                    selected_car = (car_type_t)car_focus_idx;
                    app_state = APP_CAR_CONFIRM;
                }
                last_k2_event_tick = now;
                Buzzer_BeepShort();
            }

            // Touch input for car selection.
            if (ts_click) {
                if (px >= 14 && px <= 226) {
                    // Each car row is 26 pixels tall; calculate which row was touched.
                    int8_t new_idx = (py - 64) / 26;
                    if (new_idx >= 0 && new_idx <= 6) {
                        if (new_idx == car_focus_idx) {
                            // Double-tap same car: confirm selection.
                            selected_car = (car_type_t)new_idx;
                            app_state = APP_CAR_CONFIRM;
                        } else {
                            // Single-tap different car: move focus only.
                            car_focus_idx = new_idx;
                            LCD_UpdateCarSelection();
                        }
                        Buzzer_BeepShort();
                    }
                }
            }
        }
        else if (app_state == APP_CAR_CONFIRM) {
            // Car confirmation popup.
            if (k1_click) {
                // Cancel: return to car selection.
                app_state = APP_CAR_SELECT;
                Buzzer_BeepShort();
            }
            // Confirm with 300ms debounce.
            if ((fire_pressed || k2_click) && ((now - last_k2_event_tick) >= 300)) {
                last_k2_event_tick = now;
                // Proceed to gameplay.
                app_state = APP_GAME;
                Buzzer_BeepShort();
            }
            if (ts_click) {
                if (py >= 190 && py <= 235) {
                    if (px >= 10 && px <= 115) { app_state = APP_CAR_SELECT; Buzzer_BeepShort(); }
                    else if (px >= 125 && px <= 230) { app_state = APP_GAME; Buzzer_BeepShort(); }
                }
            }
        }
        else if (app_state == APP_GAME) {
            // Active gameplay loop.
            laser_update();
            RGB_Update_From_State();

            if (selected_mode == GAME_MODE_1) {
                // Mode 1: single-player tank dueling.
                Game_Router_Task(x_raw, y_raw, k1_click, k2_click, fire_pressed);
                // Frequent LCD updates for responsive movement display.
                if (now - lcd_fast_tick >= LCD_FAST_UPDATE_MS) {
                    LCD_UpdateGameFast(x_raw, y_raw);
                    lcd_fast_tick = now;
                }
                // Slower updates for HUD elements (status, stats).
                if (now - lcd_slow_tick >= LCD_SLOW_UPDATE_MS) {
                    LCD_UpdateGameSlow(fire_pressed);
                    lcd_slow_tick = now;
                }
            }
            else if (selected_mode == GAME_MODE_2) {
                // Mode 2: drawing challenge mode.
                Mode2_Run(x_raw, y_raw, k1_click, k2_click, fire_pressed, ts_pressed, ts_click, px, py, joy_up, joy_down, joy_left, joy_right);
            }
            else if (selected_mode == GAME_MODE_3) {
                // Mode 3: arena-based multiplayer with bots.
                Mode3_Run(x_raw, y_raw, k1_click, k2_click, fire_pressed, ts_pressed, ts_click, px, py, joy_up, joy_down, joy_left, joy_right);
            }
        }

        // Default 7-Segment Telemetry Display Management
        // Only update 7-seg display when seg_mode is idle (not running other display tasks).
        if (seg_mode == SEG_IDLE) {
            if (app_state == APP_HOME) {
                // Home screen: show "00" (both digits off).
                SEG_ShowPair(0, 0, 0);
            }
            else if (app_state == APP_SETTINGS) {
                // Settings screen: show "88" (all segments on).
                SEG_ShowPair(8, 8, 0);
            }
            else if (app_state == APP_GAME) {
                // In-game telemetry display varies by mode.
                if (selected_mode == GAME_MODE_1) {
                    // Mode 1: left = mode digit (1), right = car version (0-6).
                    uint8_t left = 1;
                    uint8_t right = (uint8_t)selected_car;
                    SEG_ShowPair(left, right, 0);
                }
                else if (selected_mode == GAME_MODE_2) {
                    // Mode 2: left = mode digit (2), right = input method (1=joystick, 2=touch).
                    uint8_t left = 2;
                    uint8_t right = (m2_input_method == M2_INPUT_JOYSTICK) ? 1 : 2;
                    SEG_ShowPair(left, right, 0);
                }
                else if (selected_mode == GAME_MODE_3) {
                    // Mode 3: left = arena size (1x, 4x, 9x), right = setup phase or HP.
                    // Arena size displayed as 1, 4, or 9.
                    uint8_t arena_display = (m3_arena_size == M3_SIZE_DEFAULT) ? 1 :
                                            (m3_arena_size == M3_SIZE_NORMAL) ? 4 : 9;
                    uint8_t left = arena_display;
                    uint8_t right;
                    // Right digit shows setup phase (0=size, 1=obstacles, 2=bots) or player HP during battle.
                    if (m3_state == M3_STATE_SETUP_SIZE) right = 0;
                    else if (m3_state == M3_STATE_SETUP_OBSTACLES) right = 1;
                    else if (m3_state == M3_STATE_SETUP_BOTS) right = 2;
                    else if (m3_state == M3_STATE_BATTLE) {
                        // During battle: show HP (0-9 or 'A' for 10+).
                        right = (m3_hp >= 10) ? 10 : m3_hp;
                    }
                    else right = 0;
                    SEG_ShowPair(left, right, 0);
                }
            }
            else {
                // Menu states: show current mode selection for reference.
                uint8_t left = (uint8_t)selected_mode + 1;
                uint8_t right = (selected_mode == GAME_MODE_1) ? (uint8_t)selected_car : 0;
                SEG_ShowPair(left, right, 0);
            }
        }

        // Frame-end housekeeping tasks.
        Buzzer_Task();      // Update buzzer state (beep timing, mute state).
        SEG_Task();         // Update 7-segment display (multiplexing, digit switching).
        HAL_Delay(10);      // Wait 10 ms to maintain stable frame timing (~100 Hz loop rate).
    }
}

void SystemClock_Config(void)
{
    /* SystemClock_Config programs the clock tree so the MCU runs at the configured
     * frequency with the correct bus prescalers and flash latency.
     */

    RCC_OscInitTypeDef       o = {0};
    RCC_ClkInitTypeDef       c = {0};
    RCC_PeriphCLKInitTypeDef p = {0};
    o.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    o.HSEState       = RCC_HSE_ON;
    o.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    o.HSIState       = RCC_HSI_ON;
    o.PLL.PLLState   = RCC_PLL_ON;
    o.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    o.PLL.PLLMUL     = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&o) != HAL_OK) Error_Handler();
    c.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    c.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV2;
    c.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
    p.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    p.AdcClockSelection    = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&p) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
    /* MX_ADC1_Init configures the first ADC channel used to read one joystick axis.
     */

    ADC_ChannelConfTypeDef s = {0};
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
    s.Channel = ADC_CHANNEL_11;
    s.Rank = ADC_REGULAR_RANK_1;
    s.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &s) != HAL_OK) Error_Handler();
}

static void MX_ADC2_Init(void)
{
    /* MX_ADC2_Init configures the second ADC channel used to read the other
     * joystick axis.
     */

    ADC_ChannelConfTypeDef s = {0};
    hadc2.Instance = ADC2;
    hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) Error_Handler();
    s.Channel = ADC_CHANNEL_13;
    s.Rank = ADC_REGULAR_RANK_1;
    s.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc2, &s) != HAL_OK) Error_Handler();
}

/*static void MX_I2C2_Init(void)
{
    /* MX_I2C2_Init prepares the I2C peripheral used by external devices on the
     * board.


    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) Error_Handler();
}*/

static void MX_USART3_UART_Init(void)
{
    /* MX_USART3_UART_Init configures the UART link used to communicate with the
     * WiFi/ESP hardware.
     */

    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    /* MX_GPIO_Init configures all board GPIO pins used by buttons, lights, and
     * other digital control signals.
     */

    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1 | GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RGB_R_PORT, RGB_R_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RGB_G_PORT, RGB_G_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RGB_B_PORT, RGB_B_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BEEP_PORT, BEEP_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(T_CS_PORT, T_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(T_CLK_PORT, T_CLK_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(T_DIN_PORT, T_DIN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    g.Pin = GPIO_PIN_12; g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOD, &g);
    g.Pin = T_CS_PIN; g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(T_CS_PORT, &g);
    g.Pin = GPIO_PIN_1 | GPIO_PIN_7; HAL_GPIO_Init(GPIOE, &g);
    g.Pin = T_CLK_PIN | T_DIN_PIN; g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &g);
    g.Pin = T_DOUT_PIN | T_IRQ_PIN; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &g);
    g.Pin = LASER_PIN; g.Speed = GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(LASER_PORT, &g);
    g.Pin = RGB_R_PIN | RGB_G_PIN | RGB_B_PIN | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);
    g.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | BEEP_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = CAP_TOUCH_PIN; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CAP_TOUCH_PORT, &g);
    g.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);
    g.Pin  = JOY_SW_PIN; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(JOY_SW_PORT, &g);
    g.Pin  = BTN1_PIN | BTN2_PIN; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BTN1_PORT, &g);
    g.Pin  = K1_PIN; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(K1_PORT, &g);
    g.Pin  = K2_PIN; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(K2_PORT, &g);
}

static void MX_FSMC_Init(void)
{
    /* MX_FSMC_Init configures the external memory controller that drives the LCD
     * panel interface.
     */

    FSMC_NORSRAM_TimingTypeDef t = {0};
    hsram1.Instance = FSMC_NORSRAM_DEVICE;
    hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
    hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
    hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
    hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
    hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
    hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
    hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
    hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
    hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
    hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
    hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
    hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
    hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
    hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
    t.AddressSetupTime = 15; t.AddressHoldTime = 15; t.DataSetupTime = 255;
    t.BusTurnAroundDuration = 15; t.CLKDivision = 16; t.DataLatency = 17;
    t.AccessMode = FSMC_ACCESS_MODE_A;
    if (HAL_SRAM_Init(&hsram1, &t, NULL) != HAL_OK) Error_Handler();
    __HAL_AFIO_FSMCNADV_DISCONNECTED();
}

void Error_Handler(void) { __disable_irq(); while (1) {} }
/* Error_Handler disables interrupts and loops forever so the system remains in
 * a known safe state after an unrecoverable failure.
 */
