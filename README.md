# 🎮 ELEC3300: Tank "Wargame" - STM32F103VET6 Project

A fully-featured embedded gaming system featuring a tank combat simulator with multiple gameplay modes, WiFi connectivity, real-time UI feedback, and comprehensive sensor integration on the STM32F103VET6 microcontroller.

---

## 🚀 Project Overview

**Status:** Fully Documented & Production-Ready  
**Hardware:** STM32F103VET6 (100-pin, 512KB Flash, 64KB RAM)  
**Last Update:** Complete code documentation pass (1500+ comment lines added)  
**Repository:** https://github.com/tototofu123/stm32

### Core Features
✅ **3 Game Modes** (Mode 1: Duel, Mode 2: Drawing Challenge, Mode 3: Arena Battle)  
✅ **7 Unique Vehicles** (V0-V6 with distinct abilities)  
✅ **Real-time HUD** (LCD 320x240 + dual 7-segment telemetry)  
✅ **WiFi Control** (ESP8266 AT commands via UART3)  
✅ **Multi-input** (Joystick analog + buttons + capacitive touch + touchscreen)  
✅ **Audio/Visual Feedback** (Buzzer + RGB LED + 7-segment display)  
✅ **Laser State Machine** (5-state firing system with charge-up)  
✅ **Collision Physics** (Tank bounce-back + projectile bounce)  
✅ **AI Bots** (Mode 3 arena with difficulty-scaling enemies)

---

## 📚 Core Module Documentation

### 🎯 **Core/Inc/** - Header Files (Public Interfaces)

| File | Purpose | Key Exports | Status |
|------|---------|------------|--------|
| **main.h** | 🔧 HAL baseline & entry point | `Error_Handler()` prototype | ✅ Documented |
| **game_logic.h** | 🎮 Shared game state & enums | App/mode/car/laser state enums, car timing rules | ✅ Documented |
| **ui.h** | 🖥️ UI system & screen drawing API | 20+ screen drawing functions, color palette, focus indices | ✅ Documented |
| **mode_2.h** | 🎨 Drawing challenge mode | `Mode2_Init/Run`, input method selection | ✅ Documented |
| **mode_3.h** | ⚔️ Arena multiplayer mode | `Mode3_Init/Run`, arena sizes, bot difficulty, tile types | ✅ Documented |
| **lcd.h** | 🖼️ LCD driver primitives | 18 drawing functions (lines, rectangles, text, ellipse) | ✅ Documented |
| **peripherals.h** | 📡 Hardware helpers | WiFi/ESP AT commands, ADC reads, sensor utilities | ✅ Documented |
| **touch.h** | 👆 Touchscreen controller | XPT2046 interface (press detect, coordinate reads) | ✅ Documented |
| **seven_seg.h** | 🔢 7-segment display driver | 10 display functions (digit, pair, countdown, custom) | ✅ Documented |
| **alerts.h** | 🔊 Buzzer & RGB LED | `RGB_Set()`, `Buzzer_BeepShort/Long()`, `Buzzer_Task()` | ✅ Documented |

---

### 💻 **Core/Src/** - Source Files (Implementation)

#### **🎮 Application Layer**

| File | Lines | Purpose | Key Functions |
|------|-------|---------|---|
| **main.c** | 690 | 🔴 **CORE RUNTIME** - entry point, HAL init, main polling loop, state machine routing, input handling | `main()`, system clock config, all MX_*_Init functions, state-machine dispatch |
| **game_logic.c** | 366 | 🎲 **SHARED GAMEPLAY** - car properties, timing rules, laser state machine, motor/fire commands | `car_prime_ms()`, `car_fire_ms()`, `car_cooldown_ms()`, `laser_update()`, `Motor_SendCmd()` |

#### **🎨 User Interface Layer**

| File | Lines | Purpose | Key Functions |
|------|-------|---------|---|
| **ui.c** | 753 | 🖥️ **SCREEN DRAWING** - all menus, HUD, car preview, settings, focus highlighting | `LCD_DrawHome()`, `LCD_DrawModeSelect()`, `LCD_DrawCarSelect()`, `LCD_DrawGameLayout()`, `LCD_UpdateGameFast/Slow()` |

#### **🎮 Game Mode Implementations**

| File | Lines | Purpose | Key Functions |
|------|-------|---------|---|
| **mode_2.c** | 274 | 🎨 **DRAWING CHALLENGE** - canvas state, cursor tracking, path tracing, movement history | `Mode2_Init()`, `Mode2_ResetCanvas()`, `Mode2_Run()` |
| **mode_3.c** | 689 | ⚔️ **ARENA BATTLE** - world generation, tank/AI/projectile state, collision detection, obstacle/powerup handling | `Mode3_Init()`, `Mode3_GenerateWorld()`, `Mode3_Run()`, collision & AI logic |

#### **🔧 Hardware Driver Layer**

| File | Lines | Purpose | Key Functions |
|------|-------|---------|---|
| **lcd.c** | 214 | 🖼️ **LCD CONTROLLER** - panel init, FSMC memory interface, drawing primitives (lines, rectangles, text) | `LCD_INIT()`, `LCD_Clear()`, `LCD_DrawLine()`, `LCD_DrawString()` |
| **peripherals.c** | 115 | 📡 **HARDWARE HELPERS** - WiFi/ESP AT commands, ADC joystick reads, utility mapping | `sendAT()`, `WifiSetUp()`, `read_adc1/2()`, `map_u16()` |
| **touch.c** | 52 | 👆 **TOUCHSCREEN DRIVER** - XPT2046 SPI protocol, coordinate calibration, press detection | `TouchPressed()`, `TouchReadXRaw/Y()` |
| **seven_seg.c** | 200 | 🔢 **7-SEGMENT DISPLAY** - GPIO multiplexing, digit/segment control, telemetry display modes | `SEG_ShowPair()`, `SEG_ShowTenths()`, `SEG_Task()` |
| **alerts.c** | 62 | 🔊 **AUDIO/LED FEEDBACK** - buzzer timing, RGB LED color control, beep queue management | `Buzzer_BeepShort/Long()`, `RGB_Set()`, `Buzzer_Task()` |

---

## 📊 Data Flow Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        MAIN.C (690 lines)                   │
│                    - Entry Point & Main Loop                │
│                    - Input Polling (all sources)            │
│                    - State Machine Routing                  │
└─────────────────────────────────────────────────────────────┘
                              │
         ┌────────────────────┼────────────────────┐
         │                    │                    │
         ▼                    ▼                    ▼
  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐
  │  GAME_LOGIC.C  │  │     UI.C       │  │   MODE 2/3.C   │
  │ (366 lines)    │  │  (753 lines)   │  │  (963 lines)   │
  │                │  │                │  │                │
  │ • Car props    │  │ • All screens  │  │ • Game loops   │
  │ • Laser FSM    │  │ • HUD updates  │  │ • World gen    │
  │ • Timing rules │  │ • Focus mgmt   │  │ • Collision    │
  │ • Commands     │  │ • Colors       │  │ • AI behavior  │
  └────────────────┘  └────────────────┘  └────────────────┘
         │                    │                    │
         └────────────────────┼────────────────────┘
                              │
         ┌────────────────────┼────────────────────┐
         │                    │                    │
         ▼                    ▼                    ▼
  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐
  │   LCD.C        │  │  TOUCH.C       │  │ SEVEN_SEG.C    │
  │ (214 lines)    │  │  (52 lines)    │  │ (200 lines)    │
  │                │  │                │  │                │
  │ • Primitives   │  │ • XPT2046 SPI  │  │ • 7-seg ctrl   │
  │ • Text render  │  │ • Calibration  │  │ • Telemetry    │
  │ • Color fill   │  │ • Press detect │  │ • Digit display│
  └────────────────┘  └────────────────┘  └────────────────┘
         │                    │                    │
         └────────────────────┼────────────────────┘
                              │
         ┌────────────────────┼────────────────────┐
         │                    │                    │
         ▼                    ▼                    ▼
  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐
  │ PERIPHERALS.C  │  │   ALERTS.C     │  │  STM32 HAL     │
  │ (115 lines)    │  │   (62 lines)   │  │  (Generated)   │
  │                │  │                │  │                │
  │ • WiFi/ESP AT  │  │ • Buzzer beeps │  │ • GPIO         │
  │ • ADC reads    │  │ • RGB LED ctrl │  │ • UART/I2C     │
  │ • Map_u16      │  │ • Timing       │  │ • ADC/PWM      │
  └────────────────┘  └────────────────┘  └────────────────┘
```

---

## 🎛️ Input/Output Matrix

### **Inputs** 🔌
| Source | Type | Purpose |
|--------|------|---------|
| **Joystick (ADC1/ADC2)** | Analog (0-4095) | Vehicle movement direction & speed |
| **K1 Button (PA0)** | Digital | Menu navigation up, game movement left |
| **K2 Button (PC13)** | Digital | Menu confirm, game movement right |
| **Joystick Button (PC2)** | Digital | Firing / popup confirm (300ms debounce) |
| **Capacitive Touch (PA1)** | Digital | Quick-start from menus or special ability reset |
| **Touchscreen (XPT2046)** | SPI (12-bit X/Y) | Menu clicks, Mode 2 canvas drawing, touch aim |

### **Outputs** 📤
| Target | Type | Purpose |
|--------|------|---------|
| **LCD (ILI9341)** | FSMC (16-bit) | All visual UI, game world, HUD |
| **7-Segment (14 GPIO)** | Digital | Mode/car/HP/timer telemetry |
| **RGB LED (PB5/PB0/PB1)** | Digital | Laser state feedback (Green=firing, Red=cooldown, Yellow=charging) |
| **Buzzer (PA8)** | Digital | Beep on menu actions (80ms short, 500ms long) |
| **ESP8266 (UART3)** | Serial | AT commands for motor control (F100, B050, etc.) |

---

## 🎮 Gameplay Modes

### **Mode 1: Single-Player Tank Duel** 🎯
- **Setup:** Choose car (V0-V6), each with unique timing
- **Gameplay:** Real-time 1v1 vs ESP-controlled opponent
- **Input:** Joystick analog + buttons for movement/firing
- **HUD:** Live laser status, motion%, car specs (speed, charge, cooldown)
- **Win:** Deplete opponent HP (starts at 3, min 1)

### **Mode 2: Drawing Challenge** 🎨
- **Setup:** Choose input (joystick smooth vs touch direct)
- **Gameplay:** Trace a path on canvas, score by distance & time
- **Mechanics:** Path tracking, overlap detection, movement commands
- **Timer:** Challenge countdown, visible on 7-seg display
- **End:** Show score & stats

### **Mode 3: Arena Battle (Multiplayer Setup)** ⚔️
- **Arena Sizes:** 1x (compact), 4x (standard), 9x (huge)
- **Obstacles:** Walls, rivers (slow movement), supplements (+1 HP), traps (-1 HP)
- **Power-ups:** Speed boost (2x), wings (ignore slowdown), shield (reflect damage)
- **AI Bots:** 3 opponents, selectable difficulty (OFF, EASY, MID, HARD)
- **Mechanics:** Collision bounce (6px apart), projectile physics, FOV-based aiming
- **Display:** Live HP (0-9 or 'A' for 10+), power-up timers

---

## 🛠️ Hardware Configuration

### **GPIO Pin Map**
```
Port A:
  PA0  → K1 Button (menu nav / move left)
  PA1  → Capacitive Touch (quick-start / ability reset)
  PA4  → Laser control (active-high)
  PA5  → LSEG_A (7-seg left)
  PA6  → LSEG_B (7-seg left)
  PA7  → LSEG_E (7-seg left)
  PA8  → Buzzer (beep on actions)

Port B:
  PB0  → RGB_G (green LED)
  PB1  → RGB_B (blue LED)
  PB5  → RGB_R (red LED)
  PB6  → LSEG_G (7-seg left)
  PB7  → LSEG_F (7-seg left)
  PB10 → UART3 TX (ESP8266)
  PB11 → UART3 RX (ESP8266)
  PB12 → RSEG_G (7-seg right)
  PB13 → RSEG_F (7-seg right)
  PB14 → RSEG_A (7-seg right)
  PB15 → RSEG_B (7-seg right)

Port C:
  PC2  → Joystick Button (fire / confirm)
  PC4  → LSEG_C (7-seg left)
  PC5  → RSEG_C (7-seg right)
  PC6  → RSEG_E (7-seg right)
  PC7  → RSEG_D (7-seg right)
  PC13 → K2 Button (menu confirm / move right)

Port D:
  PD12 → LCD Backlight control
  PD13 → Touchscreen CS

Port E:
  PE0  → T_CLK (touchscreen clock)
  PE1  → LCD Reset
  PE2  → T_DIN (touchscreen data in)
  PE3  → T_DOUT (touchscreen data out)
  PE4  → T_IRQ (touchscreen interrupt)
  PE7  → LSEG_DP (7-seg decimal point left)
```

### **Analog Inputs**
```
ADC1 (PA4 input) → Joystick X-axis (0-4095, center~2200)
ADC2 (PA5 input) → Joystick Y-axis (0-4095, center~2200)
```

### **Interfaces**
```
FSMC Bank 1 → ILI9341 LCD (16-bit data bus)
  Command:  0x60000000
  Data:     0x60020000

UART3 (PB10/11) → ESP8266 AT commands (115200 baud)

I2C2 → External device (reserved)

SPI (bit-banged) → XPT2046 Touchscreen
  CS:   PD13
  CLK:  PE0
  DIN:  PE2
  DOUT: PE3
  IRQ:  PE4
```

---

## 🎯 Application States

```
APP_HOME
  ├─ K1 or Touch "START BATTLE" → APP_MODE_SELECT
  └─ K2 or Touch "PREFERENCES" → APP_SETTINGS

APP_SETTINGS (6 options)
  ├─ Theme (3 schemes)
  ├─ Audio (on/off)
  ├─ Colorblind mode (on/off)
  ├─ LED (on/off)
  ├─ 7-Segment (on/off)
  ├─ Font (default/large)
  └─ Back button → APP_HOME

APP_MODE_SELECT (3 modes)
  ├─ Mode 1 (single-player duel)
  ├─ Mode 2 (drawing challenge)
  ├─ Mode 3 (arena battle)
  ├─ Capacitive touch skips to APP_GAME (Mode 1 with defaults)
  └─ Back button → APP_HOME

APP_MODE_CONFIRM (popup)
  ├─ Mode 1 → APP_CAR_SELECT
  ├─ Mode 2/3 → APP_GAME (skip car selection)
  └─ Back button → APP_MODE_SELECT

APP_CAR_SELECT (7 cars: V0-V6)
  ├─ Each car shows name, speed, charge, fire, cooldown
  ├─ Bottom panel: color-coded by car, dynamic preview
  └─ Back button → APP_MODE_SELECT

APP_CAR_CONFIRM (popup)
  ├─ Confirm → APP_GAME
  └─ Back button → APP_CAR_SELECT

APP_GAME
  ├─ Mode 1: 1v1 tank duel
  ├─ Mode 2: Drawing challenge
  ├─ Mode 3: Arena multiplayer
  └─ End game → back to APP_HOME
```

---

## 📈 File Statistics

| Category | Count | Total Lines | Avg Size |
|----------|-------|------------|----------|
| **Header Files (.h)** | 10 | 850 | 85 LOC |
| **Source Files (.c)** | 11 | 3,115 | 283 LOC |
| **Total Documentation** | - | 1,500+ | Comments |
| **Total Project** | 21 | **4,965** | **~236 LOC/file** |

**Backup Versions:**
- `old version/4-5 7pm/` → Previous session (12 files)
- `old version/4-5 920pm/uncommented/` → Original GitHub before comments (20 files)

---

## 🔄 Development Timeline

| Date | Event |
|------|-------|
| Earlier | Initial hardware integration (motors, LCD, WiFi) |
| Earlier | Modes 2 & 3 implementation & AI/collision physics |
| Earlier | UI state machine + settings menu |
| Batch 1 | main.c & main.h: Frame loop + state routing comments |
| Batch 2 | game_logic.c/h & ui.h: Shared state + UI API docs |
| Batch 3 | ui.c + mode_2/3.h: Screen drawing + mode APIs |
| Batch 4 | lcd.h, peripherals.h, touch.h, seven_seg.h, alerts.h |
| Final | Complete documentation push (1500+ lines added) |

---

## 📖 How to Navigate the Code

1. **Start in `main.c`** – Entry point and state machine dispatcher
2. **Follow `game_logic.c`** – Shared rules (car timing, laser FSM)
3. **Check `ui.c`** – How screens are drawn and updated
4. **Pick a mode** – `mode_2.c` (drawing) or `mode_3.c` (arena)
5. **Trace hardware** – `lcd.c` (display), `peripherals.c` (WiFi/ADC), `touch.c` (touchscreen), `seven_seg.c` (telemetry), `alerts.c` (feedback)

Each file has a **detailed module-level comment block** at the top explaining its role, dependencies, and data flow.

---

## ✅ Documentation Status

✨ **All 21 target files fully documented:**
- Module-level headers (what it does, responsibility, globals, data flow)
- Function-level comments (input, output, behavior)
- Inline comments (complex logic, state transitions, hardware details)
- Zero code logic changes — comments only

**Quality:** Professional-grade documentation suitable for code reviews, maintenance, and educational purposes.

---

*Developed for ELEC3300 Embedded Systems Project. Repository: https://github.com/tototofu123/stm32*
