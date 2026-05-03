# PROJECT REPOSITORY OVERVIEW: STM32 MULTIPLAYER TANK CAR

This document provides a precise, file-by-file technical breakdown of the firmware architecture, logic flow, and hardware integration.

---

## 1. System Architecture
The system is a hybrid real-time controller for a physical car and a virtual battle arena. It utilizes an STM32F103VETx as the core processor, coordinating four major subsystems:
1.  **Human Interface:** 240x320 FSMC LCD, XPT2046 Resistive Touch, and an Analog Joystick.
2.  **Locomotion:** UART-based motor command bridge (sending 'F', 'L', 'R', 'S' protocols).
3.  **Combat:** 5V Laser diode and RGB LED status indicators.
4.  **Networking:** ESP8266 WiFi module (Dual AP+STA mode) for mobile synchronization.

---

## 2. Core Modules Breakdown (Inc & Src)

### A. System Foundation
- **main.h / main.c**:
    - **Purpose**: System entry point and hardware initialization.
    - **Logic**: Implements the Master State Machine (`app_state`). Manages the 5ms loop cycle.
    - **UI Bridge**: Routes touch coordinates and debounced button clicks to the active Game Mode.
- **peripherals.h / peripherals.c**:
    - **Purpose**: Low-level hardware drivers and WiFi management.
    - **Functions**: 
        - `read_adc1/2()`: Fast sampling for Joystick X/Y.
        - `WifiSetUp()`: Configures ESP8266 for Mode 3 (Dual Hotspot + Station).
        - `WifiScan()`: Parses `+CWLAP` responses into a list of SSIDs.
- **stm32f1xx_it.c**:
    - **Purpose**: Interrupt Service Routines. Handles UART RX interrupts for ESP8266 feedback.

### B. Game Logic & Modes
- **game_logic.h / game_logic.c**:
    - **Purpose**: Central brain for combat and movement state.
    - **Features**: 
        - **Laser Engine**: 5-stage state machine (Idle, Armed, Charging, Firing, Cooldown).
        - **Special Ability**: Capacitive-trigger cooldown reset (recharges every 10 shots).
        - **Drive Task**: Translates joystick raw ADC into speed-capped motor commands.
- **mode_2.h / mode_2.c** (Draw Fight):
    - **Purpose**: Path recording and autonomous execution.
    - **Logic**: Interpolates touch/joystick input into 8px normalized segments.
    - **Visualization**: Renders real-time distance/time stats and 8-way red direction arrows.
- **mode_3.h / mode_3.c** (Tank Arena):
    - **Purpose**: Procedural virtual multiplayer.
    - **Features**: Generates a synchronized world based on a 32-bit `seed`. Implements viewport-based rendering.

### C. Visuals & Interface
- **ui.h / ui.c**:
    - **Purpose**: High-level UI rendering engine.
    - **Screens**: 
        - **Home**: Main navigation hub.
        - **Settings**: WiFi management and IP status.
        - **Virtual Keyboard**: 6x6 touch grid for password entry.
        - **Status Bar**: Global 20px header for network and system telemetry.
- **lcd.h / lcd.c**:
    - **Purpose**: Low-level ILI9341 (FSMC) driver.
    - **Primitives**: `LCD_DrawLine`, `LCD_DrawRectangle`, `LCD_DrawChar`, `LCD_DrawString`.
- **touch.h / touch.c**:
    - **Purpose**: XPT2046 SPI-based touch driver.
    - **Calibration**: Maps 12-bit raw SPI values to 240x320 screen coordinates.

### D. Hardware Feedback
- **seven_seg.h / seven_seg.c**:
    - **Purpose**: Dual 7-segment LED display.
    - **Logic**: Displays movement characters ('F', 'L', 'A', 'S') and laser cooldown counts.
- **alerts.h / alerts.c**:
    - **Purpose**: Haptic and visual alerts.
    - **Drivers**: Non-blocking buzzer scheduler and active-low RGB LED controller.

---

## 3. Data Protocols & Communication

### UART Motor Bridge (115200 Baud)
Sends 4-byte packets to the motor controller:
- `F100` -> Move Forward at 100% speed.
- `L050` -> Rotate Left at 50% speed.
- `T001` -> Toggle Fire signal ON.

### WiFi IoT Handshake
- **Access Point**: SSID: `TANK_CAR`, Password: `password`.
- **Standard IP**: `192.168.4.1` (Serves as the Multiplayer Room ID).

---

## 4. Hardware Pin Mapping (STM32F103VETx)
| Component | Port:Pin | Type | Function |
| :--- | :--- | :--- | :--- |
| **K1 Button** | GPIOA:0 | Input | Cycle Mode / Confirm |
| **K2 Button** | GPIOC:13 | Input | Reset / Settings |
| **Capacitive Key**| GPIOA:1 | Input | Special Ability (Reset CD) |
| **Joystick Button**| GPIOC:2 | Input | FIRE Command |
| **Joystick X** | GPIOA:0 (ADC1) | Analog | Horizontal Drive |
| **Joystick Y** | GPIOA:1 (ADC2) | Analog | Vertical Drive |
| **Laser Diode** | GPIOC:4 | Output | Firing Signal |
| **RGB Red** | GPIOB:5 | Output | Cooldown Status |
| **RGB Green** | GPIOB:0 | Output | Firing Status |
| **RGB Blue** | GPIOB:1 | Output | WiFi Status |
| **Buzzer** | GPIOA:8 | Output | Haptic Feedback |

---
*End of Documentation.*
