# STM32 Multi-Mode Car Project

This is the final patch version of the STM32-based mobile platform project before code cleanup. It features a multi-mode operation system, WiFi connectivity for motor control, an integrated laser state machine, and full UI support via LCD and 7-segment displays.

## 🚀 Key Features

### 1. Multi-Mode Application Flow
The project implements a boot-up menu system:
*   **MODE SELECT**: Choose between different game modes (Mode 1: Active Gameplay, Mode 2/3: Placeholders).
*   **CAR SELECT**: Select from 7 different car variations (V0 to V6), each with unique speed caps, firing mechanics, and movement constraints.
*   **GAME**: The active execution state where sensors, motors, and peripherals interact.

### 2. Laser & Combat System
Integrated state machine for laser control:
*   **States**: Idle -> Armed -> Priming (Charging) -> Firing -> Cooldown.
*   **Variations**: Supports manual firing (JSW or K2 depending on car type) and Auto-Fire (V1 Car).
*   **Feedback**: Visual feedback via RGB LED (Green: Firing, Red: Cooldown, Yellow: Charging) and the 7-segment display for cooldown countdowns.

### 3. Buzzer Feedback (Latest Update)
Full acoustic feedback system implemented on **PA8**:
*   **Short Beep (80ms)**: Triggered on menu navigation (K1).
*   **Long Beep (500ms)**: Triggered on selection confirmation (K2).
*   **Continuous Tone**: Active during the `LASER_PRIMING` phase to simulate charging.

### 4. Connectivity & Hardware
*   **WiFi (ESP-01S)**: Configures an Access Point (`ESP8266_AP_01`) and runs a TCP server on port 80.
*   **Motor Control**: Communicates via USART3 using a protocol like `F100` (Forward 100%).
*   **Sensors**: Joystick (ADC1/ADC2), DS18B20 Temperature Sensor.
*   **Display**: ILI9341 LCD via FSMC for real-time telemetry.

## 🛠 Hardware Configuration

| Peripheral | Port | Pin |
| :--- | :--- | :--- |
| **Buzzer** | GPIOA | PA8 |
| **Laser** | GPIOC | PC4 |
| **K1 Button** | GPIOA | PA0 |
| **K2 Button** | GPIOC | PC13 |
| **Joystick SW** | GPIOC | PC2 |
| **RGB LED** | GPIOB | PB5 (R), PB0 (G), PB1 (B) |
| **DS18B20** | GPIOC | PC8 |
| **UART3 (ESP)**| GPIOB | PB10 (TX), PB11 (RX) |

## 📂 Project Structure
*   `Core/Src/main.c`: Primary application logic and state machines.
*   `Core/Src/lcd.c`: Low-level LCD driver.
*   `old version/`: Archive of previous iterations and patches.

---
*Developed as part of the STM32 development series.*
