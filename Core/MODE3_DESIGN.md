# MODE 3 & HOME SCREEN ARCHITECTURE DESIGN

## 1. Master UI Flow
The system will no longer boot directly into Mode selection. It will follow a "Modern Handheld" startup sequence:
1.  **Boot Screen:** Splash logo and hardware check (Motor, LCD, Touch, ESP8266).
2.  **Home Screen:**
    - `[ BATTLE ]` Button (Leads to the existing Mode 1/2 and new Mode 3).
    - `[ SETTINGS ]` Button (Leads to WiFi management).
    - Status Bar: Shows WiFi signal strength, current IP, and battery/state.

## 2. Settings & WiFi Management
This turns the car into a standalone IoT device.
- **Scanning:** Trigger `AT+CWLAP` via ESP8266. Parse the multi-line result to extract SSIDs and Security types.
- **Network List:** A touch-scrollable list of found WiFi networks.
- **Virtual Keyboard:**
    - Full QWERTY or Grid layout on the 240x320 LCD.
    - Input buffer to store the password.
    - Visual feedback (asterisks for password characters).
- **Connection Logic:** Use `AT+CWJAP="SSID","PASS"`. Implement a timeout and "Connection Successful/Failed" pop-up.

## 3. Mode 3: The Infinite Arena (Multiplayer)
This mode removes physical boundaries by using a virtual battlefield synchronized between the Car and a Web Client (Phone/Laptop).

### A. The "Seed" Synchronization
- **Procedural Generation:** Use a Linear Congruential Generator (LCG) or Perlin Noise.
- **The Handshake:** The Host (STM32) generates a `uint32_t seed`. It sends `SEED:12345` to the phone.
- **Identical Worlds:** Both the STM32 and the Phone use this seed to generate the exact same walls, power-ups, and spawn points.

### B. Viewport & Fairness
- **Virtual Map:** 1200x1200px (or larger).
- **Physical Viewport:** 240x320px.
- **Scrolling:** As the car moves (Joystick), the map coordinates shift. The camera "follows" the player.
- **Opponent Rendering:** The Phone player is only rendered on the STM32 LCD if their virtual coordinates fall within the Car's 240x320 viewing window.

### C. Communication Protocol (UART <-> WiFi <-> Web)
Tiny, high-frequency packets sent via UDP or WebSockets (via an ESP8266 transparent bridge):
- `P:X,Y,A` -> Player Position (X, Y, Angle).
- `F:1` -> Fire command (Physical laser + Virtual projectile).
- `H:ID,DMG` -> Hit detection notification.

### D. Hardware Feedback
- **Physical Laser:** Fires every time a virtual shot is taken.
- **Haptic Feedback:** Physical motors vibrate briefly if the virtual tank hits a virtual wall.
- **7-Segment:** Shows 'H' (Health) or current score.

## 4. Web Client (The Opponent)
- Hosted locally on the ESP8266 or a simple GitHub Pages site.
- Uses HTML5 Canvas for rendering.
- Receives the "Seed" and renders the identical Procedural Map.
- Sends touch-joystick coordinates back to the STM32.

---
**Next Step:** Once the logic is approved, we will begin implementing the `Home Screen` and `WiFi Scanner` as the foundation.
