# MODE 3 DEVELOPMENT PLAN: THE INFINITE ARENA & ADVANCED SETTINGS

This document outlines the step-by-step logic, mathematical foundation, and UI architecture for the next phase of the project.

---

## PHASE 1: THE VIRTUAL WORLD (Procedural Generation)
**Goal:** Create a massive battlefield that is identical for both the STM32 and the Phone without sending map data over WiFi.

### 1. The Mathematical Anchor (The Seed)
- **Problem:** How to ensure a wall at (500, 500) exists on both screens?
- **Solution:** Seeded Pseudo-Randomness. 
- **Implementation:**
    - The Car generates a 32-bit `m3_world_seed`.
    - It sends `SEED:[value]` to the phone client.
    - Both devices use a simple Linear Congruential Generator (LCG): `X_next = (a * X_current + c) % m`.
- **Question for myself:** Is LCG enough, or should I use a static 1D array of "Tile Types"?
    - *Decision:* A tile-based grid (e.g., 40x40 tiles of 30x30 pixels) is more efficient for collision detection.

### 2. Viewport & Camera Logic
- **Virtual Map Size:** 1200 x 1200 pixels.
- **Physical Viewport:** 240 x 320 pixels (STM32 LCD).
- **Logic:**
    - `player_world_x`, `player_world_y`: Your true position in the 1200x1200px world.
    - `cam_x = player_world_x - 120`, `cam_y = player_world_y - 160`.
    - **Rendering:** Only draw obstacles and opponents whose coordinates are within the range `[cam_x, cam_x + 240]` and `[cam_y, cam_y + 320]`.

---

## PHASE 2: MULTIPLAYER SYNCHRONIZATION
**Goal:** Real-time position and firing sync between Car and Phone.

### 1. Packet Protocol (UART Bridge)
To keep the WiFi connection fast, we use tiny 12-character strings:
- **CAR -> PHONE:** `P:X,Y,A,F` (X, Y, Angle, FiringStatus).
- **PHONE -> CAR:** `O:X,Y,A,F` (Opponent X, Y, Angle, FiringStatus).

### 2. Hit Detection (The Virtual Laser)
- **Raycasting:** When `FiringStatus == 1`, the code draws a line from the tank center in the direction of the `Angle`.
- **Collision:** Check if this line intersects the opponent's 20x20px tank bounding box.
- **Fairness:** The Car (Host) makes the final decision on "HIT" or "MISS" to prevent cheating/lag issues.

---

## PHASE 3: ADVANCED SETTINGS & KEYBOARD
**Goal:** A phone-like experience for network management.

### 1. WiFi Scanner UI
- **List View:** Consistent with Car Selection.
- **Logic:** 
    - Press K2 -> Send `AT+CWLAP`.
    - STM32 parses the raw UART stream while displaying a "Scanning..." spinner.
    - SSIDs are stored in a 2D array and displayed as touchable list items.

### 2. Virtual Keyboard UX
- **Layout:** 6 columns x 7 rows.
- **Navigation:** 
    - Row 1-4: A-Z.
    - Row 5: 0-9.
    - Row 6: Symbols & Space.
    - Row 7: `[SHIFT]` `[BACKSPACE]` `[ENTER]`.
- **Hit Detection:** Maps `px, py` from `touch.c` to the specific key index.

---

## PHASE 4: UI CONSISTENCY MANDATE
Every screen must follow these strict visual rules:
1.  **Status Bar (Y: 0-20):** Always present. Shows WiFi Mode (AP+STA) and assigned IP.
2.  **Breadcrumbs:** The top-left corner shows the current "Path" (e.g., `HOME > SETTINGS`).
3.  **Color Codes:**
    - **Interactive Items:** Blue Border + White Fill.
    - **Confirmations:** Green.
    - **Dangers/Errors:** Red.
    - **Disabled/Idle:** Grey.

---

## CRITICAL QUESTIONS & EDGE CASES
1.  **Lag Handling:** What happens if the phone misses a packet? 
    - *Plan:* Use "Dead Reckoning." If the opponent's position is missing, keep moving them in their last known direction for 100ms.
2.  **Memory Constraints:** Can the STM32 store a 1200x1200px map?
    - *Plan:* No. We only store the *Seed*. The map is "streamed" into the LCD buffer tile-by-tile during the rendering loop.
3.  **Silent Mode:** Is the buzzer truly off?
    - *Plan:* The `Buzzer_SetMute(1)` must be the first line of `Mode3_Init`.

---

## NEXT STEPS FOR IMPLEMENTATION
1.  **Step 1:** Create `Map_Generator_Task(seed)` to draw the arena.
2.  **Step 2:** Implement `Viewport_Render(cam_x, cam_y)`.
3.  **Step 3:** Setup the ESP8266 UDP Broadcast for the Phone Client.
4.  **Step 4:** Build the Web-based (HTML/JS) Tank Controller for the opponent.
