# UI/UX & Hardware Utilization Roadmap

## 1. Project Status Summary (May 4, 2026)

| Category | Item | Status | Notes |
| :--- | :--- | :--- | :--- |
| **Logic** | Mode 3 AI HP | ✅ DONE | Displayed inside tank boxes. |
| **Settings** | UI Layout | ⚠️ PARTIAL | Menu exists, needs borders & arrows. |
| **Settings** | Theme/Contrast | 💤 PAUSED | User requested to ignore for now. |
| **Inputs** | Joystick Nav | ❌ TODO | Inputs work, but don't navigate menus. |
| **Inputs** | Cap Touch | ⚠️ PARTIAL | Used for ability; need "Quick Start" logic. |
| **Display** | Font Scaling | ⚠️ PARTIAL | 2x is too large; need ~1.2x scaling. |
| **Visuals** | Dynamic BG | ❌ TODO | Need 10 FPS tank battle animation. |

## 2. The UI/UX Improvement Blueprint

### A. Aesthetic & Hierarchy
- **Theme**: "5-Year-Old Pixel Art" (Simplified trees, blocky tanks).
- **Hierarchy**:
  1. Top: Status Bar (IP/System).
  2. Center: Dynamic Hero Area (Home) or List (Menus).
  3. Bottom: Contextual HW Key hints (K1/K2 logic).
- **Colors**: Yellow boxes for idle, Green for focused.

### B. Hardware Utilization (Inputs)
- **Joystick**: Y-axis to move focus; Button to Confirm.
- **Capacitive Touch**: "Instant Action" key to bypass menus and start game immediately.
- **Focus Indicator**: A "pointer" (arrow or mini-tank) that turns green when on a selectable item.

### C. Touchscreen Overhaul (Two-Tap)
- **Interaction**:
  - Tap 1: Turn box Green (Focus).
  - Tap 2: Execute action (Confirm).
- **Visuals**: Boxes must have distinct borders to separate options clearly.

### D. Dynamic Home Screen
- **Animation**: 10 FPS loop, 10 seconds total.
- **Content**: Two tanks shooting each other, projectiles, and explosions.
- **Scenery**: Pixel-art trees and obstacles in the background layer.

### E. Font Refinement
- **Requirement**: Only 10-20% larger than default.
- **Method**: Subtler pixel repeating or "Bold" drawing in `LCD_DrawChar`.
