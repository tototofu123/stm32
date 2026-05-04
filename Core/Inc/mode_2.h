/*
 * =============================================================================
 * MODE_2.H - DRAWING CHALLENGE MODE STATE AND API
 * =============================================================================
 *
 * This header declares the state machine, enums, and function prototypes for
 * Mode 2 (the drawing/path-tracing challenge gameplay mode).
 *
 * Responsibility (mode_2.c implementation):
 * - Manage Mode 2 game loop: input selection, canvas drawing, movement tracking.
 * - Implement input-method selection screen (joystick vs touchscreen).
 * - Maintain drawing canvas state: path points, cursor position, movement history.
 * - Process player input during drawing phase and convert to movement commands.
 * - Track performance metrics: total distance, covered distance, time remaining.
 * - Draw on-screen HUD: canvas, cursor, movement arrows, stats, command history.
 * - Handle reset/restart confirmation flow.
 *
 * Key enums:
 * - mode2_state_t: Game state (INPUT_SELECT, DRAWING, RESET_CONFIRM, etc.).
 * - mode2_input_method_t: Input control type (JOYSTICK or TOUCH).
 *
 * Global variables exported:
 * - m2_state: Current Mode 2 game state.
 * - m2_input_method: Active input control method.
 * - selected_input: Temporary input choice during selection screen.
 * - m2_cmd_history: Ring buffer of last 10 movement commands.
 *
 * Data flow:
 * - main.c calls Mode2_Init() when entering Mode 2 gameplay (from APP_GAME).
 * - mode_2.c draws its own screens (Mode2InputSelect, Mode2Canvas, etc.) via ui.c.
 * - mode_2.c processes raw joystick/touch input from main.c each frame via Mode2_Run().
 * - mode_2.c updates LCD canvas, cursor, and stats on internal timers.
 * - mode_3.c has similar structure but with arena/AI instead of path drawing.
 *
 * No C++ classes; pure C state and functions.
 * =============================================================================
 */

// mode_2.h
#ifndef MODE_2_H
#define MODE_2_H

#include "main.h"
#include <stdint.h>

/*
 * =============================================================================
 * MODE 2 STATE MACHINE ENUMERATION
 * =============================================================================
 * Describes the phases of Mode 2 gameplay.
 */
typedef enum {
    M2_STATE_INPUT_SELECT = 0,  // Setup: choose joystick or touchscreen control.
    M2_STATE_DRAWING,           // Active: player tracing path on canvas.
    M2_STATE_RESET_CONFIRM,     // Popup: confirm clearing the canvas.
    M2_STATE_MOVING,            // (Reserved) potential future state.
    M2_STATE_SHOOTING,          // (Reserved) potential future state.
    M2_STATE_FINISHED           // End: mode complete, waiting for exit.
} mode2_state_t;
/*
 * M2_STATE_INPUT_SELECT: Choose control method before drawing starts.
 * M2_STATE_DRAWING: Canvas active, user moving cursor and building path.
 * M2_STATE_RESET_CONFIRM: Popup asking for canvas clear confirmation.
 * M2_STATE_FINISHED: Challenge complete or abandoned.
 */

/*
 * =============================================================================
 * INPUT METHOD ENUMERATION
 * =============================================================================
 * Determines which hardware interface controls the cursor in Mode 2.
 */
typedef enum {
    M2_INPUT_JOYSTICK = 0,  // Analog joystick moves cursor smoothly.
    M2_INPUT_TOUCH           // Touchscreen direct coordinate input.
} mode2_input_method_t;
/*
 * Joystick provides smooth, continuous movement and is easier to trace curves.
 * Touch provides direct pixel targeting but requires lifting finger to reposition.
 */

/*
 * =============================================================================
 * MODE 2 GLOBAL STATE VARIABLES
 * =============================================================================
 */
/* Game state machine */
extern mode2_state_t m2_state;              // Current Mode 2 phase.
extern mode2_input_method_t m2_input_method; // Active control method (joystick/touch).
extern mode2_input_method_t selected_input;  // Temporary input selection during setup.

/* Performance tracking */
extern char m2_cmd_history[11];  // Ring buffer: last 10 movement commands + null terminator.

/*
 * =============================================================================
 * MODE 2 FUNCTION PROTOTYPES
 * =============================================================================
 */

/*
 * Mode2_Init:
 * Initializes Mode 2: shows input-selection screen and resets all canvas state.
 * Called once when user selects Mode 2 and transitions to APP_GAME.
 */
void Mode2_Init(void);

/*
 * Mode2_ResetCanvas:
 * Clears the drawing canvas and resets cursor position and path buffers.
 * Switches from reset-confirm popup back to active drawing state.
 * Called when user confirms canvas reset or starts a new run.
 */
void Mode2_ResetCanvas(void);

/*
 * Mode2_Run:
 * Main Mode 2 game loop: processes input, updates game state, and draws HUD.
 * Called every frame from main.c's APP_GAME loop when selected_mode == GAME_MODE_2.
 * Input: joystick (joy_x, joy_y, joy_up/down/left/right), buttons (k1, k2, fire),
 *        touch (ts_pressed, ts_click, ts_x, ts_y).
 */
void Mode2_Run(uint32_t joy_x, uint32_t joy_y, uint8_t k1_click, uint8_t k2_click, 
               uint8_t fire_pressed, uint8_t ts_pressed, uint8_t ts_click, uint16_t ts_x, uint16_t ts_y,
               uint8_t joy_up, uint8_t joy_down, uint8_t joy_left, uint8_t joy_right);

#endif // MODE_2_H