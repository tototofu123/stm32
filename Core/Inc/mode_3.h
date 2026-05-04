/*
 * =============================================================================
 * MODE_3.H - ARENA MULTIPLAYER MODE STATE AND API
 * =============================================================================
 *
 * This header declares the state machine, enums, arena configuration, and
 * function prototypes for Mode 3 (the multiplayer tank arena with bots,
 * obstacles, power-ups, and environmental hazards).
 *
 * Responsibility (mode_3.c implementation):
 * - Manage Mode 3 setup flow: arena size selection, obstacle/power-up toggle,
 *   bot difficulty selection for each opponent slot.
 * - Implement world generation: tile grid, wall/river placement, power-up spawn.
 * - Maintain tank state: position, angle, HP, active power-ups (speed, wings, shield).
 * - Maintain projectile physics: velocity, collision, bounce, damage detection.
 * - Implement AI bot behavior: pathfinding, aiming, decision-making per difficulty.
 * - Handle collision detection: tank-tank (bounce), tank-projectile (damage),
 *   tank-world (obstacles), projectile-world (bounce/destroy).
 * - Draw arena viewport and HUD with tank health, power-up timers, scores.
 * - Track battle state: battles, rounds, game over conditions.
 *
 * Key enums:
 * - mode3_size_t: Arena size (DEFAULT=1x, NORMAL=4x, LARGE=9x).
 * - mode3_state_t: Game phase (SETUP_SIZE, SETUP_OBSTACLES, SETUP_BOTS, BATTLE, etc.).
 * - bot_diff_t: AI difficulty (OFF, EASY, MID, HARD).
 *
 * Tile types:
 * - TILE_EMPTY, TILE_WALL, TILE_RIVER: terrain features (movement blocking/slowing).
 * - TILE_SUPPLEMENT, TILE_SPEED, TILE_WINGS, TILE_SHIELD: power-ups (+1 HP, speed boost, etc.).
 * - TILE_TRAP: hazard (-1 HP on contact).
 *
 * Global variables exported:
 * - m3_state: Current Mode 3 game phase.
 * - m3_arena_size: Chosen arena dimensions.
 * - m3_use_*: Toggles for obstacles, power-ups, and hazards.
 * - m3_hp: Player tank health (0-10+).
 * - m3_speed_timer, m3_wings_timer, m3_shield_timer: Active power-up durations.
 * - m3_bot_diffs[3]: Difficulty level for each bot opponent.
 *
 * Data flow:
 * - main.c calls Mode3_Init() when user selects Mode 3 (from APP_GAME).
 * - mode_3.c draws setup screens (size select, obstacle toggle, bot select) via ui.c.
 * - mode_3.c processes raw joystick/touch input from main.c each frame via Mode3_Run().
 * - mode_3.c manages world simulation, AI, and renders arena viewport and HUD.
 * - mode_2.c has similar structure but simpler (no AI, no combat).
 *
 * No C++ classes; pure C state and functions.
 * =============================================================================
 */

#ifndef MODE_3_H
#define MODE_3_H

#include "main.h"
#include <stdint.h>

/*
 * =============================================================================
 * MODE 3 ARENA SIZE ENUMERATION
 * =============================================================================
 * Defines playfield dimensions for the mode 3 multiplayer arena.
 */
/*
 * Arena size options (determines viewport, tank spacing, AI complexity).
 */
typedef enum {
    M3_SIZE_DEFAULT = 0,  // 1x arena: 240x320 pixels (compact, easiest).
    M3_SIZE_NORMAL,       // 4x arena: 480x640 pixels (standard, moderate).
    M3_SIZE_LARGE         // 9x arena: 720x960 pixels (huge, hardest).
} mode3_size_t;

/*
 * =============================================================================
 * MODE 3 STATE MACHINE ENUMERATION
 * =============================================================================
 * Describes the phases of Mode 3 gameplay (setup and battle).
 */
/*
 * Setup and gameplay phases.
 */
typedef enum {
    M3_STATE_SETUP_SIZE = 0,      // Choose arena size (1x/4x/9x).
    M3_STATE_SETUP_OBSTACLES,     // Toggle obstacle/power-up types.
    M3_STATE_SETUP_BOTS,          // Set difficulty for 3 bot opponents.
    M3_STATE_INIT_ARENA,          // Generate world and spawn tanks.
    M3_STATE_BATTLE,              // Active gameplay: movement, firing, AI updates.
    M3_STATE_GAME_OVER            // End state: display results.
} mode3_state_t;

/*
 * =============================================================================
 * BOT DIFFICULTY ENUMERATION
 * =============================================================================
 * Controls AI tank behavior and decision-making aggressiveness.
 */
typedef enum {
    BOT_OFF = 0,   // Slot disabled (no AI opponent).
    BOT_EASY,      // Slow reaction, poor aim, random moves.
    BOT_MID,       // Normal reaction time, decent aim.
    BOT_HARD       // Fast reaction, accurate aiming, strategic movement.
} bot_diff_t;

/*
 * =============================================================================
 * ARENA TILE TYPE CONSTANTS
 * =============================================================================
 * Defines different terrain and object types in the arena grid.
 */
#define TILE_EMPTY      0   // Empty traversable space.
#define TILE_WALL       1   // Solid wall: blocks movement, reflects projectiles.
#define TILE_RIVER      2   // Water/slow terrain: reduces movement speed.
#define TILE_SUPPLEMENT 3   // Pickup: restores 1 HP when collected.
#define TILE_SPEED      4   // Pickup: temporary speed boost (2x movement).
#define TILE_TRAP       5   // Hazard: deals 1 HP damage on contact.
#define TILE_WINGS      6   // Pickup: ignore terrain slowdown for duration.
#define TILE_SHIELD     7   // Pickup: absorb/reflect damage for duration.

/*
 * =============================================================================
 * MODE 3 GLOBAL STATE VARIABLES
 * =============================================================================
 */
/* Game state and configuration */
extern mode3_state_t m3_state;              // Current Mode 3 game phase.
extern mode3_state_t m3_state;               // Current Mode 3 game phase.
extern mode3_size_t m3_arena_size;           // Selected arena size.

/* Arena feature toggles (chosen during setup) */
extern uint8_t      m3_use_walls;            // Include wall obstacles.
extern uint8_t      m3_use_rivers;           // Include river terrain.
extern uint8_t      m3_use_supplements;      // Include HP pickups.
extern uint8_t      m3_use_speedboosts;      // Include speed boost pickups.
extern uint8_t      m3_use_traps;            // Include damage traps.
extern uint8_t      m3_use_wings;            // Include debuff immunity pickups.
extern uint8_t      m3_use_shields;          // Include damage reflection pickups.

/* Player tank state during battle */
extern int8_t       m3_hp;                   // Player tank health (0-10+, displayed as "A" for 10+).
extern uint32_t     m3_speed_timer;          // Remaining duration of speed boost power-up.
extern uint32_t     m3_wings_timer;          // Remaining duration of terrain-debuff immunity.
extern uint32_t     m3_shield_timer;         // Remaining duration of damage reflection shield.
extern uint32_t     m3_trap_cooldowns[100][80];  // Per-tile trap cooldown (prevent repeated damage).

/* Arena generation and AI */
extern uint32_t     m3_world_seed;           // Random seed for reproducible world generation.
extern bot_diff_t   m3_bot_diffs[3];         // Difficulty level for each of 3 opponent slots.

/*
 * =============================================================================
 * MODE 3 FUNCTION PROTOTYPES
 * =============================================================================
 */

/*
 * Mode3_Init:
 * Initializes Mode 3: shows arena size selection screen and resets all state.
 * Called once when user selects Mode 3 and transitions to APP_GAME.
 */
void Mode3_Init(void);

/*
 * Mode3_Run:
 * Main Mode 3 game loop: processes input, updates world simulation, and draws arena.
 * Called every frame from main.c's APP_GAME loop when selected_mode == GAME_MODE_3.
 * Input: joystick (joy_x, joy_y, joy_up/down/left/right), buttons (k1, k2, fire),
 *        touch (ts_pressed, ts_click, ts_x, ts_y).
 * Handles: player tank movement, firing, AI tank behavior, collision detection,
 *          projectile physics, pickup collection, HUD updates.
 */
void Mode3_Run(uint32_t joy_x, uint32_t joy_y, uint8_t k1_click, uint8_t k2_click, 
               uint8_t fire_pressed, uint8_t ts_pressed, uint8_t ts_click, uint16_t ts_x, uint16_t ts_y,
               uint8_t joy_up, uint8_t joy_down, uint8_t joy_left, uint8_t joy_right);

/* UI drawing helpers for setup screens (called from mode_3.c) */
/*
 * LCD_DrawMode3Setup: Draws arena configuration selection screens.
 */
void LCD_DrawMode3Setup(void);

/*
 * LCD_UpdateMode3Setup: Incremental update for arena config focus.
 */
void LCD_UpdateMode3Setup(void);

/*
 * LCD_DrawMode3SetupBots: Draws bot difficulty selection screen.
 */
void LCD_DrawMode3SetupBots(void);

/*
 * LCD_UpdateMode3SetupBots: Incremental update for bot difficulty focus.
 */
void LCD_UpdateMode3SetupBots(void);

#endif // MODE_3_H
