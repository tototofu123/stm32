#include "mode_3.h"
#include "lcd.h"
#include "ui.h"
#include "alerts.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Local definitions for colors and types
#define MY_ORANGE     0xFD20
#define DARK_GRAY     0x4208
#define LIGHT_BLUE    0x07FF

// Arena State
mode3_state_t m3_state = M3_STATE_SETUP;
mode3_size_t  m3_arena_size = M3_SIZE_DEFAULT;
uint8_t       m3_use_walls = 1;
uint8_t       m3_use_rivers = 1;
uint32_t      m3_world_seed = 0;

// World Dimensions
static uint16_t world_w = 240;
static uint16_t world_h = 320;
static float    tank_x = 120.0f, tank_y = 160.0f;
static float    tank_angle = 0.0f;

// Surgical Redraw Tracking
static float    last_tank_x = 0, last_tank_y = 0;
static float    last_tank_angle = 0;
static int16_t  last_cam_x = -1, last_cam_y = -1;

// Camera Viewport
#define VP_Y_OFFSET 20
#define VP_HEIGHT   260
static int16_t cam_x = 0, cam_y = 0;

// Projectiles: Single high-performance bullet
static float    ball_x, ball_y, ball_vx, ball_vy;
static float    last_ball_x, last_ball_y;
static uint8_t  ball_active = 0;
static uint8_t  ball_bounces = 0;
static uint32_t ball_spawn_tick = 0;

#define TILE_SIZE 10 
static uint8_t arena_grid[80][80]; 

void Mode3_Init(void) {
    m3_state = M3_STATE_SETUP;
    Buzzer_SetMute(1); 
    LCD_DrawMode3Setup();
}

static void Mode3_GenerateWorld(void) {
    if (m3_arena_size == M3_SIZE_DEFAULT) { world_w = 240; world_h = 260; }
    else if (m3_arena_size == M3_SIZE_NORMAL) { world_w = 480; world_h = 520; }
    else { world_w = 720; world_h = 780; }

    uint8_t cols = world_w / TILE_SIZE;
    uint8_t rows = world_h / TILE_SIZE;

    // Reset grid
    for (int r = 0; r < 80; r++) {
        for (int c = 0; c < 80; c++) arena_grid[r][c] = TILE_EMPTY;
    }

    srand(m3_world_seed);
    // Increase density for 1x map to ensure obstacles show up
    int obstacle_count = (m3_arena_size == M3_SIZE_DEFAULT) ? 12 : 25;
    
    for (int i = 0; i < obstacle_count; i++) {
        uint8_t type = (rand() % 100 < 60) ? TILE_WALL : TILE_RIVER;
        if (type == TILE_WALL && !m3_use_walls) continue;
        if (type == TILE_RIVER && !m3_use_rivers) continue;

        int w = 3 + (rand() % 12);
        int h = 3 + (rand() % 12);
        int start_r = rand() % (rows - h);
        int start_c = rand() % (cols - w);

        // Avoid starting area (Smaller dead zone to ensure visibility)
        if (abs((start_c + w/2) * TILE_SIZE - world_w/2) < 40 && 
            abs((start_r + h/2) * TILE_SIZE - world_h/2) < 40) continue;

        for (int r = start_r; r < start_r + h; r++) {
            for (int c = start_c; c < start_c + w; c++) {
                if (r < 80 && c < 80) arena_grid[r][c] = type;
            }
        }
    }
    tank_x = world_w / 2.0f; tank_y = world_h / 2.0f;
    ball_active = 0; last_cam_x = -1; 
}

static void DrawGridCell(int r, int c) {
    int16_t sx = (c * TILE_SIZE) - cam_x;
    int16_t sy = (r * TILE_SIZE) - cam_y + VP_Y_OFFSET;
    if (sx > -TILE_SIZE && sx < 240 && sy > (VP_Y_OFFSET-TILE_SIZE) && sy < (VP_Y_OFFSET+VP_HEIGHT)) {
        uint16_t color = UI_BG;
        if (arena_grid[r][c] == TILE_WALL) color = DARK_GRAY;
        else if (arena_grid[r][c] == TILE_RIVER) color = LIGHT_BLUE;
        
        LCD_Clear(sx, sy, TILE_SIZE, TILE_SIZE, color);

        if (arena_grid[r][c] == TILE_WALL) {
            // Internal lines are removed. Only draw external borders.
            if (c == 0 || arena_grid[r][c-1] != TILE_WALL) {
                LCD_DrawLine(sx, sy, sx, sy+TILE_SIZE, BLACK);
            }
            if (c == (world_w/TILE_SIZE)-1 || arena_grid[r][c+1] != TILE_WALL) {
                LCD_DrawLine(sx+TILE_SIZE, sy, sx+TILE_SIZE, sy+TILE_SIZE, BLACK);
            }
            if (r == 0 || arena_grid[r-1][c] != TILE_WALL) {
                LCD_DrawLine(sx, sy, sx+TILE_SIZE, sy, BLACK);
            }
            if (r == (world_h/TILE_SIZE)-1 || arena_grid[r+1][c] != TILE_WALL) {
                LCD_DrawLine(sx, sy+TILE_SIZE, sx+TILE_SIZE, sy+TILE_SIZE, BLACK);
            }
        }
    }
}

static void Mode3_RenderArena(void) {
    cam_x = (int16_t)tank_x - 120;
    cam_y = (int16_t)tank_y - (VP_HEIGHT / 2);
    if (cam_x < 0) cam_x = 0; if (cam_x > world_w - 240) cam_x = (world_w > 240) ? world_w - 240 : 0;
    if (cam_y < 0) cam_y = 0; if (cam_y > world_h - VP_HEIGHT) cam_y = (world_h > VP_HEIGHT) ? world_h - VP_HEIGHT : 0;

    // Full Redraw if Camera moves
    if (cam_x != last_cam_x || cam_y != last_cam_y) {
        int rows = world_h / TILE_SIZE; int cols = world_w / TILE_SIZE;
        for (int r = 0; r < rows; r++) for (int c = 0; c < cols; c++) DrawGridCell(r, c);
        last_cam_x = cam_x; last_cam_y = cam_y;
    } else {
        // Surgical Clear
        float lrad = last_tank_angle * 3.14159f / 180.0f;
        int16_t ltx = (int16_t)last_tank_x - cam_x; int16_t lty = (int16_t)last_tank_y - cam_y + VP_Y_OFFSET;
        LCD_DrawLine(ltx, lty, ltx + (int16_t)(18*cos(lrad)), lty + (int16_t)(18*sin(lrad)), UI_BG);
        LCD_Clear(ltx-8, lty-8, 17, 17, UI_BG);
        
        // Robust Restore
        int tr = (int)last_tank_y / TILE_SIZE; int tc = (int)last_tank_x / TILE_SIZE;
        for(int r=tr-2; r<=tr+2; r++) for(int c=tc-2; c<=tc+2; c++) {
            if(r>=0 && r<80 && c>=0 && c<80 && arena_grid[r][c] != TILE_EMPTY) DrawGridCell(r, c);
        }
    }

    // Bullet Render
    int16_t lbx = (int16_t)last_ball_x - cam_x; int16_t lby = (int16_t)last_ball_y - cam_y + VP_Y_OFFSET;
    if(lbx>0 && lbx<240 && lby>20 && lby<280) LCD_Clear(lbx-2, lby-2, 5, 5, UI_BG);
    
    if(ball_active) {
        int16_t bx = (int16_t)ball_x - cam_x; int16_t by = (int16_t)ball_y - cam_y + VP_Y_OFFSET;
        uint16_t b_color = (ball_bounces == 0) ? RED : (ball_bounces == 1) ? YELLOW : MY_ORANGE;
        if(bx>0 && bx<240 && by>20 && by<280) LCD_Clear(bx-2, by-2, 5, 5, b_color);
        last_ball_x = ball_x; last_ball_y = ball_y;
    }

    // Tank Body
    int16_t tx = (int16_t)tank_x - cam_x; int16_t ty = (int16_t)tank_y - cam_y + VP_Y_OFFSET;
    LCD_Clear(tx-7, ty-7, 15, 15, GREEN); LCD_DrawRectangle(tx-7, ty-7, 15, 15, BLACK);
    float rad = tank_angle * 3.14159f / 180.0f;
    LCD_DrawLine(tx, ty, tx + (int16_t)(18*cos(rad)), ty + (int16_t)(18*sin(rad)), BLACK);
    last_tank_x = tank_x; last_tank_y = tank_y; last_tank_angle = tank_angle;
}

void LCD_DrawMode3Setup(void) {
    LCD_Clear(0, 0, 240, 320, UI_BG);
    LCD_DrawStatusBar();
    LCD_SetColors(BLACK, UI_BG); LCD_TEXT(60, 30, "ARENA SETUP");
    LCD_TEXT(20, 70, "MAP SIZE:");
    LCD_UpdateMode3Setup();
    LCD_Clear(0, 280, 240, 40, UI_BOTTOM); LCD_SetColors(WHITE, UI_BOTTOM); LCD_TEXT(30, 292, "K2: START BATTLE");
    LCD_SetColors(BLUE, WHITE);
}

void LCD_UpdateMode3Setup(void) {
    uint16_t s1_c = (m3_arena_size == M3_SIZE_DEFAULT) ? UI_BOX_SEL : UI_BOX_NSEL;
    uint16_t s2_c = (m3_arena_size == M3_SIZE_NORMAL) ? UI_BOX_SEL : UI_BOX_NSEL;
    uint16_t s3_c = (m3_arena_size == M3_SIZE_LARGE) ? UI_BOX_SEL : UI_BOX_NSEL;
    LCD_Clear(20, 95, 60, 30, s1_c); LCD_SetColors(BLACK, s1_c); LCD_TEXT(32, 102, "1x");
    LCD_Clear(90, 95, 60, 30, s2_c); LCD_SetColors(BLACK, s2_c); LCD_TEXT(102, 102, "4x");
    LCD_Clear(160, 95, 60, 30, s3_c); LCD_SetColors(BLACK, s3_c); LCD_TEXT(172, 102, "9x");
    
    LCD_SetColors(BLACK, UI_BG); LCD_TEXT(20, 150, "OBSTACLES:");
    
    LCD_Clear(20, 175, 200, 35, m3_use_walls ? MY_GREEN : GREY);
    LCD_SetColors(m3_use_walls ? WHITE : BLACK, m3_use_walls ? MY_GREEN : GREY);
    LCD_TEXT(40, 185, m3_use_walls ? "[X] WALLS (BLOCK)" : "[ ] WALLS (BLOCK)");
    
    LCD_Clear(20, 220, 200, 35, m3_use_rivers ? MY_GREEN : GREY);
    LCD_SetColors(m3_use_rivers ? WHITE : BLACK, m3_use_rivers ? MY_GREEN : GREY);
    LCD_TEXT(40, 230, m3_use_rivers ? "[X] RIVERS (SLOW)" : "[ ] RIVERS (SLOW)");
    LCD_SetColors(BLUE, WHITE);
}

void Mode3_Run(uint32_t joy_x, uint32_t joy_y, uint8_t k1_click, uint8_t k2_click, 
               uint8_t fire_pressed, uint8_t ts_pressed, uint8_t ts_click, uint16_t ts_x, uint16_t ts_y) {
    if (m3_state == M3_STATE_SETUP) {
        if (ts_click) {
            if (ts_y >= 95 && ts_y <= 125) {
                if (ts_x >= 20 && ts_x <= 80) m3_arena_size = M3_SIZE_DEFAULT;
                else if (ts_x >= 90 && ts_x <= 150) m3_arena_size = M3_SIZE_NORMAL;
                else if (ts_x >= 160 && ts_x <= 220) m3_arena_size = M3_SIZE_LARGE;
                LCD_UpdateMode3Setup(); Buzzer_BeepShort();
            }
            if (ts_x >= 20 && ts_x <= 220 && ts_y >= 175 && ts_y <= 210) { m3_use_walls = !m3_use_walls; LCD_UpdateMode3Setup(); Buzzer_BeepShort(); }
            if (ts_x >= 20 && ts_x <= 220 && ts_y >= 220 && ts_y <= 255) { m3_use_rivers = !m3_use_rivers; LCD_UpdateMode3Setup(); Buzzer_BeepShort(); }
        }
        if (k2_click) {
            m3_state = M3_STATE_INIT_ARENA; m3_world_seed = HAL_GetTick();
            Buzzer_BeepShort();
            LCD_Clear(0, 20, 240, 300, UI_BG); LCD_SetColors(BLACK, UI_BG); LCD_TEXT(40, 100, "GENERATING ARENA...");
            Mode3_GenerateWorld(); HAL_Delay(500); m3_state = M3_STATE_BATTLE;
            LCD_Clear(0, 0, 240, 320, UI_BG); LCD_DrawStatusBar();
        }
    }
    else if (m3_state == M3_STATE_BATTLE) {
        float speed = 2.5f; 
        int gr = (int)tank_y / TILE_SIZE; int gc = (int)tank_x / TILE_SIZE;
        if (gr>=0 && gr<80 && gc>=0 && gc<80 && arena_grid[gr][gc] == TILE_RIVER) speed = 1.0f;
        
        if (joy_y < Y_FWD_THRESH_ADC) {
            float nx = tank_x + speed * cos(tank_angle * 3.14159f / 180.0f);
            float ny = tank_y + speed * sin(tank_angle * 3.14159f / 180.0f);
            int ngr = (int)ny / TILE_SIZE, ngc = (int)nx / TILE_SIZE;
            if (ngr>=0 && ngr<80 && ngc>=0 && ngc<80 && arena_grid[ngr][ngc] != TILE_WALL && nx>10 && nx<world_w-10 && ny>10 && ny<world_h-10) { tank_x = nx; tank_y = ny; }
        }
        if (joy_x < X_LEFT_THRESH_ADC) tank_angle -= 6.0f;
        if (joy_x > X_RIGHT_THRESH_ADC) tank_angle += 6.0f;

        if (fire_pressed && !ball_active) {
            ball_x = tank_x; ball_y = tank_y;
            ball_vx = 12.0f * cos(tank_angle * 3.14159f / 180.0f);
            ball_vy = 12.0f * sin(tank_angle * 3.14159f / 180.0f);
            ball_active = 1; ball_bounces = 0; ball_spawn_tick = HAL_GetTick();
            Buzzer_BeepShort();
        }

        if (ball_active) {
            if (HAL_GetTick() - ball_spawn_tick > 5000) { ball_active = 0; }
            else {
                float nx = ball_x + ball_vx; float ny = ball_y + ball_vy;
                int ngr = (int)ny / TILE_SIZE, ngc = (int)nx / TILE_SIZE;
                
                if (ngr<0 || ngr>=80 || ngc<0 || ngc>=80 || nx<0 || nx>world_w || ny<0 || ny>world_h || arena_grid[ngr][ngc] == TILE_WALL) {
                    if (ball_bounces >= 2) { ball_active = 0; }
                    else {
                        int cur_r = (int)ball_y / TILE_SIZE, cur_c = (int)ball_x / TILE_SIZE;
                        if (ngr != cur_r) ball_vy = -ball_vy;
                        if (ngc != cur_c) ball_vx = -ball_vx;
                        ball_bounces++;
                    }
                } else { ball_x = nx; ball_y = ny; }
            }
        }
        Mode3_RenderArena();
        HAL_Delay(10);
    }
}
