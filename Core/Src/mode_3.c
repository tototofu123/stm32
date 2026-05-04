#include "mode_3.h"
#include "lcd.h"
#include "ui.h"
#include "alerts.h"
#include "peripherals.h"
#include "seven_seg.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Colors
#define MY_ORANGE     0xFD20
#define DARK_GRAY     0x4208
#define LIGHT_BLUE    0x07FF

// Arena State
mode3_state_t m3_state = M3_STATE_SETUP_SIZE;
mode3_size_t  m3_arena_size = M3_SIZE_DEFAULT;
uint8_t       m3_use_walls = 1;
uint8_t       m3_use_rivers = 1;
uint8_t       m3_use_supplements = 1;
uint8_t       m3_use_speedboosts = 1;
uint8_t       m3_use_traps = 1;
uint8_t       m3_use_wings = 1;
uint8_t       m3_use_shields = 1;

uint32_t      m3_trap_cooldowns[100][80];
uint32_t      m3_world_seed = 0;

static uint8_t m3_setup_row = 0; 
static uint8_t m3_setup_obstacle_idx = 0;

bot_diff_t    m3_bot_diffs[3] = {BOT_EASY, BOT_OFF, BOT_OFF};
static uint8_t m3_setup_bot_idx = 0;

// World Dimensions
static uint16_t world_w = 240;
static uint16_t world_h = 260;

// Viewport
#define VP_Y_OFFSET 20
static int16_t vp_height = 260;
static int16_t cam_x = 0, cam_y = 0;
static int16_t last_cam_x = -1, last_cam_y = -1;

#define TILE_SIZE 10 
static uint8_t arena_grid[100][80]; 

typedef struct {
    uint8_t active;
    uint8_t is_human;
    bot_diff_t diff;
    
    int8_t hp;
    float x;
    float y;
    float angle;
    
    uint32_t speed_timer;
    uint32_t wings_timer;
    uint32_t shield_timer;
    
    float last_x;
    float last_y;
    float last_angle;
    
    // Projectile
    uint8_t ball_active;
    float ball_x;
    float ball_y;
    float ball_vx;
    float ball_vy;
    float last_ball_x;
    float last_ball_y;
    uint8_t ball_bounces;
    uint32_t ball_spawn_tick;
    
    // AI State
    uint32_t last_decision_tick;
    float target_angle;
    uint8_t moving_fwd;
} m3_tank_t;

m3_tank_t tanks[4];
int8_t m3_hp = 10;
uint32_t m3_speed_timer = 0;
uint32_t m3_wings_timer = 0;
uint32_t m3_shield_timer = 0;

const uint16_t tank_colors[4] = {GREEN, RED, MAGENTA, MY_ORANGE};

// Forward declarations
void LCD_DrawMode3SetupSize(void);
void LCD_DrawMode3SetupObstacles(void);
void LCD_DrawMode3SetupBots(void);
void LCD_UpdateMode3SetupSize(void);
void LCD_UpdateMode3SetupObstacles(void);
void LCD_UpdateMode3SetupBots(void);
void LCD_DrawMode3HUD(void);

static void SafeClear(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    if (x + w <= 0 || x >= 240 || y + h <= VP_Y_OFFSET || y >= 320) return;
    int16_t x2 = x + w - 1;
    int16_t y2 = y + h - 1;
    if (x < 0) x = 0;
    if (y < VP_Y_OFFSET) y = VP_Y_OFFSET;
    if (x2 > 239) x2 = 239;
    if (y2 > 319) y2 = 319;
    if (x <= x2 && y <= y2) {
        LCD_Clear((uint16_t)x, (uint16_t)y, (uint16_t)(x2 - x + 1), (uint16_t)(y2 - y + 1), color);
    }
}

static void SafeDrawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
    if ((x1 < 0 && x2 < 0) || (x1 >= 240 && x2 >= 240) || 
        (y1 < VP_Y_OFFSET && y2 < VP_Y_OFFSET) || (y1 >= 320 && y2 >= 320)) return;
    
    if (x1 < 0) x1 = 0; else if (x1 > 239) x1 = 239;
    if (y1 < VP_Y_OFFSET) y1 = VP_Y_OFFSET; else if (y1 > 319) y1 = 319;
    if (x2 < 0) x2 = 0; else if (x2 > 239) x2 = 239;
    if (y2 < VP_Y_OFFSET) y2 = VP_Y_OFFSET; else if (y2 > 319) y2 = 319;
    
    LCD_DrawLine((uint16_t)x1, (uint16_t)y1, (uint16_t)x2, (uint16_t)y2, color);
}

void Mode3_Init(void) {
    m3_state = M3_STATE_SETUP_SIZE;
    m3_setup_row = 0;
    m3_setup_obstacle_idx = 0;
    m3_setup_bot_idx = 0;
    m3_hp = 10;
    m3_speed_timer = 0;
    m3_wings_timer = 0;
    m3_shield_timer = 0;
    LCD_DrawMode3SetupSize();
}

static void Mode3_GenerateWorld(void) {
    if (m3_arena_size == M3_SIZE_DEFAULT) { world_w = 240; world_h = 260; vp_height = 260; }
    else if (m3_arena_size == M3_SIZE_NORMAL) { world_w = 480; world_h = 600; vp_height = 300; }
    else { world_w = 720; world_h = 900; vp_height = 300; }

    uint8_t cols = world_w / TILE_SIZE;
    uint8_t rows = world_h / TILE_SIZE;

    for (int r = 0; r < 100; r++) {
        for (int c = 0; c < 80; c++) {
            arena_grid[r][c] = TILE_EMPTY;
            m3_trap_cooldowns[r][c] = 0;
        }
    }

    srand(m3_world_seed);

    if (m3_use_walls) {
        int num_tetris = 4 + (rand() % 3);
        if (m3_arena_size == M3_SIZE_NORMAL) num_tetris *= 4;
        if (m3_arena_size == M3_SIZE_LARGE) num_tetris *= 9;
        
        for (int i = 0; i < num_tetris; i++) {
            int r = rand() % (rows - 4);
            int c = rand() % (cols - 4);
            if (abs(r - rows/2) < 4 && abs(c - cols/2) < 4) continue;
            
            int shape = rand() % 5;
            arena_grid[r][c] = TILE_WALL;
            if (shape == 0) {
                arena_grid[r+1][c] = TILE_WALL; arena_grid[r+2][c] = TILE_WALL; arena_grid[r+3][c] = TILE_WALL;
            } else if (shape == 1) {
                arena_grid[r+1][c] = TILE_WALL; arena_grid[r][c+1] = TILE_WALL; arena_grid[r+1][c+1] = TILE_WALL;
            } else if (shape == 2) {
                arena_grid[r][c+1] = TILE_WALL; arena_grid[r][c+2] = TILE_WALL; arena_grid[r+1][c+1] = TILE_WALL;
            } else if (shape == 3) {
                arena_grid[r+1][c] = TILE_WALL; arena_grid[r+2][c] = TILE_WALL; arena_grid[r+2][c+1] = TILE_WALL;
            } else if (shape == 4) {
                arena_grid[r][c+1] = TILE_WALL; arena_grid[r+1][c+1] = TILE_WALL; arena_grid[r+1][c+2] = TILE_WALL;
            }
        }
    }

    int density = (m3_arena_size == M3_SIZE_DEFAULT) ? 40 : (m3_arena_size == M3_SIZE_NORMAL ? 150 : 350);
    for (int i = 0; i < density; i++) {
        int r = rand() % rows, c = rand() % cols;
        if (abs(r - rows/2) < 4 && abs(c - cols/2) < 4) continue;
        if (arena_grid[r][c] != TILE_EMPTY) continue;
        
        uint32_t roll = rand() % 100;
        uint8_t type = TILE_EMPTY;
        if (roll < 35) type = TILE_WALL;
        else if (roll < 65) type = TILE_RIVER;
        else if (roll < 72) type = TILE_TRAP;
        else if (roll < 78) type = TILE_SPEED;
        else if (roll < 83) type = TILE_SUPPLEMENT;
        else if (roll < 90) type = TILE_WINGS;
        else type = TILE_SHIELD;

        if (type == TILE_WALL && !m3_use_walls) continue;
        if (type == TILE_RIVER && !m3_use_rivers) continue;
        if (type == TILE_SUPPLEMENT && !m3_use_supplements) continue;
        if (type == TILE_SPEED && !m3_use_speedboosts) continue;
        if (type == TILE_TRAP && !m3_use_traps) continue;
        if (type == TILE_WINGS && !m3_use_wings) continue;
        if (type == TILE_SHIELD && !m3_use_shields) continue;

        arena_grid[r][c] = type;
        if (type == TILE_RIVER && rand() % 100 < 40) {
             if (r+1 < rows) arena_grid[r+1][c] = type;
             if (c+1 < cols) arena_grid[r][c+1] = type;
        }
    }

    for(int i=0; i<4; i++) {
        tanks[i].active = 0;
    }

    tanks[0].is_human = 1;
    tanks[0].active = 1;
    for(int i=0; i<3; i++) {
        if (m3_bot_diffs[i] != BOT_OFF) {
            tanks[i+1].is_human = 0;
            tanks[i+1].active = 1;
            tanks[i+1].diff = m3_bot_diffs[i];
        }
    }

    for(int i=0; i<4; i++) {
        if(tanks[i].active) {
            tanks[i].hp = 10;
            tanks[i].speed_timer = 0;
            tanks[i].wings_timer = 0;
            tanks[i].shield_timer = 0;
            tanks[i].ball_active = 0;
            tanks[i].angle = rand() % 360;
            tanks[i].last_decision_tick = 0;
            tanks[i].moving_fwd = 0;
            do {
                tanks[i].x = 20 + rand() % (world_w - 40);
                tanks[i].y = 20 + rand() % (world_h - 40);
                uint8_t safe = 1;
                int16_t os[4][2] = {{-7,-7},{7,-7},{-7,7},{7,7}};
                for (int j=0; j<4; j++) {
                    int r = (int)(tanks[i].y + os[j][1]) / TILE_SIZE;
                    int c = (int)(tanks[i].x + os[j][0]) / TILE_SIZE;
                    if (arena_grid[r][c] == TILE_WALL) { safe = 0; break; }
                }
                if (safe) break;
            } while (1);
            tanks[i].last_x = tanks[i].x;
            tanks[i].last_y = tanks[i].y;
            tanks[i].last_angle = tanks[i].angle;
        }
    }

    cam_x = 0; cam_y = 0;
    if (m3_arena_size != M3_SIZE_DEFAULT) { cam_x = (int16_t)tanks[0].x - 120; cam_y = (int16_t)tanks[0].y - 150; }
    last_cam_x = -1; last_cam_y = -1; 
}

static uint16_t GetTileColor(uint8_t type) {
    switch(type) {
        case TILE_WALL:       return DARK_GRAY;
        case TILE_RIVER:      return LIGHT_BLUE;
        case TILE_SUPPLEMENT: return MY_GREEN;
        case TILE_SPEED:      return YELLOW;
        case TILE_TRAP:       return RED;
        case TILE_WINGS:      return WHITE;
        case TILE_SHIELD:     return MAGENTA;
        default:              return UI_BG;
    }
}

static void DrawGridCell(int r, int c) {
    int16_t sx = (c * TILE_SIZE) - cam_x;
    int16_t sy = (r * TILE_SIZE) - cam_y + VP_Y_OFFSET;
    if (sx >= 0 && sx < 240 && sy >= VP_Y_OFFSET && sy < 320) {
        uint8_t type = arena_grid[r][c];
        uint16_t color = GetTileColor(type);
        LCD_Clear(sx, sy, TILE_SIZE, TILE_SIZE, color);
        if (type == TILE_WALL) {
            if (c == 0 || arena_grid[r][c-1] != TILE_WALL) LCD_DrawLine(sx, sy, sx, sy+TILE_SIZE, BLACK);
            if (c >= (world_w/TILE_SIZE)-1 || arena_grid[r][c+1] != TILE_WALL) LCD_DrawLine(sx+TILE_SIZE-1, sy, sx+TILE_SIZE-1, sy+TILE_SIZE, BLACK);
            if (r == 0 || arena_grid[r-1][c] != TILE_WALL) LCD_DrawLine(sx, sy, sx+TILE_SIZE, sy, BLACK);
            if (r >= (world_h/TILE_SIZE)-1 || arena_grid[r+1][c] != TILE_WALL) LCD_DrawLine(sx, sy+TILE_SIZE-1, sx+TILE_SIZE, sy+TILE_SIZE-1, BLACK);
        } else if (type >= TILE_SUPPLEMENT) { LCD_DrawRectangle(sx, sy, TILE_SIZE, TILE_SIZE, BLACK); }
    }
}

static void Mode3_BurstRefresh(void) {
    LCD_OpenWindow(0, VP_Y_OFFSET, 240, vp_height);
    LCD_Write_Cmd(CMD_SetPixel); 
    for (int y = 0; y < vp_height; y++) {
        int world_y = y + cam_y; int r = world_y / TILE_SIZE; int ly = world_y % TILE_SIZE;
        for (int x = 0; x < 240; x++) {
            int world_x = x + cam_x; int c = world_x / TILE_SIZE; int lx = world_x % TILE_SIZE;
            uint16_t color = UI_BG;
            if (r >= 0 && r < 100 && c >= 0 && c < 80) {
                uint8_t type = arena_grid[r][c]; color = GetTileColor(type);
                if (type == TILE_WALL) {
                    if ((lx == 0 && (c == 0 || arena_grid[r][c-1] != TILE_WALL)) ||
                        (lx == 9 && (c >= (world_w/TILE_SIZE)-1 || arena_grid[r][c+1] != TILE_WALL)) ||
                        (ly == 0 && (r == 0 || arena_grid[r-1][c] != TILE_WALL)) ||
                        (ly == 9 && (r >= (world_h/TILE_SIZE)-1 || arena_grid[r+1][c] != TILE_WALL))) { color = BLACK; }
                } else if (type >= TILE_SUPPLEMENT) { if (lx == 0 || lx == 9 || ly == 0 || ly == 9) color = BLACK; }
            }
            LCD_Write_Data(color); 
        }
    }
}

static void Mode3_RenderArena(void) {
    if (m3_arena_size != M3_SIZE_DEFAULT && tanks[0].active) {
        int margin_x = 60, margin_y = 80;
        if (tanks[0].x < cam_x + margin_x) { cam_x -= 40; } else if (tanks[0].x > cam_x + 240 - margin_x) { cam_x += 40; }
        if (tanks[0].y < cam_y + margin_y) { cam_y -= 40; } else if (tanks[0].y > cam_y + vp_height - margin_y) { cam_y += 40; }
        if (cam_x < 0) { cam_x = 0; } if (cam_x > world_w - 240) { cam_x = world_w - 240; }
        if (cam_y < 0) { cam_y = 0; } if (cam_y > world_h - vp_height) { cam_y = world_h - vp_height; }
    }
    
    if (cam_x != last_cam_x || cam_y != last_cam_y) {
        Mode3_BurstRefresh();
        last_cam_x = cam_x;
        last_cam_y = cam_y;
    } else {
        for(int i=0; i<4; i++) {
            if(!tanks[i].active) continue;
            float lrad = tanks[i].last_angle * 3.14159f / 180.0f;
            int16_t ltx = (int16_t)tanks[i].last_x - cam_x;
            int16_t lty = (int16_t)tanks[i].last_y - cam_y + VP_Y_OFFSET;
            SafeDrawLine(ltx, lty, ltx + (int16_t)(18*cos(lrad)), lty + (int16_t)(18*sin(lrad)), UI_BG);
            SafeClear(ltx-8, lty-8, 17, 17, UI_BG);
            int tr = (int)tanks[i].last_y / TILE_SIZE, tc = (int)tanks[i].last_x / TILE_SIZE;
            for(int r=tr-2; r<=tr+2; r++) {
                for(int c=tc-2; c<=tc+2; c++) {
                    if(r>=0 && r<100 && c>=0 && c<80 && arena_grid[r][c] != TILE_EMPTY) DrawGridCell(r, c);
                }
            }
        }
    }

    for(int i=0; i<4; i++) {
        if(!tanks[i].active) continue;
        int16_t lbx = (int16_t)tanks[i].last_ball_x - cam_x; int16_t lby = (int16_t)tanks[i].last_ball_y - cam_y + VP_Y_OFFSET;
        SafeClear(lbx-2, lby-2, 5, 5, UI_BG);
        
        if(tanks[i].ball_active) {
            int16_t bx = (int16_t)tanks[i].ball_x - cam_x; int16_t by = (int16_t)tanks[i].ball_y - cam_y + VP_Y_OFFSET;
            uint16_t b_color = (tanks[i].ball_bounces == 0) ? RED : (tanks[i].ball_bounces == 1) ? YELLOW : MY_ORANGE;
            SafeClear(bx-2, by-2, 5, 5, b_color);
            tanks[i].last_ball_x = tanks[i].ball_x; tanks[i].last_ball_y = tanks[i].ball_y;
        }

        int16_t tx = (int16_t)tanks[i].x - cam_x, ty = (int16_t)tanks[i].y - cam_y + VP_Y_OFFSET;
        SafeClear(tx-7, ty-7, 15, 15, tank_colors[i]); 
        
        SafeDrawLine(tx-7, ty-7, tx+7, ty-7, BLACK);
        SafeDrawLine(tx-7, ty+7, tx+7, ty+7, BLACK);
        SafeDrawLine(tx-7, ty-7, tx-7, ty+7, BLACK);
        SafeDrawLine(tx+7, ty-7, tx+7, ty+7, BLACK);
        
        // Draw HP inside the tank (Centered 8x16 char is too big, but we center it as much as possible)
        if (tanks[i].hp > 0) {
            char hp_char = (tanks[i].hp >= 10) ? 'X' : (tanks[i].hp + '0'); // 'X' for 10
            LCD_SetColors(BLACK, tank_colors[i]);
            LCD_DrawChar(tx - 3, ty - 8, hp_char);
        }
        
        float rad = tanks[i].angle * 3.14159f / 180.0f;
        SafeDrawLine(tx, ty, tx + (int16_t)(18 * cos(rad)), ty + (int16_t)(18 * sin(rad)), BLACK);
        
        tanks[i].last_x = tanks[i].x; tanks[i].last_y = tanks[i].y; tanks[i].last_angle = tanks[i].angle;
    }
}

void LCD_DrawMode3SetupSize(void) {
    LCD_Clear(0, 0, 240, 320, UI_BG); LCD_DrawStatusBar(); LCD_DrawBackButton();
    LCD_SetColors(BLACK, UI_BG); LCD_TEXT(60, 30, "ARENA SETUP (1/3)");
    LCD_UpdateMode3SetupSize();
}

void LCD_UpdateMode3SetupSize(void) {
    seg_mode = SEG_M3_OBSTACLES;
    uint8_t sz = (m3_arena_size == M3_SIZE_DEFAULT) ? 1 : (m3_arena_size == M3_SIZE_NORMAL ? 4 : 9);
    SEG_ShowPair(0, sz, 0);
    LCD_SetColors(BLUE, UI_BG); LCD_TEXT(20, 70, "MAP SIZE:");
    uint16_t s1_c = (m3_arena_size == M3_SIZE_DEFAULT) ? UI_BOX_SEL : UI_BOX_NSEL;
    uint16_t s2_c = (m3_arena_size == M3_SIZE_NORMAL) ? UI_BOX_SEL : UI_BOX_NSEL;
    uint16_t s3_c = (m3_arena_size == M3_SIZE_LARGE) ? UI_BOX_SEL : UI_BOX_NSEL;
    LCD_Clear(20, 95, 60, 30, s1_c); LCD_SetColors(BLACK, s1_c); LCD_TEXT(32, 102, "1x");
    LCD_Clear(90, 95, 60, 30, s2_c); LCD_SetColors(BLACK, s2_c); LCD_TEXT(102, 102, "4x");
    LCD_Clear(160, 95, 60, 30, s3_c); LCD_SetColors(BLACK, s3_c); LCD_TEXT(172, 102, "9x");
    LCD_Clear(20, 275, 200, 35, BLUE); LCD_SetColors(WHITE, BLUE); LCD_TEXT(55, 285, "NEXT: OBSTACLES");
}

void LCD_DrawMode3SetupObstacles(void) {
    LCD_Clear(0, 0, 240, 320, UI_BG); LCD_DrawStatusBar(); LCD_DrawBackButton();
    LCD_SetColors(BLACK, UI_BG); LCD_TEXT(60, 30, "ARENA SETUP (2/3)");
    LCD_UpdateMode3SetupObstacles();
}

void LCD_UpdateMode3SetupObstacles(void) {
    uint8_t bits = 0;
    if (m3_use_walls) { bits |= 0x01; }
    if (m3_use_rivers) { bits |= 0x02; }
    if (m3_use_supplements) { bits |= 0x04; }
    if (m3_use_speedboosts) { bits |= 0x08; }
    if (m3_use_traps) { bits |= 0x10; }
    if (m3_use_wings) { bits |= 0x20; }
    if (m3_use_shields) { bits |= 0x40; }
    SEG_ShowCustom(0, bits);
    const char* names[] = {"WALLS", "RIVERS", "SUPPLEMENT", "SPEED", "TRAP", "WINGS", "SHIELD"};
    uint8_t* vals[] = {&m3_use_walls, &m3_use_rivers, &m3_use_supplements, &m3_use_speedboosts, &m3_use_traps, &m3_use_wings, &m3_use_shields};
    uint16_t colors[] = {DARK_GRAY, LIGHT_BLUE, MY_GREEN, YELLOW, RED, WHITE, MAGENTA};

    LCD_Clear(0, 65, 20, 7*28, UI_BG);
    LCD_Clear(220, 65, 20, 7*28, UI_BG);

    for (int i=0; i<7; i++) {
        int y = 65 + i*28;
        uint16_t box_color = (m3_setup_obstacle_idx == i) ? UI_BOX_SEL : (*vals[i] ? MY_GREEN : GREY);
        uint16_t text_color = (*vals[i] || m3_setup_obstacle_idx == i) ? WHITE : BLACK;
        
        LCD_Clear(20, y, 200, 24, box_color);
        LCD_SetColors(text_color, box_color);
        
        char buf[32]; snprintf(buf, sizeof(buf), "%s %s", *vals[i] ? "[X]" : "[ ]", names[i]);
        LCD_TEXT(55, y + 4, buf);
        
        LCD_Clear(32, y + 4, 16, 16, colors[i]);
        LCD_DrawRectangle(32, y + 4, 16, 16, BLACK);

        if (m3_setup_obstacle_idx == i) {
            LCD_SetColors(BLACK, UI_BG);
            LCD_TEXT(2, y + 4, "->");
            LCD_TEXT(222, y + 4, "<-");
        }
    }
    LCD_Clear(20, 275, 200, 35, BLUE); LCD_SetColors(WHITE, BLUE); LCD_TEXT(55, 285, "NEXT: PLAYERS");
}

void LCD_DrawMode3SetupBots(void) {
    LCD_Clear(0, 0, 240, 320, UI_BG); LCD_DrawStatusBar(); LCD_DrawBackButton();
    LCD_SetColors(BLACK, UI_BG); LCD_TEXT(60, 30, "ARENA SETUP (3/3)");
    LCD_UpdateMode3SetupBots();
}

void LCD_UpdateMode3SetupBots(void) {
    LCD_Clear(0, 65, 20, 4*40, UI_BG);
    LCD_Clear(220, 65, 20, 4*40, UI_BG);

    for (int i=0; i<3; i++) {
        int y = 65 + i*40;
        uint16_t box_color = (m3_setup_bot_idx == i) ? UI_BOX_SEL : MY_GREEN;
        LCD_Clear(20, y, 200, 32, box_color);
        LCD_SetColors(BLACK, box_color);
        
        char buf[32];
        if (m3_bot_diffs[i] == BOT_OFF) {
            snprintf(buf, sizeof(buf), "BOT %d: OFF", i+1);
            LCD_SetColors(GREY, box_color);
        } else {
            const char* diffs[] = {"", "EASY", "MID", "HARD"};
            snprintf(buf, sizeof(buf), "BOT %d: %s", i+1, diffs[m3_bot_diffs[i]]);
        }
        LCD_TEXT(60, y + 8, buf);

        if (m3_setup_bot_idx == i) {
            LCD_SetColors(BLACK, UI_BG);
            LCD_TEXT(2, y + 8, "->");
            LCD_TEXT(222, y + 8, "<-");
        }
    }
    LCD_Clear(20, 275, 200, 35, BLUE); LCD_SetColors(WHITE, BLUE); LCD_TEXT(55, 285, "START BATTLE");
}

void LCD_DrawMode3HUD(void) {
    LCD_SetColors(BLACK, UI_TOP);
    char buf[16]; snprintf(buf, sizeof(buf), "HP:%d", tanks[0].hp);
    LCD_Clear(165, 2, 73, 16, UI_TOP); LCD_TEXT(165, 4, buf);
    LCD_Clear(100, 2, 60, 16, UI_TOP); int ox = 100; uint32_t now = HAL_GetTick();
    if (tanks[0].shield_timer > now) { LCD_Clear(ox, 4, 12, 12, MAGENTA); LCD_DrawRectangle(ox, 4, 12, 12, BLACK); ox += 14; }
    if (tanks[0].wings_timer > now) { LCD_Clear(ox, 4, 12, 12, WHITE); LCD_DrawRectangle(ox, 4, 12, 12, BLACK); ox += 14; }
    if (tanks[0].speed_timer > now) { LCD_Clear(ox, 4, 12, 12, YELLOW); LCD_DrawRectangle(ox, 4, 12, 12, BLACK); ox += 14; }
}

void Mode3_Run(uint32_t joy_x, uint32_t joy_y, uint8_t k1_click, uint8_t k2_click, 
               uint8_t fire_pressed, uint8_t ts_pressed, uint8_t ts_click, uint16_t ts_x, uint16_t ts_y,
               uint8_t joy_up, uint8_t joy_down, uint8_t joy_left, uint8_t joy_right) {
    uint32_t now = HAL_GetTick();

    // Global back button for setup phases
    if (ts_click && ts_x < 60 && ts_y < 50) {
        if (m3_state == M3_STATE_SETUP_SIZE || m3_state == M3_STATE_SETUP_OBSTACLES || m3_state == M3_STATE_SETUP_BOTS) {
            app_state = APP_MODE_SELECT;
            Buzzer_BeepShort();
            return;
        }
    }

    if (m3_state == M3_STATE_SETUP_SIZE) {
        if (joy_left || joy_right) { m3_arena_size = (mode3_size_t)((m3_arena_size + (joy_right ? 1 : 2)) % 3); LCD_UpdateMode3SetupSize(); Buzzer_BeepShort(); }
        if (k2_click || joy_down) { m3_state = M3_STATE_SETUP_OBSTACLES; LCD_DrawMode3SetupObstacles(); Buzzer_BeepShort(); }
        if (ts_click) {
            if (ts_y >= 95 && ts_y <= 125) {
                if (ts_x >= 20 && ts_x <= 80) m3_arena_size = M3_SIZE_DEFAULT;
                else if (ts_x >= 90 && ts_x <= 150) m3_arena_size = M3_SIZE_NORMAL;
                else if (ts_x >= 160 && ts_x <= 220) m3_arena_size = M3_SIZE_LARGE;
                LCD_UpdateMode3SetupSize(); Buzzer_BeepShort();
            } else if (ts_y >= 275) { m3_state = M3_STATE_SETUP_OBSTACLES; LCD_DrawMode3SetupObstacles(); Buzzer_BeepShort(); }
        }
    }
    else if (m3_state == M3_STATE_SETUP_OBSTACLES) {
        if (joy_up) { if(m3_setup_obstacle_idx > 0) m3_setup_obstacle_idx--; else m3_setup_obstacle_idx = 6; LCD_UpdateMode3SetupObstacles(); Buzzer_BeepShort(); }
        if (joy_down) { if(m3_setup_obstacle_idx < 6) m3_setup_obstacle_idx++; else m3_setup_obstacle_idx = 0; LCD_UpdateMode3SetupObstacles(); Buzzer_BeepShort(); }
        if (k1_click) {
            uint8_t* v[] = {&m3_use_walls, &m3_use_rivers, &m3_use_supplements, &m3_use_speedboosts, &m3_use_traps, &m3_use_wings, &m3_use_shields};
            *v[m3_setup_obstacle_idx] = !(*v[m3_setup_obstacle_idx]); LCD_UpdateMode3SetupObstacles(); Buzzer_BeepShort();
        }
        if (ts_click) { 
            if (ts_y >= 65 && ts_y < 261) { 
                uint8_t tapped_idx = (ts_y - 65) / 28; 
                static uint32_t last_tap_time = 0;
                if (m3_setup_obstacle_idx != tapped_idx) {
                    m3_setup_obstacle_idx = tapped_idx; LCD_UpdateMode3SetupObstacles(); Buzzer_BeepShort(); last_tap_time = now;
                } else {
                    if (now - last_tap_time < 800) {
                        uint8_t* v[] = {&m3_use_walls, &m3_use_rivers, &m3_use_supplements, &m3_use_speedboosts, &m3_use_traps, &m3_use_wings, &m3_use_shields};
                        *v[m3_setup_obstacle_idx] = !(*v[m3_setup_obstacle_idx]); 
                        LCD_UpdateMode3SetupObstacles(); Buzzer_BeepShort(); last_tap_time = 0; 
                    } else { last_tap_time = now; }
                }
            } else if (ts_y >= 275) { k2_click = 1; }
        }
        if (k2_click) {
            m3_state = M3_STATE_SETUP_BOTS; Buzzer_BeepShort();
            LCD_DrawMode3SetupBots();
        }
    }
    else if (m3_state == M3_STATE_SETUP_BOTS) {
        if (joy_up) { if(m3_setup_bot_idx > 0) m3_setup_bot_idx--; else m3_setup_bot_idx = 2; LCD_UpdateMode3SetupBots(); Buzzer_BeepShort(); }
        if (joy_down) { if(m3_setup_bot_idx < 2) m3_setup_bot_idx++; else m3_setup_bot_idx = 0; LCD_UpdateMode3SetupBots(); Buzzer_BeepShort(); }
        
        if (k1_click || joy_right) {
            m3_bot_diffs[m3_setup_bot_idx] = (bot_diff_t)((m3_bot_diffs[m3_setup_bot_idx] + 1) % 4);
            LCD_UpdateMode3SetupBots(); Buzzer_BeepShort();
        } else if (joy_left) {
            m3_bot_diffs[m3_setup_bot_idx] = (bot_diff_t)((m3_bot_diffs[m3_setup_bot_idx] + 3) % 4);
            LCD_UpdateMode3SetupBots(); Buzzer_BeepShort();
        }
        
        if (ts_click) {
            if (ts_y >= 65 && ts_y <= 185) {
                uint8_t tapped_idx = (ts_y - 65) / 40;
                static uint32_t last_tap_time2 = 0;
                if (m3_setup_bot_idx != tapped_idx) {
                    m3_setup_bot_idx = tapped_idx; LCD_UpdateMode3SetupBots(); Buzzer_BeepShort(); last_tap_time2 = now;
                } else {
                    if (now - last_tap_time2 < 800) {
                        m3_bot_diffs[m3_setup_bot_idx] = (bot_diff_t)((m3_bot_diffs[m3_setup_bot_idx] + 1) % 4);
                        LCD_UpdateMode3SetupBots(); Buzzer_BeepShort();
                        last_tap_time2 = 0;
                    } else { last_tap_time2 = now; }
                }
            } else if (ts_y >= 275) { k2_click = 1; }
        }
        if (k2_click) {
            m3_state = M3_STATE_INIT_ARENA; m3_world_seed = now; Buzzer_BeepShort();
            LCD_Clear(0, 20, 240, 300, UI_BG); LCD_SetColors(BLACK, UI_BG); LCD_TEXT(40, 100, "GENERATING ARENA...");
            Mode3_GenerateWorld(); HAL_Delay(500); m3_state = M3_STATE_BATTLE;
            LCD_Clear(0, 0, 240, 320, UI_BG); LCD_DrawStatusBar(); LCD_DrawMode3HUD();
        }
    }
    else if (m3_state == M3_STATE_BATTLE) {
        // Run AI & Input
        for (int i=0; i<4; i++) {
            if (!tanks[i].active || tanks[i].hp <= 0) continue;
            
            float speed = (tanks[i].speed_timer > now) ? 4.0f : 2.5f; 
            int gr = (int)tanks[i].y / TILE_SIZE, gc = (int)tanks[i].x / TILE_SIZE;
            if (gr>=0 && gr<100 && gc>=0 && gc<80) {
                uint8_t tile = arena_grid[gr][gc];
                if (tile == TILE_RIVER && tanks[i].wings_timer <= now) speed *= 0.4f;
                if (tile >= TILE_SUPPLEMENT) {
                    if (tile == TILE_SUPPLEMENT) { tanks[i].hp++; if(tanks[i].hp>10) tanks[i].hp=10; arena_grid[gr][gc] = TILE_EMPTY; }
                    else if (tile == TILE_SPEED) { tanks[i].speed_timer = now + 5000; arena_grid[gr][gc] = TILE_EMPTY; }
                    else if (tile == TILE_WINGS) { tanks[i].wings_timer = now + 5000; arena_grid[gr][gc] = TILE_EMPTY; }
                    else if (tile == TILE_SHIELD) { tanks[i].shield_timer = now + 5000; arena_grid[gr][gc] = TILE_EMPTY; }
                    else if (tile == TILE_TRAP && now >= m3_trap_cooldowns[gr][gc]) { if (tanks[i].shield_timer <= now) tanks[i].hp--; m3_trap_cooldowns[gr][gc] = now + 10000; }
                    if (i == 0) { LCD_DrawMode3HUD(); Buzzer_BeepShort(); }
                }
            }

            if (tanks[i].is_human) {
                // Enhanced deadzone check to prevent drift
                if (joy_y < Y_FWD_THRESH_ADC - 150) { 
                    tanks[i].moving_fwd = 1;
                } else {
                    tanks[i].moving_fwd = 0;
                }
                
                if (joy_x < X_LEFT_THRESH_ADC - 150) { 
                    tanks[i].angle -= 6.0f; 
                } else if (joy_x > X_RIGHT_THRESH_ADC + 150) { 
                    tanks[i].angle += 6.0f; 
                }
                
                if (fire_pressed && !tanks[i].ball_active) {
                    tanks[i].ball_x = tanks[i].x; tanks[i].ball_y = tanks[i].y;
                    tanks[i].ball_vx = 12.0f * cos(tanks[i].angle * 3.14159f / 180.0f);
                    tanks[i].ball_vy = 12.0f * sin(tanks[i].angle * 3.14159f / 180.0f);
                    tanks[i].ball_active = 1; tanks[i].ball_bounces = 0; tanks[i].ball_spawn_tick = now; Buzzer_BeepShort();
                }
            } else {
                // AI Logic
                if (now - tanks[i].last_decision_tick > ((tanks[i].diff == BOT_EASY) ? 1000 : ((tanks[i].diff == BOT_MID) ? 500 : 250))) {
                    tanks[i].last_decision_tick = now;
                    if (tanks[i].diff == BOT_EASY) {
                        tanks[i].target_angle = rand() % 360;
                        tanks[i].moving_fwd = (rand() % 100 < 70) ? 1 : 0;
                        if (rand() % 100 < 20 && !tanks[i].ball_active) {
                            tanks[i].ball_x = tanks[i].x; tanks[i].ball_y = tanks[i].y;
                            tanks[i].ball_vx = 12.0f * cos(tanks[i].angle * 3.14159f / 180.0f);
                            tanks[i].ball_vy = 12.0f * sin(tanks[i].angle * 3.14159f / 180.0f);
                            tanks[i].ball_active = 1; tanks[i].ball_bounces = 0; tanks[i].ball_spawn_tick = now;
                        }
                    } else {
                        // Mid & Hard logic: aim at player 0
                        float dx = tanks[0].x - tanks[i].x;
                        float dy = tanks[0].y - tanks[i].y;
                        tanks[i].target_angle = atan2(dy, dx) * 180.0f / 3.14159f;
                        float dist = sqrt(dx*dx + dy*dy);
                        tanks[i].moving_fwd = (dist > 40.0f) ? 1 : 0;
                        
                        if (tanks[i].diff == BOT_HARD) {
                            // Avoid walls by nudging target angle
                            int tr = (int)(tanks[i].y + 15*sin(tanks[i].target_angle * 3.14159f / 180.0f)) / TILE_SIZE;
                            int tc = (int)(tanks[i].x + 15*cos(tanks[i].target_angle * 3.14159f / 180.0f)) / TILE_SIZE;
                            if (tr>=0 && tr<100 && tc>=0 && tc<80 && arena_grid[tr][tc] == TILE_WALL) {
                                tanks[i].target_angle += (rand() % 2 == 0) ? 45.0f : -45.0f;
                            }
                        }

                        // Fire if pointing roughly at target
                        float adiff = fabs(tanks[i].angle - tanks[i].target_angle);
                        while(adiff > 180) adiff -= 360;
                        adiff = fabs(adiff);
                        if (adiff < 20.0f && !tanks[i].ball_active && (rand() % 100 < ((tanks[i].diff == BOT_HARD) ? 80 : 30))) {
                            tanks[i].ball_x = tanks[i].x; tanks[i].ball_y = tanks[i].y;
                            tanks[i].ball_vx = 12.0f * cos(tanks[i].angle * 3.14159f / 180.0f);
                            tanks[i].ball_vy = 12.0f * sin(tanks[i].angle * 3.14159f / 180.0f);
                            tanks[i].ball_active = 1; tanks[i].ball_bounces = 0; tanks[i].ball_spawn_tick = now;
                        }
                    }
                }
                // Steer towards target
                float adiff = tanks[i].target_angle - tanks[i].angle;
                while(adiff > 180.0f) adiff -= 360.0f;
                while(adiff < -180.0f) adiff += 360.0f;
                if (adiff > 6.0f) tanks[i].angle += 6.0f;
                else if (adiff < -6.0f) tanks[i].angle -= 6.0f;
            }

            if (tanks[i].moving_fwd) {
                float nx = tanks[i].x + speed * cos(tanks[i].angle * 3.14159f / 180.0f);
                float ny = tanks[i].y + speed * sin(tanks[i].angle * 3.14159f / 180.0f);
                uint8_t can = 1; int16_t os[4][2] = {{-7,-7},{7,-7},{-7,7},{7,7}};
                for (int j=0; j<4; j++) {
                    float cx=nx+os[j][0], cy=ny+os[j][1]; int r=(int)cy/TILE_SIZE, c=(int)cx/TILE_SIZE;
                    if (cx<2 || cx>world_w-2 || cy<2 || cy>world_h-2 || (r>=0 && r<100 && c>=0 && c<80 && arena_grid[r][c]==TILE_WALL)) { can=0; break; }
                }
                if (can) {
                    for(int k=0; k<4; k++) {
                        if (k == i || !tanks[k].active || tanks[k].hp <= 0) continue;
                        if (abs((int)nx - (int)tanks[k].x) < 15 && abs((int)ny - (int)tanks[k].y) < 15) { can = 0; break; }
                    }
                }
                if (can) { tanks[i].x = nx; tanks[i].y = ny; }
            }
            
            // Proximity bounce-back collision with other tanks
            for(int k=0; k<4; k++) {
                if (k == i || !tanks[k].active || tanks[k].hp <= 0) continue;
                float dist = sqrt((tanks[i].x - tanks[k].x) * (tanks[i].x - tanks[k].x) + 
                                  (tanks[i].y - tanks[k].y) * (tanks[i].y - tanks[k].y));
                if (dist < 12.0f && dist > 0.1f) {
                    // Bounce back and lose 1 HP each
                    float dx = (tanks[i].x - tanks[k].x) / dist;
                    float dy = (tanks[i].y - tanks[k].y) / dist;
                    tanks[i].x += dx * 3.0f;
                    tanks[i].y += dy * 3.0f;
                    tanks[k].x -= dx * 3.0f;
                    tanks[k].y -= dy * 3.0f;
                    if (tanks[i].shield_timer <= now) tanks[i].hp--;
                    if (tanks[k].shield_timer <= now) tanks[k].hp--;
                    if (k == 0 || i == 0) LCD_DrawMode3HUD();
                    Buzzer_BeepShort();
                }
            }

            if (tanks[i].ball_active) {
                if (now - tanks[i].ball_spawn_tick > 5000) tanks[i].ball_active = 0;
                else {
                    float nx = tanks[i].ball_x + tanks[i].ball_vx, ny = tanks[i].ball_y + tanks[i].ball_vy; 
                    int ngr = (int)ny / TILE_SIZE, ngc = (int)nx / TILE_SIZE;
                    
                    // Collision with tanks (with minimum distance check to prevent penetration)
                    if (now - tanks[i].ball_spawn_tick > 200) {
                        for(int k=0; k<4; k++) {
                            if(!tanks[k].active || tanks[k].hp <= 0) continue;
                            float shot_dist = sqrt((nx - tanks[k].x) * (nx - tanks[k].x) + 
                                                   (ny - tanks[k].y) * (ny - tanks[k].y));
                            // Only damage if shot is at least 10 pixels away from firer
                            float firer_dist = sqrt((tanks[i].x - tanks[k].x) * (tanks[i].x - tanks[k].x) + 
                                                    (tanks[i].y - tanks[k].y) * (tanks[i].y - tanks[k].y));
                            if (shot_dist < 8 && firer_dist >= 10.0f) { 
                                if (tanks[k].shield_timer <= now) tanks[k].hp--; 
                                tanks[i].ball_active = 0; 
                                if(k==0) LCD_DrawMode3HUD(); 
                                Buzzer_BeepShort(); 
                                break; 
                            }
                        }
                    }

                    if (tanks[i].ball_active) {
                        if (ngr<0 || ngr>=100 || ngc<0 || ngc>=80 || nx<0 || nx > world_w || ny<0 || ny > world_h || (ngr>=0 && ngr<100 && ngc>=0 && ngc<80 && arena_grid[ngr][ngc] == TILE_WALL)) {
                            if (tanks[i].ball_bounces >= 2) tanks[i].ball_active = 0; 
                            else { int cur_r = (int)tanks[i].ball_y / TILE_SIZE, cur_c = (int)tanks[i].ball_x / TILE_SIZE; if (ngr != cur_r) { tanks[i].ball_vy = -tanks[i].ball_vy; } if (ngc != cur_c) { tanks[i].ball_vx = -tanks[i].ball_vx; } tanks[i].ball_bounces++; }
                        } else { tanks[i].ball_x = nx; tanks[i].ball_y = ny; }
                    }
                }
            }
        }
        
        m3_hp = tanks[0].hp; // Sync for compatibility

        uint8_t active_bots_configured = 0;
        uint8_t alive_bots = 0;
        for(int i=1; i<4; i++) {
            if(tanks[i].active) {
                active_bots_configured++;
                if (tanks[i].hp > 0) alive_bots++;
            }
        }

        if (tanks[0].hp <= 0) { m3_state = M3_STATE_GAME_OVER; LCD_Clear(20, 100, 200, 100, RED); LCD_SetColors(WHITE, RED); LCD_TEXT(60, 130, "DEFEAT!"); LCD_TEXT(60, 150, "K2: RESTART"); }
        else if (active_bots_configured > 0 && alive_bots == 0) { m3_state = M3_STATE_GAME_OVER; LCD_Clear(20, 100, 200, 100, MY_GREEN); LCD_SetColors(WHITE, MY_GREEN); LCD_TEXT(60, 130, "VICTORY!"); LCD_TEXT(60, 150, "K2: RESTART"); }

        Mode3_RenderArena();
    } else if (m3_state == M3_STATE_GAME_OVER && k2_click) { Mode3_Init(); }
}
