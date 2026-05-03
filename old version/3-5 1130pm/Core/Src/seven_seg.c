#include "seven_seg.h"

// Initialize state variables
seg_mode_t seg_mode = SEG_IDLE;
uint32_t   seg_tick = 0;
int        seg_tenths = 0;
uint8_t    seg_left = 0;
uint8_t    seg_right = 0;
uint8_t    seg_dp = 0;

void SEG_WritePin(GPIO_TypeDef *port, uint16_t pin, uint8_t on)
{
    HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void SEG_AllOff(void)
{
    SEG_WritePin(LSEG_A_PORT, LSEG_A_PIN, 0);
    SEG_WritePin(LSEG_B_PORT, LSEG_B_PIN, 0);
    SEG_WritePin(LSEG_C_PORT, LSEG_C_PIN, 0);
    SEG_WritePin(LSEG_D_PORT, LSEG_D_PIN, 0);
    SEG_WritePin(LSEG_E_PORT, LSEG_E_PIN, 0);
    SEG_WritePin(LSEG_F_PORT, LSEG_F_PIN, 0);
    SEG_WritePin(LSEG_G_PORT, LSEG_G_PIN, 0);
    SEG_WritePin(LSEG_DP_PORT, LSEG_DP_PIN, 0);

    SEG_WritePin(RSEG_A_PORT, RSEG_A_PIN, 0);
    SEG_WritePin(RSEG_B_PORT, RSEG_B_PIN, 0);
    SEG_WritePin(RSEG_C_PORT, RSEG_C_PIN, 0);
    SEG_WritePin(RSEG_D_PORT, RSEG_D_PIN, 0);
    SEG_WritePin(RSEG_E_PORT, RSEG_E_PIN, 0);
    SEG_WritePin(RSEG_F_PORT, RSEG_F_PIN, 0);
    SEG_WritePin(RSEG_G_PORT, RSEG_G_PIN, 0);
}

void SEG_ShowLeft(uint8_t d, uint8_t dp)
{
    static const uint8_t lut[10][7] = {
        {1,1,1,1,1,1,0}, // 0
        {0,1,1,0,0,0,0}, // 1
        {1,1,0,1,1,0,1}, // 2
        {1,1,1,1,0,0,1}, // 3
        {0,1,1,0,0,1,1}, // 4
        {1,0,1,1,0,1,1}, // 5
        {1,0,1,1,1,1,1}, // 6
        {1,1,1,0,0,0,0}, // 7
        {1,1,1,1,1,1,1}, // 8
        {1,1,1,1,0,1,1}  // 9
    };

    if (d > 9) d = 0;

    SEG_WritePin(LSEG_A_PORT, LSEG_A_PIN, lut[d][0]);
    SEG_WritePin(LSEG_B_PORT, LSEG_B_PIN, lut[d][1]);
    SEG_WritePin(LSEG_C_PORT, LSEG_C_PIN, lut[d][2]);
    SEG_WritePin(LSEG_D_PORT, LSEG_D_PIN, lut[d][3]);
    SEG_WritePin(LSEG_E_PORT, LSEG_E_PIN, lut[d][4]);
    SEG_WritePin(LSEG_F_PORT, LSEG_F_PIN, lut[d][5]);
    SEG_WritePin(LSEG_G_PORT, LSEG_G_PIN, lut[d][6]);
    SEG_WritePin(LSEG_DP_PORT, LSEG_DP_PIN, dp ? 1 : 0);
}

void SEG_ShowRight(uint8_t d)
{
    static const uint8_t lut[10][7] = {
        {1,1,1,1,1,1,0}, // 0
        {0,1,1,0,0,0,0}, // 1
        {1,1,0,1,1,0,1}, // 2
        {1,1,1,1,0,0,1}, // 3
        {0,1,1,0,0,1,1}, // 4
        {1,0,1,1,0,1,1}, // 5
        {1,0,1,1,1,1,1}, // 6
        {1,1,1,0,0,0,0}, // 7
        {1,1,1,1,1,1,1}, // 8
        {1,1,1,1,0,1,1}  // 9
    };

    if (d > 9) d = 0;

    SEG_WritePin(RSEG_A_PORT, RSEG_A_PIN, lut[d][0]);
    SEG_WritePin(RSEG_B_PORT, RSEG_B_PIN, lut[d][1]);
    SEG_WritePin(RSEG_C_PORT, RSEG_C_PIN, lut[d][2]);
    SEG_WritePin(RSEG_D_PORT, RSEG_D_PIN, lut[d][3]);
    SEG_WritePin(RSEG_E_PORT, RSEG_E_PIN, lut[d][4]);
    SEG_WritePin(RSEG_F_PORT, RSEG_F_PIN, lut[d][5]);
    SEG_WritePin(RSEG_G_PORT, RSEG_G_PIN, lut[d][6]);
}

void SEG_ShowPair(uint8_t left, uint8_t right, uint8_t dp)
{
    seg_left = left;
    seg_right = right;
    seg_dp = dp;
    SEG_ShowLeft(left, dp);
    SEG_ShowRight(right);
}

void SEG_ShowTenths(int t)
{
    if (t < 0) t = 0;
    if (t > 99) t = 99;
    SEG_ShowPair((uint8_t)(t / 10), (uint8_t)(t % 10), 1);
}

void SEG_ShowCmd(char cmd)
{
    static const uint8_t segments[4][7] = {
        {0,0,0,1,1,1,0}, // L
        {1,0,0,0,1,1,1}, // F
        {1,0,1,1,0,1,1}, // S (same as 5)
        {1,1,1,0,1,1,1}  // A
    };

    uint8_t idx = 2; // Default to S
    if (cmd == 'L') idx = 0;
    else if (cmd == 'F') idx = 1;
    else if (cmd == 'S') idx = 2;
    else if (cmd == 'A' || cmd == 'R') idx = 3;

    seg_mode = SEG_MODE2_CMD;

    // Show on both segments
    for (int i = 0; i < 7; i++) {
        SEG_WritePin((i == 0) ? LSEG_A_PORT : (i == 1) ? LSEG_B_PORT : (i == 2) ? LSEG_C_PORT : (i == 3) ? LSEG_D_PORT : (i == 4) ? LSEG_E_PORT : (i == 5) ? LSEG_F_PORT : LSEG_G_PORT,
                     (i == 0) ? LSEG_A_PIN : (i == 1) ? LSEG_B_PIN : (i == 2) ? LSEG_C_PIN : (i == 3) ? LSEG_D_PIN : (i == 4) ? LSEG_E_PIN : (i == 5) ? LSEG_F_PIN : LSEG_G_PIN,
                     segments[idx][i]);
        SEG_WritePin((i == 0) ? RSEG_A_PORT : (i == 1) ? RSEG_B_PORT : (i == 2) ? RSEG_C_PORT : (i == 3) ? RSEG_D_PORT : (i == 4) ? RSEG_E_PORT : (i == 5) ? RSEG_F_PORT : RSEG_G_PORT,
                     (i == 0) ? RSEG_A_PIN : (i == 1) ? RSEG_B_PIN : (i == 2) ? RSEG_C_PIN : (i == 3) ? RSEG_D_PIN : (i == 4) ? RSEG_E_PIN : (i == 5) ? RSEG_F_PIN : RSEG_G_PIN,
                     segments[idx][i]);
    }
    SEG_WritePin(LSEG_DP_PORT, LSEG_DP_PIN, 0);
}

void SEG_StartCooldownCountdown(uint32_t cooldown_ms)
{
    seg_mode = SEG_JSW_CD;
    seg_tenths = (int)(cooldown_ms / 100U);
    if (seg_tenths < 0) seg_tenths = 0;
    if (seg_tenths > 99) seg_tenths = 99;
    seg_tick = HAL_GetTick();
    SEG_ShowTenths(seg_tenths);
}

void SEG_Task(void)
{
    uint32_t now = HAL_GetTick();

    switch (seg_mode)
    {
    case SEG_JSW_CD:
        while ((now - seg_tick) >= 100U)
        {
            seg_tick += 100U;
            if (seg_tenths > 0)
            {
                seg_tenths--;
                SEG_ShowTenths(seg_tenths);
            }
            else
            {
                SEG_ShowTenths(0);
                seg_mode = SEG_ZERO_HOLD;
                seg_tick = now;
                break;
            }
        }
        break;

    case SEG_ZERO_HOLD:
        if ((now - seg_tick) >= 1000U)
        {
            SEG_ShowPair(0, 0, 0);
            seg_mode = SEG_IDLE;
        }
        break;

    case SEG_IDLE:
    default:
        break;
    }
}