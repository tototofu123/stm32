#ifndef SEVEN_SEG_H
#define SEVEN_SEG_H

#include "main.h"
#include <stdint.h>

// Left Segment Pins
#define LSEG_A_PIN          GPIO_PIN_5
#define LSEG_A_PORT         GPIOA
#define LSEG_B_PIN          GPIO_PIN_6
#define LSEG_B_PORT         GPIOA
#define LSEG_C_PIN          GPIO_PIN_4
#define LSEG_C_PORT         GPIOC
#define LSEG_D_PIN          GPIO_PIN_4
#define LSEG_D_PORT         GPIOA
#define LSEG_E_PIN          GPIO_PIN_7
#define LSEG_E_PORT         GPIOA
#define LSEG_F_PIN          GPIO_PIN_7
#define LSEG_F_PORT         GPIOB
#define LSEG_G_PIN          GPIO_PIN_6
#define LSEG_G_PORT         GPIOB
#define LSEG_DP_PIN         GPIO_PIN_7
#define LSEG_DP_PORT        GPIOE

// Right Segment Pins
#define RSEG_A_PIN          GPIO_PIN_14
#define RSEG_A_PORT         GPIOB
#define RSEG_B_PIN          GPIO_PIN_15
#define RSEG_B_PORT         GPIOB
#define RSEG_C_PIN          GPIO_PIN_5
#define RSEG_C_PORT         GPIOC
#define RSEG_D_PIN          GPIO_PIN_7
#define RSEG_D_PORT         GPIOC
#define RSEG_E_PIN          GPIO_PIN_6
#define RSEG_E_PORT         GPIOC
#define RSEG_F_PIN          GPIO_PIN_13
#define RSEG_F_PORT         GPIOB
#define RSEG_G_PIN          GPIO_PIN_12
#define RSEG_G_PORT         GPIOB

// Display Modes
typedef enum {
    SEG_IDLE = 0,
    SEG_K1_COUNT,
    SEG_K2_SHOW88,
    SEG_JSW_CD,
    SEG_ZERO_HOLD
} seg_mode_t;

// Extern Globals (So main.c can interact with the current state)
extern seg_mode_t seg_mode;
extern uint32_t   seg_tick;
extern int        seg_tenths;
extern uint8_t    seg_left;
extern uint8_t    seg_right;
extern uint8_t    seg_dp;

// Function Prototypes
void SEG_WritePin(GPIO_TypeDef *port, uint16_t pin, uint8_t on);
void SEG_AllOff(void);
void SEG_ShowLeft(uint8_t d, uint8_t dp);
void SEG_ShowRight(uint8_t d);
void SEG_ShowPair(uint8_t left, uint8_t right, uint8_t dp);
void SEG_ShowTenths(int t);
void SEG_StartCooldownCountdown(uint32_t cooldown_ms);
void SEG_Task(void);

#endif // SEVEN_SEG_H