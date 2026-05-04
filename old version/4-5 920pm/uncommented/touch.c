#include "touch.h"

// --- Private Helper Functions ---
static void T_Delay(void)
{
    for (volatile int i = 0; i < 12; i++) __NOP();
}

static void T_WriteBit(uint8_t b)
{
    HAL_GPIO_WritePin(T_DIN_PORT, T_DIN_PIN, b ? GPIO_PIN_SET : GPIO_PIN_RESET);
    T_Delay();
    HAL_GPIO_WritePin(T_CLK_PORT, T_CLK_PIN, GPIO_PIN_SET);
    T_Delay();
    HAL_GPIO_WritePin(T_CLK_PORT, T_CLK_PIN, GPIO_PIN_RESET);
}

static uint8_t T_ReadBit(void)
{
    HAL_GPIO_WritePin(T_CLK_PORT, T_CLK_PIN, GPIO_PIN_SET);
    T_Delay();
    HAL_GPIO_WritePin(T_CLK_PORT, T_CLK_PIN, GPIO_PIN_RESET);
    T_Delay();
    return (HAL_GPIO_ReadPin(T_DOUT_PORT, T_DOUT_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

static void T_WriteByte(uint8_t data)
{
    for (int i = 7; i >= 0; i--) T_WriteBit((data >> i) & 1U);
}

static uint16_t XPT2046_Read12(uint8_t cmd)
{
    uint16_t v = 0;
    HAL_GPIO_WritePin(T_CS_PORT, T_CS_PIN, GPIO_PIN_RESET);
    T_WriteByte(cmd);
    for (int i = 0; i < 16; i++)
    {
        v <<= 1;
        v |= T_ReadBit();
    }
    HAL_GPIO_WritePin(T_CS_PORT, T_CS_PIN, GPIO_PIN_SET);
    return (v >> 4) & 0x0FFF;
}

// --- Public Functions ---
uint8_t TouchPressed(void)
{
    return (HAL_GPIO_ReadPin(T_IRQ_PORT, T_IRQ_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
}

uint16_t TouchReadXRaw(void)
{
    return XPT2046_Read12(XPT_CMD_X);
}

uint16_t TouchReadYRaw(void)
{
    return XPT2046_Read12(XPT_CMD_Y);
}
