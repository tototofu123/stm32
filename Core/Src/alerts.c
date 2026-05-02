#include "alerts.h"

// Buzzer state trackers
uint8_t  buzzer_active   = 0U;
uint32_t buzzer_tick     = 0U;
uint32_t buzzer_duration = 0U;

void RGB_Set(uint8_t r, uint8_t g, uint8_t b)
{
    // The RGB LED is active-low, so 1 turns it ON (GPIO_PIN_RESET) and 0 turns it OFF (GPIO_PIN_SET)
    HAL_GPIO_WritePin(RGB_R_PORT, RGB_R_PIN, r ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(RGB_G_PORT, RGB_G_PIN, g ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(RGB_B_PORT, RGB_B_PIN, b ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Buzzer_Set(uint8_t on)
{
    HAL_GPIO_WritePin(BEEP_PORT, BEEP_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Buzzer_BeepShort(void)
{
    buzzer_active = 1U;
    buzzer_tick = HAL_GetTick();
    buzzer_duration = BEEP_SHORT_MS;
    Buzzer_Set(1);
}

void Buzzer_BeepLong(void)
{
    buzzer_active = 1U;
    buzzer_tick = HAL_GetTick();
    buzzer_duration = BEEP_LONG_MS;
    Buzzer_Set(1);
}

void Buzzer_Task(void)
{
    uint32_t now = HAL_GetTick();

    if (buzzer_active)
    {
        if ((now - buzzer_tick) < buzzer_duration)
        {
            Buzzer_Set(1);
        }
        else
        {
            buzzer_active = 0U;
            buzzer_duration = 0U;
            Buzzer_Set(0);
        }
    }
    else
    {
        Buzzer_Set(0);
    }
}