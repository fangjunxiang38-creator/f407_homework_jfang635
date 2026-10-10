#include "LED.hpp"


static GPIO_TypeDef *const LED_PORT[LED_COUNT] =
{
    GPIOH,      /* LED_RED   → PH12 */
    GPIOH,      /* LED_GREEN → PH11 */
    GPIOH,      /* LED_BLUE  → PH10 */
};

static const uint16_t LED_PIN[LED_COUNT] =
{
    GPIO_PIN_12,
    GPIO_PIN_11,
    GPIO_PIN_10,
};

/*点亮极性：高电平点亮、低电平熄灭*/
#define LED_LEVEL_ON    GPIO_PIN_SET
#define LED_LEVEL_OFF   GPIO_PIN_RESET

void LED_On(LED_Color color)
{
    HAL_GPIO_WritePin(LED_PORT[color], LED_PIN[color], LED_LEVEL_ON);
}

void LED_Off(LED_Color color)
{
    HAL_GPIO_WritePin(LED_PORT[color], LED_PIN[color], LED_LEVEL_OFF);
}

void LED_Toggle(LED_Color color)
{
    HAL_GPIO_TogglePin(LED_PORT[color], LED_PIN[color]);
}

void LED_AllOff(void)
{
    for (uint8_t i = 0; i < LED_COUNT; i++)
    {
        HAL_GPIO_WritePin(LED_PORT[i], LED_PIN[i], LED_LEVEL_OFF);
    }
}