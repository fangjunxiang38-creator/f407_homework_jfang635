#ifndef LED_HPP
#define LED_HPP

#include "main.h"

/**
 * @brief 板上三色灯
 * @note  GPIO 已经由 CubeMX 的 MX_GPIO_Init() 初始化好了，
 *        这个模块只负责"点亮 / 熄灭 / 翻转"的封装
 */
enum LED_Color
{
    LED_RED = 0,
    LED_GREEN,
    LED_BLUE,
    LED_COUNT,      /* = 3，顺便给数组定大小 */
};

void LED_On(LED_Color color);
void LED_Off(LED_Color color);
void LED_Toggle(LED_Color color);
void LED_AllOff(void);

#endif /* LED_HPP */