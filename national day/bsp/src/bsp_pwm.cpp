//
// Created by cosmosmount on 2025/9/2.
//

#include "bsp_pwm.hpp"
#define TIMER_CLOCK_HZ  168000000U

void PWM_Init(void)
{
    // TODO: 根据实际使用的定时器和通道完成 PWM 初始化。
    PWM_Start(&htim1,  TIM_CHANNEL_1);
    PWM_Start(&htim10, TIM_CHANNEL_1);
}

void PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{

    // TODO: 调用 HAL 启动指定定时器通道的 PWM，并检查返回状态。
    if (htim == nullptr) return;
    HAL_TIM_PWM_Start(htim, Channel);
}

void PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    // TODO: 调用 HAL 停止指定定时器通道的 PWM。
    if (htim == nullptr) return;
    HAL_TIM_PWM_Stop(htim, Channel);
}

void PWM_SetPeriod(TIM_HandleTypeDef *htim, float period_s)
{
    // TODO: 根据定时器时钟和预分频值，将秒转换为 ARR 并更新周期。
    if (htim == nullptr || period_s <= 0.0f) return;

    uint32_t arr = (uint32_t)(period_s * TIMER_CLOCK_HZ / (htim->Instance->PSC + 1U)) - 1U;

    __HAL_TIM_SET_AUTORELOAD(htim, arr);   /* 写入 ARR */
    __HAL_TIM_SET_COUNTER(htim, 0);        /* 计数器归零，新周期立刻生效 */
}

void PWM_SetDutyRatio(TIM_HandleTypeDef *htim, float dutyratio, uint32_t channel)
{
    // TODO: 将 [0, 1] 占空比换算为比较值，写入指定通道。
    if (htim == nullptr) return;

    /* 把占空比限制在 0~1 */
    if (dutyratio < 0.0f) dutyratio = 0.0f;
    if (dutyratio > 1.0f) dutyratio = 1.0f;

    /* CCR = 占空比 × (ARR+1) */
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim) + 1U;
    __HAL_TIM_SET_COMPARE(htim, channel, (uint32_t)(dutyratio * arr));
}
