#pragma once
#include <cstdint>
#include <cmath>

enum class RefShape { Constant, Step, Triangle, PeriodicSmoothStep };

// t_ms 是本次实验开始后的时间。输出单位与 low/high 一致。
// Step: delay_ms 前为 low，之后为 high（单次阶跃）。
// Triangle: low -> high -> low，周期 period_ms。
struct RefCurve {
    RefShape shape = RefShape::Step;
    float low = 0.0f;
    float high = 1000.0f;
    uint32_t delay_ms = 1000;
    uint32_t period_ms = 4000;

    float Value(uint32_t t_ms) const {
        if (shape == RefShape::Constant) return high;
        if (shape == RefShape::Step) return t_ms < delay_ms ? low : high;
        if (period_ms < 2) return low;

        if (shape == RefShape::PeriodicSmoothStep) {
            // Match RM26_F4's signal(): hold low for 1/5 period, then use a
            // half-cosine rise to high; the next period starts again at low.
            constexpr float hold_ratio = 0.2f;
            constexpr float pi = 3.14159265358979323846f;
            const float phase = static_cast<float>(t_ms % period_ms) /
                                static_cast<float>(period_ms);
            if (phase < hold_ratio) return low;
            const float progress = (phase - hold_ratio) / (1.0f - hold_ratio);
            const float smooth = 0.5f * (1.0f - std::cos(pi * progress));
            return low + (high - low) * smooth;
        }

        const float phase = static_cast<float>(t_ms % period_ms) / period_ms;
        const float ramp = phase < 0.5f ? 2.0f * phase : 2.0f * (1.0f - phase);
        return low + (high - low) * ramp;
    }
};