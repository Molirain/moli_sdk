#include "main.h" // 具体的寄存器和 HAL API 集中在这里
#include <moli/target_hal/stm32/pwm.hpp>

namespace moli::port::stm32 {

Pwm::Pwm(TIM_HandleTypeDef *htim, uint32_t ch) : htim_(htim), ch_(ch) {}

static bool is_channel_enabled(TIM_HandleTypeDef *htim, uint32_t ch) {
    const uint32_t ccer = htim->Instance->CCER;
    switch (ch) {
    case TIM_CHANNEL_1:
        return (ccer & TIM_CCER_CC1E) != 0U;
    case TIM_CHANNEL_2:
        return (ccer & TIM_CCER_CC2E) != 0U;
    case TIM_CHANNEL_3:
        return (ccer & TIM_CCER_CC3E) != 0U;
    case TIM_CHANNEL_4:
        return (ccer & TIM_CCER_CC4E) != 0U;
    case TIM_CHANNEL_5:
        return (ccer & TIM_CCER_CC5E) != 0U;
    case TIM_CHANNEL_6:
        return (ccer & TIM_CCER_CC6E) != 0U;
    default:
        return false;
    }
}

void Pwm::begin() {
    __HAL_TIM_SET_COMPARE(htim_, ch_, 0);
    if (!is_channel_enabled(htim_, ch_)) {
        HAL_TIM_PWM_Start(htim_, ch_);
    }
}

void Pwm::set_duty(float duty) {
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim_);
    __HAL_TIM_SET_COMPARE(htim_, ch_, static_cast<uint32_t>(duty * arr));
}

float Pwm::get_duty() const {
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim_);
    uint32_t ccr = __HAL_TIM_GET_COMPARE(htim_, ch_);
    return static_cast<float>(ccr) / static_cast<float>(arr);
}

} // namespace moli::port::stm32