#include "main.h" // 具体的寄存器和 HAL API 集中在这里
#include <moli/target_hal/stm32/pwm.hpp>

namespace moli::port::stm32 {

Pwm::Pwm(TIM_HandleTypeDef *htim, uint32_t ch1, uint32_t ch2)
    : htim_(htim), ch1_(ch1), ch2_(ch2) {}

void Pwm::begin() {
    __HAL_TIM_SET_COMPARE(htim_, ch1_, 0);
    __HAL_TIM_SET_COMPARE(htim_, ch2_, 0);
    HAL_TIM_PWM_Start(htim_, ch1_);
    HAL_TIM_PWM_Start(htim_, ch2_);
}

void Pwm::set_duty(float duty1, float duty2) {
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim_);
    __HAL_TIM_SET_COMPARE(htim_, ch1_, static_cast<uint32_t>(duty1 * arr));
    __HAL_TIM_SET_COMPARE(htim_, ch2_, static_cast<uint32_t>(duty2 * arr));
}

} // namespace moli::port::stm32