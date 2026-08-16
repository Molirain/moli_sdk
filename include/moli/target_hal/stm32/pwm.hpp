#pragma once
#include <cstdint>

// 前向声明，避免引入庞大的 stm32h7xx_hal.h
struct __TIM_HandleTypeDef;
using TIM_HandleTypeDef = struct __TIM_HandleTypeDef;

namespace moli::port::stm32 {

class Pwm {
  public:
    Pwm(TIM_HandleTypeDef *htim, uint32_t ch);

    void begin();
    void set_duty(float duty);
    float get_duty() const;

  private:
    TIM_HandleTypeDef *htim_;
    uint32_t ch_;
};

} // namespace moli::port::stm32