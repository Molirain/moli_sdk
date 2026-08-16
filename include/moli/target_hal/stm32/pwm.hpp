#pragma once
#include <cstdint>

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