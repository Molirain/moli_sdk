#pragma once
#include <moli/hal/pwm.hpp>

namespace moli::drivers {
template <hal::PwmHardware PwmHw> class Servo {
  public:
    Servo(PwmHw pwm) : pwm_(pwm) {}
    void begin();
    void setAngle(float angle);
    float getAngle() const;

  private:
    PwmHw pwm_;
};

// 以下为实现
template <hal::PwmHardware PwmHw> void Servo<PwmHw>::begin() { pwm_.begin(); }

template <hal::PwmHardware PwmHw> void Servo<PwmHw>::setAngle(float angle) {
    // 将角度映射到占空比范围
    float duty = (angle / 180.0f) * (1.0f - 0.05f) + 0.05f; // 5% ~ 95%
    pwm_.set_duty(duty);
}

template <hal::PwmHardware PwmHw> float Servo<PwmHw>::getAngle() const {
    // 将占空比映射到角度范围
    float duty = pwm_.get_duty();
    return (duty - 0.05f) / (1.0f - 0.05f) * 180.0f;
}
} // namespace moli::drivers