#pragma once
#include <algorithm>
#include <moli/hal/pwm.hpp>

namespace moli::drivers {

// 通用 50Hz 舵机驱动
// 角度范围: -135° ~ +135°，居中 0° 对应 1.5ms 脉宽
// 占空比范围: 2.5% (0.5ms) ~ 12.5% (2.5ms)
template <hal::PwmHardware PwmHw> class Servo {
  public:
    explicit Servo(PwmHw pwm) : pwm_(pwm) {}

    void begin() { pwm_.begin(); }

    void setAngle(float angle) {
        angle_ = std::clamp(angle, -135.0f, 135.0f);
        pwm_.set_duty(angleToDuty(angle_));
    }

    float getAngle() const { return angle_; }

    void center() { setAngle(0.0f); }

  private:
    PwmHw pwm_;
    float angle_ = 0.0f;

    static float angleToDuty(float angle) {
        // us = 1500 + angle * (1000 / 135)
        float us = 1500.0f + angle * (1000.0f / 135.0f);
        // duty = us / 20000
        return us / 20000.0f;
    }
};

} // namespace moli::drivers