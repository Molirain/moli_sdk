#pragma once
#include "moli/hal/pwm.hpp"
#include <algorithm>
#include <cmath>
#include <concepts>

namespace moli::drivers {

// 两个模板参数：
// 1. PwmHw: 遵循 PwmHardware 规范的单通道硬件类
// 2. T: 浮点精度类型（默认为 float，也可以指定为 double）
template <hal::PwmHardware PwmHw, std::floating_point T = float> class DCMotor {
  private:
    PwmHw pwm_a_;
    PwmHw pwm_b_;
    T deadband_;
    T *trim_;
    bool should_trim_;

    T apply_trim(T speed) const {
        if (speed == static_cast<T>(0) || !trim_)
            return static_cast<T>(0);
        T abs_speed = std::abs(speed);
        int idx = static_cast<int>(abs_speed);
        T ratio = abs_speed - static_cast<T>(idx);

        T val = trim_[idx] + (trim_[idx + 1] - trim_[idx]) * ratio;
        return (speed > static_cast<T>(0)) ? val : -val;
    }

  public:
    DCMotor(PwmHw pwm_a, PwmHw pwm_b, T deadband = static_cast<T>(15.0),
            T *trim = nullptr, bool should_trim = false)
        : pwm_a_(pwm_a), pwm_b_(pwm_b), deadband_(deadband), trim_(trim),
          should_trim_(should_trim) {}

    void begin() {
        pwm_a_.begin();
        pwm_b_.begin();
    }

    void setSpeed(T speed) {
        // 1. 根据当前精度做限幅
        speed =
            std::clamp(speed, static_cast<T>(-100.0), static_cast<T>(100.0));

        if (speed == static_cast<T>(0)) {
            pwm_a_.set_duty(0.0f);
            pwm_b_.set_duty(0.0f);
            return;
        }

        // 2. 通用死区与映射计算
        T var = std::abs(speed);
        var = deadband_ + (var / static_cast<T>(100.0)) *
                              (static_cast<T>(100.0) - deadband_);
        if (should_trim_) {
            var = apply_trim(var);
        }

        // 无论内部怎么算，转换为底层 PWM 接收的比例
        float duty = static_cast<float>(var / static_cast<T>(100.0));

        if (speed > static_cast<T>(0)) {
            pwm_a_.set_duty(duty);
            pwm_b_.set_duty(0.0f);
        } else {
            pwm_a_.set_duty(0.0f);
            pwm_b_.set_duty(duty);
        }
    }
};

} // namespace moli::drivers