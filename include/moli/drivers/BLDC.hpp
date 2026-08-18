#pragma once
// ============================================================================
// BLDC 无刷电机薄封装：组合 SimpleFOC 的 BLDCMotor（FOC 算法）与
// moli::drivers::foc::BldcDriver3Pwm（驱动适配），传感器通过 linkSensor 注入。
// 风格对齐 dc_motor.hpp / servo.hpp：header-only、按值持有硬件、begin()
// 初始化。
// ============================================================================
#include <moli/drivers/foc/bldc_driver3pwm.hpp>
#include <moli/hal/clock.hpp>
#include <moli/hal/pwm.hpp>

#include "BLDCMotor.h"
#include "common/base_classes/Sensor.h"

namespace moli::drivers {

// PwmHw 满足 moli::hal::PwmHardware concept（三相 PWM）
// ClockHw 满足 moli::hal::ClockHardware concept（时间源，构造时注入 SimpleFOC）
template <hal::PwmHardware PwmHw, hal::ClockHardware ClockHw> class BLDC {
  public:
    // @param pwm_a/pwm_b/pwm_c 三相 PWM 硬件实例
    // @param pole_pairs        电机极对数
    // @param phase_resistance  相电阻 [Ω]（可选，用于参数辨识）
    // @param kv                电机 KV 值 rpm/V（可选）
    // @param phase_inductance  相电感 [H]（可选）
    BLDC(PwmHw pwm_a, PwmHw pwm_b, PwmHw pwm_c, int pole_pairs,
         float phase_resistance = NOT_SET, float kv = NOT_SET,
         float phase_inductance = NOT_SET)
        : driver_(pwm_a, pwm_b, pwm_c),
          motor_(pole_pairs, phase_resistance, kv, phase_inductance) {
        // 注入平台时钟到 SimpleFOC 全局时间函数指针（见 arduino_compat.h）
        ::g_micros_fn = &ClockHw::now_us;
        ::g_delay_ms_fn = &ClockHw::delay_ms;
        ::g_delay_us_fn = &ClockHw::delay_us;
        motor_.linkDriver(&driver_);
    }

    // 初始化驱动硬件（三相 PWM 启动）
    void begin() { driver_.init(); }

    // 注入位置传感器（AS5600 / AS5047D 等，继承 SimpleFOC::Sensor）
    void linkSensor(Sensor *sensor) { motor_.linkSensor(sensor); }

    // 执行传感器零点对齐（需在 begin() + linkSensor() 之后调用一次）
    int initFOC() { return motor_.initFOC(); }

    // 运行 FOC 电流环（需高频周期调用，典型 1~20kHz）
    void loopFOC() { motor_.loopFOC(); }

    // 执行运动控制环（按 motor_.controller 模式：torque/velocity/angle）
    void move(float target = NOT_SET) { motor_.move(target); }

    // 暴露底层 SimpleFOC 电机对象，供高级配置（PID、controller 模式等）
    BLDCMotor &motor() { return motor_; }

  private:
    foc::BldcDriver3Pwm<PwmHw> driver_;
    BLDCMotor motor_;
};

} // namespace moli::drivers
