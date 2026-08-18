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

#include <cmath>
#include <math.h>

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

    // ======================== 角度环（0~360° 单圈位置伺服）
    // ======================== 设置目标角度（度）。内部自动：
    //   1. 归一化到 [0, 360)
    //   2. 选择最短旋转路径（避免从 350° 反转 340° 到 10°）
    //   3. 切换到 SimpleFOC 级联角度环（MotionControlType::angle）
    void setAngle(float angle_deg) {
        target_angle_deg_ = normalizeDeg(angle_deg);
        motor_.controller = MotionControlType::angle;
        motor_.target = shortestTarget(target_angle_deg_);
    }

    // 当前机械角度（度），归一化到 [0, 360)
    float getAngle() { return normalizeDeg(motor_.shaftAngle() * kRadToDeg); }

    // 将当前位置标定为零点（0°）。调用后 setAngle(0) 即回到该位置。
    void setZero() {
        if (motor_.sensor != nullptr) {
            motor_.sensor_offset = static_cast<float>(motor_.sensor_direction) *
                                   motor_.sensor->getAngle();
        }
        // 零点变了，重新同步目标的圈数
        motor_.target = shortestTarget(target_angle_deg_);
    }

    // 角度环 P 增益（越大响应越"硬"，过大易振荡）
    void setAngleP(float p) { motor_.P_angle.P = p; }

    // 角度环输出的最大角速度限幅 [rad/s]
    void setVelocityLimit(float limit_rad_s) {
        motor_.velocity_limit = limit_rad_s;
    }

    // 暴露底层 SimpleFOC 电机对象，供高级配置（PID、controller 模式等）
    BLDCMotor &motor() { return motor_; }

  private:
    static constexpr float kRadToDeg = 180.0f / _PI; // 弧度 → 度

    foc::BldcDriver3Pwm<PwmHw> driver_;
    BLDCMotor motor_;
    float target_angle_deg_ = 0.0f; // 最近一次 setAngle 的目标（度）

    // 归一化到 [0, 360)
    static float normalizeDeg(float deg) {
        float r = fmodf(deg, 360.0f);
        return r < 0.0f ? r + 360.0f : r;
    }

    // 把目标角度（度）换算成「离当前累计角度最近」的等效目标（弧度），
    // 从而让 SimpleFOC 角度环总是走最短路径。
    float shortestTarget(float angle_deg) {
        const float target = angle_deg * _PI / 180.0f;
        const float current = motor_.shaftAngle(); // 累计角度（可超 2π）
        const float rev = roundf((current - target) / _2PI);
        return target + rev * _2PI;
    }
};

} // namespace moli::drivers
