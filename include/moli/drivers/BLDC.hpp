#pragma once
// ============================================================================
// BLDC 无刷电机薄封装：组合 SimpleFOC 的 BLDCMotor（FOC 算法）与
// moli::drivers::foc::BldcDriver3Pwm（驱动适配），传感器通过 linkSensor 注入。
// 风格对齐 dc_motor.hpp / servo.hpp：header-only、按值持有硬件、begin()
// 初始化。
//
// 功率级管理（模板参数默认 NullPin，不接线即为空操作）：
//   - en_sys / en_pwm：芯片与 PWM 输出使能引脚，极性由 en_active_high 决定；
//     begin() 后默认关断，调用 enable()/disable() 控制。
//   - fault：外部中断引脚（低有效为常见，见 begin 的 fault_edge 参数）。
//     中断触发后默认立即拉低两个使能引脚切断功率并锁存故障标志；
//     可通过 setFaultHandler() 挂载自定义回调完全接管（ISR 上下文，
//     注意只做轻量操作）。
// ============================================================================
#include <moli/drivers/foc/bldc_driver3pwm.hpp>
#include <moli/hal/clock.hpp>
#include <moli/hal/gpio.hpp>
#include <moli/hal/pwm.hpp>

#include <cmath>
#include <math.h>

#include "BLDCMotor.h"
#include "common/base_classes/Sensor.h"

namespace moli::drivers {

// PwmHw 满足 moli::hal::PwmHardware concept（三相 PWM）
// ClockHw 满足 moli::hal::ClockHardware concept（时间源，构造时注入 SimpleFOC）
template <hal::PwmHardware PwmHw, hal::ClockHardware ClockHw,
          hal::OutputPin EnablePinHw = hal::NullPin,
          hal::InterruptPin FaultPinHw = hal::NullPin>
class BLDC {
  public:
    // 故障回调签名：self = 触发故障的 BLDC 实例（可访问全部公开 API），
    // context = setFaultHandler 时用户传入的任意数据。
    // 在 EXTI 中断上下文中执行，务必保持轻量（无锁、无堆分配、无长延时）。
    using FaultHandler = void (*)(BLDC *self, void *context);

    // @param pwm_a/pwm_b/pwm_c 三相 PWM 硬件实例
    // @param pole_pairs        电机极对数
    // @param phase_resistance  相电阻 [Ω]（可选，用于参数辨识）
    // @param kv                电机 KV 值 rpm/V（可选）
    // @param phase_inductance  相电感 [H]（可选）
    // @param en_sys            芯片使能引脚（默认 NullPin：不接线）
    // @param en_pwm            PWM 输出使能引脚（默认 NullPin：不接线）
    // @param fault             故障中断引脚（默认 NullPin：不接线）
    // @param en_active_high    使能引脚有效电平：true=高电平使能，
    //                          false=低电平使能（反相使能）
    BLDC(PwmHw pwm_a, PwmHw pwm_b, PwmHw pwm_c, int pole_pairs,
         float phase_resistance = NOT_SET, float kv = NOT_SET,
         float phase_inductance = NOT_SET, EnablePinHw en_sys = EnablePinHw{},
         EnablePinHw en_pwm = EnablePinHw{}, FaultPinHw fault = FaultPinHw{},
         bool en_active_high = true)
        : driver_(pwm_a, pwm_b, pwm_c),
          motor_(pole_pairs, phase_resistance, kv, phase_inductance),
          en_sys_(en_sys), en_pwm_(en_pwm), fault_(fault),
          en_active_high_(en_active_high) {
        // 注入平台时钟到 SimpleFOC 全局时间函数指针（见 arduino_compat.h）
        ::g_micros_fn = &ClockHw::now_us;
        ::g_delay_ms_fn = &ClockHw::delay_ms;
        ::g_delay_us_fn = &ClockHw::delay_us;
        motor_.linkDriver(&driver_);
    }

    // 初始化驱动硬件（三相 PWM 启动）。
    // @param fault_edge 故障触发沿：低有效驱动用 Falling（默认），
    //                   高有效驱动用 Rising。
    // 上电后功率级默认处于关断状态，需调用 enable() 才输出。
    void begin(hal::TriggerEdge fault_edge = hal::TriggerEdge::Falling) {
        driver_.init();
        setEnabled(false); // 安全上电：默认关断功率级
        fault_.attach_interrupt(fault_edge, &faultThunk, this);
        fault_.enable_interrupt();
    }

    // 使能功率级：两个使能引脚输出有效电平（极性由 en_active_high 决定）
    void enable() { setEnabled(true); }

    // 关断功率级：两个使能引脚输出无效电平（电机自由滑行/抱闸由硬件决定）
    void disable() { setEnabled(false); }

    // 当前使能状态（软件镜像）
    bool isEnabled() const { return enabled_; }

    // 故障锁存标志：EXTI 中断触发后置位，主循环可轮询
    bool isFaulted() const { return faulted_; }

    // 清除故障锁存并恢复使能状态。
    // 注意：硬件故障未消失时中断会立即再次触发（这是期望的保护行为）。
    void clearFault() {
        faulted_ = false;
        if (enabled_) {
            setEnabled(true);
        }
    }

    // 挂载自定义故障回调，完全接管默认动作（默认动作 = 立即拉低使能引脚）。
    // 传 handler = nullptr 恢复默认动作。context 原样透传给回调。
    void setFaultHandler(FaultHandler handler, void *context = nullptr) {
        fault_handler_ = handler;
        fault_ctx_ = context;
    }

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

    // 当前机械转速 [rad/s]
    float getVelocity() { return motor_.shaftVelocity(); }

    // 驱动器健康状态：未锁存故障时返回 true
    bool getDriverHealth() const { return !faulted_; }

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
    EnablePinHw en_sys_;         // 芯片使能
    EnablePinHw en_pwm_;         // PWM 输出使能
    FaultPinHw fault_;           // 故障，低电平代表出故障，中断中立即拉低使能
    bool en_active_high_ = true; // 使能引脚有效电平（true=高有效）
    bool enabled_ = false;       // 当前使能状态（软件镜像）
    bool faulted_ = false;       // 故障锁存标志（ISR 置位，主循环轮询）
    float target_angle_deg_ = 0.0f; // 最近一次 setAngle 的目标（度）

    // 用户可替换的故障回调（nullptr → 默认动作：拉低使能）
    FaultHandler fault_handler_ = nullptr;
    void *fault_ctx_ = nullptr;

    // EXTI 回调只能拿到 void*，这里恢复 BLDC* 再分发
    static void faultThunk(void *ctx) { static_cast<BLDC *>(ctx)->onFault(); }

    // 故障分发（ISR 上下文）：锁存标志 + 执行用户回调或默认动作
    void onFault() {
        faulted_ = true;
        if (fault_handler_ != nullptr) {
            fault_handler_(this, fault_ctx_); // 用户完全接管
            return;
        }
        setEnabled(false); // 默认动作：立即切断功率（纯 GPIO 写，ISR 安全）
    }

    // 按极性写两个使能引脚（NullPin 为 no-op）
    void setEnabled(bool en) {
        enabled_ = en;
        if (en_active_high_) {
            if (en) {
                en_sys_.set_high();
                en_pwm_.set_high();
            } else {
                en_sys_.set_low();
                en_pwm_.set_low();
            }
        } else {
            if (en) {
                en_sys_.set_low();
                en_pwm_.set_low();
            } else {
                en_sys_.set_high();
                en_pwm_.set_high();
            }
        }
    }

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
