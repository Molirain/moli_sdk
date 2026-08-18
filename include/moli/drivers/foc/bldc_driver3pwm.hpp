#pragma once
// ============================================================================
// BLDC 3-PWM 驱动适配：把 SimpleFOC 的 BLDCDriver 抽象接到 moli::hal::Pwm。
// 持有 3 个 PwmHardware 实例，setPwm 直接写占空比，不经过 hardware_api。
// ============================================================================
#include <algorithm>
#include <moli/hal/pwm.hpp>

#include "common/base_classes/BLDCDriver.h"
#include "common/defaults.h"
#include "common/foc_utils.h"

namespace moli::drivers::foc {

// PwmHw 满足 moli::hal::PwmHardware concept
template <hal::PwmHardware PwmHw> class BldcDriver3Pwm : public BLDCDriver {
  public:
    BldcDriver3Pwm(PwmHw pwm_a, PwmHw pwm_b, PwmHw pwm_c,
                   float voltage_power_supply = DEF_POWER_SUPPLY)
        : pwm_a_(pwm_a), pwm_b_(pwm_b), pwm_c_(pwm_c) {
        this->voltage_power_supply = voltage_power_supply;
        this->voltage_limit = voltage_power_supply;
        this->pwm_frequency = NOT_SET; // 频率由 TIM 外设决定，此处无意义
    }

    int init() override {
        pwm_a_.begin();
        pwm_b_.begin();
        pwm_c_.begin();
        setPwm(0.0f, 0.0f, 0.0f);
        initialized = true;
        return 1; // SimpleFOC 约定：非 -1 即成功
    }

    void enable() override { setPwm(0.0f, 0.0f, 0.0f); }

    void disable() override { setPwm(0.0f, 0.0f, 0.0f); }

    void setPwm(float Ua, float Ub, float Uc) override {
        // 电压限幅，计算占空比（[0,1]）
        Ua = _constrain(Ua, 0.0f, voltage_limit);
        Ub = _constrain(Ub, 0.0f, voltage_limit);
        Uc = _constrain(Uc, 0.0f, voltage_limit);

        dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
        dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
        dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

        pwm_a_.set_duty(dc_a);
        pwm_b_.set_duty(dc_b);
        pwm_c_.set_duty(dc_c);
    }

    // 3-PWM 驱动无独立相位使能引脚，空实现（保持抽象契约）
    void setPhaseState(PhaseState, PhaseState, PhaseState) override {}

  private:
    PwmHw pwm_a_;
    PwmHw pwm_b_;
    PwmHw pwm_c_;
};

} // namespace moli::drivers::foc
