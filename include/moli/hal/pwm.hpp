#pragma once
#include <concepts>

namespace moli::hal {

// 约束：底层硬件必须提供 begin() 以及能接收 0.0 ~ 1.0 占空比的 set_duty()
template <typename T>
concept DualPwmHardware = requires(T hw, float duty1, float duty2) {
    { hw.begin() } -> std::same_as<void>;
    { hw.set_duty(duty1, duty2) } -> std::same_as<void>;
};

} // namespace moli::hal