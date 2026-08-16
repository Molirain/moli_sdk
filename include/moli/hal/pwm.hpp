#pragma once
#include <concepts>

namespace moli::hal {

// 约束：底层硬件必须提供 begin() 以及能接收 0.0 ~ 1.0 占空比的 set_duty()
template <typename T>
concept PwmHardware = requires(T hw, float duty) {
    { hw.begin() } -> std::same_as<void>;
    { hw.set_duty(duty) } -> std::same_as<void>;
    { hw.get_duty() } -> std::same_as<float>;
};

} // namespace moli::hal