#pragma once
#include <concepts>
#include <cstdint>

namespace moli::hal {

// 约束：底层必须提供微秒级单调时钟与阻塞延时能力。
// 采用静态接口（无实例），因为时间源是芯片级唯一的全局资源，
// 上层（如 arduino_compat 的 micros()/delay()）通过 CurrentMcuClock 别名调用。
template <typename T>
concept ClockHardware = requires(T clk, std::uint32_t v) {
    { T::now_us() } -> std::same_as<std::uint32_t>;
    { T::now_ms() } -> std::same_as<std::uint32_t>;
    { T::delay_us(v) } -> std::same_as<void>;
    { T::delay_ms(v) } -> std::same_as<void>;
};

} // namespace moli::hal
