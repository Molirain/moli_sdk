#pragma once
#include <concepts>

/*
 * GPIO 支持输入/输出/混合以及中断
 */

namespace moli::hal {

template <typename T>
concept OutputPin = requires(T pin, bool state) {
    { pin.write(state) } -> std::same_as<void>;
    { pin.set_high() } -> std::same_as<void>;
    { pin.set_low() } -> std::same_as<void>;
};

template <typename T>
concept InputPin = requires(T pin) {
    { pin.read() } -> std::same_as<bool>;
};

template <typename T>
concept GpioPin = OutputPin<T> && InputPin<T>;

// 触发类型
enum class TriggerEdge : uint8_t {
    Rising,  // 上升沿
    Falling, // 下降沿
    Both     // 双边沿（上升 + 下降）
};

// 中断回调函数指针类型
using ExtiCallback = void (*)(void *context);

template <typename T>
concept InterruptPin =
    requires(T pin, TriggerEdge edge, ExtiCallback cb, void *ctx) {
        { pin.attach_interrupt(edge, cb, ctx) } -> std::same_as<void>;
        { pin.detach_interrupt() } -> std::same_as<void>;
        { pin.enable_interrupt() } -> std::same_as<void>;
        { pin.disable_interrupt() } -> std::same_as<void>;
    };

// 空引脚（Null Object 模式）：不连接任何实际 GPIO，用于模板参数占位。
// 同时满足 OutputPin / InputPin / InterruptPin 三个 concept。
struct NullPin {
    void write(bool) {}
    void set_high() {}
    void set_low() {}
    void toggle() {}
    bool read() const { return false; }
    void attach_interrupt(TriggerEdge, ExtiCallback, void *) {}
    void detach_interrupt() {}
    void enable_interrupt() {}
    void disable_interrupt() {}
};

} // namespace moli::hal