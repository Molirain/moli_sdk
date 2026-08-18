#pragma once
#include <cstdint>
#include <moli/hal/clock.hpp>

namespace moli::port::stm32 {

// STM32 单调时钟实现：
// - now_us 基于 DWT 周期计数器（微秒分辨率，无中断依赖）
// - delay_us 基于 DWT 忙等（微秒级，不阻塞调度器）
// - delay_ms 基于 FreeRTOS vTaskDelay（让出 CPU，供长延时使用）
class Clock {
  public:
    // 初始化 DWT 周期计数器，幂等
    static void begin() noexcept;

    static std::uint32_t now_us() noexcept;
    static std::uint32_t now_ms() noexcept;

    static void delay_us(std::uint32_t us) noexcept;
    static void delay_ms(std::uint32_t ms) noexcept;

  private:
    static std::uint32_t cycles_per_us_;
};

static_assert(hal::ClockHardware<Clock>);

} // namespace moli::port::stm32
