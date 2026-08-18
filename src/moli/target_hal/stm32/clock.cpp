#include "main.h" // 集中引入 HAL / CMSIS 头
#include <moli/target_hal/stm32/clock.hpp>

#include "FreeRTOS.h"
#include "task.h"

namespace moli::port::stm32 {

std::uint32_t Clock::cycles_per_us_ = 0;

void Clock::begin() noexcept {
    if (cycles_per_us_ != 0) {
        return; // 已初始化
    }
    // 使能 DWT 跟踪并启动周期计数器
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    cycles_per_us_ = SystemCoreClock / 1000000U;
}

std::uint32_t Clock::now_us() noexcept {
    if (cycles_per_us_ == 0) {
        begin();
    }
    return DWT->CYCCNT / cycles_per_us_;
}

std::uint32_t Clock::now_ms() noexcept {
    return HAL_GetTick();
}

void Clock::delay_us(std::uint32_t us) noexcept {
    if (cycles_per_us_ == 0) {
        begin();
    }
    const std::uint32_t start = DWT->CYCCNT;
    const std::uint32_t ticks = us * cycles_per_us_;
    while ((DWT->CYCCNT - start) < ticks) {
    }
}

void Clock::delay_ms(std::uint32_t ms) noexcept {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

} // namespace moli::port::stm32
