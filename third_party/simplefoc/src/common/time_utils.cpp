#include "time_utils.h"

// ============================================================================
// SimpleFOC 的「硬件特定」时间实现。
// 通过 arduino_compat.h 声明的全局函数指针调用平台时钟，
// 由上层 BLDC 模板在构造时注入（concept 约束 moli::hal::ClockHardware）。
// ============================================================================

void _delay(unsigned long ms) {
    if (g_delay_ms_fn) {
        g_delay_ms_fn(static_cast<uint32_t>(ms));
    }
}

void _delayMicroseconds(unsigned long us) {
    if (g_delay_us_fn) {
        g_delay_us_fn(static_cast<uint32_t>(us));
    }
}

unsigned long _micros() {
    if (g_micros_fn) {
        return g_micros_fn();
    }
    return 0;
}
