#pragma once

// clang-format off
// ============================================================================
// 平台识别（显式列出，不使用 else 兜底，以免误判未来新增平台）
// ----------------------------------------------------------------------------
// 检测顺序很重要：Arduino-ESP32 会同时定义 ESP32 与 ARDUINO 两个宏，
// 因此必须优先判断 ESP32。
// ============================================================================

#if defined(ESP32)
    #define MOLI_PLATFORM_ESP32 1
#elif defined(ARDUINO)
    #define MOLI_PLATFORM_ARDUINO 1
#elif defined(STM32)
    #define MOLI_PLATFORM_STM32 1
#else
    #error "Moli-SDK: Unsupported MCU Target!"
#endif

// ----------------------------------------------------------------------------
// PWM 端口层类型映射
// ----------------------------------------------------------------------------
#if defined(MOLI_PLATFORM_STM32)
    #include "target_hal/stm32/pwm.hpp"
    // STM32 全系列共用 HAL，外设 API 一致，统一映射到同一个 Pwm 类型
    using CurrentMcuPwm = moli::port::stm32::Pwm;
#endif
// clang-format on