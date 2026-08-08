#pragma once

// clang-format off
#if defined(STM32H750xx)
    #include "target_hal/hal_stm32h750.hpp"
    using CurrentMcuPwm = moli::port::stm32::H750Pwm;
    // H750 拥有双精度 FPU，默认推荐 double
    using DefaultFloat = double; 
#elif defined(STM32G431xx)
    #include "target_hal/hal_stm32g431.hpp"
    using CurrentMcuPwm = moli::port::stm32::G431Pwm;
    // G431 只有单精度 FPU，默认推荐 float
    using DefaultFloat = float;
#else
    #error "Moli-SDK: Unsupported MCU Target!"
#endif
// clang-format on