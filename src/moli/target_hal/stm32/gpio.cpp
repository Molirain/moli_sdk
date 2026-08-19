#include "main.h"
#include <moli/target_hal/stm32/gpio.hpp>

// ============================================================================
// 兼容全系 STM32 的 EXTI 寄存器命名：
//   - G4 系：RTSR1/FTSR1/IMR1（位名 TR0/IM0）
//   - H7 系：RTSR1/FTSR1/IMR1（位名 RT0/MR0）
//   - F1/F4 等老系列：RTSR/FTSR/IMR（位名 TR0/MR0）
// 通过 CMSIS 位域宏是否存在来判定，而非平台宏（更精确、可移植）。
// ============================================================================
#if defined(EXTI_RTSR1_TR0) || defined(EXTI_RTSR1_RT0)
    #define MOLI_EXTI_RTSR (EXTI->RTSR1)
    #define MOLI_EXTI_FTSR (EXTI->FTSR1)
#else
    #define MOLI_EXTI_RTSR (EXTI->RTSR)
    #define MOLI_EXTI_FTSR (EXTI->FTSR)
#endif
#if defined(EXTI_IMR1_IM0) || defined(EXTI_IMR1_MR0)
    #define MOLI_EXTI_IMR (EXTI->IMR1)
#else
    #define MOLI_EXTI_IMR (EXTI->IMR)
#endif

namespace {
// 静态分配 16 个中断槽位，0 动态内存申请
struct ExtiSlot {
    moli::hal::ExtiCallback callback{nullptr};
    void *context{nullptr};
};

ExtiSlot g_exti_table[16];
} // namespace

namespace moli::port::stm32 {

void Gpio::write(bool state) {
    HAL_GPIO_WritePin(port_, pin_, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Gpio::set_high() { HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET); }

void Gpio::set_low() { HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET); }

void Gpio::toggle() { HAL_GPIO_TogglePin(port_, pin_); }

bool Gpio::read() const {
    return HAL_GPIO_ReadPin(port_, pin_) == GPIO_PIN_SET;
}

uint8_t ExtiPin::pin_to_index(uint16_t pin) {
    return static_cast<uint8_t>(
        __builtin_ctz(pin)); // 编译器内置硬件指令计算 pin 掩码对应的 0~15 索引
}

ExtiPin::ExtiPin(GPIO_TypeDef *port, uint16_t pin)
    : port_(port), pin_(pin), pin_index_(pin_to_index(pin)) {}

ExtiPin::~ExtiPin() { detach_interrupt(); }

void ExtiPin::attach_interrupt(moli::hal::TriggerEdge edge,
                               moli::hal::ExtiCallback callback,
                               void *context) {
    g_exti_table[pin_index_].callback = callback;
    g_exti_table[pin_index_].context = context;

    // 软件配置 EXTI 边沿（RTSR/FTSR），以传入的 edge 为唯一真源，
    // 覆盖 CubeMX 初始化时的触发沿配置，二者保持一致。
    switch (edge) {
    case moli::hal::TriggerEdge::Rising:
        MOLI_EXTI_RTSR |= pin_; // 上升沿
        MOLI_EXTI_FTSR &= ~pin_;
        break;
    case moli::hal::TriggerEdge::Falling:
        MOLI_EXTI_RTSR &= ~pin_;
        MOLI_EXTI_FTSR |= pin_; // 下降沿
        break;
    case moli::hal::TriggerEdge::Both:
        MOLI_EXTI_RTSR |= pin_;
        MOLI_EXTI_FTSR |= pin_; // 双边沿
        break;
    }
}

void ExtiPin::detach_interrupt() {
    disable_interrupt();
    g_exti_table[pin_index_] = {nullptr, nullptr};
}

void ExtiPin::enable_interrupt() {
    MOLI_EXTI_IMR |= pin_; // 使能 EXTI 中断线
}

void ExtiPin::disable_interrupt() {
    MOLI_EXTI_IMR &= ~pin_; // 屏蔽 EXTI 中断线
}

} // namespace moli::port::stm32

// 桥接 STM32 官方 HAL 库的中断回调
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    uint8_t idx = static_cast<uint8_t>(__builtin_ctz(GPIO_Pin));
    if (idx < 16 && g_exti_table[idx].callback != nullptr) {
        g_exti_table[idx].callback(g_exti_table[idx].context);
    }
}

// 宏仅在本编译单元使用，用完即弃，避免污染后续包含的头文件
#undef MOLI_EXTI_RTSR
#undef MOLI_EXTI_FTSR
#undef MOLI_EXTI_IMR