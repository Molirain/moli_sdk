#pragma once
#include <cstdint>
#include <moli/hal/gpio.hpp>

namespace moli::port::stm32 {

// 普通 GPIO
class Gpio {
  public:
    Gpio(GPIO_TypeDef *port, uint16_t pin) : port_(port), pin_(pin) {};
    void write(bool state);
    void set_high();
    void set_low();
    void toggle();
    bool read() const;

  private:
    GPIO_TypeDef *port_;
    uint16_t pin_;
};

// 外部中断 GPIO（EXTI）
class ExtiPin {
  public:
    ExtiPin(GPIO_TypeDef *port, uint16_t pin);
    ~ExtiPin();

    // 挂载静态函数/普通函数
    void attach_interrupt(moli::hal::TriggerEdge edge,
                          moli::hal::ExtiCallback callback,
                          void *context = nullptr);

    // 模板糖：直接绑定类的成员函数（无需手动写 C 桥接包装）
    template <typename T, void (T::*MemberFn)()>
    void attach_member(T *instance, moli::hal::TriggerEdge edge) {
        attach_interrupt(
            edge, [](void *ctx) { (static_cast<T *>(ctx)->*MemberFn)(); },
            instance);
    }

    void detach_interrupt();
    void enable_interrupt();
    void disable_interrupt();

  private:
    GPIO_TypeDef *port_;
    uint16_t pin_;
    uint8_t pin_index_; // 0 ~ 15

    static uint8_t pin_to_index(uint16_t pin);
};

} // namespace moli::port::stm32