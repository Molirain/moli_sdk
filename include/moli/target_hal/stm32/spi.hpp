#pragma once
#include <cstdint>
#include <expected>
#include <moli/hal/spi.hpp>

// 前置声明 STM32 HAL 类型，避免头文件引入完整 HAL
struct __SPI_HandleTypeDef;
typedef struct __SPI_HandleTypeDef SPI_HandleTypeDef;
// 注意：GPIO_TypeDef 在 HAL 中是匿名 struct（无 tag），无法安全前置声明，
// 直接依赖使用方先 include main.h（与 i2c.hpp 的 GPIO_TypeDef 用法一致）。

namespace moli::port::stm32 {

// STM32 SPI 外设封装：负责 CS 引脚管理与同步全双工收发。
// 外设本身（SCK/MOSI/MISO 复用与时钟）由用户在 MX_SPI_Init 中完成，
// 本类只补充软件片选 CS 的 GPIO 初始化，遵循 Pwm 的「MX 负责外设、驱动负责启动」约定。
class Spi {
  public:
    Spi(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, std::uint16_t cs_pin);

    std::expected<void, hal::SpiError> begin() noexcept;

    void select() noexcept;   // CS 拉低，开始一个帧
    void deselect() noexcept; // CS 拉高，结束一个帧

    std::uint8_t transfer(std::uint8_t byte) noexcept;
    std::uint16_t transfer16(std::uint16_t word) noexcept;

  private:
    SPI_HandleTypeDef *hspi_;
    GPIO_TypeDef *cs_port_;
    std::uint16_t cs_pin_;
};

static_assert(hal::SpiHardware<Spi>);

} // namespace moli::port::stm32
