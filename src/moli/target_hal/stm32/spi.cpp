#include "main.h" // 集中引入 HAL / CMSIS 头
#include <moli/target_hal/stm32/spi.hpp>

namespace moli::port::stm32 {

Spi::Spi(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, std::uint16_t cs_pin)
    : hspi_(hspi), cs_port_(cs_port), cs_pin_(cs_pin) {}

std::expected<void, hal::SpiError> Spi::begin() noexcept {
    if (hspi_ == nullptr || cs_port_ == nullptr || cs_pin_ == 0) {
        return std::unexpected(hal::SpiError::InvalidArgument);
    }

    // 软件片选 CS 初始化：推挽输出、默认高电平（未选中）
    GPIO_InitTypeDef gpio{};
    gpio.Pin = cs_pin_;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(cs_port_, &gpio);
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);

    return {};
}

void Spi::select() noexcept {
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
}

void Spi::deselect() noexcept {
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
}

std::uint8_t Spi::transfer(std::uint8_t byte) noexcept {
    std::uint8_t rx = 0;
    HAL_SPI_TransmitReceive(hspi_, &byte, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

std::uint16_t Spi::transfer16(std::uint16_t word) noexcept {
    const std::uint8_t tx[2] = {static_cast<std::uint8_t>(word >> 8),
                                static_cast<std::uint8_t>(word & 0xFF)};
    std::uint8_t rx[2] = {0, 0};
    HAL_SPI_TransmitReceive(hspi_, const_cast<std::uint8_t *>(tx), rx, 2,
                            HAL_MAX_DELAY);
    return static_cast<std::uint16_t>((rx[0] << 8) | rx[1]);
}

} // namespace moli::port::stm32
