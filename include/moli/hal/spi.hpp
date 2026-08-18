#pragma once
#include <concepts>
#include <cstdint>
#include <expected>

namespace moli::hal {

enum class SpiError : std::uint8_t {
    Timeout,         // 传输超时
    BusError,        // 总线时序异常
    NotInitialized,  // 外设未使能或参数无效
    InvalidArgument, // 空缓冲区或非法参数
    HardwareError    // 底层硬件故障
};

// 约束：底层必须提供同步全双工 SPI 收发能力。
// 同步语义对齐 PwmHardware：磁编码器读寄存器是微秒级阻塞操作，
// 无需像 I2c 那样引入异步 Transfer 句柄与 RTOS 集成。
// CS 由 SPI 外设自身管理（它知道自己的片选引脚），暴露 select/deselect。
template <typename T>
concept SpiHardware = requires(T hw, std::uint8_t b, std::uint16_t w) {
    { hw.begin() } -> std::same_as<std::expected<void, SpiError>>;
    { hw.select() } -> std::same_as<void>;
    { hw.deselect() } -> std::same_as<void>;
    { hw.transfer(b) } -> std::same_as<std::uint8_t>;
    { hw.transfer16(w) } -> std::same_as<std::uint16_t>;
};

} // namespace moli::hal
