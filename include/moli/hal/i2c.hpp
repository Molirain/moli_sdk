#pragma once

#include <chrono>
#include <concepts>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>

namespace moli::hal {

enum class I2cError : std::uint8_t {
    Busy,            // 硬件总线已被占用
    Timeout,         // 传输超时
    Nack,            // 地址或数据未收到应答
    BusError,        // 总线时序异常 (BERR)
    ArbitrationLost, // 总线仲裁丢失 (ARLO)
    Overrun,         // 溢出错误 (OVR)
    NotInitialized,  // 外设未使能或参数无效
    InvalidArgument, // 空缓冲区或非法地址
    HardwareError,   // 底层 DMA / 控制器硬件故障
    Cancelled        // 传输已被中止
};

// 强类型 7-bit / 10-bit I2C 地址
class I2cAddress {
  public:
    [[nodiscard]] static constexpr std::expected<I2cAddress, I2cError>
    from_7bit(std::uint8_t addr_7bit) noexcept {
        if (addr_7bit > 0x7F) {
            return std::unexpected(I2cError::InvalidArgument);
        }
        return I2cAddress{static_cast<std::uint16_t>(addr_7bit << 1)};
    }

    [[nodiscard]] static constexpr I2cAddress
    from_7bit_unchecked(std::uint8_t addr_7bit) noexcept {
        return I2cAddress{static_cast<std::uint16_t>((addr_7bit & 0x7F) << 1)};
    }

    [[nodiscard]] static constexpr I2cAddress
    from_raw_hal_unchecked(std::uint16_t hal_addr) noexcept {
        return I2cAddress{hal_addr};
    }

    [[nodiscard]] constexpr std::uint16_t raw() const noexcept {
        return hal_address_;
    }

  private:
    constexpr explicit I2cAddress(std::uint16_t hal_addr) noexcept
        : hal_address_(hal_addr) {}
    std::uint16_t hal_address_{0};
};

// 基础异步传输句柄契约 (Minimal Async Handle)
template <typename T>
concept I2cTransferHandle = requires(T handle) {
    {
        handle.poll()
    } -> std::same_as<std::optional<std::expected<void, I2cError>>>;
    { handle.cancel() } -> std::same_as<void>;
    { handle.is_done() } -> std::same_as<bool>;
};

// 支持 RTOS 挂起等待的句柄契约 (Waitable Handle)
template <typename T>
concept WaitableI2cTransferHandle =
    I2cTransferHandle<T> &&
    requires(T handle, std::chrono::milliseconds timeout) {
        { handle.wait(timeout) } -> std::same_as<std::expected<void, I2cError>>;
    };

// 基础异步 I2C 硬件契约
template <typename T>
concept AsyncI2cHardware =
    requires(T bus, I2cAddress address, std::span<const std::uint8_t> tx,
             std::span<std::uint8_t> rx) {
        typename T::Transfer;
        requires I2cTransferHandle<typename T::Transfer>;

        { bus.begin() } -> std::same_as<std::expected<void, I2cError>>;

        {
            bus.start_write(address, tx)
        } -> std::same_as<std::expected<typename T::Transfer, I2cError>>;

        {
            bus.start_read(address, rx)
        } -> std::same_as<std::expected<typename T::Transfer, I2cError>>;

        {
            bus.start_write_read(address, tx, rx)
        } -> std::same_as<std::expected<typename T::Transfer, I2cError>>;
    };

// 独立可选能力：总线硬件级恢复契约
template <typename T>
concept I2cBusRecovery = requires(T bus) {
    { bus.recover_bus() } -> std::same_as<bool>;
};

} // namespace moli::hal