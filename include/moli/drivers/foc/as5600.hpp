#pragma once
// ============================================================================
// AS5600 磁编码器适配（I2C）：继承 SimpleFOC 的 Sensor，读取 12bit 角度寄存器。
// 内部通过 moli::hal 的异步 I2C + wait() 实现同步读，不依赖 TwoWire/Wire。
// ============================================================================
#include <array>
#include <moli/hal/i2c.hpp>

#include "common/base_classes/Sensor.h"
#include "common/foc_utils.h"

namespace moli::drivers::foc {

// AS5600 寄存器配置（与 SimpleFOC 的 AS5600_I2C 一致）
inline constexpr std::uint8_t kAs5600Addr7bit = 0x36;
inline constexpr std::uint8_t kAs5600AngleReg = 0x0C;
inline constexpr int kAs5600BitResolution = 12;

// I2cHw 满足 moli::hal::AsyncI2cHardware + WaitableI2cTransferHandle
template <hal::AsyncI2cHardware I2cHw>
    requires hal::WaitableI2cTransferHandle<typename I2cHw::Transfer>
class As5600 : public Sensor {
  public:
    explicit As5600(I2cHw i2c)
        : i2c_(i2c), cpr_(_powtwo(kAs5600BitResolution)) {}

    // 初始化：开启 I2C 总线 + 传感器基线采样
    void init() {
        i2c_.begin();
        Sensor::init();
    }

    // 读取原始角度计数值
    int getRawCount() {
        const auto addr = hal::I2cAddress::from_7bit_unchecked(kAs5600Addr7bit);
        std::array<std::uint8_t, 2> rx{};

        const std::uint8_t reg = kAs5600AngleReg;
        auto tfer =
            i2c_.start_write_read(addr, std::span<const std::uint8_t>(&reg, 1),
                                  std::span<std::uint8_t>(rx));
        if (!tfer) {
            return -1; // 负值触发 Sensor::update 的错误信号
        }
        auto result = tfer->wait();
        if (!result) {
            return -1;
        }

        // 高字节高 4 位有效（12bit 左对齐），低字节全有效
        return static_cast<int>(((rx[0] & 0x0F) << 8) | rx[1]);
    }

  protected:
    float getSensorAngle() override {
        const int raw = getRawCount();
        if (raw < 0) {
            return -1.0f;
        }
        return (static_cast<float>(raw) / cpr_) * _2PI;
    }

  private:
    I2cHw i2c_;
    float cpr_;
};

} // namespace moli::drivers::foc
