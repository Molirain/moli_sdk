#pragma once
// ============================================================================
// AS5047D 磁编码器适配（SPI）：继承 SimpleFOC 的 Sensor，读取 14bit
// 角度寄存器。 内部通过
// moli::hal::Spi（select/transfer16/deselect）实现，不依赖 SPIClass。
// ============================================================================
#include <moli/hal/spi.hpp>

#include "common/base_classes/Sensor.h"
#include "common/foc_utils.h"

namespace moli::drivers::foc {

// AS5047D 读取命令（14bit 角度 + R/W 位 + 奇偶校验位）
inline constexpr std::uint16_t kAs5047dReadCmd = 0x7FFE; // 0b0111111111111110
inline constexpr int kAs5047dBitResolution = 14;
inline constexpr int kAs5047dDataStartBit = 13;

// SpiHw 满足 moli::hal::SpiHardware
template <hal::SpiHardware SpiHw> class As5047d : public Sensor {
  public:
    explicit As5047d(SpiHw spi)
        : spi_(spi), cpr_(_powtwo(kAs5047dBitResolution)) {}

    void init() {
        spi_.begin();
        Sensor::init();
    }

    int getRawCount() {
        // 16bit 全双工：发送读命令，同时收到上一帧数据（AS5047D 特性）
        spi_.select();
        const std::uint16_t value = spi_.transfer16(kAs5047dReadCmd);
        spi_.deselect();

        // 去除命令位（bit15 奇偶 + bit14 R/W），保留 14bit 角度
        const std::uint16_t angle = value & 0x3FFF;
        return static_cast<int>(angle);
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
    SpiHw spi_;
    float cpr_;
};

} // namespace moli::drivers::foc
