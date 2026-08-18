#pragma once
// ============================================================================
// Arduino 兼容层：让 SimpleFOC（Arduino 风格源码）在 Moli-SDK 上零改动编译。
// 本文件只做「符号垫片」——纯类型定义 + 全局函数指针，不依赖任何具体平台。
//   - 时间 -> 通过 g_micros_fn / g_delay_ms_fn / g_delay_us_fn 函数指针注入，
//             由上层 BLDC 模板在构造时填入平台时钟实现（concept 约束）。
//   - GPIO -> 暂为占位（moli::hal 尚无 GPIO 抽象，当前 FOC 路径未用到）
// 注意：min/max/abs 是 SimpleFOC 直接依赖的宏（BLDCMotor/Sensor/foc_utils
// 调用）， 保留为函数式宏；调用方在混用 std:: 版本时需用 (std::min)
// 形式规避展开。
// ============================================================================
#include <cstddef>
#include <cstdint>
#include <math.h> // fabs / fabsf / log（SimpleFOC 多处使用，原 Arduino.h 会带入）

// ---- Arduino 基础类型 ----
typedef std::uint16_t word;
typedef std::uint8_t byte;

// ---- 宏（SimpleFOC 直接调用，必须保留）----
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))

// ---- 时间注入点 ----
// SimpleFOC 的 _micros()/_delay()/_delayMicroseconds() 是全局符号，
// 通过函数指针由上层 BLDC 模板在构造时注入平台时钟实现（concept 约束
// ClockHardware）。 兼容层因此不依赖任何具体平台，时钟类型由 BLDC 的 ClockHw
// 模板参数决定。
using MicrosFn = uint32_t (*)();
using DelayMsFn = void (*)(uint32_t);
using DelayUsFn = void (*)(uint32_t);

inline MicrosFn g_micros_fn = nullptr;
inline DelayMsFn g_delay_ms_fn = nullptr;
inline DelayUsFn g_delay_us_fn = nullptr;

// ---- GPIO（占位：moli::hal 尚无 GPIO 抽象，当前编译单元未使用）----
#define LOW 0
#define HIGH 1
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return LOW; }

// ---- 闪存字符串（AVR 概念）。不完整类型，用于与 const char* 区分重载 ----
struct __FlashStringHelper;
#define F(x) (reinterpret_cast<const __FlashStringHelper *>(x))

// ---- 字符串拼接辅助（SimpleFOCDebug 调用 .c_str()）----
class StringSumHelper {
  public:
    explicit StringSumHelper(const char *s = "") : s_(s) {}
    const char *c_str() const { return s_; }

  private:
    const char *s_;
};

// ---- 输出抽象（SimpleFOC 监控 + 调试打印用到的最小重载集）----
class Print {
  public:
    virtual std::size_t write(uint8_t) { return 0; }

    virtual void print(char) {}
    virtual void print(const char *) {}
    virtual void print(int) {}
    virtual void print(float) {}
    virtual void print(float, int) {} // 带小数位
    virtual void print(const __FlashStringHelper *s) {
        print(reinterpret_cast<const char *>(s));
    }

    virtual void println() { print("\r\n"); }
    virtual void println(char c) {
        print(c);
        print("\r\n");
    }
    virtual void println(const char *s) {
        print(s);
        print("\r\n");
    }
    virtual void println(int v) {
        print(v);
        print("\r\n");
    }
    virtual void println(float v) {
        print(v);
        print("\r\n");
    }
    virtual void println(const __FlashStringHelper *s) {
        println(reinterpret_cast<const char *>(s));
    }
};
inline Print Serial;
// ---- arduino_compat.h 结束 ----