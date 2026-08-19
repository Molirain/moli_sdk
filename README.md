# Moli-SDK

我的嵌入式驱动积累。基于 C++23、concept 约束的零成本抽象，跨平台嵌入式外设驱动库。

## 设计理念

```
┌─────────────────────────────────────────────────────┐
│  drivers/  （驱动层，header-only 模板，面向用户）      │
│    dc_motor.hpp  servo.hpp  BLDC.hpp                │
│    foc/ as5600.hpp as5047d.hpp bldc_driver3pwm.hpp  │
├─────────────────────────────────────────────────────┤
│  hal/      （抽象层，concept 约束，与平台无关）        │
│    pwm.hpp  i2c.hpp  spi.hpp  clock.hpp  gpio.hpp   │
├─────────────────────────────────────────────────────┤
│  target_hal/  （平台实现层，具体外设封装）            │
│    stm32/  pwm  i2c  spi  clock  gpio               │
└─────────────────────────────────────────────────────┘
```

- **驱动层**：header-only 模板，按值持有硬件对象，通过 concept 约束模板参数，零运行时开销。
- **抽象层**：用 C++20 concept 定义「某个外设长什么样」（如 `PwmHardware`），不关心具体芯片。
- **平台实现层**：实现抽象层要求的接口，末尾用 `static_assert` 自我校验符合 concept。
- **平台选择**：`moli_target.hpp` 根据编译宏（`STM32` / `ESP32` / `ARDUINO`）映射出 `CurrentMcuPwm`、`CurrentMcuI2c`、`CurrentMcuSpi`、`CurrentMcuClock`、`CurrentMcuGpio`、`CurrentMcuExtiPin` 别名。

> 依赖 C++23，编译时需 `-std=gnu++23`。

## 目前支持

- **直流电机**（A4950 式双 PWM 驱动）
- **舵机**（50Hz 标准 PWM 舵机）
- **FOC 无刷电机**（基于 [SimpleFOC](https://github.com/simplefoc/Arduino-FOC)，已裁剪移植；支持功率级使能 + 故障中断保护）
- **磁编码器**
  - AS5600（I2C）
  - AS5047D（SPI）
- **GPIO / 外部中断（EXTI）**（输出、输入、边沿中断，全系 STM32 寄存器兼容）

---

## 目录结构

```
moli_sdk/
├── library.json                 # PlatformIO 库清单（含 SimpleFOC 编译脚本）
├── foc_build.py                 # SimpleFOC 源码编译（库构建阶段自动执行）
├── include/moli/
│   ├── moli_target.hpp          # 平台识别 + 类型别名映射
│   ├── hal/                     # 抽象层（concept）
│   │   ├── pwm.hpp              #   PwmHardware
│   │   ├── i2c.hpp              #   AsyncI2cHardware / I2cError / I2cAddress
│   │   ├── spi.hpp              #   SpiHardware / SpiError
│   │   ├── clock.hpp            #   ClockHardware
│   │   └── gpio.hpp             #   OutputPin / InputPin / GpioPin / InterruptPin
│   ├── target_hal/stm32/        # STM32 平台实现
│   │   ├── pwm.hpp / i2c.hpp / spi.hpp / clock.hpp / gpio.hpp
│   └── drivers/                 # 驱动层（header-only）
│       ├── dc_motor.hpp
│       ├── servo.hpp
│       ├── BLDC.hpp             # FOC 无刷电机薄封装（含功率级/故障管理）
│       └── foc/
│           ├── arduino_compat.h # SimpleFOC 兼容垫片
│           ├── bldc_driver3pwm.hpp
│           ├── as5600.hpp
│           └── as5047d.hpp
├── src/moli/target_hal/stm32/   # 平台实现源码
│   ├── pwm.cpp / i2c.cpp / spi.cpp / clock.cpp / gpio.cpp
└── third_party/simplefoc/       # 裁剪后的 SimpleFOC 算法核心
```

---

## 快速开始（PlatformIO）

在项目的 `platformio.ini` 引入本库：

```ini
[env:your_board]
platform = ststm32
framework = stm32cube
build_flags = -std=gnu++23 -DSTM32
lib_deps =
    symlink://D:/githubProjects/moli_sdk
```

> 说明：
> - `-DSTM32` 触发 `moli_target.hpp` 选择 STM32 平台实现。
> - `clock.cpp` 的 `delay_ms()` 依赖 FreeRTOS 的 `vTaskDelay`，请确保工程包含 FreeRTOS（或改用它实现）。
> - SimpleFOC 的源码会在库构建阶段被 `foc_build.py` 自动编译，无需手动处理。

---

## FOC 无刷电机（BLDC）

### 依赖

- 3 路 PWM 输出（三相桥驱动，如半桥 / H 桥驱动芯片）
- 一个磁编码器：AS5600（I2C）或 AS5047D（SPI）
- （可选）使能引脚：`en_sys`（芯片使能）+ `en_pwm`（PWM 输出使能），不接则传空
- （可选）故障引脚：`fault`（驱动芯片故障输出，如 NFLP），不接则传空
- 已初始化的 STM32 HAL 外设句柄（`TIM_HandleTypeDef`、`I2C_HandleTypeDef` / `SPI_HandleTypeDef`）

### 头文件

```cpp
#include "main.h"                       // HAL 外设句柄
#include <moli/moli_target.hpp>          // CurrentMcuPwm 等别名
#include <moli/drivers/BLDC.hpp>         // BLDC 薄封装
#include <moli/drivers/foc/as5600.hpp>   // 或 as5047d.hpp
```

### 实例化

`BLDC` 共 4 个模板参数，均由 concept 约束：

| 模板参数 | 约束 | 说明 |
|---------|------|------|
| `PwmHw` | `PwmHardware` | 三相 PWM 硬件 |
| `ClockHw` | `ClockHardware` | 时间源（构造时注入 SimpleFOC 时钟） |
| `EnablePinHw` | `OutputPin` | 使能引脚类型（默认 `hal::NullPin`，en_sys/en_pwm 共用） |
| `FaultPinHw` | `InterruptPin` | 故障中断引脚类型（默认 `hal::NullPin`） |

```cpp
// 最简：不接使能/故障引脚
using BLDC = moli::drivers::BLDC<CurrentMcuPwm, CurrentMcuClock>;
using Encoder = moli::drivers::foc::As5600<CurrentMcuI2c>;   // I2C 编码器
// using Encoder = moli::drivers::foc::As5047d<CurrentMcuSpi>; // SPI 编码器

// 三相 PWM 挂在 htim1 的 CH1/CH2/CH3
BLDC motor(
    CurrentMcuPwm(&htim1, TIM_CHANNEL_1),
    CurrentMcuPwm(&htim1, TIM_CHANNEL_2),
    CurrentMcuPwm(&htim1, TIM_CHANNEL_3),
    7,          // 极对数 pole_pairs
    0.5f,       // 相电阻 R [Ω]（可选，用于参数辨识）
    100.0f,     // KV 值 rpm/V（可选）
    0.0001f     // 相电感 L [H]（可选）
);

// 编码器：AS5600 挂 hi2c1（I2C 总线为 RAII 资源，需持久对象，encoder 持引用）
CurrentMcuI2c i2c(&hi2c1);
Encoder encoder(i2c);
// AS5047D 挂 hspi1，片选 GPIOC Pin4：
// Encoder encoder(CurrentMcuSpi(&hspi1, GPIOC, GPIO_PIN_4));
```

接使能/故障引脚时，类型与构造参数同步扩展：

```cpp
using BLDC = moli::drivers::BLDC<CurrentMcuPwm, CurrentMcuClock,
                                 CurrentMcuGpio, CurrentMcuExtiPin>;

BLDC motor(
    CurrentMcuPwm(&htim1, TIM_CHANNEL_1),
    CurrentMcuPwm(&htim1, TIM_CHANNEL_2),
    CurrentMcuPwm(&htim1, TIM_CHANNEL_3),
    7,
    /*phase_resistance=*/NOT_SET, /*kv=*/NOT_SET, /*phase_inductance=*/NOT_SET,
    CurrentMcuGpio(GPIOB, GPIO_PIN_1),   // en_sys：芯片使能
    CurrentMcuGpio(GPIOB, GPIO_PIN_13),  // en_pwm：PWM 输出使能
    CurrentMcuExtiPin(GPIOB, GPIO_PIN_12), // fault：故障中断（低有效）
    /*en_active_high=*/true              // 使能引脚极性：true=高有效（默认）
);
```

> 引脚未接线时全部默认 `hal::NullPin`（Null Object），对应操作自动为空，无需任何 `#ifdef`。

### 初始化

```cpp
motor.begin();          // 启动三相 PWM + 挂载故障中断（默认下降沿）
encoder.init();         // 初始化编码器（I2C/SPI 总线 + 基线采样）
motor.linkSensor(&encoder);

motor.initFOC();        // 传感器零点对齐（仅需一次，会转动电机）
motor.enable();         // 上电功率级（使能引脚输出有效电平）
```

> `begin()` 后功率级**默认处于关断状态**（安全上电），对齐完成后需显式 `enable()` 才输出。

### 控制模式

通过底层 `BLDCMotor` 配置控制模式（`torque` / `velocity` / `angle`）：

```cpp
// 位置控制（角度闭环）
motor.motor().controller = MotionControlType::angle;
motor.motor().target = 1.0f;   // 目标角 1 rad

// 速度控制
// motor.motor().controller = MotionControlType::velocity;
// motor.motor().target = 10.0f;  // 目标角速度 10 rad/s

// 力矩控制（电压环）
// motor.motor().controller = MotionControlType::torque;
// motor.motor().target = 2.0f;   // 目标电压 2V
```

### 角度环（0~360° 位置伺服）

`BLDC` 内置了角度环封装，直接用「度」控制，自动归一化并走最短路径：

```cpp
// 把上电位置标定为 0°
motor.setZero();

// 转到任意角度（单位：度，自动最短路径）
motor.setAngle(90.0f);    // 转到 90°
motor.setAngle(350.0f);   // 从 90° 正转 260° 到 350°（不会反转 340°）

// 读取当前角度（度，[0, 360)）
float angle = motor.getAngle();

// 调参
motor.setAngleP(20.0f);            // 角度环 P 增益（越大越"硬"，过大易振荡）
motor.setVelocityLimit(20.0f);     // 角度环输出的最大角速度 [rad/s]
```

对应的方法一览：

| 方法 | 说明 |
|------|------|
| `setAngle(float deg)` | 设置目标角（度），自动归一化 `[0,360)` + 最短路径 + 切到角度环 |
| `getAngle()` | 当前机械角（度），归一化到 `[0,360)` |
| `setZero()` | 将当前位置标定为 0° |
| `setAngleP(float p)` | 角度环 P 增益 |
| `setVelocityLimit(float rad_s)` | 角度环输出速度限幅 |

> 注意：`setAngle()` 依赖 `initFOC()` 已完成，请按「实例化 → 初始化 → 控制」的顺序调用。

### 功率级管理与故障保护

`BLDC` 内置功率级使能控制与故障中断处理（模板参数接 `EnablePinHw` / `FaultPinHw` 时生效，不接则为空操作）：

| 方法 | 说明 |
|------|------|
| `enable()` / `disable()` | 使能 / 关断功率级（两个使能引脚输出有效 / 无效电平） |
| `isEnabled()` | 当前使能状态（软件镜像） |
| `isFaulted()` | 故障锁存标志（中断触发后置位，主循环可轮询） |
| `clearFault()` | 清除故障锁存并恢复使能状态 |
| `setFaultHandler(FaultHandler, ctx)` | 挂载自定义故障回调，完全接管默认动作 |

**故障中断行为**（`begin(fault_edge = TriggerEdge::Falling)` 挂载）：

1. 故障引脚触发（低有效驱动默认 `Falling`，高有效传 `Rising`）→ EXTI 中断
2. 默认动作：**立即拉低两个使能引脚**切断功率（纯 GPIO 写，ISR 安全）+ 锁存 `isFaulted()`
3. 可用 `setFaultHandler()` 替换为自定义处理——回调签名 `void(*)(BLDC *self, void *context)`，在 ISR 上下文执行，务必保持轻量（无锁、无堆分配、无长延时）

```cpp
// 自定义故障处理（完全接管默认动作）
void onFault(BldcMotor *self, void *ctx) {
    self->disable();           // 切断功率
    // ... 上报 / 记录 / 点亮指示灯
}
motor.setFaultHandler(&onFault);

// 主循环轮询故障
if (motor.isFaulted()) {
    // 处理完成后清除，硬件故障未消失会立即再次触发（预期的保护行为）
    motor.clearFault();
}
```

> 注意：`HAL_GPIO_EXTI_Callback` 为全工程共享的中断桥（`gpio.cpp`），同一引脚不可被多个 `ExtiPin` 重复挂载。

### 运行循环

SimpleFOC 的标准双环结构：`loopFOC()` 是**电流环**（需高频调用，1~20kHz），`move()` 是**运动控制环**（可低频，如 1kHz）。

```cpp
// 放在定时器中断 / 高优先级 RTOS 任务里
motor.loopFOC();   // 读编码器 + FOC 换相 + 输出三相电压
motor.move();      // 按 controller 模式执行运动控制
```

典型 RTOS 任务示例：

```cpp
void StartMotionTask(void *argument) {
    motor.begin();
    encoder.init();
    motor.linkSensor(&encoder);
    motor.initFOC();

    motor.motor().controller = MotionControlType::velocity;
    motor.motor().target = 10.0f;

    for (;;) {
        motor.loopFOC();
        motor.move();
        vTaskDelay(pdMS_TO_TICKS(1));   // 约 1kHz 运动环
    }
}
```

> 若需要更高的 `loopFOC` 频率，建议把 `loopFOC()` 放进定时器中断回调，`move()` 放任务里。

### 调参

SimpleFOC 的 PID 参数通过 `motor.motor()` 暴露：

```cpp
// 速度环 PID
motor.motor().PID_velocity.P = 0.5f;
motor.motor().PID_velocity.I = 10.0f;
motor.motor().PID_velocity.D = 0.0f;

// 角度环 P（级联：角度环 → 速度环）
motor.motor().P_angle.P = 20.0f;

// 限幅
motor.motor().voltage_limit = 12.0f;    // 电压限幅 [V]
motor.motor().velocity_limit = 20.0f;   // 角速度限幅 [rad/s]
```

---

## GPIO 与外部中断（EXTI）

抽象层 `hal/gpio.hpp` 定义 GPIO concept 与中断回调：

```cpp
template <typename T>
concept OutputPin = requires(T pin, bool state) {   // write / set_high / set_low
    { pin.write(state) } -> std::same_as<void>;
    { pin.set_high() } -> std::same_as<void>;
    { pin.set_low() } -> std::same_as<void>;
};

template <typename T>
concept InterruptPin = requires(T pin, TriggerEdge edge,
                                ExtiCallback cb, void *ctx) {
    { pin.attach_interrupt(edge, cb, ctx) } -> std::same_as<void>;
    { pin.detach_interrupt() } -> std::same_as<void>;
    { pin.enable_interrupt() } -> std::same_as<void>;
    { pin.disable_interrupt() } -> std::same_as<void>;
};
```

- `TriggerEdge`：`Rising` / `Falling` / `Both` 三档触发沿。
- `ExtiCallback`：`void (*)(void *context)`——回调只能拿 `void*`，驱动层内部用静态 thunk 恢复 `this` 再分发（见 `BLDC` 的故障处理）。
- `NullPin`：Null Object 模式，同时满足全部 GPIO concept，未接线的模板参数直接用它占位。

### STM32 实现（`ExtiPin`）

```cpp
#include <moli/target_hal/stm32/gpio.hpp>

moli::hal::stm32::ExtiPin fault(GPIOB, GPIO_PIN_12);

// 挂载普通函数（回调签名 void(*)(void*)）
fault.attach_interrupt(moli::hal::TriggerEdge::Falling,
                       [](void *ctx) { /* ... */ }, nullptr);

// 模板糖：直接绑成员函数，无需手写 C 桥接
fault.attach_member<MyClass, &MyClass::onFault>(&obj,
                                                moli::hal::TriggerEdge::Falling);

fault.enable_interrupt();   // 使能 EXTI 中断线
fault.disable_interrupt();  // 屏蔽
fault.detach_interrupt();   // 卸载回调 + 屏蔽 + 清槽位
```

要点：

- **边沿以软件传入为准**：`attach_interrupt(edge, ...)` 会直接写 EXTI 边沿寄存器（`RTSR1/FTSR1`），覆盖 CubeMX 初始化时的触发沿——`edge` 是唯一真源，改沿无需重新生成 CubeMX 代码。
- **全系 STM32 寄存器兼容**：`gpio.cpp` 顶部通过 CMSIS 位域宏存在性自动选择 `RTSR1/FTSR1/IMR1`（G4/H7）或 `RTSR/FTSR/IMR`（F1/F4 等老系列），宏在文件末尾 `#undef`，不泄漏。
- **静态槽位表**：16 个 EXTI 线静态分配回调槽，零动态内存；`HAL_GPIO_EXTI_Callback`（`stm32g4xx_it.c` 中调用）按引脚索引分发。
- 析构自动 `detach_interrupt()`，防止对象销毁后中断回调悬垂。

---

## 磁编码器

### AS5600（I2C）

```cpp
#include <moli/drivers/foc/as5600.hpp>

using Encoder = moli::drivers::foc::As5600<CurrentMcuI2c>;

// I2c 是 RAII 总线资源（不可拷贝），需作为持久对象；encoder 持有其引用
CurrentMcuI2c i2c(&hi2c1);
Encoder encoder(i2c);

encoder.init();
float angle = encoder.getAngle();   // 机械角（含多圈累计），单位 rad
```

- 12 位分辨率，I2C 地址 `0x36`。
- 内部通过异步 I2C + `wait()` 实现同步读，兼容 `moli::hal::AsyncI2cHardware`。

### AS5047D（SPI）

```cpp
#include <moli/drivers/foc/as5047d.hpp>

using Encoder = moli::drivers::foc::As5047d<CurrentMcuSpi>;
Encoder encoder(CurrentMcuSpi(&hspi1, GPIOC, GPIO_PIN_4));  // CS = PC4

encoder.init();
float angle = encoder.getAngle();   // 单位 rad
```

- 14 位分辨率，SPI 模式 1。
- 片选由 `Spi` 外设封装管理（`select`/`deselect`），无需手动拉 GPIO。

---

## 直流电机（DC Motor）

A4950 式双 PWM 驱动（一路控制正转、一路控制反转）：

```cpp
#include <moli/drivers/dc_motor.hpp>

using Motor = moli::drivers::DCMotor<CurrentMcuPwm>;

// pwm_a 正向，pwm_b 反向，死区 15（0~100）
Motor motor(CurrentMcuPwm(&htim1, TIM_CHANNEL_1),
            CurrentMcuPwm(&htim1, TIM_CHANNEL_2), 15.0f);

motor.begin();
motor.setSpeed(50.0f);   // 50% 正转
motor.setSpeed(-50.0f);  // 50% 反转
motor.setSpeed(0.0f);    // 停止
```

---

## 舵机（Servo）

标准 50Hz PWM 舵机：

```cpp
#include <moli/drivers/servo.hpp>

using Servo = moli::drivers::Servo<CurrentMcuPwm>;

Servo servo(CurrentMcuPwm(&htim3, TIM_CHANNEL_1));

servo.begin();
servo.setAngle(90.0f);   // 转到 90°
servo.center();          // 回中 0°
```

- 角度范围 -135° ~ +135°，居中 0° 对应 1.5ms 脉宽。

---

## 抽象层约定（扩展新平台）

每个外设都遵循「concept 定义 → 平台实现 → 别名映射」三步：

1. **`hal/xxx.hpp`** 定义 concept，例如：

```cpp
template <typename T>
concept PwmHardware = requires(T hw, float duty) {
    { hw.begin() } -> std::same_as<void>;
    { hw.set_duty(duty) } -> std::same_as<void>;
    { hw.get_duty() } -> std::same_as<float>;
};
```

2. **`target_hal/xxx/yyy.hpp` + `.cpp`** 实现类，末尾自校验：

```cpp
class Pwm { ... };
static_assert(hal::PwmHardware<Pwm>);
```

3. **`moli_target.hpp`** 按平台映射别名：

```cpp
#if defined(STM32)
    using CurrentMcuPwm = moli::port::stm32::Pwm;
#endif
```

要支持新芯片，只需新增 `target_hal/<platform>/` 实现并在 `moli_target.hpp` 加一个分支，驱动层与抽象层零改动。