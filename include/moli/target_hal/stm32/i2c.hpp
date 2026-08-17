#pragma once

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <expected>
#include <moli/hal/i2c.hpp>
#include <optional>
#include <span>

struct __I2C_HandleTypeDef;
typedef struct __I2C_HandleTypeDef I2C_HandleTypeDef;

namespace moli::port::stm32 {

class I2c;

// 事务控制块 (TCB)：封装单个事务的完整上下文与生命周期
struct TransferControlBlock {
    enum class Status : std::uint8_t {
        Unused,
        Allocating, // 槽位已被锁定，正在初始化字段
        Pending,    // 字段已发布，硬件传输中
        Success,    // 传输正常完成
        Error,      // 发生总线/硬件错误
        Cancelled   // 传输已被中止完成
    };

    std::atomic<std::uint32_t> session_id{0};
    std::atomic<Status> status{Status::Unused};
    std::atomic<hal::I2cError> error{hal::I2cError::HardwareError};
    std::atomic<TaskHandle_t> waiting_task{nullptr};
    std::atomic<std::uint8_t> ref_count{0}; // 硬件活跃引用 + 用户句柄引用
    std::atomic<I2c *> bus{nullptr};        // 反向指针（I2c 析构时置空）

    // Sequential 传输所需的事务私有上下文（彻底移出 I2c 主实例）
    hal::I2cAddress seq_addr{hal::I2cAddress::from_raw_hal_unchecked(0)};
    std::span<std::uint8_t> seq_rx{};

    void retain() noexcept {
        ref_count.fetch_add(1, std::memory_order_relaxed);
    }
    void release() noexcept;
};

// RAII 异步传输句柄
class I2cTransfer {
  public:
    I2cTransfer() noexcept = default;
    ~I2cTransfer();

    I2cTransfer(I2cTransfer &&other) noexcept;
    I2cTransfer &operator=(I2cTransfer &&other) noexcept;
    I2cTransfer(const I2cTransfer &) = delete;
    I2cTransfer &operator=(const I2cTransfer &) = delete;

    [[nodiscard]] std::optional<std::expected<void, hal::I2cError>>
    poll() noexcept;
    [[nodiscard]] std::expected<void, hal::I2cError>
    wait(std::chrono::milliseconds timeout =
             std::chrono::milliseconds(100)) noexcept;
    void cancel() noexcept;
    [[nodiscard]] bool is_done() const noexcept;

  private:
    friend class I2c;
    I2cTransfer(TransferControlBlock *tcb, std::uint32_t session_id) noexcept;

    TransferControlBlock *tcb_{nullptr};
    std::uint32_t session_id_{0};
    bool user_settled_{false};
    std::expected<void, hal::I2cError> cached_result_{};
};

class I2c {
  public:
    using Transfer = I2cTransfer;

    enum class DriverMode : std::uint8_t { Interrupt, Dma };

    struct BusPins {
        GPIO_TypeDef *scl_port;
        std::uint16_t scl_pin;
        std::uint32_t scl_af; // 复用功能映射 (如 GPIO_AF4_I2C1)
        GPIO_TypeDef *sda_port;
        std::uint16_t sda_pin;
        std::uint32_t sda_af;

        constexpr BusPins() noexcept
            : scl_port(nullptr), scl_pin(0), scl_af(0), sda_port(nullptr),
              sda_pin(0), sda_af(0) {}
    };

    I2c(I2C_HandleTypeDef *hi2c, DriverMode mode = DriverMode::Dma,
        BusPins pins = {}) noexcept;
    ~I2c();

    I2c(const I2c &) = delete;
    I2c &operator=(const I2c &) = delete;

    std::expected<void, hal::I2cError> begin() noexcept;

    std::expected<Transfer, hal::I2cError>
    start_write(hal::I2cAddress address,
                std::span<const std::uint8_t> tx) noexcept;

    std::expected<Transfer, hal::I2cError>
    start_read(hal::I2cAddress address, std::span<std::uint8_t> rx) noexcept;

    std::expected<Transfer, hal::I2cError>
    start_write_read(hal::I2cAddress address, std::span<const std::uint8_t> tx,
                     std::span<std::uint8_t> rx) noexcept;

    // 硬件级 9-Pulse 总线恢复（互斥独占总线）
    bool recover_bus() noexcept;

    // C 中断回调静态路由入口
    static void dispatch_isr_completed(I2C_HandleTypeDef *hi2c) noexcept;
    static void dispatch_isr_error(I2C_HandleTypeDef *hi2c) noexcept;
    static void dispatch_isr_aborted(I2C_HandleTypeDef *hi2c) noexcept;

  private:
    friend class I2cTransfer;
    friend struct TransferControlBlock;

    enum class HardwareOp : std::uint8_t {
        Idle,
        Writing,
        Reading,
        SeqWriting, // Sequential 第一阶段 (写)
        SeqReading, // Sequential 第二阶段 (读，Repeated-Start)
        Cancelling
    };

    TransferControlBlock *allocate_tcb() noexcept;
    void finalize_transaction_from_isr(
        TransferControlBlock::Status status,
        hal::I2cError err = hal::I2cError::HardwareError) noexcept;
    void cancel_active_transaction(std::uint32_t session_id) noexcept;

    void handle_isr_completed() noexcept;
    void handle_isr_error() noexcept;
    void handle_isr_aborted() noexcept;

    static hal::I2cError map_hal_error(std::uint32_t hal_error_code) noexcept;
    static void dwt_delay_us(std::uint32_t us) noexcept;

    I2C_HandleTypeDef *hi2c_;
    DriverMode mode_;
    BusPins pins_;

    // TCB 槽位池：管理事务记录与延迟查询；物理总线依然由 bus_lock_
    // 保证单事务独占
    static constexpr std::size_t kMaxTcbSlots = 4;
    std::array<TransferControlBlock, kMaxTcbSlots> tcb_pool_{};
    std::atomic<std::uint32_t> session_counter_{0};

    // 物理硬件执行状态
    std::atomic<HardwareOp> hw_op_{HardwareOp::Idle};
    std::atomic<TransferControlBlock *> active_tcb_{nullptr};

    // 物理总线硬件锁（二值信号量，支持 Task 获取与 ISR 释放）
    StaticSemaphore_t sem_buffer_{};
    SemaphoreHandle_t bus_lock_{nullptr};

    // 静态实例分发表
    static constexpr std::size_t kMaxInstances = 4;
    static std::array<I2c *, kMaxInstances> instances_;
    static void register_instance(I2c *inst) noexcept;
    static void unregister_instance(I2c *inst) noexcept;
    static I2c *get_instance(I2C_HandleTypeDef *hi2c) noexcept;
};

static_assert(hal::AsyncI2cHardware<I2c>);
static_assert(hal::WaitableI2cTransferHandle<I2c::Transfer>);
static_assert(hal::I2cBusRecovery<I2c>);

} // namespace moli::port::stm32