#include "main.h"
#include <moli/target_hal/stm32/i2c.hpp>

namespace moli::port::stm32 {

std::array<I2c *, I2c::kMaxInstances> I2c::instances_{nullptr};

// ======================= TransferControlBlock 实现 =======================

void TransferControlBlock::release() noexcept {
    if (ref_count.fetch_sub(1, std::memory_order_acq_rel) == 1) {
        // 最后一个所有者退出（硬件与句柄均已归还），重置槽位
        waiting_task.store(nullptr, std::memory_order_relaxed);
        bus.store(nullptr, std::memory_order_relaxed);
        seq_rx = {};
        status.store(Status::Unused, std::memory_order_release);
    }
}

// ======================= I2cTransfer 实现 =======================

I2cTransfer::I2cTransfer(TransferControlBlock *tcb,
                         std::uint32_t session_id) noexcept
    : tcb_(tcb), session_id_(session_id), user_settled_(false) {
    if (tcb_ != nullptr) {
        tcb_->retain();
    }
}

I2cTransfer::~I2cTransfer() {
    if (!user_settled_ && tcb_ != nullptr) {
        cancel();
    }
    if (tcb_ != nullptr) {
        tcb_->release();
    }
}

I2cTransfer::I2cTransfer(I2cTransfer &&other) noexcept
    : tcb_(other.tcb_), session_id_(other.session_id_),
      user_settled_(other.user_settled_), cached_result_(other.cached_result_) {
    other.tcb_ = nullptr;
    other.session_id_ = 0;
    other.user_settled_ = true;
}

I2cTransfer &I2cTransfer::operator=(I2cTransfer &&other) noexcept {
    if (this != &other) {
        if (!user_settled_ && tcb_ != nullptr) {
            cancel();
        }
        if (tcb_ != nullptr) {
            tcb_->release();
        }

        tcb_ = other.tcb_;
        session_id_ = other.session_id_;
        user_settled_ = other.user_settled_;
        cached_result_ = other.cached_result_;

        other.tcb_ = nullptr;
        other.session_id_ = 0;
        other.user_settled_ = true;
    }
    return *this;
}

std::optional<std::expected<void, hal::I2cError>> I2cTransfer::poll() noexcept {
    if (user_settled_)
        return cached_result_;
    if (tcb_ == nullptr ||
        tcb_->session_id.load(std::memory_order_relaxed) != session_id_) {
        return std::unexpected(hal::I2cError::InvalidArgument);
    }

    const auto s = tcb_->status.load(std::memory_order_acquire);
    if (s == TransferControlBlock::Status::Pending ||
        s == TransferControlBlock::Status::Allocating) {
        return std::nullopt;
    }

    user_settled_ = true;
    if (s == TransferControlBlock::Status::Success) {
        cached_result_ = {};
    } else if (s == TransferControlBlock::Status::Cancelled) {
        cached_result_ = std::unexpected(hal::I2cError::Cancelled);
    } else {
        cached_result_ =
            std::unexpected(tcb_->error.load(std::memory_order_relaxed));
    }
    return cached_result_;
}

std::expected<void, hal::I2cError>
I2cTransfer::wait(std::chrono::milliseconds timeout) noexcept {
    if (user_settled_)
        return cached_result_;
    if (tcb_ == nullptr ||
        tcb_->session_id.load(std::memory_order_relaxed) != session_id_) {
        return std::unexpected(hal::I2cError::InvalidArgument);
    }

    // 1. 清空当前 Task 历史残留通知
    ulTaskNotifyTake(pdTRUE, 0);

    // 2. 注册等待 Task 句柄（全序同步）
    tcb_->waiting_task.store(xTaskGetCurrentTaskHandle(),
                             std::memory_order_seq_cst);

    // 3. 检查当前是否仍在 Pending 阶段
    if (tcb_->status.load(std::memory_order_seq_cst) ==
        TransferControlBlock::Status::Pending) {
        const TickType_t ticks = pdMS_TO_TICKS(timeout.count());
        const std::uint32_t notified = ulTaskNotifyTake(pdTRUE, ticks);

        if (notified == 0 && tcb_->status.load(std::memory_order_seq_cst) ==
                                 TransferControlBlock::Status::Pending) {
            cancel(); // 超时触发异步中止请求
        }
    }

    tcb_->waiting_task.store(nullptr, std::memory_order_relaxed);
    auto res = poll();
    return res.value_or(std::unexpected(hal::I2cError::Timeout));
}

void I2cTransfer::cancel() noexcept {
    if (user_settled_ || tcb_ == nullptr)
        return;

    // 仅提交中止请求，最终状态迁移由 Abort 完成中断回调推进
    I2c *bus_ptr = tcb_->bus.load(std::memory_order_acquire);
    if (bus_ptr != nullptr) {
        bus_ptr->cancel_active_transaction(session_id_);
    }
}

bool I2cTransfer::is_done() const noexcept {
    if (user_settled_)
        return true;
    if (tcb_ == nullptr)
        return true;
    const auto s = tcb_->status.load(std::memory_order_acquire);
    return s != TransferControlBlock::Status::Pending &&
           s != TransferControlBlock::Status::Allocating;
}

// ========================== I2c Driver 实现 ==========================

I2c::I2c(I2C_HandleTypeDef *hi2c, DriverMode mode, BusPins pins) noexcept
    : hi2c_(hi2c), mode_(mode), pins_(pins) {
    bus_lock_ = xSemaphoreCreateBinaryStatic(&sem_buffer_);
    xSemaphoreGive(bus_lock_);
    register_instance(this);
}

I2c::~I2c() {
    unregister_instance(this);
    for (auto &tcb : tcb_pool_) {
        tcb.bus.store(nullptr, std::memory_order_release);
    }
    if (bus_lock_ != nullptr) {
        vSemaphoreDelete(bus_lock_);
    }
}

void I2c::dwt_delay_us(std::uint32_t us) noexcept {
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    const std::uint32_t start = DWT->CYCCNT;
    const std::uint32_t cycles = us * (SystemCoreClock / 1000000UL);
    while ((DWT->CYCCNT - start) < cycles) {
    }
}

std::expected<void, hal::I2cError> I2c::begin() noexcept {
    if (hi2c_ == nullptr)
        return std::unexpected(hal::I2cError::NotInitialized);
    if (__HAL_I2C_GET_FLAG(hi2c_, I2C_FLAG_BUSY)) {
        recover_bus();
    }
    return {};
}

TransferControlBlock *I2c::allocate_tcb() noexcept {
    for (auto &tcb : tcb_pool_) {
        auto expected_status = TransferControlBlock::Status::Unused;
        // 1. CAS 获取独占槽位
        if (tcb.status.compare_exchange_strong(
                expected_status, TransferControlBlock::Status::Allocating,
                std::memory_order_acquire, std::memory_order_relaxed)) {
            // 2. 初始化字段
            tcb.ref_count.store(1, std::memory_order_relaxed); // 硬件活跃所有权
            tcb.bus.store(this, std::memory_order_relaxed);
            tcb.waiting_task.store(nullptr, std::memory_order_relaxed);
            tcb.error.store(hal::I2cError::HardwareError,
                            std::memory_order_relaxed);
            tcb.session_id.store(
                session_counter_.fetch_add(1, std::memory_order_relaxed) + 1,
                std::memory_order_relaxed);
            tcb.seq_rx = {};
            return &tcb;
        }
    }
    return nullptr;
}

void I2c::register_instance(I2c *inst) noexcept {
    for (auto &slot : instances_) {
        if (slot == nullptr) {
            slot = inst;
            return;
        }
    }
}

void I2c::unregister_instance(I2c *inst) noexcept {
    for (auto &slot : instances_) {
        if (slot == inst) {
            slot = nullptr;
            return;
        }
    }
}

I2c *I2c::get_instance(I2C_HandleTypeDef *hi2c) noexcept {
    for (auto *inst : instances_) {
        if (inst != nullptr && inst->hi2c_ == hi2c) {
            return inst;
        }
    }
    return nullptr;
}

void I2c::dispatch_isr_completed(I2C_HandleTypeDef *hi2c) noexcept {
    if (auto *inst = get_instance(hi2c))
        inst->handle_isr_completed();
}

void I2c::dispatch_isr_error(I2C_HandleTypeDef *hi2c) noexcept {
    if (auto *inst = get_instance(hi2c))
        inst->handle_isr_error();
}

void I2c::dispatch_isr_aborted(I2C_HandleTypeDef *hi2c) noexcept {
    if (auto *inst = get_instance(hi2c))
        inst->handle_isr_aborted();
}

hal::I2cError I2c::map_hal_error(std::uint32_t hal_error_code) noexcept {
    if (hal_error_code & HAL_I2C_ERROR_AF)
        return hal::I2cError::Nack;
    if (hal_error_code & HAL_I2C_ERROR_BERR)
        return hal::I2cError::BusError;
    if (hal_error_code & HAL_I2C_ERROR_ARLO)
        return hal::I2cError::ArbitrationLost;
    if (hal_error_code & HAL_I2C_ERROR_OVR)
        return hal::I2cError::Overrun;
    if (hal_error_code & HAL_I2C_ERROR_TIMEOUT)
        return hal::I2cError::Timeout;
    return hal::I2cError::HardwareError;
}

void I2c::finalize_transaction_from_isr(TransferControlBlock::Status status,
                                        hal::I2cError err) noexcept {
    TransferControlBlock *tcb =
        active_tcb_.exchange(nullptr, std::memory_order_acq_rel);
    if (tcb == nullptr)
        return;

    if (status == TransferControlBlock::Status::Error) {
        tcb->error.store(err, std::memory_order_relaxed);
    }
    tcb->status.store(status, std::memory_order_release);

    TaskHandle_t task = tcb->waiting_task.load(std::memory_order_seq_cst);
    BaseType_t woken = pdFALSE;

    if (task != nullptr) {
        vTaskNotifyGiveFromISR(task, &woken);
    }

    hw_op_.store(HardwareOp::Idle, std::memory_order_relaxed);
    xSemaphoreGiveFromISR(bus_lock_, &woken);
    tcb->release(); // 释放硬件活跃所有权

    portYIELD_FROM_ISR(woken);
}

std::expected<I2cTransfer, hal::I2cError>
I2c::start_write(hal::I2cAddress address,
                 std::span<const std::uint8_t> tx) noexcept {

    if (tx.empty())
        return std::unexpected(hal::I2cError::InvalidArgument);
    if (hi2c_ == nullptr)
        return std::unexpected(hal::I2cError::NotInitialized);

    if (xSemaphoreTake(bus_lock_, 0) != pdTRUE) {
        return std::unexpected(hal::I2cError::Busy);
    }

    auto *tcb = allocate_tcb();
    if (tcb == nullptr) {
        xSemaphoreGive(bus_lock_);
        return std::unexpected(hal::I2cError::Busy);
    }

    active_tcb_.store(tcb, std::memory_order_release);
    hw_op_.store(HardwareOp::Writing, std::memory_order_relaxed);

    // 字段准备完毕，正式发布为 Pending 状态
    tcb->status.store(TransferControlBlock::Status::Pending,
                      std::memory_order_release);

    auto *buf = const_cast<std::uint8_t *>(tx.data());
    const auto size = static_cast<std::uint16_t>(tx.size());

    HAL_StatusTypeDef status =
        (mode_ == DriverMode::Dma)
            ? HAL_I2C_Master_Transmit_DMA(hi2c_, address.raw(), buf, size)
            : HAL_I2C_Master_Transmit_IT(hi2c_, address.raw(), buf, size);

    if (status != HAL_OK) {
        active_tcb_.store(nullptr, std::memory_order_relaxed);
        hw_op_.store(HardwareOp::Idle, std::memory_order_relaxed);
        tcb->error.store(status == HAL_BUSY ? hal::I2cError::Busy
                                            : hal::I2cError::HardwareError,
                         std::memory_order_relaxed);
        tcb->status.store(TransferControlBlock::Status::Error,
                          std::memory_order_release);
        tcb->release();
        xSemaphoreGive(bus_lock_);
        return std::unexpected(status == HAL_BUSY
                                   ? hal::I2cError::Busy
                                   : hal::I2cError::HardwareError);
    }

    return I2cTransfer(tcb, tcb->session_id.load(std::memory_order_relaxed));
}

std::expected<I2cTransfer, hal::I2cError>
I2c::start_read(hal::I2cAddress address, std::span<std::uint8_t> rx) noexcept {

    if (rx.empty())
        return std::unexpected(hal::I2cError::InvalidArgument);
    if (hi2c_ == nullptr)
        return std::unexpected(hal::I2cError::NotInitialized);

    if (xSemaphoreTake(bus_lock_, 0) != pdTRUE) {
        return std::unexpected(hal::I2cError::Busy);
    }

    auto *tcb = allocate_tcb();
    if (tcb == nullptr) {
        xSemaphoreGive(bus_lock_);
        return std::unexpected(hal::I2cError::Busy);
    }

    active_tcb_.store(tcb, std::memory_order_release);
    hw_op_.store(HardwareOp::Reading, std::memory_order_relaxed);

    tcb->status.store(TransferControlBlock::Status::Pending,
                      std::memory_order_release);

    auto *buf = rx.data();
    const auto size = static_cast<std::uint16_t>(rx.size());

    HAL_StatusTypeDef status =
        (mode_ == DriverMode::Dma)
            ? HAL_I2C_Master_Receive_DMA(hi2c_, address.raw(), buf, size)
            : HAL_I2C_Master_Receive_IT(hi2c_, address.raw(), buf, size);

    if (status != HAL_OK) {
        active_tcb_.store(nullptr, std::memory_order_relaxed);
        hw_op_.store(HardwareOp::Idle, std::memory_order_relaxed);
        tcb->error.store(status == HAL_BUSY ? hal::I2cError::Busy
                                            : hal::I2cError::HardwareError,
                         std::memory_order_relaxed);
        tcb->status.store(TransferControlBlock::Status::Error,
                          std::memory_order_release);
        tcb->release();
        xSemaphoreGive(bus_lock_);
        return std::unexpected(status == HAL_BUSY
                                   ? hal::I2cError::Busy
                                   : hal::I2cError::HardwareError);
    }

    return I2cTransfer(tcb, tcb->session_id.load(std::memory_order_relaxed));
}

std::expected<I2cTransfer, hal::I2cError>
I2c::start_write_read(hal::I2cAddress address, std::span<const std::uint8_t> tx,
                      std::span<std::uint8_t> rx) noexcept {

    if (tx.empty() || rx.empty())
        return std::unexpected(hal::I2cError::InvalidArgument);
    if (hi2c_ == nullptr)
        return std::unexpected(hal::I2cError::NotInitialized);

    if (xSemaphoreTake(bus_lock_, 0) != pdTRUE) {
        return std::unexpected(hal::I2cError::Busy);
    }

    auto *tcb = allocate_tcb();
    if (tcb == nullptr) {
        xSemaphoreGive(bus_lock_);
        return std::unexpected(hal::I2cError::Busy);
    }

    // 将 Sequential 读阶段参数保存在 TCB 结构中
    tcb->seq_addr = address;
    tcb->seq_rx = rx;

    active_tcb_.store(tcb, std::memory_order_release);
    hw_op_.store(HardwareOp::SeqWriting, std::memory_order_relaxed);

    tcb->status.store(TransferControlBlock::Status::Pending,
                      std::memory_order_release);

    auto *buf = const_cast<std::uint8_t *>(tx.data());
    const auto size = static_cast<std::uint16_t>(tx.size());

    // 第一阶段：Sequential 发送 (I2C_FIRST_FRAME，维持总线占用，不产生 STOP)
    HAL_StatusTypeDef status =
        (mode_ == DriverMode::Dma)
            ? HAL_I2C_Master_Seq_Transmit_DMA(hi2c_, address.raw(), buf, size,
                                              I2C_FIRST_FRAME)
            : HAL_I2C_Master_Seq_Transmit_IT(hi2c_, address.raw(), buf, size,
                                             I2C_FIRST_FRAME);

    if (status != HAL_OK) {
        active_tcb_.store(nullptr, std::memory_order_relaxed);
        hw_op_.store(HardwareOp::Idle, std::memory_order_relaxed);
        tcb->error.store(status == HAL_BUSY ? hal::I2cError::Busy
                                            : hal::I2cError::HardwareError,
                         std::memory_order_relaxed);
        tcb->status.store(TransferControlBlock::Status::Error,
                          std::memory_order_release);
        tcb->release();
        xSemaphoreGive(bus_lock_);
        return std::unexpected(status == HAL_BUSY
                                   ? hal::I2cError::Busy
                                   : hal::I2cError::HardwareError);
    }

    return I2cTransfer(tcb, tcb->session_id.load(std::memory_order_relaxed));
}

void I2c::cancel_active_transaction(std::uint32_t session_id) noexcept {
    TransferControlBlock *tcb = active_tcb_.load(std::memory_order_acquire);
    if (tcb != nullptr &&
        tcb->session_id.load(std::memory_order_relaxed) == session_id) {
        auto cur_op = hw_op_.load(std::memory_order_relaxed);
        if (cur_op != HardwareOp::Idle && cur_op != HardwareOp::Cancelling) {
            hw_op_.store(HardwareOp::Cancelling, std::memory_order_relaxed);
            HAL_I2C_Master_Abort_IT(hi2c_, 0);
        }
    }
}

void I2c::handle_isr_completed() noexcept {
    auto current_op = hw_op_.load(std::memory_order_relaxed);

    if (current_op == HardwareOp::SeqWriting) {
        TransferControlBlock *tcb = active_tcb_.load(std::memory_order_acquire);
        if (tcb == nullptr) {
            finalize_transaction_from_isr(TransferControlBlock::Status::Error,
                                          hal::I2cError::HardwareError);
            return;
        }

        // 第一阶段写完成，立即触发 Sequential 读 (I2C_LAST_FRAME 产生
        // Repeated-Start)
        hw_op_.store(HardwareOp::SeqReading, std::memory_order_relaxed);
        auto *buf = tcb->seq_rx.data();
        const auto size = static_cast<std::uint16_t>(tcb->seq_rx.size());

        HAL_StatusTypeDef status =
            (mode_ == DriverMode::Dma)
                ? HAL_I2C_Master_Seq_Receive_DMA(hi2c_, tcb->seq_addr.raw(),
                                                 buf, size, I2C_LAST_FRAME)
                : HAL_I2C_Master_Seq_Receive_IT(hi2c_, tcb->seq_addr.raw(), buf,
                                                size, I2C_LAST_FRAME);

        if (status == HAL_OK)
            return;

        finalize_transaction_from_isr(TransferControlBlock::Status::Error,
                                      hal::I2cError::HardwareError);
        return;
    }

    if (hi2c_->ErrorCode == HAL_I2C_ERROR_NONE) {
        finalize_transaction_from_isr(TransferControlBlock::Status::Success);
    } else {
        finalize_transaction_from_isr(TransferControlBlock::Status::Error,
                                      map_hal_error(hi2c_->ErrorCode));
    }
}

void I2c::handle_isr_error() noexcept {
    finalize_transaction_from_isr(TransferControlBlock::Status::Error,
                                  map_hal_error(hi2c_->ErrorCode));
}

void I2c::handle_isr_aborted() noexcept {
    finalize_transaction_from_isr(TransferControlBlock::Status::Cancelled);
}

bool I2c::recover_bus() noexcept {
    // 必须首先获取硬件总线锁，防止与后台正在进行的 DMA/IT 事务冲突
    if (xSemaphoreTake(bus_lock_, 0) != pdTRUE) {
        return false;
    }

    if (pins_.scl_port == nullptr || pins_.sda_port == nullptr) {
        __HAL_I2C_DISABLE(hi2c_);
        hi2c_->Instance->CR1 |= I2C_CR1_SWRST;
        dwt_delay_us(10);
        hi2c_->Instance->CR1 &= ~I2C_CR1_SWRST;
        __HAL_I2C_ENABLE(hi2c_);
        xSemaphoreGive(bus_lock_);
        return true;
    }

    __HAL_I2C_DISABLE(hi2c_);

    // 1. 引脚切为开漏输出模式
    GPIO_InitTypeDef gpio_init{};
    gpio_init.Pin = pins_.scl_pin;
    gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(pins_.scl_port, &gpio_init);

    gpio_init.Pin = pins_.sda_pin;
    HAL_GPIO_Init(pins_.sda_port, &gpio_init);

    HAL_GPIO_WritePin(pins_.sda_port, pins_.sda_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(pins_.scl_port, pins_.scl_pin, GPIO_PIN_SET);

    // 2. 发送最多 9 个 SCL 脉冲，使用 DWT 硬件时钟精确保障 5us 高低电平
    for (int i = 0; i < 9; ++i) {
        HAL_GPIO_WritePin(pins_.scl_port, pins_.scl_pin, GPIO_PIN_RESET);
        dwt_delay_us(5);
        HAL_GPIO_WritePin(pins_.scl_port, pins_.scl_pin, GPIO_PIN_SET);
        dwt_delay_us(5);

        if (HAL_GPIO_ReadPin(pins_.sda_port, pins_.sda_pin) == GPIO_PIN_SET) {
            break;
        }
    }

    // 3. 产生标准 STOP 条件
    HAL_GPIO_WritePin(pins_.sda_port, pins_.sda_pin, GPIO_PIN_RESET);
    dwt_delay_us(5);
    HAL_GPIO_WritePin(pins_.scl_port, pins_.scl_pin, GPIO_PIN_SET);
    dwt_delay_us(5);
    HAL_GPIO_WritePin(pins_.sda_port, pins_.sda_pin, GPIO_PIN_SET);
    dwt_delay_us(5);

    // 4. 恢复引脚 Alternate Function 复用模式
    gpio_init.Mode = GPIO_MODE_AF_OD;
    gpio_init.Alternate = pins_.scl_af;
    HAL_GPIO_Init(pins_.scl_port, &gpio_init);

    gpio_init.Alternate = pins_.sda_af;
    HAL_GPIO_Init(pins_.sda_port, &gpio_init);

    __HAL_I2C_ENABLE(hi2c_);
    xSemaphoreGive(bus_lock_);
    return true;
}

} // namespace moli::port::stm32

// ========================== C 中断回调统一转发 ==========================
extern "C" {

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    moli::port::stm32::I2c::dispatch_isr_completed(hi2c);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    moli::port::stm32::I2c::dispatch_isr_completed(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    moli::port::stm32::I2c::dispatch_isr_error(hi2c);
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c) {
    moli::port::stm32::I2c::dispatch_isr_aborted(hi2c);
}

} // extern "C"