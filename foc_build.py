# SimpleFOC 源码编译（库构建阶段，由 library.json 的 build.extraScript 调用）
#
# 从 moli_sdk/third_party/simplefoc/src 编译 FOC 算法核心。
# 该目录已裁剪：只保留算法核心与基类，被 moli::drivers::foc 适配层
# 替代的驱动/传感器实现已删除，因此无需 src_filter 排除，全量编译即可。
#
# 依赖：
#   - include path 需包含 third_party/simplefoc/src（本脚本加入）
#   - arduino_compat.h 通过 <moli/drivers/foc/arduino_compat.h> 引用，库 include 自动可及
#   - FreeRTOS 头路径由使用方 pre 脚本提供（clock.cpp 引入 vTaskDelay）

Import("env", "pio_lib_builder")

import os

lib_path = pio_lib_builder.path
simplefoc_src = os.path.join(lib_path, "third_party", "simplefoc", "src")

# 1. SimpleFOC 头文件路径（其内部用 "common/base_classes/..." 相对路径）
env.Append(CPPPATH=[simplefoc_src])

# 2. 全量编译（目录内已无需要排除的源码）
env.Append(
    LIBS=env.BuildLibrary(
        os.path.join("$BUILD_DIR", "SimpleFOC"),
        simplefoc_src,
        src_filter=["+<*>"],
    )
)
