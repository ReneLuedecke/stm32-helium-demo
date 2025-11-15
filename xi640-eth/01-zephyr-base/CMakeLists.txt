# SPDX-License-Identifier: Apache-2.0
# Xi 640 ETH - Minimal Boot Test

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(xi640_eth_minimal VERSION 0.1.0)

# Application sources - minimal test only
target_sources(app PRIVATE
    src/main.c
)

# Temporarily disabled for minimal test
# target_sources(app PRIVATE src/thermal_frame_generator.c)
# target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
# target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/modules/thermal_simd)
# add_subdirectory(modules/thermal_simd)
# target_link_libraries(app PRIVATE thermal_simd)

# ========================================
# ARM Helium MVE Support via Compiler Flags
# (Zephyr Kconfig does not have CONFIG_ARM_MVE_FP)
# ========================================
target_compile_options(app PRIVATE
    -march=armv8.1-m.main+mve.fp+fp.dp
    -mfpu=auto
    -mfloat-abi=hard
)

# Optimization flags for maximum performance
target_compile_options(app PRIVATE
    -O3
    -ffast-math
    -funroll-loops
    -ftree-vectorize
)
