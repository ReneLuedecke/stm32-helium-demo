# Xi 640 ETH Thermal Processing Pipeline
# Complete integration: Frame Generation + MVE Processing
# Target: 128+ FPS on STM32N6570-DK

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(xi640_eth_thermal)

# Main application sources
target_sources(app PRIVATE
    src/main.c
)

# Link thermal_simd module (Session 2 MVE processing)
add_subdirectory(modules/thermal_simd)
target_link_libraries(app PRIVATE thermal_simd)

# Enable ARM Helium MVE optimizations for application
target_compile_options(app PRIVATE
    -march=armv8.1-m.main+mve.fp
    -mfpu=auto
    -mfloat-abi=hard
    -O3
    -ffast-math
)
