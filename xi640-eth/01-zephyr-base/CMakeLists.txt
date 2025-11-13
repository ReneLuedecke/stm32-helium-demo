# SPDX-License-Identifier: Apache-2.0
# Xi 640 ETH Thermal Camera - Session 1: Zephyr Base + Synthetic Frame Generator

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(xi640_zephyr_base VERSION 0.1.0)

# Application sources
target_sources(app PRIVATE
  src/main.c
  src/thermal_frame_generator.c
  src/memory_manager.c
)

# Include directories
target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
