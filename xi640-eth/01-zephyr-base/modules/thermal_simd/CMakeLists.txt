# Thermal SIMD module CMake configuration
# ARM Helium MVE-optimized thermal processing

zephyr_library_named(thermal_simd)

zephyr_library_sources(
    thermal_simd.c
)

zephyr_library_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# ========================================
# ARM Helium MVE Support via Compiler Flags
# (Zephyr Kconfig does not have CONFIG_ARM_MVE_FP)
# ========================================
zephyr_library_compile_options(
    -march=armv8.1-m.main+mve.fp+fp.dp
    -mfpu=auto
    -mfloat-abi=hard
    -O3
    -ffast-math
    -funroll-loops
    -ftree-vectorize
)

# Link math library for Planck calculations
zephyr_library_link_libraries(m)
