# Thermal SIMD module CMake configuration
# ARM Helium MVE-optimized thermal processing

zephyr_library_named(thermal_simd)

zephyr_library_sources(
    thermal_simd.c
)

zephyr_library_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Enable ARM Helium MVE optimizations
zephyr_library_compile_options(
    -march=armv8.1-m.main+mve.fp
    -mfpu=auto
    -mfloat-abi=hard
)
