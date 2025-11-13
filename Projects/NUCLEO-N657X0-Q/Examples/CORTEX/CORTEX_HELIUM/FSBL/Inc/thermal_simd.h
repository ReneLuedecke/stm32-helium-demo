/**
 * @file thermal_simd.h
 * @brief ARM Helium MVE-optimized thermal image processing pipeline
 * @author Claude Code Session 2
 * @date 2025-11-13
 *
 * This module provides high-performance thermal frame processing using
 * ARM Helium (MVE) SIMD instructions for STM32N6 (600 MHz Cortex-M55).
 *
 * Features:
 * - 16 pixels parallel processing with MVE intrinsics
 * - Dark frame / gain / emissivity correction
 * - Planck radiation law temperature conversion
 * - Target: 640×480 @ 128 FPS (6.4 ms processing time)
 *
 * Processing Pipeline:
 * 1. Dark frame subtraction (ADC - dark)
 * 2. Gain correction (Q15 fixed-point multiply)
 * 3. Emissivity correction (Q15 fixed-point multiply)
 * 4. Offset + flag reference addition
 * 5. Planck LUT temperature lookup
 */

#ifndef THERMAL_SIMD_H
#define THERMAL_SIMD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <arm_mve.h>

// ═══════════════════════════════════════════════════════════════════
// CONFIGURATION
// ═══════════════════════════════════════════════════════════════════

#define THERMAL_HPIX 640    ///< Horizontal resolution
#define THERMAL_VPIX 480    ///< Vertical resolution
#define THERMAL_LUT_SIZE 65536  ///< Planck LUT entries (16-bit ADC)

// Planck radiation law constants
#define PLANCK_C1 1.191042e8f   ///< First radiation constant (W·μm⁴/m²·sr)
#define PLANCK_C2 1.4387752e4f  ///< Second radiation constant (μm·K)

// ═══════════════════════════════════════════════════════════════════
// DATA STRUCTURES
// ═══════════════════════════════════════════════════════════════════

/**
 * @brief Thermal processing context (holds calibration data pointers)
 */
typedef struct {
    const uint16_t *planck_lut;       ///< Planck LUT (65536 entries, AXISRAM1)
    const uint16_t (*dark_frame)[THERMAL_HPIX];   ///< Dark frame (480×640, AXISRAM2)
    const uint16_t (*gain_frame)[THERMAL_HPIX];   ///< Gain frame (480×640, AXISRAM2)
    const uint16_t *offset_line;      ///< Offset line (640 values, reused per line)
    const uint16_t (*emissivity)[THERMAL_HPIX];   ///< Emissivity map (480×640, XSPI)
    uint16_t flag_adc;                ///< Flag temperature ADC value (reference)
} ThermalProcessingContext_t;

/**
 * @brief Processing statistics
 */
typedef struct {
    uint32_t frame_count;             ///< Total frames processed
    uint32_t total_cycles;            ///< Total CPU cycles spent
    uint32_t last_frame_cycles;       ///< Cycles for last frame
    float avg_cycles_per_frame;       ///< Average cycles per frame
    float avg_ms_per_frame;           ///< Average ms per frame (@ 600 MHz)
    float fps;                        ///< Frames per second
} ThermalProcessingStats_t;

// ═══════════════════════════════════════════════════════════════════
// API FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

/**
 * @brief Initialize thermal processing context
 * @param ctx Processing context to initialize
 * @param planck_lut Pointer to Planck LUT (65536 entries)
 * @param dark_frame Pointer to dark frame (480×640)
 * @param gain_frame Pointer to gain frame (480×640)
 * @param offset_line Pointer to offset line (640 values)
 * @param emissivity Pointer to emissivity map (480×640)
 * @param flag_adc Flag temperature ADC value
 */
void Thermal_SIMD_Init(
    ThermalProcessingContext_t *ctx,
    const uint16_t *planck_lut,
    const uint16_t (*dark_frame)[THERMAL_HPIX],
    const uint16_t (*gain_frame)[THERMAL_HPIX],
    const uint16_t *offset_line,
    const uint16_t (*emissivity)[THERMAL_HPIX],
    uint16_t flag_adc
);

/**
 * @brief Process single thermal line (MVE-optimized, 8 pixels parallel)
 * @param ctx Processing context
 * @param sensor_data Input raw ADC data (640 pixels)
 * @param line_index Line number (0-479)
 * @param output Output temperature data (640 pixels, encoded as uint16_t)
 *
 * Processing steps:
 * 1. Dark frame subtraction: corr = sensor_data - dark
 * 2. Gain correction: val = corr * gain (Q15)
 * 3. Emissivity correction: val = val * emissivity (Q15)
 * 4. Add flag + offset: val = val + flag + offset
 * 5. Planck LUT lookup: output = planck_lut[val]
 *
 * Performance: ~3000 cycles/line @ 600 MHz (5 μs/line)
 * Expected: 480 lines × 5 μs = 2.4 ms total
 */
void Thermal_SIMD_ProcessLine(
    const ThermalProcessingContext_t *ctx,
    const uint16_t *sensor_data,
    uint32_t line_index,
    uint16_t *output
);

/**
 * @brief Process full thermal frame (480×640)
 * @param ctx Processing context
 * @param input_frame Input raw ADC frame (480×640)
 * @param output_frame Output temperature frame (480×640)
 * @return Processing time in CPU cycles
 *
 * Expected performance: ~1.4M cycles @ 600 MHz = 2.4 ms
 * Allows 128 FPS (7.8 ms period - 2.4 ms processing = 5.4 ms margin)
 */
uint32_t Thermal_SIMD_ProcessFrame(
    const ThermalProcessingContext_t *ctx,
    const uint16_t (*input_frame)[THERMAL_HPIX],
    uint16_t (*output_frame)[THERMAL_HPIX]
);

/**
 * @brief Process frame with statistics tracking
 * @param ctx Processing context
 * @param input_frame Input raw ADC frame
 * @param output_frame Output temperature frame
 * @param stats Statistics structure to update
 * @return Processing time in CPU cycles
 */
uint32_t Thermal_SIMD_ProcessFrameWithStats(
    const ThermalProcessingContext_t *ctx,
    const uint16_t (*input_frame)[THERMAL_HPIX],
    uint16_t (*output_frame)[THERMAL_HPIX],
    ThermalProcessingStats_t *stats
);

/**
 * @brief Generate Planck radiation LUT
 * @param lut Output LUT buffer (65536 entries)
 * @param wavelength Wavelength in micrometers (e.g., 10.0 for LWIR)
 * @param temp_min Minimum temperature in Celsius
 * @param temp_max Maximum temperature in Celsius
 *
 * Planck's law: B(λ,T) = C₁ / (λ⁵(e^(C₂/λT) - 1))
 * Output format: uint16_t = (T_celsius * 100 + 10000)
 * Example: 25.5°C → 12550
 */
void Thermal_SIMD_GeneratePlanckLUT(
    uint16_t *lut,
    float wavelength,
    float temp_min,
    float temp_max
);

/**
 * @brief Decode temperature from uint16_t encoding
 * @param encoded Encoded temperature value
 * @return Temperature in Celsius
 */
static inline float Thermal_DecodeTemperature(uint16_t encoded) {
    return ((float)encoded - 10000.0f) / 100.0f;
}

/**
 * @brief Encode temperature to uint16_t format
 * @param temp_celsius Temperature in Celsius
 * @return Encoded temperature value
 */
static inline uint16_t Thermal_EncodeTemperature(float temp_celsius) {
    int32_t val = (int32_t)(temp_celsius * 100.0f + 10000.0f);
    if (val < 0) val = 0;
    if (val > 65535) val = 65535;
    return (uint16_t)val;
}

/**
 * @brief Reset statistics
 * @param stats Statistics structure to reset
 */
void Thermal_SIMD_ResetStats(ThermalProcessingStats_t *stats);

/**
 * @brief Print statistics to console
 * @param stats Statistics to print
 */
void Thermal_SIMD_PrintStats(const ThermalProcessingStats_t *stats);

#ifdef __cplusplus
}
#endif

#endif // THERMAL_SIMD_H
