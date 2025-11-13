/**
 * @file thermal_simd.h
 * @brief ARM Helium MVE-optimized thermal image processing for Xi 640 ETH
 *
 * High-performance thermal frame processing using ARM Helium (MVE) SIMD instructions
 * for the STM32N6570 @ 600 MHz targeting 128+ FPS throughput.
 */

#ifndef THERMAL_SIMD_H
#define THERMAL_SIMD_H

#include <stdint.h>
#include <stdbool.h>
#include <arm_mve.h>

/* Xi 640 ETH sensor specifications */
#define HPIX 640
#define VPIX 480
#define FRAME_SIZE (HPIX * VPIX)

/* Planck radiation constants */
#define PLANCK_C1 1.191042e8f
#define PLANCK_C2 1.4387752e4f
#define WAVELENGTH_UM 10.0f  // Long-wave infrared: 10 μm

/* Fixed-point Q15 format (1 sign + 15 fractional bits) */
#define Q15_SCALE 32768.0f
#define FLOAT_TO_Q15(x) ((int16_t)((x) * Q15_SCALE))
#define Q15_TO_FLOAT(x) (((float)(x)) / Q15_SCALE)

/**
 * @brief Raw thermal frame from sensor (14-bit ADC values)
 */
typedef struct {
    uint16_t pixels[FRAME_SIZE];
    uint32_t timestamp;
    uint32_t frame_id;
} thermal_frame_t;

/**
 * @brief Processed temperature frame (Celsius * 100)
 */
typedef struct {
    int16_t temps[FRAME_SIZE];  // Temperature in 0.01°C units
    float min_temp;
    float max_temp;
    float avg_temp;
    uint32_t timestamp;
    uint32_t frame_id;
} temperature_frame_t;

/**
 * @brief Calibration parameters for thermal processing
 */
typedef struct {
    float offset;           // Dark frame offset
    float scale;            // Gain correction scale
    float t_amb;            // Ambient temperature (°C)
    float emissivity;       // Surface emissivity (0.0-1.0)
    bool use_pixel_cal;     // Enable per-pixel calibration
    int16_t gain_map[FRAME_SIZE];   // Q15 per-pixel gain (if enabled)
    int16_t offset_map[FRAME_SIZE]; // Q15 per-pixel offset (if enabled)
} thermal_calibration_t;

/**
 * @brief SIMD processing context
 */
typedef struct {
    thermal_calibration_t cal;
    float planck_lut[65536];  // Temperature LUT for ADC → °C
    uint32_t frames_processed;
    uint64_t total_cycles;
} Thermal_SIMD_Context_t;

/**
 * @brief Initialize thermal SIMD processing context
 *
 * @param ctx Context to initialize
 * @param cal Calibration parameters
 */
void Thermal_SIMD_Init(Thermal_SIMD_Context_t *ctx, const thermal_calibration_t *cal);

/**
 * @brief Process raw thermal frame with MVE optimization
 *
 * Performs:
 * 1. Dark frame subtraction
 * 2. Gain/offset correction (Q15 fixed-point)
 * 3. Planck LUT temperature conversion
 * 4. Statistics calculation (min/max/avg)
 *
 * @param ctx Processing context
 * @param raw_frame Input raw frame
 * @param temp_frame Output temperature frame
 */
void Thermal_SIMD_ProcessFrame(Thermal_SIMD_Context_t *ctx,
                               const thermal_frame_t *raw_frame,
                               temperature_frame_t *temp_frame);

/**
 * @brief Generate synthetic thermal frame for testing
 *
 * Creates a test pattern with hot spot gradient
 *
 * @param frame Output frame
 * @param frame_id Frame sequence number
 */
void generate_synthetic_frame(thermal_frame_t *frame, uint32_t frame_id);

/**
 * @brief Get processing performance metrics
 *
 * @param ctx Processing context
 * @param avg_cycles_out Average cycles per frame
 * @param fps_out Estimated FPS
 */
void Thermal_SIMD_GetStats(const Thermal_SIMD_Context_t *ctx,
                           uint32_t *avg_cycles_out,
                           float *fps_out);

#endif /* THERMAL_SIMD_H */
