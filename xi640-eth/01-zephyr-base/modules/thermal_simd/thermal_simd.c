/**
 * @file thermal_simd.c
 * @brief ARM Helium MVE-optimized thermal processing implementation
 */

#include "thermal_simd.h"
#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>

/* CPU frequency for FPS calculation (STM32N6570 @ 600 MHz) */
#define CPU_FREQ_MHZ 600.0f

/**
 * @brief Calculate Planck radiation law temperature
 *
 * B(λ,T) = C₁ / (λ⁵(e^(C₂/λT) - 1))
 *
 * @param adc_value Raw ADC value (0-65535)
 * @return Temperature in Celsius
 */
static float planck_to_temperature(uint16_t adc_value)
{
    if (adc_value == 0) return -273.15f;

    float radiance = (float)adc_value;
    float lambda5 = powf(WAVELENGTH_UM, 5.0f);

    // Solve for T: adc ∝ C₁ / (λ⁵(e^(C₂/λT) - 1))
    // Simplified approximation for typical thermal ranges
    float exp_term = PLANCK_C1 / (lambda5 * radiance);
    if (exp_term <= 1.0f) return -273.15f;

    float temp_k = PLANCK_C2 / (WAVELENGTH_UM * logf(exp_term + 1.0f));
    return temp_k - 273.15f;
}

void Thermal_SIMD_Init(Thermal_SIMD_Context_t *ctx, const thermal_calibration_t *cal)
{
    memcpy(&ctx->cal, cal, sizeof(thermal_calibration_t));
    ctx->frames_processed = 0;
    ctx->total_cycles = 0;

    // Generate Planck LUT for fast temperature conversion
    for (uint32_t i = 0; i < 65536; i++) {
        ctx->planck_lut[i] = planck_to_temperature((uint16_t)i);
    }
}

/**
 * @brief Process single line with MVE vectorization (8 pixels per iteration)
 *
 * Uses ARM Helium intrinsics for 8× throughput improvement
 */
static inline void process_line_mve(const uint16_t *raw_line,
                                     int16_t *temp_line,
                                     const float *planck_lut,
                                     float offset,
                                     float scale,
                                     int line_width)
{
    // Bounds check to prevent undefined behavior
    if (line_width > HPIX) {
        line_width = HPIX;
    }

    int i = 0;

    // Process 8 pixels per iteration with MVE
    for (i = 0; i <= line_width - 8; i += 8) {
        // Load 8 raw pixels
        uint16x8_t raw = vld1q_u16(&raw_line[i]);

        // Convert to float vector for processing
        float32x4_t raw_low = vcvtq_f32_u32(vmovlbq_u16(raw));
        float32x4_t raw_high = vcvtq_f32_u32(vmovltq_u16(raw));

        // Apply offset subtraction
        float32x4_t offset_vec = vdupq_n_f32(offset);
        raw_low = vsubq_f32(raw_low, offset_vec);
        raw_high = vsubq_f32(raw_high, offset_vec);

        // Apply gain scaling
        float32x4_t scale_vec = vdupq_n_f32(scale);
        raw_low = vmulq_f32(raw_low, scale_vec);
        raw_high = vmulq_f32(raw_high, scale_vec);

        // Convert back to uint16 for LUT indexing
        uint32x4_t idx_low = vcvtq_u32_f32(raw_low);
        uint32x4_t idx_high = vcvtq_u32_f32(raw_high);

        // Clamp to valid LUT range
        idx_low = vminq_u32(idx_low, vdupq_n_u32(65535));
        idx_high = vminq_u32(idx_high, vdupq_n_u32(65535));

        // LUT lookup (scalar fallback - MVE doesn't have gather load for float)
        float temps[8];
        temps[0] = planck_lut[vgetq_lane_u32(idx_low, 0)];
        temps[1] = planck_lut[vgetq_lane_u32(idx_low, 1)];
        temps[2] = planck_lut[vgetq_lane_u32(idx_low, 2)];
        temps[3] = planck_lut[vgetq_lane_u32(idx_low, 3)];
        temps[4] = planck_lut[vgetq_lane_u32(idx_high, 0)];
        temps[5] = planck_lut[vgetq_lane_u32(idx_high, 1)];
        temps[6] = planck_lut[vgetq_lane_u32(idx_high, 2)];
        temps[7] = planck_lut[vgetq_lane_u32(idx_high, 3)];

        // Convert to int16 (temperature * 100)
        for (int j = 0; j < 8; j++) {
            temp_line[i + j] = (int16_t)(temps[j] * 100.0f);
        }
    }

    // Handle remaining pixels (scalar)
    for (; i < line_width; i++) {
        float corrected = ((float)raw_line[i] - offset) * scale;
        uint16_t idx = (uint16_t)(corrected < 0 ? 0 : (corrected > 65535 ? 65535 : corrected));
        temp_line[i] = (int16_t)(planck_lut[idx] * 100.0f);
    }
}

void Thermal_SIMD_ProcessFrame(Thermal_SIMD_Context_t *ctx,
                               const thermal_frame_t *raw_frame,
                               temperature_frame_t *temp_frame)
{
    uint32_t start_cycles = k_cycle_get_32();

    // Copy metadata
    temp_frame->timestamp = raw_frame->timestamp_us;
    temp_frame->frame_id = raw_frame->frame_number;

    // Process all lines with MVE
    for (int y = 0; y < VPIX; y++) {
        const uint16_t *raw_line = &raw_frame->pixels[y * HPIX];
        int16_t *temp_line = &temp_frame->temps[y * HPIX];

        process_line_mve(raw_line, temp_line, ctx->planck_lut,
                        ctx->cal.offset, ctx->cal.scale, HPIX);
    }

    // Calculate statistics (vectorized min/max/sum)
    int16_t min_val = 32767;
    int16_t max_val = -32768;
    int64_t sum = 0;

    for (int i = 0; i < FRAME_SIZE; i += 8) {
        int16x8_t temps = vld1q_s16(&temp_frame->temps[i]);
        min_val = (int16_t)vminvq_s16(min_val, temps);
        max_val = (int16_t)vmaxvq_s16(max_val, temps);
        sum += (int64_t)vaddvq_s16(temps);
    }

    temp_frame->min_temp = min_val / 100.0f;
    temp_frame->max_temp = max_val / 100.0f;
    temp_frame->avg_temp = sum / (FRAME_SIZE * 100.0f);

    // Update performance counters
    uint32_t cycles = k_cycle_get_32() - start_cycles;
    ctx->total_cycles += cycles;
    ctx->frames_processed++;
}

void generate_synthetic_frame(thermal_frame_t *frame, uint32_t frame_id)
{
    frame->frame_number = frame_id;
    frame->timestamp_us = k_cycle_get_32();

    // Generate test pattern: radial gradient with hot spot
    int cx = HPIX / 2;
    int cy = VPIX / 2;
    float phase = (frame_id % 360) * 3.14159f / 180.0f;

    for (int y = 0; y < VPIX; y++) {
        for (int x = 0; x < HPIX; x++) {
            int dx = x - cx;
            int dy = y - cy;
            float dist = sqrtf(dx*dx + dy*dy);

            // Radial gradient (20°C - 85°C range)
            float temp_c = 25.0f + 60.0f * (1.0f - dist / 400.0f);
            temp_c += 10.0f * sinf(phase + dist / 50.0f);  // Animated ripple

            // Clamp to valid range
            if (temp_c < 20.0f) temp_c = 20.0f;
            if (temp_c > 85.0f) temp_c = 85.0f;

            // Convert to ADC value (simplified inverse Planck)
            uint16_t adc = (uint16_t)((temp_c - 20.0f) * 1000.0f + 2048.0f);

            frame->pixels[y * HPIX + x] = adc;
        }
    }
}

void Thermal_SIMD_GetStats(const Thermal_SIMD_Context_t *ctx,
                           uint32_t *avg_cycles_out,
                           float *fps_out)
{
    if (ctx->frames_processed == 0) {
        *avg_cycles_out = 0;
        *fps_out = 0.0f;
        return;
    }

    *avg_cycles_out = (uint32_t)(ctx->total_cycles / ctx->frames_processed);
    float ms_per_frame = *avg_cycles_out / (CPU_FREQ_MHZ * 1000.0f);
    *fps_out = 1000.0f / ms_per_frame;
}
