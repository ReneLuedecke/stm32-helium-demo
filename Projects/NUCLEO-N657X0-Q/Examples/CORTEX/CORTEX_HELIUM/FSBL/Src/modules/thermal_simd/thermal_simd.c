/**
 * @file thermal_simd.c
 * @brief ARM Helium MVE-optimized thermal image processing implementation
 * @author Claude Code Session 2
 * @date 2025-11-13
 *
 * This implementation uses ARM Helium (MVE) SIMD instructions to achieve
 * high-performance thermal frame processing on STM32N6 Cortex-M55.
 *
 * Key optimizations:
 * - 8 pixels processed in parallel with MVE vector intrinsics
 * - Q15 fixed-point arithmetic for gain/emissivity correction
 * - Saturating arithmetic to prevent overflow
 * - Gather load for Planck LUT (vectorized table lookup)
 *
 * Expected performance @ 600 MHz:
 * - Single line (640 pixels): ~3000 cycles = 5 μs
 * - Full frame (480×640): ~1.4M cycles = 2.4 ms
 * - Achieves 128 FPS target (7.8 ms period, 5.4 ms margin)
 */

#include "thermal_simd.h"
#include <math.h>
#include <stdio.h>

// ═══════════════════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════════════════

void Thermal_SIMD_Init(
    ThermalProcessingContext_t *ctx,
    const uint16_t *planck_lut,
    const uint16_t (*dark_frame)[THERMAL_HPIX],
    const uint16_t (*gain_frame)[THERMAL_HPIX],
    const uint16_t *offset_line,
    const uint16_t (*emissivity)[THERMAL_HPIX],
    uint16_t flag_adc
) {
    ctx->planck_lut = planck_lut;
    ctx->dark_frame = dark_frame;
    ctx->gain_frame = gain_frame;
    ctx->offset_line = offset_line;
    ctx->emissivity = emissivity;
    ctx->flag_adc = flag_adc;
}

// ═══════════════════════════════════════════════════════════════════
// MVE-OPTIMIZED LINE PROCESSING
// ═══════════════════════════════════════════════════════════════════

/**
 * @brief Process single thermal line with MVE SIMD (8 pixels parallel)
 *
 * This function is the heart of the thermal processing pipeline.
 * It processes 640 pixels in 80 iterations (8 pixels/iteration).
 *
 * MVE intrinsics used:
 * - vld1q_u16: Load 8×16-bit values
 * - vqsubq_u16: Saturating subtract (dark frame correction)
 * - vqrdmulhq_s16: Q15 saturating multiply (gain/emissivity)
 * - vqaddq_u16: Saturating add (offset + flag)
 * - vldrhq_gather_shifted_offset_u16: Vectorized LUT lookup
 * - vst1q_u16: Store 8×16-bit values
 *
 * Performance analysis:
 * - 80 iterations × ~37 cycles/iteration ≈ 3000 cycles
 * - @ 600 MHz: 3000 cycles = 5 μs per line
 * - Full frame: 480 lines × 5 μs = 2.4 ms
 */
void Thermal_SIMD_ProcessLine(
    const ThermalProcessingContext_t *ctx,
    const uint16_t *sensor_data,
    uint32_t line_index,
    uint16_t *output
) {
    // Prepare line pointers
    const uint16_t *dark = ctx->dark_frame[line_index];
    const uint16_t *gain = ctx->gain_frame[line_index];
    const uint16_t *offset = ctx->offset_line;
    const uint16_t *emiss = ctx->emissivity[line_index];
    const uint16_t *planck_lut = ctx->planck_lut;

    // Broadcast flag ADC value to vector
    const uint16x8_t flag_vec = vdupq_n_u16(ctx->flag_adc);

    // Process 8 pixels at a time (640 pixels = 80 iterations)
    for (uint32_t x = 0; x < THERMAL_HPIX; x += 8)
    {
        // ═══════════════════════════════════════════════════════════
        // STEP 1: Load input data (8 pixels)
        // ═══════════════════════════════════════════════════════════
        uint16x8_t adc   = vld1q_u16(&sensor_data[x]);  // Raw ADC values
        uint16x8_t darkv = vld1q_u16(&dark[x]);         // Dark frame
        uint16x8_t gainv = vld1q_u16(&gain[x]);         // Gain coefficients (Q15)
        uint16x8_t offv  = vld1q_u16(&offset[x]);       // Offset values
        uint16x8_t emissv = vld1q_u16(&emiss[x]);       // Emissivity (Q15)

        // ═══════════════════════════════════════════════════════════
        // STEP 2: Dark frame correction (saturating subtract)
        // ═══════════════════════════════════════════════════════════
        // corr = adc - dark (saturates at 0 if adc < dark)
        uint16x8_t corr = vqsubq_u16(adc, darkv);

        // ═══════════════════════════════════════════════════════════
        // STEP 3: Gain correction (Q15 fixed-point multiply)
        // ═══════════════════════════════════════════════════════════
        // Q15 format: 1 sign bit + 15 fractional bits
        // Range: [-1.0, 0.9999...] mapped to [-32768, 32767]
        // vqrdmulhq_s16: (a * b) >> 15 with rounding and saturation
        int16x8_t  corr_s = vreinterpretq_s16_u16(corr);
        int16x8_t  gain_s = vreinterpretq_s16_u16(gainv);
        int16x8_t  mul_s  = vqrdmulhq_s16(corr_s, gain_s);
        uint16x8_t val    = vreinterpretq_u16_s16(mul_s);

        // ═══════════════════════════════════════════════════════════
        // STEP 4: Emissivity correction (Q15 multiply)
        // ═══════════════════════════════════════════════════════════
        // Emissivity typically 0.95-1.0 (31129-32767 in Q15)
        int16x8_t val_s   = vreinterpretq_s16_u16(val);
        int16x8_t emiss_s = vreinterpretq_s16_u16(emissv);
        int16x8_t emiss_mul = vqrdmulhq_s16(val_s, emiss_s);
        val = vreinterpretq_u16_s16(emiss_mul);

        // ═══════════════════════════════════════════════════════════
        // STEP 5: Add flag reference + offset (saturating add)
        // ═══════════════════════════════════════════════════════════
        val = vqaddq_u16(val, flag_vec);
        val = vqaddq_u16(val, offv);

        // ═══════════════════════════════════════════════════════════
        // STEP 6: Planck LUT lookup (vectorized gather load)
        // ═══════════════════════════════════════════════════════════
        // vldrhq_gather_shifted_offset_u16 performs:
        // out[i] = planck_lut[val[i]] for i = 0..7
        // This is a single instruction that replaces 8 scalar loads!
        uint16x8_t out = vldrhq_gather_shifted_offset_u16(planck_lut, val);

        // ═══════════════════════════════════════════════════════════
        // STEP 7: Store result (8 pixels)
        // ═══════════════════════════════════════════════════════════
        vst1q_u16(&output[x], out);
    }
}

// ═══════════════════════════════════════════════════════════════════
// FRAME PROCESSING
// ═══════════════════════════════════════════════════════════════════

uint32_t Thermal_SIMD_ProcessFrame(
    const ThermalProcessingContext_t *ctx,
    const uint16_t (*input_frame)[THERMAL_HPIX],
    uint16_t (*output_frame)[THERMAL_HPIX]
) {
    // Read cycle counter (assumes DWT is already initialized in main)
    #define DWT_CYCCNT ((volatile uint32_t *)0xE0001004)
    uint32_t start_cycles = *DWT_CYCCNT;

    // Process all 480 lines
    for (uint32_t line = 0; line < THERMAL_VPIX; line++) {
        Thermal_SIMD_ProcessLine(
            ctx,
            input_frame[line],
            line,
            output_frame[line]
        );
    }

    uint32_t cycles = *DWT_CYCCNT - start_cycles;
    return cycles;
}

uint32_t Thermal_SIMD_ProcessFrameWithStats(
    const ThermalProcessingContext_t *ctx,
    const uint16_t (*input_frame)[THERMAL_HPIX],
    uint16_t (*output_frame)[THERMAL_HPIX],
    ThermalProcessingStats_t *stats
) {
    uint32_t cycles = Thermal_SIMD_ProcessFrame(ctx, input_frame, output_frame);

    // Update statistics
    stats->frame_count++;
    stats->total_cycles += cycles;
    stats->last_frame_cycles = cycles;
    stats->avg_cycles_per_frame = (float)stats->total_cycles / (float)stats->frame_count;
    stats->avg_ms_per_frame = stats->avg_cycles_per_frame / 600000.0f;  // @ 600 MHz
    stats->fps = 1000.0f / stats->avg_ms_per_frame;

    return cycles;
}

// ═══════════════════════════════════════════════════════════════════
// PLANCK LUT GENERATION
// ═══════════════════════════════════════════════════════════════════

/**
 * @brief Generate Planck radiation law lookup table
 *
 * Planck's law for blackbody radiation:
 *   B(λ,T) = C₁ / (λ⁵ * (exp(C₂/(λ*T)) - 1))
 *
 * Where:
 *   C₁ = 1.191042e8 W·μm⁴/m²·sr (first radiation constant)
 *   C₂ = 1.4387752e4 μm·K (second radiation constant)
 *   λ = wavelength in micrometers
 *   T = absolute temperature in Kelvin
 *
 * This function creates a 65536-entry LUT mapping ADC values to temperatures.
 * The LUT is inverted: given an ADC value (proportional to radiance),
 * it returns the corresponding temperature.
 *
 * Output encoding: uint16_t = (T_celsius * 100 + 10000)
 * Examples:
 *   -100.0°C → 0
 *     0.0°C → 10000
 *    25.5°C → 12550
 *   100.0°C → 20000
 *   555.35°C → 65535
 */
void Thermal_SIMD_GeneratePlanckLUT(
    uint16_t *lut,
    float wavelength,
    float temp_min,
    float temp_max
) {
    const float lambda5 = powf(wavelength, 5.0f);
    const float c2_lambda = PLANCK_C2 / wavelength;

    // Pre-compute radiance range
    float T_min_K = temp_min + 273.15f;
    float T_max_K = temp_max + 273.15f;

    float B_min = PLANCK_C1 / (lambda5 * (expf(c2_lambda / T_max_K) - 1.0f));
    float B_max = PLANCK_C1 / (lambda5 * (expf(c2_lambda / T_min_K) - 1.0f));

    // Generate LUT: for each ADC value, compute corresponding temperature
    for (uint32_t adc = 0; adc < THERMAL_LUT_SIZE; adc++) {
        // Map ADC to radiance (linear interpolation)
        float t = (float)adc / (float)(THERMAL_LUT_SIZE - 1);
        float B = B_min + t * (B_max - B_min);

        // Invert Planck's law to get temperature from radiance
        // B = C₁ / (λ⁵ * (exp(C₂/(λ*T)) - 1))
        // → exp(C₂/(λ*T)) = C₁/(λ⁵*B) + 1
        // → C₂/(λ*T) = ln(C₁/(λ⁵*B) + 1)
        // → T = C₂ / (λ * ln(C₁/(λ⁵*B) + 1))

        float ratio = PLANCK_C1 / (lambda5 * B) + 1.0f;
        float T_K = c2_lambda / logf(ratio);
        float T_C = T_K - 273.15f;

        // Encode temperature: uint16_t = (T * 100 + 10000)
        lut[adc] = Thermal_EncodeTemperature(T_C);
    }
}

// ═══════════════════════════════════════════════════════════════════
// STATISTICS
// ═══════════════════════════════════════════════════════════════════

void Thermal_SIMD_ResetStats(ThermalProcessingStats_t *stats) {
    stats->frame_count = 0;
    stats->total_cycles = 0;
    stats->last_frame_cycles = 0;
    stats->avg_cycles_per_frame = 0.0f;
    stats->avg_ms_per_frame = 0.0f;
    stats->fps = 0.0f;
}

void Thermal_SIMD_PrintStats(const ThermalProcessingStats_t *stats) {
    printf("\n╔═══════════════════════════════════════════════════════════╗\n");
    printf("║ THERMAL PROCESSING STATISTICS (MVE SIMD)                  ║\n");
    printf("╠═══════════════════════════════════════════════════════════╣\n");
    printf("║ Frames processed:    %10u                        ║\n", stats->frame_count);
    printf("║ Last frame cycles:   %10u                        ║\n", stats->last_frame_cycles);
    printf("║ Last frame time:     %10.3f ms                    ║\n",
           (float)stats->last_frame_cycles / 600000.0f);
    printf("║ Avg cycles/frame:    %10.1f                        ║\n", stats->avg_cycles_per_frame);
    printf("║ Avg time/frame:      %10.3f ms                    ║\n", stats->avg_ms_per_frame);
    printf("║ Throughput:          %10.1f FPS                   ║\n", stats->fps);
    printf("╚═══════════════════════════════════════════════════════════╝\n");
}
