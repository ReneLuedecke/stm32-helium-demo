/**
 * @file thermal_frame_generator.c
 * @brief Synthetic thermal frame generator implementation
 */

#include "thermal_frame_generator.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>
#include <math.h>

#define MAX_HOTSPOTS 10

/* Generator state */
static struct {
    thermal_pattern_type_t pattern;
    thermal_hotspot_t hotspots[MAX_HOTSPOTS];
    uint8_t hotspot_count;
    uint32_t frame_counter;
    uint16_t min_adc;
    uint16_t max_adc;
    uint32_t sum_adc;
} gen_state;

/**
 * @brief Fast square root approximation for hot spot decay calculation
 */
static inline uint16_t fast_distance(int16_t dx, int16_t dy)
{
    /* Manhattan distance approximation (faster than true Euclidean) */
    uint16_t abs_dx = abs(dx);
    uint16_t abs_dy = abs(dy);
    return abs_dx + abs_dy - (MIN(abs_dx, abs_dy) >> 1);
}

/**
 * @brief Generate gradient pattern (cold→hot)
 */
static void generate_gradient(thermal_frame_t *frame)
{
    for (int y = 0; y < THERMAL_HEIGHT; y++) {
        /* Linear gradient from 20% to 80% of ADC range */
        uint16_t line_value = THERMAL_ADC_MIN +
            ((THERMAL_ADC_MAX * 6 / 10) * y) / THERMAL_HEIGHT +
            (THERMAL_ADC_MAX * 2 / 10);

        for (int x = 0; x < THERMAL_WIDTH; x++) {
            frame->pixels[y * THERMAL_WIDTH + x] = line_value;
        }
    }
}

/**
 * @brief Generate hot spot pattern with radial decay
 */
static void generate_hotspot(thermal_frame_t *frame)
{
    /* Base temperature (ambient) */
    uint16_t ambient = THERMAL_ADC_MAX / 3;

    /* Fill with ambient first */
    for (int i = 0; i < THERMAL_PIXELS; i++) {
        frame->pixels[i] = ambient;
    }

    /* Apply each hot spot */
    for (uint8_t h = 0; h < gen_state.hotspot_count; h++) {
        thermal_hotspot_t *hs = &gen_state.hotspots[h];

        for (int y = 0; y < THERMAL_HEIGHT; y++) {
            for (int x = 0; x < THERMAL_WIDTH; x++) {
                int16_t dx = x - hs->x;
                int16_t dy = y - hs->y;
                uint16_t dist = fast_distance(dx, dy);

                if (dist < hs->radius) {
                    /* Radial decay: temp = peak * (1 - dist/radius)^2 */
                    uint32_t decay = hs->radius - dist;
                    uint32_t temp = ambient +
                        ((hs->temp_peak - ambient) * decay * decay) /
                        (hs->radius * hs->radius);

                    /* Accumulate (multiple hot spots can overlap) */
                    uint32_t current = frame->pixels[y * THERMAL_WIDTH + x];
                    frame->pixels[y * THERMAL_WIDTH + x] = MIN(current + temp - ambient,
                                                                THERMAL_ADC_MAX);
                }
            }
        }
    }
}

/**
 * @brief Generate checkerboard pattern
 */
static void generate_checkerboard(thermal_frame_t *frame)
{
    const uint16_t block_size = 40; /* 40×40 pixel blocks */
    const uint16_t cold_val = THERMAL_ADC_MAX / 4;
    const uint16_t hot_val = THERMAL_ADC_MAX * 3 / 4;

    for (int y = 0; y < THERMAL_HEIGHT; y++) {
        for (int x = 0; x < THERMAL_WIDTH; x++) {
            bool is_hot = ((x / block_size) + (y / block_size)) & 1;
            frame->pixels[y * THERMAL_WIDTH + x] = is_hot ? hot_val : cold_val;
        }
    }
}

/**
 * @brief Generate random thermal noise
 */
static void generate_noise(thermal_frame_t *frame)
{
    /* LFSR-based pseudo-random number generator (fast) */
    static uint32_t lfsr = 0xACE1u;

    for (int i = 0; i < THERMAL_PIXELS; i++) {
        /* Galois LFSR */
        lfsr = (lfsr >> 1) ^ (-(lfsr & 1u) & 0xD0000001u);

        /* Map to ADC range (30-70% for realistic thermal noise) */
        frame->pixels[i] = (THERMAL_ADC_MAX * 3 / 10) +
                          (lfsr % (THERMAL_ADC_MAX * 4 / 10));
    }
}

/**
 * @brief Generate moving hot spot (animated)
 */
static void generate_moving_hotspot(thermal_frame_t *frame)
{
    /* Circular motion */
    float angle = (gen_state.frame_counter * 0.05f);
    int16_t center_x = THERMAL_WIDTH / 2;
    int16_t center_y = THERMAL_HEIGHT / 2;
    int16_t radius = THERMAL_WIDTH / 4;

    thermal_hotspot_t moving_hs = {
        .x = center_x + (int16_t)(radius * cosf(angle)),
        .y = center_y + (int16_t)(radius * sinf(angle)),
        .temp_peak = THERMAL_ADC_MAX * 9 / 10,
        .radius = 80
    };

    /* Generate base ambient */
    uint16_t ambient = THERMAL_ADC_MAX / 3;
    for (int i = 0; i < THERMAL_PIXELS; i++) {
        frame->pixels[i] = ambient;
    }

    /* Apply moving hot spot */
    for (int y = 0; y < THERMAL_HEIGHT; y++) {
        for (int x = 0; x < THERMAL_WIDTH; x++) {
            int16_t dx = x - moving_hs.x;
            int16_t dy = y - moving_hs.y;
            uint16_t dist = fast_distance(dx, dy);

            if (dist < moving_hs.radius) {
                uint32_t decay = moving_hs.radius - dist;
                uint32_t temp = ambient +
                    ((moving_hs.temp_peak - ambient) * decay * decay) /
                    (moving_hs.radius * moving_hs.radius);

                frame->pixels[y * THERMAL_WIDTH + x] = MIN(temp, THERMAL_ADC_MAX);
            }
        }
    }
}

/**
 * @brief Calculate frame statistics
 */
static void calculate_stats(thermal_frame_t *frame)
{
    gen_state.min_adc = THERMAL_ADC_MAX;
    gen_state.max_adc = THERMAL_ADC_MIN;
    gen_state.sum_adc = 0;

    for (int i = 0; i < THERMAL_PIXELS; i++) {
        uint16_t val = frame->pixels[i];
        if (val < gen_state.min_adc) gen_state.min_adc = val;
        if (val > gen_state.max_adc) gen_state.max_adc = val;
        gen_state.sum_adc += val;
    }
}

/* ========================================
 * PUBLIC API IMPLEMENTATION
 * ======================================== */

int thermal_generator_init(thermal_pattern_type_t pattern)
{
    gen_state.pattern = pattern;
    gen_state.hotspot_count = 0;
    gen_state.frame_counter = 0;

    /* Add default hot spots for HOTSPOT pattern */
    if (pattern == PATTERN_HOTSPOT) {
        thermal_hotspot_t default_spots[] = {
            { .x = 320, .y = 240, .temp_peak = THERMAL_ADC_MAX * 9 / 10, .radius = 100 },
            { .x = 160, .y = 120, .temp_peak = THERMAL_ADC_MAX * 7 / 10, .radius = 60 },
            { .x = 480, .y = 360, .temp_peak = THERMAL_ADC_MAX * 8 / 10, .radius = 80 }
        };

        for (int i = 0; i < 3; i++) {
            thermal_generator_add_hotspot(&default_spots[i]);
        }
    }

    printk("Thermal frame generator initialized (pattern=%d)\n", pattern);
    return 0;
}

int thermal_generator_create_frame(thermal_frame_t *frame)
{
    if (!frame) return -1;

    /* Set metadata */
    frame->frame_number = gen_state.frame_counter++;
    frame->timestamp_us = k_uptime_get();

    /* Generate pattern */
    switch (gen_state.pattern) {
    case PATTERN_GRADIENT:
        generate_gradient(frame);
        break;
    case PATTERN_HOTSPOT:
        generate_hotspot(frame);
        break;
    case PATTERN_CHECKERBOARD:
        generate_checkerboard(frame);
        break;
    case PATTERN_NOISE:
        generate_noise(frame);
        break;
    case PATTERN_MOVING_HOTSPOT:
        generate_moving_hotspot(frame);
        break;
    default:
        generate_gradient(frame);
    }

    /* Calculate statistics */
    calculate_stats(frame);

    return 0;
}

void thermal_generator_set_pattern(thermal_pattern_type_t pattern)
{
    gen_state.pattern = pattern;
    printk("Pattern changed to %d\n", pattern);
}

int thermal_generator_add_hotspot(const thermal_hotspot_t *hotspot)
{
    if (gen_state.hotspot_count >= MAX_HOTSPOTS) {
        return -1;
    }

    gen_state.hotspots[gen_state.hotspot_count++] = *hotspot;
    return 0;
}

void thermal_generator_clear_hotspots(void)
{
    gen_state.hotspot_count = 0;
}

void thermal_generator_get_stats(uint16_t *min_val, uint16_t *max_val, uint32_t *avg_val)
{
    if (min_val) *min_val = gen_state.min_adc;
    if (max_val) *max_val = gen_state.max_adc;
    if (avg_val) *avg_val = gen_state.sum_adc / THERMAL_PIXELS;
}
