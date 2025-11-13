/**
 * @file thermal_frame_generator.h
 * @brief Synthetic thermal frame generator for Xi 640 ETH development
 *
 * Generates test patterns when physical thermal sensor is not available:
 * - Linear gradients (cold to hot)
 * - Hot spot patterns with radial decay
 * - Checkerboard thermal patterns
 * - Random thermal noise
 *
 * Frame format: 640×480 @ 14-bit (0-16383 ADC values)
 */

#ifndef THERMAL_FRAME_GENERATOR_H
#define THERMAL_FRAME_GENERATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "thermal_types.h"  /* Common thermal frame type definitions */

/* Frame dimensions */
#define THERMAL_WIDTH  640
#define THERMAL_HEIGHT 480
#define THERMAL_PIXELS (THERMAL_WIDTH * THERMAL_HEIGHT)

/* 14-bit ADC range */
#define THERMAL_ADC_MIN 0
#define THERMAL_ADC_MAX 16383

/* thermal_frame_t is defined in thermal_types.h */

/* Test pattern types */
typedef enum {
    PATTERN_GRADIENT,      /* Linear cold→hot gradient */
    PATTERN_HOTSPOT,       /* Central hot spot with decay */
    PATTERN_CHECKERBOARD,  /* Checkerboard pattern */
    PATTERN_NOISE,         /* Random thermal noise */
    PATTERN_MOVING_HOTSPOT /* Animated hot spot */
} thermal_pattern_type_t;

/* Hot spot configuration */
typedef struct {
    int16_t x;           /* Center X coordinate */
    int16_t y;           /* Center Y coordinate */
    uint16_t temp_peak;  /* Peak temperature (ADC value) */
    uint16_t radius;     /* Decay radius in pixels */
} thermal_hotspot_t;

/**
 * @brief Initialize thermal frame generator
 * @param pattern Initial test pattern type
 * @return 0 on success, negative on error
 */
int thermal_generator_init(thermal_pattern_type_t pattern);

/**
 * @brief Generate synthetic thermal frame
 * @param frame Output buffer (must be pre-allocated)
 * @return 0 on success, negative on error
 */
int thermal_generator_create_frame(thermal_frame_t *frame);

/**
 * @brief Set test pattern type
 * @param pattern New pattern type
 */
void thermal_generator_set_pattern(thermal_pattern_type_t pattern);

/**
 * @brief Add hot spot to current pattern
 * @param hotspot Hot spot configuration
 * @return 0 on success, negative if max hot spots reached
 */
int thermal_generator_add_hotspot(const thermal_hotspot_t *hotspot);

/**
 * @brief Clear all hot spots
 */
void thermal_generator_clear_hotspots(void);

/**
 * @brief Get current frame statistics
 * @param min_val Output: minimum ADC value in last frame
 * @param max_val Output: maximum ADC value in last frame
 * @param avg_val Output: average ADC value in last frame
 */
void thermal_generator_get_stats(uint16_t *min_val, uint16_t *max_val, uint32_t *avg_val);

#endif /* THERMAL_FRAME_GENERATOR_H */
