/**
 * @file thermal_types.h
 * @brief Common thermal frame type definitions for Xi 640 ETH
 *
 * Shared between thermal_frame_generator and thermal_simd modules
 * to avoid type conflicts.
 */

#ifndef THERMAL_TYPES_H
#define THERMAL_TYPES_H

#include <stdint.h>

/* Xi 640 ETH sensor specifications */
#define FRAME_WIDTH  640
#define FRAME_HEIGHT 480
#define FRAME_PIXELS (FRAME_WIDTH * FRAME_HEIGHT)

/**
 * @brief Raw thermal frame from sensor (14-bit ADC values)
 *
 * Single definition used by both frame generator and thermal SIMD modules
 */
typedef struct {
    uint16_t pixels[FRAME_PIXELS];  // Flattened array for efficient processing
    uint32_t frame_number;          // Frame sequence number (also used as frame_id)
    uint64_t timestamp_us;          // Timestamp in microseconds (also used as timestamp)
} thermal_frame_t;

/**
 * @brief Processed temperature frame (Celsius * 100)
 *
 * Output from thermal SIMD processing pipeline
 */
typedef struct {
    int16_t temps[FRAME_PIXELS];  // Temperature in 0.01°C units (Celsius * 100)
    uint32_t frame_id;            // Frame sequence number
    uint64_t timestamp;           // Timestamp
    float min_temp;               // Minimum temperature in frame
    float max_temp;               // Maximum temperature in frame
    float avg_temp;               // Average temperature in frame
} temperature_frame_t;

#endif /* THERMAL_TYPES_H */
