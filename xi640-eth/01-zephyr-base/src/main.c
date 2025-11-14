/**
 * @file main.c
 * @brief Xi 640 ETH - Session 1: Thermal Frame Generator Test
 *
 * Simplified version without custom memory regions (uses standard RAM)
 * - Thermal frame generator with test patterns
 * - Performance monitoring
 * - 50 FPS target
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/mem_mgmt/mem_attr.h>

#include "thermal_types.h"           /* Common thermal frame types (include first!) */
#include "thermal_frame_generator.h"
#include "thermal_simd.h"

/* ========================================
 * GLOBAL CONTEXT AND BUFFERS
 * ======================================== */
/* Place large buffers in external RAM */

#ifndef __extram
#define __extram __attribute__((section(".psram_data")))
#endif


static __extram  thermal_frame_t frame_buffer;
static __extram  temperature_frame_t temp_frame_buffer;

static Thermal_SIMD_Context_t thermal_ctx;  /* Small, stays in internal RAM */

/* ========================================
 * CONFIGURATION
 * ======================================== */
#define FRAME_RATE_HZ        50    /* Target: 50 FPS */
#define FRAME_PERIOD_MS      (1000 / FRAME_RATE_HZ)
#define STATS_INTERVAL_MS    5000  /* Print stats every 5 seconds */

/* ========================================
 * PERFORMANCE MONITORING
 * ======================================== */
typedef struct {
    uint64_t frames_generated;
    uint64_t total_generation_time_ms;
    uint64_t min_generation_time_ms;
    uint64_t max_generation_time_ms;
    uint64_t last_stats_time;
} performance_stats_t;

static performance_stats_t perf_stats = {0};

/**
 * @brief Update performance statistics
 */
static void update_performance_stats(uint64_t generation_time_ms)
{
    perf_stats.frames_generated++;
    perf_stats.total_generation_time_ms += generation_time_ms;

    if (perf_stats.min_generation_time_ms == 0 ||
        generation_time_ms < perf_stats.min_generation_time_ms) {
        perf_stats.min_generation_time_ms = generation_time_ms;
    }

    if (generation_time_ms > perf_stats.max_generation_time_ms) {
        perf_stats.max_generation_time_ms = generation_time_ms;
    }
}

/**
 * @brief Print performance statistics
 */
static void print_statistics(void)
{
    uint64_t now = k_uptime_get();
    uint64_t elapsed_s = (now - perf_stats.last_stats_time) / 1000;

    if (elapsed_s == 0) elapsed_s = 1;

    uint64_t avg_gen_time = perf_stats.total_generation_time_ms /
                           (perf_stats.frames_generated > 0 ? perf_stats.frames_generated : 1);

    uint32_t fps = (perf_stats.frames_generated * 1000) / (now - perf_stats.last_stats_time);

    printk("\n");
    printk("============================================================\n");
    printk(" Xi 640 ETH - THERMAL FRAME GENERATOR STATISTICS\n");
    printk("============================================================\n");
    printk(" Frames generated:    %llu\n", perf_stats.frames_generated);
    printk(" Current FPS:         %u (target: %d)\n", fps, FRAME_RATE_HZ);
    printk("\n");
    printk(" Frame Generation Time:\n");
    printk("   Average:           %llu ms\n", avg_gen_time);
    printk("   Min:               %llu ms\n", perf_stats.min_generation_time_ms);
    printk("   Max:               %llu ms\n", perf_stats.max_generation_time_ms);
    printk("\n");

    /* Get frame statistics */
    uint16_t min_adc, max_adc;
    uint32_t avg_adc;
    thermal_generator_get_stats(&min_adc, &max_adc, &avg_adc);

    printk(" Thermal Data (ADC 14-bit):\n");
    printk("   Min value:         %u / 16383\n", min_adc);
    printk("   Max value:         %u / 16383\n", max_adc);
    printk("   Average:           %u\n", avg_adc);
    printk("\n");

    /* Get thermal SIMD processing stats */
    uint32_t avg_cycles;
    float processing_fps;
    Thermal_SIMD_GetStats(&thermal_ctx, &avg_cycles, &processing_fps);

    printk(" Thermal Processing (ARM Helium MVE):\n");
    printk("   Avg cycles:        %u\n", avg_cycles);
    printk("   Processing FPS:    %.1f\n", (double)processing_fps);
    printk("   Temperature range: %.2f°C - %.2f°C\n",
           (double)temp_frame_buffer.min_temp, (double)temp_frame_buffer.max_temp);
    printk("   Average temp:      %.2f°C\n", (double)temp_frame_buffer.avg_temp);
    printk("============================================================\n");
    printk("\n");

    perf_stats.last_stats_time = now;
}

/* ========================================
 * FRAME CAPTURE THREAD
 * ======================================== */
#define CAPTURE_STACK_SIZE 4096
#define CAPTURE_PRIORITY 5

K_THREAD_STACK_DEFINE(capture_stack, CAPTURE_STACK_SIZE);
static struct k_thread capture_thread_data;

/**
 * @brief Frame capture thread (simulates thermal sensor)
 */
static void capture_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    printk("Capture thread started (target: %d FPS)\n", FRAME_RATE_HZ);

    while (1) {
        /* Generate synthetic thermal frame */
        uint64_t start_time = k_uptime_get();
        int ret = thermal_generator_create_frame(&frame_buffer);
        uint64_t generation_time = k_uptime_get() - start_time;

        if (ret < 0) {
            printk("ERROR: Frame generation failed\n");
            k_sleep(K_MSEC(FRAME_PERIOD_MS));
            continue;
        }

        /* Process frame with thermal SIMD pipeline */
        Thermal_SIMD_ProcessFrame(&thermal_ctx, &frame_buffer, &temp_frame_buffer);

        /* Update statistics */
        update_performance_stats(generation_time);

        /* Periodic statistics */
        if (k_uptime_get() - perf_stats.last_stats_time >= STATS_INTERVAL_MS) {
            print_statistics();
        }

        /* Wait for next frame period */
        k_sleep(K_MSEC(FRAME_PERIOD_MS));
    }
}

/* ========================================
 * MAIN
 * ======================================== */
int main(void)
{
    printk("\n");
    printk("============================================================\n");
    printk("           Xi 640 ETH Thermal Camera System\n");
    printk("        SESSION 1: Zephyr Base + Frame Generator\n");
    printk("============================================================\n");
    printk(" Board:    STM32N6570-DK\n");
    printk(" Sensor:   640x480 @ 14-bit (synthetic mode)\n");
    printk(" Target:   50 FPS baseline\n");
    printk(" Build:    " __DATE__ " " __TIME__ "\n");
    printk("============================================================\n");
    printk("\n");

    /* Frame buffers placed in external RAM via __extram attribute */
    printk("[0/3] Frame buffers (external RAM):\n");
    printk("   Raw frame:  %zu bytes\n", sizeof(thermal_frame_t));
    printk("   Temp frame: %zu bytes\n", sizeof(temperature_frame_t));
    printk("   Total:      %zu bytes (%.1f MB)\n",
           sizeof(thermal_frame_t) + sizeof(temperature_frame_t),
           (sizeof(thermal_frame_t) + sizeof(temperature_frame_t)) / 1048576.0);

    /* Initialize thermal frame generator */
    printk("\n[1/3] Initializing thermal frame generator...\n");

    thermal_pattern_type_t pattern = PATTERN_GRADIENT;

    if (thermal_generator_init(pattern) < 0) {
        printk("FATAL: Thermal generator initialization failed\n");
        return -1;
    }
    printk("OK - Thermal generator ready (pattern=%d)\n", pattern);

    /* Initialize thermal SIMD processing pipeline */
    printk("\n[2/3] Initializing thermal SIMD pipeline...\n");

    thermal_calibration_t cal = {
        .offset = 2048.0f,      // Dark frame offset
        .scale = 1.0f,          // Gain correction scale
        .t_amb = 25.0f,         // Ambient temperature
        .emissivity = 0.95f,    // Surface emissivity
        .use_pixel_cal = false  // Disable per-pixel calibration for now
    };

    Thermal_SIMD_Init(&thermal_ctx, &cal);
    printk("OK - Thermal SIMD pipeline ready (ARM Helium MVE enabled)\n");

    /* Start frame capture thread */
    printk("\n[3/3] Starting capture thread...\n");
    k_thread_create(&capture_thread_data, capture_stack,
                    K_THREAD_STACK_SIZEOF(capture_stack),
                    capture_thread_entry,
                    NULL, NULL, NULL,
                    CAPTURE_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&capture_thread_data, "capture");
    printk("OK - Capture thread started\n");

    /* Initialize statistics timer */
    perf_stats.last_stats_time = k_uptime_get();

    printk("\n");
    printk("============================================================\n");
    printk(" SYSTEM READY - Generating thermal frames @ %d FPS\n", FRAME_RATE_HZ);
    printk("============================================================\n");
    printk("\n");

    /* Main thread becomes idle */
    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}
