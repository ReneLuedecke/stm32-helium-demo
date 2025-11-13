/**
 * @file main.c
 * @brief Xi 640 ETH - Complete Thermal Processing Pipeline
 *
 * Session 1 + Session 2 Integration:
 * - High-speed synthetic frame generation (Session 1)
 * - ARM Helium MVE thermal processing (Session 2)
 * - Target: 128+ FPS sustained throughput
 *
 * STM32N6570-DK @ 600 MHz with ARM Helium (MVE) SIMD
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "thermal_simd.h"

#define USE_THERMAL_SIMD_MODULE 1
#define STATS_INTERVAL 100  // Print statistics every N frames

// Global processing buffers
static thermal_frame_t raw_frame;
static temperature_frame_t temp_frame;
static Thermal_SIMD_Context_t simd_ctx;

void main(void)
{
    printk("\n");
    printk("========================================\n");
    printk("Xi 640 ETH - Complete Pipeline Test\n");
    printk("Session 1 + Session 2 Integration\n");
    printk("========================================\n");
    printk("Hardware: STM32N6570-DK @ 600 MHz\n");
    printk("Sensor:   Xi 640 ETH (640×480 thermal)\n");
    printk("Target:   128+ FPS sustained\n");
    printk("========================================\n\n");

    // Initialize thermal calibration parameters
    thermal_calibration_t cal = {
        .offset = 2048.0f,
        .scale = 0.01f,
        .t_amb = 25.0f,
        .emissivity = 0.95f,
        .use_pixel_cal = false
    };

    // Initialize MVE thermal processing
    Thermal_SIMD_Init(&simd_ctx, &cal);
    printk("✅ Thermal SIMD initialized\n");
    printk("   - Planck LUT: 65536 entries\n");
    printk("   - MVE vectorization: 8 pixels/iteration\n");
    printk("   - Processing: %d×%d frames\n\n", HPIX, VPIX);

    uint32_t frame_count = 0;
    uint64_t total_gen_time = 0;
    uint64_t total_process_time = 0;

    printk("Starting thermal processing pipeline...\n\n");

    // Main processing loop
    while (1) {
        // =============================================================
        // SESSION 1: Generate synthetic thermal frame
        // =============================================================
        uint32_t gen_start = k_cycle_get_32();
        generate_synthetic_frame(&raw_frame, frame_count);
        uint32_t gen_cycles = k_cycle_get_32() - gen_start;

        // =============================================================
        // SESSION 2: Process with MVE thermal pipeline
        // =============================================================
        uint32_t proc_start = k_cycle_get_32();
        Thermal_SIMD_ProcessFrame(&simd_ctx, &raw_frame, &temp_frame);
        uint32_t proc_cycles = k_cycle_get_32() - proc_start;

        // Update cumulative statistics
        total_gen_time += gen_cycles;
        total_process_time += proc_cycles;
        frame_count++;

        // Print detailed statistics every STATS_INTERVAL frames
        if (frame_count % STATS_INTERVAL == 0) {
            // Calculate average times (in milliseconds)
            float avg_gen_ms = (float)(total_gen_time / frame_count) / 600000.0f;
            float avg_proc_ms = (float)(total_process_time / frame_count) / 600000.0f;
            float total_ms = avg_gen_ms + avg_proc_ms;
            float fps = 1000.0f / total_ms;

            printk("\n=== Frame %u Statistics ===\n", frame_count);
            printk("Generation:  %.2f ms (%6.1f FPS) [Session 1]\n",
                   avg_gen_ms, 1000.0f / avg_gen_ms);
            printk("Processing:  %.2f ms (%6.1f FPS) [Session 2]\n",
                   avg_proc_ms, 1000.0f / avg_proc_ms);
            printk("───────────────────────────────────────\n");
            printk("Total:       %.2f ms (%6.1f FPS)\n", total_ms, fps);
            printk("───────────────────────────────────────\n");
            printk("Temperature: %.1f°C - %.1f°C (avg: %.1f°C)\n",
                   temp_frame.min_temp, temp_frame.max_temp, temp_frame.avg_temp);

            // Check if we're meeting the 128 FPS target
            if (fps >= 128.0f) {
                printk("✅ Target 128 FPS maintained! (%.1f FPS)\n", fps);
            } else {
                printk("⚠️  Below target: %.1f / 128 FPS (%.1f%% of goal)\n",
                       fps, (fps / 128.0f) * 100.0f);
            }
        }

        // Small delay to prevent log spam (can be removed for max throughput)
        k_msleep(1);
    }
}
