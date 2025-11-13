/**
 * @file main.c
 * @brief Xi 640 ETH - Session 1: Zephyr Base + Synthetic Frame Generator
 *
 * Architecture:
 * - STM32N6570-DK @ 150 MHz with Zephyr RTOS 3.7
 * - 640×480 thermal sensor simulation (14-bit ADC)
 * - Memory layout: AXISRAM (DMA) + HyperRAM (framebuffer) + HyperFlash (calibration)
 * - Target: 50 FPS baseline (will scale to 125-160 FPS in Session 2)
 *
 * Flow:
 * 1. Initialize memory regions and frame pool
 * 2. Start synthetic frame generator
 * 3. Timer-driven frame capture (simulating VSYNC)
 * 4. Performance monitoring and statistics
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/timing/timing.h>

#include "thermal_frame_generator.h"
#include "memory_manager.h"

/* ========================================
 * CONFIGURATION
 * ======================================== */
#define FRAME_RATE_HZ        50    /* Target: 50 FPS (20ms period) */
#define FRAME_PERIOD_MS      (1000 / FRAME_RATE_HZ)
#define STATS_INTERVAL_MS    5000  /* Print stats every 5 seconds */

/* LED for status indication */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(LED0_NODE, gpios, {0});

/* ========================================
 * PERFORMANCE MONITORING
 * ======================================== */
typedef struct {
    uint64_t frames_generated;
    uint64_t total_generation_time_us;
    uint64_t min_generation_time_us;
    uint64_t max_generation_time_us;
    uint32_t current_fps;
    uint64_t last_stats_time;
} performance_stats_t;

static performance_stats_t perf_stats = {0};

/**
 * @brief Update performance statistics
 */
static void update_performance_stats(uint64_t generation_time_us)
{
    perf_stats.frames_generated++;
    perf_stats.total_generation_time_us += generation_time_us;

    if (perf_stats.min_generation_time_us == 0 ||
        generation_time_us < perf_stats.min_generation_time_us) {
        perf_stats.min_generation_time_us = generation_time_us;
    }

    if (generation_time_us > perf_stats.max_generation_time_us) {
        perf_stats.max_generation_time_us = generation_time_us;
    }
}

/**
 * @brief Print performance statistics
 */
static void print_statistics(void)
{
    uint64_t now = k_uptime_get();
    uint64_t elapsed_s = (now - perf_stats.last_stats_time) / 1000;

    if (elapsed_s == 0) elapsed_s = 1; /* Avoid division by zero */

    uint64_t avg_gen_time = perf_stats.total_generation_time_us /
                           MAX(perf_stats.frames_generated, 1);

    uint32_t fps = (perf_stats.frames_generated * 1000) / (now - perf_stats.last_stats_time);

    printk("\n╔════════════════════════════════════════════════════════════╗\n");
    printk("║ Xi 640 ETH - PERFORMANCE STATISTICS                       ║\n");
    printk("╠════════════════════════════════════════════════════════════╣\n");
    printk("║ Frames generated:    %10llu                         ║\n", perf_stats.frames_generated);
    printk("║ Current FPS:         %10u (target: %d)              ║\n", fps, FRAME_RATE_HZ);
    printk("║                                                            ║\n");
    printk("║ Frame Generation Time:                                     ║\n");
    printk("║   Average:           %10llu µs                       ║\n", avg_gen_time);
    printk("║   Min:               %10llu µs                       ║\n", perf_stats.min_generation_time_us);
    printk("║   Max:               %10llu µs                       ║\n", perf_stats.max_generation_time_us);
    printk("║                                                            ║\n");

    /* Get frame statistics */
    uint16_t min_adc, max_adc;
    uint32_t avg_adc;
    thermal_generator_get_stats(&min_adc, &max_adc, &avg_adc);

    printk("║ Thermal Data (ADC 14-bit):                                 ║\n");
    printk("║   Min value:         %10u / 16383                   ║\n", min_adc);
    printk("║   Max value:         %10u / 16383                   ║\n", max_adc);
    printk("║   Average:           %10u                           ║\n", avg_adc);
    printk("╚════════════════════════════════════════════════════════════╝\n\n");

    /* Print memory usage */
    memory_print_map();

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
 * @brief Frame capture thread (simulates DCMIPP VSYNC interrupt)
 *
 * In production, this would be triggered by hardware VSYNC signal from thermal sensor.
 * Here we use periodic timer to simulate 50 Hz frame rate.
 */
static void capture_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    printk("Capture thread started (target: %d FPS)\n", FRAME_RATE_HZ);

    uint64_t next_frame_time = k_uptime_get();
    bool led_state = false;

    while (1) {
        /* Wait for next frame period */
        k_sleep(K_MSEC(FRAME_PERIOD_MS));
        next_frame_time += FRAME_PERIOD_MS;

        /* Toggle LED to indicate activity */
        if (led.port) {
            led_state = !led_state;
            gpio_pin_set_dt(&led, led_state);
        }

        /* Acquire frame buffer from pool */
        thermal_frame_t *frame = memory_acquire_capture_frame();
        if (!frame) {
            printk("WARNING: No free frame buffer (frame dropped)\n");
            continue;
        }

        /* Generate synthetic thermal frame */
        uint64_t start_time = k_uptime_get();
        int ret = thermal_generator_create_frame(frame);
        uint64_t generation_time = k_uptime_get() - start_time;

        if (ret < 0) {
            printk("ERROR: Frame generation failed\n");
            memory_release_frame(frame);
            continue;
        }

        /* Update statistics */
        update_performance_stats(generation_time);

        /* Release frame for processing (in Session 2, Helium MVE will process here) */
        memory_release_frame(frame);

        /* Periodic statistics */
        if (k_uptime_get() - perf_stats.last_stats_time >= STATS_INTERVAL_MS) {
            print_statistics();
        }
    }
}

/* ========================================
 * SHELL COMMANDS (Optional)
 * ======================================== */
#ifdef CONFIG_SHELL
#include <zephyr/shell/shell.h>

static int cmd_pattern(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_print(sh, "Usage: pattern <0-4>");
        shell_print(sh, "  0=gradient, 1=hotspot, 2=checkerboard, 3=noise, 4=moving");
        return -1;
    }

    int pattern = atoi(argv[1]);
    if (pattern < 0 || pattern > 4) {
        shell_print(sh, "Invalid pattern (must be 0-4)");
        return -1;
    }

    thermal_generator_set_pattern((thermal_pattern_type_t)pattern);
    shell_print(sh, "Pattern set to %d", pattern);
    return 0;
}

static int cmd_stats(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    print_statistics();
    return 0;
}

SHELL_CMD_REGISTER(pattern, NULL, "Set test pattern", cmd_pattern);
SHELL_CMD_REGISTER(stats, NULL, "Show statistics", cmd_stats);
#endif

/* ========================================
 * MAIN
 * ======================================== */
int main(void)
{
    printk("\n");
    printk("╔════════════════════════════════════════════════════════════╗\n");
    printk("║                                                            ║\n");
    printk("║           Xi 640 ETH Thermal Camera System                ║\n");
    printk("║        SESSION 1: Zephyr Base + Frame Generator           ║\n");
    printk("║                                                            ║\n");
    printk("║  Board:    STM32N6570-DK @ 150 MHz                        ║\n");
    printk("║  RTOS:     Zephyr 3.7 LTS                                 ║\n");
    printk("║  Sensor:   640×480 @ 14-bit (synthetic mode)              ║\n");
    printk("║  Target:   50 FPS baseline                                ║\n");
    printk("║                                                            ║\n");
    printk("╚════════════════════════════════════════════════════════════╝\n");
    printk("\n");

    /* Initialize LED */
    if (led.port) {
        if (!gpio_is_ready_dt(&led)) {
            printk("ERROR: LED device not ready\n");
        } else {
            gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
            printk("✓ LED initialized\n");
        }
    }

    /* Initialize memory manager */
    printk("\n[1/3] Initializing memory manager...\n");
    if (memory_manager_init() < 0) {
        printk("FATAL: Memory manager initialization failed\n");
        return -1;
    }
    printk("✓ Memory manager ready\n");

    /* Initialize thermal frame generator */
    printk("\n[2/3] Initializing thermal frame generator...\n");
#ifdef CONFIG_THERMAL_PATTERN_HOTSPOT
    thermal_pattern_type_t pattern = PATTERN_HOTSPOT;
#elif defined(CONFIG_THERMAL_PATTERN_CHECKERBOARD)
    thermal_pattern_type_t pattern = PATTERN_CHECKERBOARD;
#elif defined(CONFIG_THERMAL_PATTERN_NOISE)
    thermal_pattern_type_t pattern = PATTERN_NOISE;
#else
    thermal_pattern_type_t pattern = PATTERN_GRADIENT;
#endif

    if (thermal_generator_init(pattern) < 0) {
        printk("FATAL: Thermal generator initialization failed\n");
        return -1;
    }
    printk("✓ Thermal generator ready (pattern=%d)\n", pattern);

    /* Start frame capture thread */
    printk("\n[3/3] Starting capture thread...\n");
    k_thread_create(&capture_thread_data, capture_stack,
                    K_THREAD_STACK_SIZEOF(capture_stack),
                    capture_thread_entry,
                    NULL, NULL, NULL,
                    CAPTURE_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&capture_thread_data, "capture");
    printk("✓ Capture thread started\n");

    printk("\n╔════════════════════════════════════════════════════════════╗\n");
    printk("║ SYSTEM READY - Generating thermal frames @ %d FPS         ║\n", FRAME_RATE_HZ);
    printk("╚════════════════════════════════════════════════════════════╝\n\n");

    /* Initialize statistics timer */
    perf_stats.last_stats_time = k_uptime_get();

    /* Main thread becomes idle (Zephyr shell will be available if enabled) */
    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}
