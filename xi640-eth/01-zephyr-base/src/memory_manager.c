/**
 * @file memory_manager.c
 * @brief Memory manager implementation
 */

#include "memory_manager.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/flash.h>
#include <string.h>

/* Memory region descriptors */
static memory_region_t regions[4] = {
    { .base_addr = (void *)AXISRAM1_BASE, .size = AXISRAM_SIZE, .used = 0, .name = "AXISRAM1" },
    { .base_addr = (void *)AXISRAM2_BASE, .size = AXISRAM_SIZE, .used = 0, .name = "AXISRAM2" },
    { .base_addr = (void *)AXISRAM3_BASE, .size = AXISRAM_SIZE, .used = 0, .name = "AXISRAM3" },
    { .base_addr = (void *)HYPERRAM_BASE, .size = HYPERRAM_SIZE, .used = 0, .name = "HYPERRAM" }
};

/* Frame pool (allocated from HyperRAM) */
static frame_pool_t frame_pool;

/* Mutex for frame pool access */
K_MUTEX_DEFINE(frame_pool_mutex);

/**
 * @brief Align value to specified alignment
 */
static inline uint32_t align_up(uint32_t value, uint32_t align)
{
    return (value + align - 1) & ~(align - 1);
}

/**
 * @brief Find frame index by pointer
 */
static int find_frame_index(thermal_frame_t *frame)
{
    for (int i = 0; i < FRAME_POOL_COUNT; i++) {
        if (frame_pool.frames[i] == frame) {
            return i;
        }
    }
    return -1;
}

/* ========================================
 * PUBLIC API IMPLEMENTATION
 * ======================================== */

int memory_manager_init(void)
{
    printk("Initializing memory manager...\n");

    /* Allocate frame pool from HyperRAM */
    for (int i = 0; i < FRAME_POOL_COUNT; i++) {
        frame_pool.frames[i] = (thermal_frame_t *)memory_allocate(3, sizeof(thermal_frame_t), 64);
        if (!frame_pool.frames[i]) {
            printk("ERROR: Failed to allocate frame %d\n", i);
            return -1;
        }
        frame_pool.locks[i] = false;
        memset(frame_pool.frames[i], 0, sizeof(thermal_frame_t));
    }

    frame_pool.current_capture_idx = 0;
    frame_pool.current_process_idx = 0;
    frame_pool.current_display_idx = 0;

    printk("Frame pool allocated: %d frames × %u bytes = %u KB\n",
           FRAME_POOL_COUNT, sizeof(thermal_frame_t),
           (FRAME_POOL_COUNT * sizeof(thermal_frame_t)) / 1024);

    memory_print_map();
    return 0;
}

void *memory_allocate(uint8_t region, uint32_t size, uint32_t align)
{
    if (region >= 4) return NULL;

    memory_region_t *reg = &regions[region];

    /* Align current position */
    uint32_t aligned_used = align_up(reg->used, align);

    /* Check if enough space */
    if (aligned_used + size > reg->size) {
        printk("ERROR: %s out of memory (need %u, have %u)\n",
               reg->name, size, reg->size - aligned_used);
        return NULL;
    }

    void *ptr = (uint8_t *)reg->base_addr + aligned_used;
    reg->used = aligned_used + size;

    return ptr;
}

frame_pool_t *memory_get_frame_pool(void)
{
    return &frame_pool;
}

thermal_frame_t *memory_acquire_capture_frame(void)
{
    k_mutex_lock(&frame_pool_mutex, K_FOREVER);

    /* Find next unlocked frame for capture */
    for (int i = 0; i < FRAME_POOL_COUNT; i++) {
        uint8_t idx = (frame_pool.current_capture_idx + i) % FRAME_POOL_COUNT;
        if (!frame_pool.locks[idx]) {
            frame_pool.locks[idx] = true;
            frame_pool.current_capture_idx = (idx + 1) % FRAME_POOL_COUNT;
            k_mutex_unlock(&frame_pool_mutex);
            return frame_pool.frames[idx];
        }
    }

    k_mutex_unlock(&frame_pool_mutex);
    return NULL; /* All frames locked */
}

thermal_frame_t *memory_acquire_process_frame(void)
{
    k_mutex_lock(&frame_pool_mutex, K_FOREVER);

    /* Get frame that was last captured */
    uint8_t idx = (frame_pool.current_capture_idx - 1 + FRAME_POOL_COUNT) % FRAME_POOL_COUNT;
    if (frame_pool.locks[idx]) {
        frame_pool.current_process_idx = idx;
        k_mutex_unlock(&frame_pool_mutex);
        return frame_pool.frames[idx];
    }

    k_mutex_unlock(&frame_pool_mutex);
    return NULL;
}

thermal_frame_t *memory_acquire_display_frame(void)
{
    k_mutex_lock(&frame_pool_mutex, K_FOREVER);

    /* Get frame that was last processed */
    uint8_t idx = frame_pool.current_process_idx;
    if (frame_pool.locks[idx]) {
        frame_pool.current_display_idx = idx;
        k_mutex_unlock(&frame_pool_mutex);
        return frame_pool.frames[idx];
    }

    k_mutex_unlock(&frame_pool_mutex);
    return NULL;
}

void memory_release_frame(thermal_frame_t *frame)
{
    k_mutex_lock(&frame_pool_mutex, K_FOREVER);

    int idx = find_frame_index(frame);
    if (idx >= 0) {
        frame_pool.locks[idx] = false;
    }

    k_mutex_unlock(&frame_pool_mutex);
}

void memory_get_stats(uint8_t region, uint32_t *used, uint32_t *total)
{
    if (region >= 4) return;

    if (used) *used = regions[region].used;
    if (total) *total = regions[region].size;
}

int memory_write_calibration(uint32_t offset, const void *data, uint32_t size)
{
    /* Direct memory-mapped write to HyperFlash */
    volatile uint8_t *flash_addr = (volatile uint8_t *)(HYPERFLASH_BASE + offset);

    /* TODO: In production, add sector erase + write verification */
    memcpy((void *)flash_addr, data, size);

    printk("Calibration data written: offset=0x%08X, size=%u bytes\n", offset, size);
    return 0;
}

int memory_read_calibration(uint32_t offset, void *data, uint32_t size)
{
    /* Direct memory-mapped read from HyperFlash */
    volatile uint8_t *flash_addr = (volatile uint8_t *)(HYPERFLASH_BASE + offset);

    memcpy(data, (void *)flash_addr, size);

    return 0;
}

void memory_print_map(void)
{
    printk("\n=== MEMORY MAP ===\n");
    printk("Region       Base        Size      Used      Free      Usage\n");
    printk("----------------------------------------------------------------\n");

    for (int i = 0; i < 4; i++) {
        memory_region_t *reg = &regions[i];
        uint32_t free = reg->size - reg->used;
        uint32_t usage_percent = (reg->used * 100) / reg->size;

        printk("%-12s 0x%08X  %7u  %7u  %7u  %3u%%\n",
               reg->name,
               (uint32_t)reg->base_addr,
               reg->size,
               reg->used,
               free,
               usage_percent);
    }

    printk("\nFrame Pool: %d frames × %u bytes (HyperRAM)\n",
           FRAME_POOL_COUNT, sizeof(thermal_frame_t));
    printk("  Capture idx: %d\n", frame_pool.current_capture_idx);
    printk("  Process idx: %d\n", frame_pool.current_process_idx);
    printk("  Display idx: %d\n", frame_pool.current_display_idx);
    printk("  Locks: [%d %d %d]\n",
           frame_pool.locks[0], frame_pool.locks[1], frame_pool.locks[2]);
    printk("================================================================\n\n");
}
