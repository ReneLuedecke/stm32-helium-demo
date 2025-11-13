/**
 * @file memory_manager.h
 * @brief Memory management for Xi 640 ETH thermal pipeline
 *
 * Manages memory allocation across:
 * - AXISRAM1 (1MB): DMA buffers for DCMIPP capture
 * - AXISRAM2 (1MB): Processing buffers (gain, dark frame, offset)
 * - AXISRAM3 (1MB): Bad pixel correction data
 * - HyperRAM (64MB): Frame pool for triple buffering
 * - HyperFlash (128MB): Calibration data persistence
 */

#ifndef MEMORY_MANAGER_H
#define MEMORY_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "thermal_frame_generator.h"

/* Memory region base addresses (from device tree) */
#define AXISRAM1_BASE  0x24000000  /* DMA buffers */
#define AXISRAM2_BASE  0x24100000  /* Processing */
#define AXISRAM3_BASE  0x24200000  /* Calibration */
#define HYPERRAM_BASE  0x90000000  /* Frame pool */
#define HYPERFLASH_BASE 0x70000000 /* Persistent storage */

/* Region sizes */
#define AXISRAM_SIZE   (1024 * 1024)  /* 1 MB each */
#define HYPERRAM_SIZE  (64 * 1024 * 1024)   /* 64 MB */
#define HYPERFLASH_SIZE (128 * 1024 * 1024) /* 128 MB */

/* Frame buffer constants */
#define FRAME_SIZE_BYTES (640 * 480 * 2)  /* 614,400 bytes per frame */
#define FRAME_POOL_COUNT 3                /* Triple buffering */

/* Memory pool structure */
typedef struct {
    void *base_addr;
    uint32_t size;
    uint32_t used;
    const char *name;
} memory_region_t;

/* Frame pool management */
typedef struct {
    thermal_frame_t *frames[FRAME_POOL_COUNT];
    uint8_t current_capture_idx;
    uint8_t current_process_idx;
    uint8_t current_display_idx;
    bool locks[FRAME_POOL_COUNT];
} frame_pool_t;

/**
 * @brief Initialize all memory regions
 * @return 0 on success, negative on error
 */
int memory_manager_init(void);

/**
 * @brief Allocate memory from specific region
 * @param region Region identifier (0=AXISRAM1, 1=AXISRAM2, 2=AXISRAM3, 3=HYPERRAM)
 * @param size Size in bytes
 * @param align Alignment requirement (must be power of 2)
 * @return Pointer to allocated memory, or NULL on failure
 */
void *memory_allocate(uint8_t region, uint32_t size, uint32_t align);

/**
 * @brief Get frame pool instance
 * @return Pointer to frame pool
 */
frame_pool_t *memory_get_frame_pool(void);

/**
 * @brief Acquire frame for capture (from DMA)
 * @return Pointer to frame, or NULL if none available
 */
thermal_frame_t *memory_acquire_capture_frame(void);

/**
 * @brief Acquire frame for processing
 * @return Pointer to frame, or NULL if none available
 */
thermal_frame_t *memory_acquire_process_frame(void);

/**
 * @brief Acquire frame for display/transmission
 * @return Pointer to frame, or NULL if none available
 */
thermal_frame_t *memory_acquire_display_frame(void);

/**
 * @brief Release frame back to pool
 * @param frame Frame to release
 */
void memory_release_frame(thermal_frame_t *frame);

/**
 * @brief Get memory usage statistics
 * @param region Region identifier
 * @param used Output: bytes used
 * @param total Output: total bytes
 */
void memory_get_stats(uint8_t region, uint32_t *used, uint32_t *total);

/**
 * @brief Write calibration data to HyperFlash
 * @param offset Offset in bytes from HyperFlash base
 * @param data Source data
 * @param size Data size in bytes
 * @return 0 on success, negative on error
 */
int memory_write_calibration(uint32_t offset, const void *data, uint32_t size);

/**
 * @brief Read calibration data from HyperFlash
 * @param offset Offset in bytes from HyperFlash base
 * @param data Destination buffer
 * @param size Data size in bytes
 * @return 0 on success, negative on error
 */
int memory_read_calibration(uint32_t offset, void *data, uint32_t size);

/**
 * @brief Print memory map and usage statistics
 */
void memory_print_map(void);

#endif /* MEMORY_MANAGER_H */
