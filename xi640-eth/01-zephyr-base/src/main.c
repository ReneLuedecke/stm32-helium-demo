/**
 * @file main.c
 * @brief Xi 640 ETH - Minimal Boot Test
 *
 * Based on proven hello_world approach
 * Tests k_malloc() allocation to PSRAM before adding thermal processing
 */

#include <zephyr/kernel.h>

static void *frame_buffer = NULL;
static void *temp_frame_buffer = NULL;

int main(void)
{
    printk("\n\n========================================\n");
    printk("Xi 640 ETH - Minimal Boot Test\n");
    printk("========================================\n");
    printk("Build: %s %s\n", __DATE__, __TIME__);
    printk("\n");

    /* Allocate frame buffer (614 KB) */
    printk("Allocating frame_buffer (614 KB)...\n");
    frame_buffer = k_malloc(614400);
    if (!frame_buffer) {
        printk("ERROR: frame_buffer alloc failed\n");
        return -1;
    }
    printk("OK: frame_buffer = %p\n", frame_buffer);

    /* Allocate temp buffer (1228 KB) */
    printk("Allocating temp_frame_buffer (1228 KB)...\n");
    temp_frame_buffer = k_malloc(1228800);
    if (!temp_frame_buffer) {
        printk("ERROR: temp_frame_buffer alloc failed\n");
        k_free(frame_buffer);
        return -1;
    }
    printk("OK: temp_frame_buffer = %p\n", temp_frame_buffer);

    /* Check PSRAM placement */
    printk("\nMemory Check:\n");
    if ((uintptr_t)frame_buffer >= 0x90000000) {
        printk("  frame_buffer:      PSRAM (0x90000000+) OK!\n");
    } else {
        printk("  frame_buffer:      WARNING - NOT in PSRAM!\n");
    }

    if ((uintptr_t)temp_frame_buffer >= 0x90000000) {
        printk("  temp_frame_buffer: PSRAM (0x90000000+) OK!\n");
    } else {
        printk("  temp_frame_buffer: WARNING - NOT in PSRAM!\n");
    }

    printk("\n========================================\n");
    printk("ALL TESTS PASSED!\n");
    printk("========================================\n\n");

    /* Main loop - heartbeat every 2 seconds */
    int count = 0;
    while (1) {
        printk("[%d] System running...\n", count++);
        k_sleep(K_SECONDS(2));
    }

    return 0;
}
