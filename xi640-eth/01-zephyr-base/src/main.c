/**
 * @file main.c
 * @brief Xi 640 ETH - Session 1: Minimal UART Test
 *
 * MINIMAL VERSION: Test if UART console works at all
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>

int main(void)
{
    int counter = 0;

    /* Wait a bit for UART to be ready */
    k_msleep(100);

    printk("\n\n");
    printk("==========================================================\n");
    printk("  Xi 640 ETH - UART TEST\n");
    printk("  Board: STM32N6570-DK\n");
    printk("  Zephyr: %s\n", KERNEL_VERSION_STRING);
    printk("==========================================================\n");
    printk("\n");

    printk("If you see this, UART is working!\n");
    printk("\n");

    while (1) {
        printk("Counter: %d\n", counter++);
        k_msleep(1000);
    }

    return 0;
}
