/*
 * Copyright (c) 2025 Xi 640 ETH Project
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>

int main(void)
{
	printf("Hello World! Xi 640 ETH - STM32N6570-DK\n");
	printf("UART Test - If you see this, it works!\n");
	printf("\n");

	int counter = 0;
	while (1) {
		printf("Counter: %d\n", counter++);
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
