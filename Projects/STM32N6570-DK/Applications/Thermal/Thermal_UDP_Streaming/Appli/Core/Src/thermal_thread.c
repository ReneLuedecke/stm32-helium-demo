/**
 * @file    thermal_thread.c
 * @brief   ThreadX thermal processing thread implementation
 */

#include "tx_api.h"
#include "thermal_processing.h"
#include "xspi_nor.h"
#include "stm32n6570_discovery.h"
#include <stdio.h>

// LED Fallback-Defines
#ifndef LED_ORANGE
  #define LED_ORANGE 0
#endif
#ifndef LED_GREEN
  #define LED_GREEN 1
#endif
#ifndef LED_RED
  #define LED_RED 2
#endif

#define THERMAL_THREAD_STACK_SIZE  4096
#define THERMAL_THREAD_PRIORITY    5

TX_THREAD ThermalThread;
TX_EVENT_FLAGS_GROUP thermal_event_flags;
TX_SEMAPHORE frame_ready_semaphore;

static UCHAR thermal_thread_stack[THERMAL_THREAD_STACK_SIZE];

VOID Thermal_Thread_Entry(ULONG thread_input)
{
  UINT status;
  ULONG actual_flags;

  (void)thread_input;

  printf("[Thermal] Thread started (Prio %d)\n", THERMAL_THREAD_PRIORITY);

  // XSPI Init (stub)
  XSPI_NOR_Init_All();

  // Load calibration (stub for now)
  printf("[Thermal] Using default calibration (STUB)\n");
  thermal_init();

  printf("[Thermal] Initialization complete. Waiting for frames...\n");
  BSP_LED_On(LED_ORANGE);

  while (1)
  {
    status = tx_event_flags_get(&thermal_event_flags,
                                0x01,
                                TX_OR_CLEAR,
                                &actual_flags,
                                TX_WAIT_FOREVER);
    if (status != TX_SUCCESS)
    {
      printf("[Thermal] ERROR: Event wait failed (%u)\n", status);
      continue;
    }

    BSP_LED_Toggle(LED_ORANGE);

    // Process frame (stub for now)
    thermal_frame_process();

    // Signal UDP thread
    status = tx_semaphore_put(&frame_ready_semaphore);
    if (status != TX_SUCCESS)
    {
      printf("[Thermal] ERROR: Semaphore put failed (%u)\n", status);
    }
  }
}

UINT App_Thermal_Init(TX_BYTE_POOL *byte_pool)
{
  UINT ret;
  (void)byte_pool;

  ret = tx_event_flags_create(&thermal_event_flags, "Thermal Events");
  if (ret != TX_SUCCESS)
  {
    printf("[Thermal] ERROR: Event flags creation failed (%u)\n", ret);
    return ret;
  }

  ret = tx_semaphore_create(&frame_ready_semaphore, "Frame Ready", 0);
  if (ret != TX_SUCCESS)
  {
    printf("[Thermal] ERROR: Semaphore creation failed (%u)\n", ret);
    return ret;
  }

  ret = tx_thread_create(&ThermalThread,
                         "Thermal Thread",
                         Thermal_Thread_Entry,
                         0,
                         thermal_thread_stack,
                         THERMAL_THREAD_STACK_SIZE,
                         THERMAL_THREAD_PRIORITY,
                         THERMAL_THREAD_PRIORITY,
                         TX_NO_TIME_SLICE,
                         TX_AUTO_START);
  if (ret != TX_SUCCESS)
  {
    printf("[Thermal] ERROR: Thread creation failed (%u)\n", ret);
    return ret;
  }

  printf("[Thermal] Thread created successfully\n");
  return TX_SUCCESS;
}
