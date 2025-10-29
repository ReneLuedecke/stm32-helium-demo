/**
 * @file    app_thermal.h
 * @brief   Thermal processing thread interface
 */

#ifndef APP_THERMAL_H
#define APP_THERMAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tx_api.h"

#define THERMAL_EVENT_FRAME_READY  0x01UL

extern TX_THREAD ThermalThread;
extern TX_EVENT_FLAGS_GROUP thermal_event_flags;
extern TX_SEMAPHORE frame_ready_semaphore;

UINT App_Thermal_Init(TX_BYTE_POOL *byte_pool);
VOID Thermal_Thread_Entry(ULONG thread_input);

#ifdef __cplusplus
}
#endif

#endif /* APP_THERMAL_H */
