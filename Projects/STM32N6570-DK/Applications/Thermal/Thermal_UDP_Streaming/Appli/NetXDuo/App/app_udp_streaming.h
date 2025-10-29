/**
 * @file    app_udp_streaming.h
 * @brief   UDP streaming thread interface for thermal frames
 */

#ifndef APP_UDP_STREAMING_H
#define APP_UDP_STREAMING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tx_api.h"
#include "nx_api.h"
#include "thermal_processing.h"

#define UDP_SERVER_PORT           6100U
#define UDP_PACKET_PAYLOAD_SIZE   1400U
#define THERMAL_FRAME_SIZE_BYTES  FRAME_SIZE_BYTES
#define PACKETS_PER_FRAME         ((THERMAL_FRAME_SIZE_BYTES + UDP_PACKET_PAYLOAD_SIZE - 1U) / UDP_PACKET_PAYLOAD_SIZE)

#define UDP_STREAMING_THREAD_PRIORITY    8U
#define UDP_STREAMING_THREAD_STACK_SIZE  4096U

typedef struct
{
  uint32_t frame_number;
  uint16_t packet_number;
  uint16_t total_packets;
  uint32_t timestamp_ms;
} udp_frame_header_t;

extern TX_THREAD UdpStreamingThread;
extern TX_SEMAPHORE frame_ready_semaphore;

UINT App_UDP_Streaming_Init(TX_BYTE_POOL *byte_pool);
VOID UDP_Streaming_Thread_Entry(ULONG thread_input);

#ifdef __cplusplus
}
#endif

#endif /* APP_UDP_STREAMING_H */
