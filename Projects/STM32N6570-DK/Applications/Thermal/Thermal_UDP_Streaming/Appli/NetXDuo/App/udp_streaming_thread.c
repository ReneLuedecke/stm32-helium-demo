/**
 * @file    udp_streaming_thread.c
 * @brief   UDP streaming thread for thermal frames
 */

#include "app_udp_streaming.h"
#include "app_netxduo.h"
#include "thermal_processing.h"

#include <stdio.h>
#include <string.h>

extern NX_IP IpInstance;
extern NX_PACKET_POOL AppPool;

TX_THREAD UdpStreamingThread;

static NX_UDP_SOCKET UdpSocket;
static UCHAR *udp_thread_stack = NULL;

static uint32_t frame_counter = 0U;
static uint32_t total_frames_sent = 0U;
static uint32_t total_bytes_sent = 0U;

static UINT send_fragmented_frame(ULONG dest_ip, UINT dest_port, const volatile uint16_t *frame_data);

UINT App_UDP_Streaming_Init(TX_BYTE_POOL *byte_pool)
{
  UINT ret;

  ret = tx_byte_allocate(byte_pool,
                         (VOID **)&udp_thread_stack,
                         UDP_STREAMING_THREAD_STACK_SIZE,
                         TX_NO_WAIT);
  if (ret != TX_SUCCESS)
  {
    printf("[UDP] ERROR: Stack allocation failed (%u)\n", ret);
    return ret;
  }

  ret = tx_thread_create(&UdpStreamingThread,
                         "UDP Streaming Thread",
                         UDP_Streaming_Thread_Entry,
                         0UL,
                         udp_thread_stack,
                         UDP_STREAMING_THREAD_STACK_SIZE,
                         UDP_STREAMING_THREAD_PRIORITY,
                         UDP_STREAMING_THREAD_PRIORITY,
                         TX_NO_TIME_SLICE,
                         TX_AUTO_START);
  if (ret != TX_SUCCESS)
  {
    printf("[UDP] ERROR: Thread creation failed (%u)\n", ret);
    (void)tx_byte_release(udp_thread_stack);
    udp_thread_stack = NULL;
    return ret;
  }

  printf("[UDP] Thread created successfully (Prio %u)\n", UDP_STREAMING_THREAD_PRIORITY);
  return TX_SUCCESS;
}

VOID UDP_Streaming_Thread_Entry(ULONG thread_input)
{
  UINT status;
  ULONG ip_address;
  ULONG network_mask;
  (void)thread_input;

  printf("[UDP] Thread started (Prio %u)\n", UDP_STREAMING_THREAD_PRIORITY);

  tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND * 2U);

  status = nx_ip_address_get(&IpInstance, &ip_address, &network_mask);
  if (status != NX_SUCCESS)
  {
    printf("[UDP] ERROR: Failed to get IP address (%u)\n", status);
    return;
  }

  PRINT_IP_ADDRESS(ip_address);

  status = nx_udp_socket_create(&IpInstance,
                                &UdpSocket,
                                "UDP Streaming Socket",
                                NX_IP_NORMAL,
                                NX_FRAGMENT_OKAY,
                                NX_IP_TIME_TO_LIVE,
                                10U);
  if (status != NX_SUCCESS)
  {
    printf("[UDP] ERROR: Socket creation failed (%u)\n", status);
    return;
  }

  status = nx_udp_socket_bind(&UdpSocket, UDP_SERVER_PORT, TX_WAIT_FOREVER);
  if (status != NX_SUCCESS)
  {
    printf("[UDP] ERROR: Socket bind failed (%u)\n", status);
    (void)nx_udp_socket_delete(&UdpSocket);
    return;
  }

  printf("[UDP] Socket bound to port %u\n", UDP_SERVER_PORT);
  printf("[UDP] Frame size: %lu bytes (%lu packets @ %u bytes)\n",
         (unsigned long)THERMAL_FRAME_SIZE_BYTES,
         (unsigned long)PACKETS_PER_FRAME,
         UDP_PACKET_PAYLOAD_SIZE);
  printf("[UDP] Waiting for thermal frames...\n");

  while (1)
  {
    status = tx_semaphore_get(&frame_ready_semaphore, TX_WAIT_FOREVER);
    if (status != TX_SUCCESS)
    {
      printf("[UDP] ERROR: Semaphore get failed (%u)\n", status);
      continue;
    }

    ULONG dest_ip = IP_ADDRESS(192, 168, 1, 255);
    UINT dest_port = UDP_SERVER_PORT;

    status = send_fragmented_frame(dest_ip, dest_port, (const volatile uint16_t *)res);
    if (status != NX_SUCCESS)
    {
      printf("[UDP] ERROR: Frame send failed (%u)\n", status);
    }
  }
}

static UINT send_fragmented_frame(ULONG dest_ip, UINT dest_port, const volatile uint16_t *frame_data)
{
  UINT status;
  NX_PACKET *packet;
  udp_frame_header_t header;
  const uint8_t *data_ptr = (const uint8_t *)frame_data;
  uint32_t bytes_remaining = THERMAL_FRAME_SIZE_BYTES;
  uint16_t packet_num;
  uint32_t timestamp_start = tx_time_get();

  header.frame_number = frame_counter++;
  header.total_packets = (uint16_t)PACKETS_PER_FRAME;
  header.timestamp_ms = timestamp_start * 10U;

  for (packet_num = 0U; packet_num < PACKETS_PER_FRAME; packet_num++)
  {
    status = nx_packet_allocate(&AppPool, &packet, NX_UDP_PACKET, TX_WAIT_FOREVER);
    if (status != NX_SUCCESS)
    {
      printf("[UDP] ERROR: Packet allocation failed (%u)\n", status);
      return status;
    }

    header.packet_number = packet_num;

    status = nx_packet_data_append(packet,
                                   &header,
                                   sizeof(header),
                                   &AppPool,
                                   TX_WAIT_FOREVER);
    if (status != NX_SUCCESS)
    {
      printf("[UDP] ERROR: Header append failed (%u)\n", status);
      nx_packet_release(packet);
      return status;
    }

    uint32_t payload_size = (bytes_remaining > UDP_PACKET_PAYLOAD_SIZE)
                              ? UDP_PACKET_PAYLOAD_SIZE
                              : bytes_remaining;

    status = nx_packet_data_append(packet,
                                   (VOID *)data_ptr,
                                   payload_size,
                                   &AppPool,
                                   TX_WAIT_FOREVER);
    if (status != NX_SUCCESS)
    {
      printf("[UDP] ERROR: Data append failed (%u)\n", status);
      nx_packet_release(packet);
      return status;
    }

    status = nx_udp_socket_send(&UdpSocket, packet, dest_ip, dest_port);
    if (status != NX_SUCCESS)
    {
      printf("[UDP] ERROR: Send failed on packet %u (%u)\n", packet_num, status);
      nx_packet_release(packet);
      return status;
    }

    data_ptr += payload_size;
    bytes_remaining -= payload_size;
  }

  total_frames_sent++;
  total_bytes_sent += THERMAL_FRAME_SIZE_BYTES;

  uint32_t duration_ms = (tx_time_get() - timestamp_start) * 10U;

  if ((total_frames_sent % 50U) == 0U)
  {
    printf("[UDP] Frame %lu: %lu packets, %lu ms\n",
           (unsigned long)header.frame_number,
           (unsigned long)PACKETS_PER_FRAME,
           (unsigned long)duration_ms);
    printf("[UDP] Stats: %lu frames, %lu MB total\n",
           (unsigned long)total_frames_sent,
           (unsigned long)(total_bytes_sent / (1024U * 1024U)));
  }

  return NX_SUCCESS;
}
