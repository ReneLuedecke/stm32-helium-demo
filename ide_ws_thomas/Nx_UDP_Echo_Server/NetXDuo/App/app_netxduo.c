/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_netxduo.c
  * @author  MCD Application Team
  * @brief   NetXDuo applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_netxduo.h"

/* Private includes ----------------------------------------------------------*/
#include "nxd_dhcp_client.h"
/* USER CODE BEGIN Includes */
#include   "main.h"
#include "arm_mve.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TX_THREAD      NxAppThread;
NX_PACKET_POOL NxAppPool;
NX_IP          NetXDuoEthIpInstance;
TX_SEMAPHORE   DHCPSemaphore;
NX_DHCP        DHCPClient;

/* USER CODE BEGIN PV */
TX_THREAD      AppUDPThread;
TX_THREAD      AppLinkThread;

NX_UDP_SOCKET  UDPSocket;

ULONG          IpAddress;
ULONG          NetMask;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static VOID nx_app_thread_entry (ULONG thread_input);
static VOID ip_address_change_notify_callback(NX_IP *ip_instance, VOID *ptr);
/* USER CODE BEGIN PFP */
static VOID App_UDP_Thread_Entry(ULONG thread_input);
static VOID App_Link_Thread_Entry(ULONG thread_input);
UINT  _nxe_packet_data_appendth(NX_PACKET *packet_ptr, VOID *data_start, ULONG data_size,
                              NX_PACKET_POOL *pool_ptr, ULONG wait_option);

/* USER CODE END PFP */

/**
  * @brief  Application NetXDuo Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT MX_NetXDuo_Init(VOID *memory_ptr)
{
  UINT ret = NX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;
  CHAR *pointer;

  /* USER CODE BEGIN MX_NetXDuo_MEM_POOL */
  /* USER CODE END MX_NetXDuo_MEM_POOL */

  /* USER CODE BEGIN 0 */
  printf("Nx_UDP_Echo_Server application started..\n");
  /* USER CODE END 0 */

  /* Initialize the NetXDuo system. */
  nx_system_initialize();

    /* Allocate the memory for packet_pool.  */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, NX_APP_PACKET_POOL_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  /* Create the Packet pool to be used for packet allocation,
   * If extra NX_PACKET are to be used the NX_APP_PACKET_POOL_SIZE should be increased
   */
  ret = nx_packet_pool_create(&NxAppPool, "NetXDuo App Pool", DEFAULT_PAYLOAD_SIZE, pointer, NX_APP_PACKET_POOL_SIZE);

  if (ret != NX_SUCCESS)
  {
    return NX_POOL_ERROR;
  }

    /* Allocate the memory for Ip_Instance */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, Nx_IP_INSTANCE_THREAD_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

   /* Create the main NX_IP instance */
  ret = nx_ip_create(&NetXDuoEthIpInstance, "NetX Ip instance", NX_APP_DEFAULT_IP_ADDRESS, NX_APP_DEFAULT_NET_MASK, &NxAppPool, nx_stm32_eth_driver,
                     pointer, Nx_IP_INSTANCE_THREAD_SIZE, NX_APP_INSTANCE_PRIORITY);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

    /* Allocate the memory for ARP */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_ARP_CACHE_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  /* Enable the ARP protocol and provide the ARP cache size for the IP instance */

  /* USER CODE BEGIN ARP_Protocol_Initialization */

  /* USER CODE END ARP_Protocol_Initialization */

  ret = nx_arp_enable(&NetXDuoEthIpInstance, (VOID *)pointer, DEFAULT_ARP_CACHE_SIZE);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

  /* Enable the ICMP */

  /* USER CODE BEGIN ICMP_Protocol_Initialization */

  /* USER CODE END ICMP_Protocol_Initialization */

  ret = nx_icmp_enable(&NetXDuoEthIpInstance);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

  /* Enable TCP Protocol */

  /* USER CODE BEGIN TCP_Protocol_Initialization */

  /* USER CODE END TCP_Protocol_Initialization */

  ret = nx_tcp_enable(&NetXDuoEthIpInstance);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

  /* Enable the UDP protocol required for  DHCP communication */

  /* USER CODE BEGIN UDP_Protocol_Initialization */
  /* Allocate the app UDP thread entry pool. */
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, Nx_IP_INSTANCE_THREAD_SIZE, TX_NO_WAIT);

  if (ret != TX_SUCCESS)
  {
    return NX_NOT_ENABLED;
  }

  /* create the UDP server thread */
  ret = tx_thread_create(&AppUDPThread, "App UDP Thread", App_UDP_Thread_Entry, 0, pointer, Nx_IP_INSTANCE_THREAD_SIZE,
                        NX_APP_INSTANCE_PRIORITY, NX_APP_INSTANCE_PRIORITY, TX_NO_TIME_SLICE, TX_DONT_START);

  if (ret != TX_SUCCESS)
  {
    return NX_NOT_ENABLED;
  }
  /* USER CODE END UDP_Protocol_Initialization */

  ret = nx_udp_enable(&NetXDuoEthIpInstance);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

   /* Allocate the memory for main thread   */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, NX_APP_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  /* Create the main thread */
  ret = tx_thread_create(&NxAppThread, "NetXDuo App thread", nx_app_thread_entry , 0, pointer, NX_APP_THREAD_STACK_SIZE,
                         NX_APP_THREAD_PRIORITY, NX_APP_THREAD_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);

  if (ret != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }

  /* Create the DHCP client */

  /* USER CODE BEGIN DHCP_Protocol_Initialization */

  /* USER CODE END DHCP_Protocol_Initialization */

  ret = nx_dhcp_create(&DHCPClient, &NetXDuoEthIpInstance, "DHCP Client");

  if (ret != NX_SUCCESS)
  {
    return NX_DHCP_ERROR;
  }

  /* set DHCP notification callback  */
  ret = tx_semaphore_create(&DHCPSemaphore, "DHCP Semaphore", 0);

  if (ret != TX_SUCCESS)
  {
    return TX_SEMAPHORE_ERROR;
  }

  /* USER CODE BEGIN MX_NetXDuo_Init */
  /* Allocate the memory for Link thread   */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer,NX_APP_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  /* create the Link thread */
  ret = tx_thread_create(&AppLinkThread, "App Link Thread", App_Link_Thread_Entry, 0, pointer, NX_APP_THREAD_STACK_SIZE,
                         LINK_PRIORITY, LINK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);

  if (ret != TX_SUCCESS)
  {
    return NX_NOT_ENABLED;
  }
  /* USER CODE END MX_NetXDuo_Init */

  return ret;
}

/**
* @brief  ip address change callback.
* @param ip_instance: NX_IP instance
* @param ptr: user data
* @retval none
*/
static VOID ip_address_change_notify_callback(NX_IP *ip_instance, VOID *ptr)
{
  /* USER CODE BEGIN ip_address_change_notify_callback */
  /* release the semaphore as soon as an IP address is available */
  if (nx_ip_address_get(&NetXDuoEthIpInstance, &IpAddress, &NetMask) != NX_SUCCESS)
  {
    /* USER CODE BEGIN IP address change callback error */
    Error_Handler();
    /* USER CODE END IP address change callback error */
  }
  if(IpAddress != NULL_ADDRESS)
  {
    tx_semaphore_put(&DHCPSemaphore);
  }
  /* USER CODE END ip_address_change_notify_callback */
}

/**
* @brief  Main thread entry.
* @param thread_input: ULONG user argument used by the thread entry
* @retval none
*/
static VOID nx_app_thread_entry (ULONG thread_input)
{
  /* USER CODE BEGIN Nx_App_Thread_Entry 0 */

  /* USER CODE END Nx_App_Thread_Entry 0 */

  UINT ret = NX_SUCCESS;

  /* USER CODE BEGIN Nx_App_Thread_Entry 1 */

  /* USER CODE END Nx_App_Thread_Entry 1 */

  /* register the IP address change callback */
  ret = nx_ip_address_change_notify(&NetXDuoEthIpInstance, ip_address_change_notify_callback, NULL);
  if (ret != NX_SUCCESS)
  {
    /* USER CODE BEGIN IP address change callback error */
    Error_Handler();
    /* USER CODE END IP address change callback error */
  }
#if(0)
  /* start the DHCP client */
  ret = nx_dhcp_start(&DHCPClient);
  if (ret != NX_SUCCESS)
  {
    /* USER CODE BEGIN DHCP client start error */
    Error_Handler();
    /* USER CODE END DHCP client start error */
  }
  printf("Looking for DHCP server ..\n");
  /* wait until an IP address is ready */
  if(tx_semaphore_get(&DHCPSemaphore, TX_WAIT_FOREVER) != TX_SUCCESS)
  {
    /* USER CODE BEGIN DHCPSemaphore get error */
    Error_Handler();
    /* USER CODE END DHCPSemaphore get error */
  }

  /* USER CODE BEGIN Nx_App_Thread_Entry 2 */
#endif
  PRINT_IP_ADDRESS(IpAddress);

  /* Now the network is correctly initialized, start the UDP server thread */
  tx_thread_resume(&AppUDPThread);

  /* this thread is not needed any more, we relinquish it */
  tx_thread_relinquish();
  /* USER CODE END Nx_App_Thread_Entry 2 */

}
/* USER CODE BEGIN 1 */
UCHAR data_buffer[1288];
//UCHAR data_buffer2[770];
#define     PACKET_SIZE    1400
#define     POOL_SIZE       ((sizeof(NX_PACKET) + PACKET_SIZE) * 4)

#define __NON_CACHEABLE __attribute__((section(".noncacheable")))

UCHAR pool_buffer[POOL_SIZE] __NON_CACHEABLE __attribute__((aligned(8)));
//UCHAR pool_buffer[POOL_SIZE] __attribute__((aligned(8)));

#define dlen 0x4b000
extern uint32_t address;
//extern uint16_t raw[dlen] __attribute__((aligned(8)));
extern uint16_t raw[dlen];
//extern volatile uint16_t res[dlen] __NON_CACHEABLE __attribute__((aligned(8)));
extern uint16_t dark[dlen];
extern uint16_t gain[dlen];
volatile uint16_t *offset = (void*)(XSPI2_BASE + 0x6800000);
volatile uint16_t *emissivity = (void*)(XSPI2_BASE + 0x6900000);
extern uint16_t planck[0x10000];
//extern __attribute__((section(".xdata"))) uint16_t planck[0x10000];
extern uint16_t planck_offset;

extern DMA_HandleTypeDef handle_HPDMA1_Channel12;
extern __IO uint32_t transferErrorDetected;    /* Set to 1 if an error transfer is detected */
extern __IO uint32_t transferCompleteDetected; /* Set to 1 if transfer is correctly completed */

uint16_t roim[9]={0,2,1,0,0,2,1,3,0};







static VOID App_UDP_Thread_Entry(ULONG thread_input)
{
  UINT ret;
  ULONG bytes_read;
  UINT source_port,z,x,f;
  volatile UINT i;
  uint16_t alpha=0x8234;

  ULONG source_ip_address;
  NX_PACKET *data_packet;
  NX_PACKET *my_packet;
  NX_PACKET_POOL          pool_0;

	uint16_t *e;
	uint8_t *dst;
	uint32_t k;

	const uint16x8_t flag_vec = vdupq_n_u16(0xa39);

  /* create the UDP socket */
  ret = nx_udp_socket_create(&NetXDuoEthIpInstance, &UDPSocket, "UDP Server Socket", NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, QUEUE_MAX_SIZE);

  if (ret != NX_SUCCESS)
  {
     Error_Handler();
  }
  ret =  nx_packet_pool_create(&pool_0, "NetX Packet Pool", PACKET_SIZE, pool_buffer, POOL_SIZE);

  if (ret != NX_SUCCESS)
  {
     Error_Handler();
  }

  ret = nx_udp_socket_bind(&UDPSocket, 2727, TX_WAIT_FOREVER);

  if (ret != NX_SUCCESS)
  {
     Error_Handler();
  }
  else
  {
    printf("UDP Server listening on PORT %d.. \n", DEFAULT_PORT);
  }
      f=0;
      for(i=0;i<(1288>>1);i++)
      {
    	  data_buffer[2*i]=0x40;
    	  data_buffer[2*i+1]=4;
      }
      while(1)
      {
	      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13,GPIO_PIN_SET);
	  	  for (z=0;z<480;z++)
	  	  {
    		  ret =  nx_packet_allocate(&pool_0, &my_packet, NX_UDP_PACKET, TX_WAIT_FOREVER);
    		  if (ret != NX_SUCCESS)
    		  {
    			  Error_Handler();
    		  }
	 	 	  data_buffer[2]=z & 0xff;
	 	 	  data_buffer[3]=(z>>8) & 0xff;
	 	 	  dst=(uint8_t*)_nx_packet_data_appendth(my_packet, &data_buffer[0], sizeof(data_buffer), &pool_0, TX_WAIT_FOREVER);
	 	 	  memcpy(dst,data_buffer, 8);
	 	 	  dst+=8;
	 	 	  e=(uint16_t *) dst;
	 	 	  //here starts the calculation of one line
	      	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6,GPIO_PIN_SET);
	 	 	  for (uint32_t x = z*640; x < (z+1)*640; x += 8)
	 	 	  {
	 	         // ═══════════════════════════════════════════════════════════
	 	         // Load input data (8 pixels at once)
	 	         // ═══════════════════════════════════════════════════════════
	 	         uint16x8_t adc   = vld1q_u16(&raw[x]);
	 	         uint16x8_t darkv = vld1q_u16(&dark[x]);
	 	         uint16x8_t gainv = vld1q_u16(&gain[x]);
	 	         uint16x8_t offv  = vld1q_u16(&offset[x]);
	 	         uint16x8_t emiss = vld1q_u16(&emissivity[x]);

	 	         // ═══════════════════════════════════════════════════════════
	 	         // Dark frame correction
	 	         // ═══════════════════════════════════════════════════════════
//	 	         uint16x8_t corr = vqsubq_u16(adc, darkv);
	 	         uint16x8_t corr = vsubq_u16(adc, darkv);

	 	         // ═══════════════════════════════════════════════════════════
	 	         // Gain correction (Q15 signed multiply)
	 	         // ═══════════════════════════════════════════════════════════
	 	         int16x8_t  corr_s = vreinterpretq_s16_u16(corr);
	 	         int16x8_t  gain_s = vreinterpretq_s16_u16(gainv);
	 	         int16x8_t  mul_s  = vqrdmulhq_s16(corr_s, gain_s);
	 	         uint16x8_t val    = vreinterpretq_u16_s16(mul_s);

	 	         // ═══════════════════════════════════════════════════════════
	 	         // Emissivity correction (Q15 signed multiply for unsigned data)
	 	         // ═══════════════════════════════════════════════════════════
	 	         int16x8_t val_s   = vreinterpretq_s16_u16(val);
	 	         int16x8_t emiss_s = vreinterpretq_s16_u16(emiss);
	 	         int16x8_t emiss_mul = vqrdmulhq_s16(val_s, emiss_s);
	 	         val = vreinterpretq_u16_s16(emiss_mul);

	 	         // ═══════════════════════════════════════════════════════════
	 	         // Add flag reference + offset
	 	         // ═══════════════════════════════════════════════════════════
//	 	         val = vqaddq_u16(val, flag_vec);
//	 	         val = vqaddq_u16(val, offv);
	 	         val = vaddq_u16(val, flag_vec);
	 	         val = vaddq_u16(val, offv);

	 	         // ═══════════════════════════════════════════════════════════
	 	         // Planck LUT lookup
	 	         // ═══════════════════════════════════════════════════════════
	 	         uint16x8_t out = vldrhq_gather_shifted_offset_u16(planck, val);

	 	         // ═══════════════════════════════════════════════════════════
	 	         // Store result
	 	         // ═══════════════════════════════════════════════════════════
	 	         vst1q_u16(e, out);
	 	         e+=8;
	 	 	  }
	 	 	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6,GPIO_PIN_RESET);
	 	 	  if (z==f) memcpy(dst, &data_buffer[8], sizeof(data_buffer)-8);
     	      do
    	      {
     	    	  ret=nx_udp_socket_send(&UDPSocket, my_packet,0xc0a80064, 4096); //192.168.0.100
    	      }
    	      while (ret!=NX_SUCCESS);
    	      if (z==240) HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13,GPIO_PIN_RESET);
    		//  HAL_Delay(10);
	  	  }
	  	  f++;
	  	  if (f>479) f=0;
      }
}

/**
* @brief  Link thread entry
* @param thread_input: ULONG thread parameter
* @retval none
*/
static VOID App_Link_Thread_Entry(ULONG thread_input)
{
  ULONG actual_status;
  UINT linkdown = 0, status;

  while(1)
  {
    /* Send request to check if the Ethernet cable is connected. */
    status = nx_ip_interface_status_check(&NetXDuoEthIpInstance, 0, NX_IP_LINK_ENABLED,
                                      &actual_status, 10);

    if(status == NX_SUCCESS)
    {
      if(linkdown == 1)
      {
        linkdown = 0;

        /* The network cable is connected. */
        printf("The network cable is connected.\n");

        /* Send request to enable PHY Link. */
        nx_ip_driver_direct_command(&NetXDuoEthIpInstance, NX_LINK_ENABLE,
                                      &actual_status);

        /* Send request to check if an address is resolved. */
        status = nx_ip_interface_status_check(&NetXDuoEthIpInstance, 0, NX_IP_ADDRESS_RESOLVED,
                                      &actual_status, 10);
        if(status == NX_SUCCESS)
        {
          /* Stop DHCP */
          nx_dhcp_stop(&DHCPClient);

          /* Reinitialize DHCP */
          nx_dhcp_reinitialize(&DHCPClient);

          /* Start DHCP */
          nx_dhcp_start(&DHCPClient);

          /* wait until an IP address is ready */
          if(tx_semaphore_get(&DHCPSemaphore, TX_WAIT_FOREVER) != TX_SUCCESS)
          {
            /* USER CODE BEGIN DHCPSemaphore get error */
            Error_Handler();
            /* USER CODE END DHCPSemaphore get error */
          }

          PRINT_IP_ADDRESS(IpAddress);
        }
        else
        {
          /* Set the DHCP Client's remaining lease time to 0 seconds to trigger an immediate renewal request for a DHCP address. */
          nx_dhcp_client_update_time_remaining(&DHCPClient, 0);
        }
      }
    }
    else
    {
      if(0 == linkdown)
      {
        linkdown = 1;
        /* The network cable is not connected. */
        printf("The network cable is not connected.\n");
        nx_ip_driver_direct_command(&NetXDuoEthIpInstance, NX_LINK_DISABLE,
                                      &actual_status);
      }
    }

    tx_thread_sleep(NX_APP_CABLE_CONNECTION_CHECK_PERIOD);
  }
}
/* USER CODE END 1 */
