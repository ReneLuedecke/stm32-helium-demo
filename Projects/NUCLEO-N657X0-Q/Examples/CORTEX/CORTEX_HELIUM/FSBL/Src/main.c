/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "arm_mve.h"
#include "stdio.h"
#include <stdint.h>
#include <string.h>
#include "xspi_nor.h"

/* Private defines -----------------------------------------------------------*/



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

UART_HandleTypeDef huart1;
XSPI_HandleTypeDef hxspi2;

#define HPIX 640
#define VPIX 480
#define N (HPIX * VPIX)


//__attribute__((aligned(16), section(".axisram1"))) uint16_t src_u16[N];
//__attribute__((aligned(16), section(".axisram1"))) uint16_t gain_u16[N];
//__attribute__((aligned(16), section(".axisram1"))) uint16_t off_u16[N];
//__attribute__((aligned(16), section(".axisram1"))) uint16_t dst_u16[N];
//__attribute__((aligned(16), section(".axisram1"))) uint16_t temp_u16[N];
//__attribute__((aligned(16), section(".axisram1"))) uint16_t planck_table[65536];

__attribute__((aligned(16), section(".axisram1"))) uint16_t src_u16[N];  // 614 KB
__attribute__((aligned(16), section(".axisram1"))) uint16_t planck_table[65536];  // 131 KB

/* Cold data in RAM (2048 KB) */
__attribute__((aligned(16))) uint16_t gain_u16[N];  // 614 KB
__attribute__((aligned(16))) uint16_t dst_u16[N];   // 614 KB
__attribute__((aligned(16))) uint16_t off_u16[N];
__attribute__((aligned(16))) uint16_t temp_u16[N];

#define LED1_SET()    (GPIOG->BSRR = LED1_Pin)
#define LED1_RESET()  (GPIOG->BSRR = (LED1_Pin << 16))
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOG
#define BUFFERSIZE_XSPI                  256


/* XSPI Test Buffers */
uint8_t testTxBuffer[BUFFERSIZE_XSPI];   // Zu schreibende Daten
uint8_t testRxBuffer[BUFFERSIZE_XSPI];   // Gelesene Daten



static void process_image_full(void);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_XSPI2_Init(void);
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Definitions for UART terminal */
#if !defined(TERMINAL_IO)

UART_HandleTypeDef huart1;
static void MX_USART1_UART_Init(void);

#if defined(__ICCARM__)
__ATTRIBUTES size_t __write(int, const unsigned char *, size_t);
#endif /* __ICCARM__ */

#if defined(__ICCARM__)
/* New definition from EWARM V9, compatible with EWARM8 */
int iar_fputc(int ch);
#define PUTCHAR_PROTOTYPE int iar_fputc(int ch)
#elif defined ( __CC_ARM ) || defined(__ARMCC_VERSION)
/* ARM Compiler 5/6*/
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#elif defined(__GNUC__)
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#endif /* __ICCARM__ */
#endif /* TERMINAL_IO */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

/* USER CODE BEGIN PFP */
void process_mve(const uint16_t *src, const uint16_t *gain,
                              uint16_t *dst, int n, int16_t offset)
{
  int i = 0;
  const int step = 8;
  const uint8_t shift = 15;  // Q15 format

  // Constants for SIMD
  const uint32x4_t rnd = vdupq_n_u32(1u << (shift - 1));  // Rounding: 0x4000
  const int32x4_t off32 = vdupq_n_s32(offset);
  const int32x4_t zeroS = vdupq_n_s32(0);
  const int32x4_t negSh = vdupq_n_s32(-15);

  // SIMD loop: process 8 pixels at a time
  for (; i <= n - step; i += step)
  {
    uint16x8_t s16 = vld1q_u16(src + i);
    uint16x8_t g16 = vld1q_u16(gain + i);

    // 16x16 -> 32-bit multiply (TRUE 32-bit intermediate!)
    uint32x4_t lo32 = vmullbq_int_u16(s16, g16);  // Lower 4 elements
    uint32x4_t hi32 = vmulltq_int_u16(s16, g16);  // Upper 4 elements

    // Add rounding
    lo32 = vaddq_u32(lo32, rnd);
    hi32 = vaddq_u32(hi32, rnd);

    // Right shift by 15 (Q15 division)
    lo32 = vshlq_u32(lo32, negSh);
    hi32 = vshlq_u32(hi32, negSh);

    // Add offset with saturation
    int32x4_t loS = vqaddq_s32(vreinterpretq_s32_u32(lo32), off32);
    int32x4_t hiS = vqaddq_s32(vreinterpretq_s32_u32(hi32), off32);

    // Clamp to [0, 65535]
    loS = vmaxq_s32(loS, zeroS);
    hiS = vmaxq_s32(hiS, zeroS);

    // Convert back to u32 for narrow
    uint32x4_t loU = vreinterpretq_u32_s32(loS);
    uint32x4_t hiU = vreinterpretq_u32_s32(hiS);

    // Narrow: u32 -> u16 with saturation
    uint16x8_t out = vdupq_n_u16(0);
    out = vqmovnbq_u32(out, loU);  // Bottom 4 lanes
    out = vqmovntq_u32(out, hiU);  // Top 4 lanes

    vst1q_u16(dst + i, out);
  }

  // Scalar tail for remaining pixels
  for (; i < n; ++i) {
    uint32_t prod = (uint32_t)src[i] * (uint32_t)gain[i];
    uint32_t rounded = prod + (1u << (shift - 1));
    uint32_t shifted = rounded >> shift;
    int32_t result = (int32_t)shifted + (int32_t)offset;

    if (result < 0) result = 0;
    if (result > 65535) result = 65535;

    dst[i] = (uint16_t)result;
  }
}

void planck_lut(const uint16_t *src, uint16_t *dst, int n)
{
  for (int i = 0; i < n; i++) {
    dst[i] = planck_table[src[i]];
  }
}

void planck_lut_mve(const uint16_t *src, uint16_t *dst, int n)
{
    int i = 0;
    for (; i <= n - 8; i += 8)
    {
        uint16x8_t idx = vld1q_u16(src + i);
        uint16x8_t val = vldrhq_gather_shifted_offset_u16(planck_table, idx);
        vst1q_u16(dst + i, val);
    }
    for (; i < n; ++i) dst[i] = planck_table[src[i]];
}

static inline uint16_t gain_to_q15(float gain)
{
  if (gain < 0.0f) gain = 0.0f;
  if (gain > 1.9999f) gain = 1.9999f;
  return (uint16_t)(gain * 32768.0f + 0.5f);
}

int main(void)
{
  // System init
  SCB_EnableICache();
  SCB_EnableDCache();
  HAL_Init();
  MPU->CTRL = 0;
  __DSB();
  __ISB();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_XSPI2_Init();
  MX_USART1_UART_Init();

  /* ============================================ */
  /* XSPI Initialisierung                         */
  /* ============================================ */
  printf("\n\n");
  printf("================================================\n");
  printf("  XSPI NOR Flash Test\n");
  printf("================================================\n\n");

  printf("Initializing XSPI2 NOR Flash...\n");
  XSPI_NOR_Init_All();
  printf("XSPI2 initialized successfully!\n\n");

  /* ============================================ */
  /* XSPI Test: Write & Read 256 Bytes           */
  /* ============================================ */

  printf("================================================\n");
  printf("  TEST: Write 256 bytes to XSPI\n");
  printf("================================================\n");

  // 1. Testdaten vorbereiten
  printf("Preparing test data (0x00 to 0xFF)...\n");
  for (int i = 0; i < 256; i++) {
    testTxBuffer[i] = (uint8_t)i;
  }
  printf("Test data ready!\n\n");

  // 2. Speicher löschen
  printf("Erasing sector at address 0x00000000...\n");
  if (XSPI_NOR_EraseSector(0x00000000) != HAL_OK) {
    printf("ERROR: Erase failed!\n");
    Error_Handler();
  }
  printf("Erase successful!\n\n");

  // 3. Schreiben
  printf("Writing 256 bytes to address 0x00000000...\n");
  if (XSPI_NOR_Write(testTxBuffer, 0x00000000, BUFFERSIZE_XSPI) != HAL_OK) {
    printf("ERROR: Write failed!\n");
    Error_Handler();
  }
  printf("Write successful!\n\n");

  // 4. Lesen
  printf("Reading 256 bytes from address 0x00000000...\n");
  if (XSPI_NOR_Read(testRxBuffer, 0x00000000, BUFFERSIZE_XSPI) != HAL_OK) {
    printf("ERROR: Read failed!\n");
    Error_Handler();
  }
  printf("Read successful!\n\n");

  // 5. Vergleichen
  printf("================================================\n");
  printf("  Verification\n");
  printf("================================================\n");

  int errors = 0;
  for (int i = 0; i < BUFFERSIZE_XSPI; i++) {
    if (testTxBuffer[i] != testRxBuffer[i]) {
      printf("ERROR at byte %d: wrote 0x%02X, read 0x%02X\n",
             i, testTxBuffer[i], testRxBuffer[i]);
      errors++;
    }
  }

  if (errors == 0) {
    printf("\n✓ SUCCESS! All 256 bytes match!\n");
    printf("XSPI NOR Flash is working correctly.\n\n");
  } else {
    printf("\n✗ FAILURE! %d byte(s) mismatch!\n\n", errors);
    Error_Handler();
  }

  printf("First 16 bytes written:  ");
  for (int i = 0; i < 16; i++) {
    printf("%02X ", testTxBuffer[i]);
  }
  printf("\n");

  printf("First 16 bytes read:     ");
  for (int i = 0; i < 16; i++) {
    printf("%02X ", testRxBuffer[i]);
  }
  printf("\n\n");

  printf("Test complete!\n");
  printf("================================================\n\n");

  printf("\n");
  printf("================================================\n");
  printf("  STM32N6 14-Bit Sensor\n");
  printf("  Format: Q15 (Gain 0.0 - 2.0)\n");
  printf("================================================\n");
  printf("Resolution: %ux%u = %u pixels\n", HPIX, VPIX, N);
  printf("CPU:        600 MHz\n");
  printf("SIMD:       ARM MVE (Helium)\n");
  printf("Memory:     AXISRAM1\n\n");

  /* ============================================ */
  /* STEP 1: Initialize Planck Lookup Table      */
  /* ============================================ */

  printf("Initializing Planck lookup table...\n");

  // Example: Simple gamma correction (you can replace with real Planck)
  for (uint32_t i = 0; i < 65536; i++) {
    // Linear for now - replace with your Planck formula
    planck_table[i] = (uint16_t)i;

    // Example gamma 2.2:
    // float normalized = (float)i / 65535.0f;
    // float gamma = powf(normalized, 1.0f / 2.2f);
    // planck_table[i] = (uint16_t)(gamma * 65535.0f);
  }

  printf("Planck table ready!\n\n");

  /* ============================================ */
  /* STEP 2: Setup Test Image                    */
  /* ============================================ */

  printf("Generating test pattern...\n");

  // Simulate 14-bit sensor data
  for (uint32_t i = 0; i < N; i++) {
    // Test pattern: gradient
    uint32_t x = i % HPIX;
    uint32_t y = i / HPIX;
    src_u16[i] = (uint16_t)((x * 16383) / HPIX);  // 0 to 16383

    // Per-pixel gain (uniform for now)
    float gain_float = 1.0f;  // Unity gain
    gain_u16[i] = gain_to_q15(gain_float);
  }

  printf("Test pattern ready!\n\n");

  /* ============================================ */
  /* STEP 3: Benchmark                           */
  /* ============================================ */

  printf("================================================\n");
  printf("  BENCHMARK: Full Pipeline\n");
  printf("================================================\n\n");

  printf("Test 1: Unity gain (1.0)\n");
  for (uint32_t i = 0; i < N; i++) {
    gain_u16[i] = gain_to_q15(1.0f);  // 0x8000
  }

  __DSB();
  __ISB();
  DWT->CTRL |= 1;
  uint32_t t0 = DWT->CYCCNT;

  // Step 1: Apply gain + offset
  process_mve(src_u16, gain_u16, temp_u16, N, 0);


  uint32_t t1 = DWT->CYCCNT;

  //planck_lut(temp_u16, dst_u16, N);
  planck_lut_mve(temp_u16, dst_u16, N);

  uint32_t t2 = DWT->CYCCNT;

  uint32_t cyc_gain = (t1 - t0);
  uint32_t cyc_planck = (t2 - t1);
  uint32_t cyc_total = (t2 - t0);

  printf("  Gain/Offset: %u cycles (%u ms)\n",
         (unsigned)cyc_gain, (unsigned)(cyc_gain / 600000));
  printf("  Planck LUT:  %u cycles (%u ms)\n",
         (unsigned)cyc_planck, (unsigned)(cyc_planck / 600000));
  printf("  Total:       %u cycles (%u ms)\n",
         (unsigned)cyc_total, (unsigned)(cyc_total / 600000));
  printf("  FPS:         %u\n\n",
         (unsigned)(600000000 / cyc_total));

  printf("================================================\n");
  printf("  CONTINUOUS PROCESSING\n");
  printf("================================================\n");
  printf("Running at maximum speed (~23 FPS)\n");
  printf("LED toggles every frame\n");
  printf("Format: Q15 (gain 0.0 - 2.0)\n\n");

  // LED Test
  printf("Testing LED...\n");
  for (int i = 0; i < 3; i++) {
    LED1_SET();
    HAL_Delay(200);
    LED1_RESET();
    HAL_Delay(200);
  }
  printf("LED OK!\n\n");

  // Setup for continuous operation
  for (uint32_t i = 0; i < N; i++) {
    src_u16[i] = 8000 + (i % 8000);
    gain_u16[i] = gain_to_q15(1.0f);
  }

  uint32_t frame_count = 0;
  uint8_t led_state = 0;

  while (1)
  {
    // Process frame at maximum speed
    process_mve(src_u16, gain_u16, temp_u16, N, 0);
    planck_lut_mve(temp_u16, dst_u16, N);

    // Toggle LED every 10 frames (~2 Hz, visible)
    //frame_count++;
    //if (frame_count >= 10) {
      if (led_state) {
        LED1_RESET();
        led_state = 0;
      } else {
        LED1_SET();
        led_state = 1;
      }
      //frame_count = 0;
    //}
  }
}
/* USER CODE BEGIN CLK 1 */

/*         The system Clock is configured as follows :
              CPU Clock source               = IC1_CK
              System bus Clock source        = IC2_IC6_IC11_CK
              CPUCLK (sysa_ck) (Hz)          = 600000000
              SYSCLK AXI (sysb_ck) (Hz)      = 400000000
              SYSCLK NPU (sysc_ck) (Hz)      = 300000000
              SYSCLK AXISRAM3/4/5/6 (sysd_ck) (Hz) = 400000000
              HCLKx(Hz)                      = 200000000
              PCLKx(Hz)                      = 200000000
              AHB Prescaler                  = 2
              APB1 Prescaler                 = 1
              APB2 Prescaler                 = 1
              APB4 Prescaler                 = 1
              APB5 Prescaler                 = 1
              PLL1 State                     = ON
              PLL1 clock source              = HSI
              PLL1 M                         = 4
              PLL1 N                         = 75
              PLL1 P1                        = 1
              PLL1 P2                        = 1
              PLL1 FRACN                     = 0
              PLL2 State                     = BYPASS
              PLL2 clock source              = HSI
              PLL3 State                     = BYPASS
              PLL3 clock source              = HSI
              PLL4 State                     = BYPASS
              PLL4 clock source              = HSI
*/
/* USER CODE END CLK 1 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the System Power Supply
  */
  if (HAL_PWREx_ConfigSupply(PWR_EXTERNAL_SOURCE_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }

  /* Enable HSI */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Get current CPU/System buses clocks configuration and if necessary switch
 to intermediate HSI clock to ensure target clock can be set
  */
  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct);
  if ((RCC_ClkInitStruct.CPUCLKSource == RCC_CPUCLKSOURCE_IC1) ||
     (RCC_ClkInitStruct.SYSCLKSource == RCC_SYSCLKSOURCE_IC2_IC6_IC11))
  {
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK);
    RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
    {
      /* Initialization Error */
      Error_Handler();
    }
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
  RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL1.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL1.PLLM = 4;
  RCC_OscInitStruct.PLL1.PLLN = 75;
  RCC_OscInitStruct.PLL1.PLLFractional = 0;
  RCC_OscInitStruct.PLL1.PLLP1 = 1;
  RCC_OscInitStruct.PLL1.PLLP2 = 1;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_CPUCLK|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2|RCC_CLOCKTYPE_PCLK5
                              |RCC_CLOCKTYPE_PCLK4;
  RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_IC1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_IC2_IC6_IC11;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
  RCC_ClkInitStruct.APB5CLKDivider = RCC_APB5_DIV1;
  RCC_ClkInitStruct.IC1Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC1Selection.ClockDivider = 2;
  RCC_ClkInitStruct.IC2Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC2Selection.ClockDivider = 3;
  RCC_ClkInitStruct.IC6Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC6Selection.ClockDivider = 4;
  RCC_ClkInitStruct.IC11Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC11Selection.ClockDivider = 3;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Configure GPIO pin Output Level */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#if !defined(TERMINAL_IO)

#if defined(__ICCARM__)
size_t __write(int file, unsigned char const *ptr, size_t len)
{
  size_t idx;
  unsigned char const *pdata = ptr;

  for (idx = 0; idx < len; idx++)
  {
    iar_fputc((int)*pdata);
    pdata++;
  }
  return len;
}
#endif /* __ICCARM__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of putchar here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
#endif /* TERMINAL_IO */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
/**
  * @brief XSPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_XSPI2_Init(void)
{
  XSPIM_CfgTypeDef sXspiManagerCfg = {0};

  /* XSPI2 parameter configuration*/
  hxspi2.Instance = XSPI2;
  hxspi2.Init.FifoThresholdByte = 4;
  hxspi2.Init.MemoryMode = HAL_XSPI_SINGLE_MEM;
  hxspi2.Init.MemoryType = HAL_XSPI_MEMTYPE_MACRONIX;
  hxspi2.Init.MemorySize = HAL_XSPI_SIZE_512MB;
  hxspi2.Init.ChipSelectHighTimeCycle = 1;
  hxspi2.Init.FreeRunningClock = HAL_XSPI_FREERUNCLK_DISABLE;
  hxspi2.Init.ClockMode = HAL_XSPI_CLOCK_MODE_0;
  hxspi2.Init.WrapSize = HAL_XSPI_WRAP_NOT_SUPPORTED;
  hxspi2.Init.ClockPrescaler = 2;  // ← ÄNDERN von 1 zu 2 (600MHz/3 = 200MHz)
  hxspi2.Init.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_HALFCYCLE;  // ← ÄNDERN für DTR
  hxspi2.Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_ENABLE;
  hxspi2.Init.ChipSelectBoundary = HAL_XSPI_BONDARYOF_NONE;
  hxspi2.Init.MaxTran = 0;
  hxspi2.Init.Refresh = 0;
  hxspi2.Init.MemorySelect = HAL_XSPI_CSSEL_NCS1;

  if (HAL_XSPI_Init(&hxspi2) != HAL_OK) {
    Error_Handler();
  }

  sXspiManagerCfg.nCSOverride = HAL_XSPI_CSSEL_OVR_NCS1;
  sXspiManagerCfg.IOPort = HAL_XSPIM_IOPORT_2;
  sXspiManagerCfg.Req2AckTime = 1;

  if (HAL_XSPIM_Config(&hxspi2, &sXspiManagerCfg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    Error_Handler();
  }
}
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
