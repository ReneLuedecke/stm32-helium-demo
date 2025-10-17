/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_mve.h"
#include "stdio.h"
#include <stdint.h>
#include <string.h>
#include "xspi_nor.h"
#include "stm32n6xx_nucleo_xspi.h"
#include "mx25um51245g.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HPIX 640
#define VPIX 480
#define N (HPIX * VPIX)

#define LED1_SET()    (GPIOG->BSRR = LED1_Pin)
#define LED1_RESET()  (GPIOG->BSRR = (LED1_Pin << 16))
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOG
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
XSPI_HandleTypeDef hxspi2;

/* USER CODE BEGIN PV */
__attribute__((aligned(16), section(".axisram1"))) uint16_t src_u16[N];
__attribute__((aligned(16), section(".axisram1"))) uint16_t planck_table[65536];

__attribute__((aligned(16))) uint16_t gain_u16[N];
__attribute__((aligned(16))) uint16_t dst_u16[N];
__attribute__((aligned(16))) uint16_t off_u16[N];
__attribute__((aligned(16))) uint16_t temp_u16[N];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_XSPI2_Init(void);

/* USER CODE BEGIN PFP */
static const char* bsp_err_str(int32_t e);
static inline uint16_t gain_to_q15(float gain);
void process_mve(const uint16_t *src, const uint16_t *gain, uint16_t *dst, int n, int16_t offset);
void planck_lut_mve(const uint16_t *src, uint16_t *dst, int n);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if !defined(TERMINAL_IO)
#if defined(__ICCARM__)
int iar_fputc(int ch);
#define PUTCHAR_PROTOTYPE int iar_fputc(int ch)
#elif defined ( __CC_ARM ) || defined(__ARMCC_VERSION)
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#elif defined(__GNUC__)
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#endif
#endif

static const char* bsp_err_str(int32_t e)
{
    switch(e){
        case  0:  return "BSP_ERROR_NONE";
        case -1:  return "BSP_ERROR_NO_INIT";
        case -2:  return "BSP_ERROR_WRONG_PARAM";
        case -3:  return "BSP_ERROR_BUSY";
        case -4:  return "BSP_ERROR_PERIPH_FAILURE";
        case -5:  return "BSP_ERROR_COMPONENT_FAILURE";
        case -6:  return "BSP_ERROR_UNKNOWN_FAILURE";
        case -7:  return "BSP_ERROR_FEATURE_NOT_SUPPORTED";
        case -8:  return "BSP_ERROR_TIMEOUT";
        default:  return "BSP_ERROR_OTHER";
    }
}

static inline uint16_t gain_to_q15(float gain)
{
    if (gain < 0.0f) gain = 0.0f;
    if (gain > 1.9999f) gain = 1.9999f;
    return (uint16_t)(gain * 32768.0f + 0.5f);
}

void process_mve(const uint16_t *src, const uint16_t *gain, uint16_t *dst, int n, int16_t offset)
{
    int i = 0;
    const int step = 8;
    const uint8_t shift = 15;

    const uint32x4_t rnd = vdupq_n_u32(1u << (shift - 1));
    const int32x4_t off32 = vdupq_n_s32(offset);
    const int32x4_t zeroS = vdupq_n_s32(0);
    const int32x4_t negSh = vdupq_n_s32(-15);

    for (; i <= n - step; i += step)
    {
        uint16x8_t s16 = vld1q_u16(src + i);
        uint16x8_t g16 = vld1q_u16(gain + i);

        uint32x4_t lo32 = vmullbq_int_u16(s16, g16);
        uint32x4_t hi32 = vmulltq_int_u16(s16, g16);

        lo32 = vaddq_u32(lo32, rnd);
        hi32 = vaddq_u32(hi32, rnd);

        lo32 = vshlq_u32(lo32, negSh);
        hi32 = vshlq_u32(hi32, negSh);

        int32x4_t loS = vqaddq_s32(vreinterpretq_s32_u32(lo32), off32);
        int32x4_t hiS = vqaddq_s32(vreinterpretq_s32_u32(hi32), off32);

        loS = vmaxq_s32(loS, zeroS);
        hiS = vmaxq_s32(hiS, zeroS);

        uint32x4_t loU = vreinterpretq_u32_s32(loS);
        uint32x4_t hiU = vreinterpretq_u32_s32(hiS);

        uint16x8_t out = vdupq_n_u16(0);
        out = vqmovnbq_u32(out, loU);
        out = vqmovntq_u32(out, hiU);

        vst1q_u16(dst + i, out);
    }

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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  SCB_EnableICache();
  SCB_EnableDCache();
  MPU->CTRL = 0;
  __DSB();
  __ISB();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_XSPI2_Init();

  /* USER CODE BEGIN 2 */

  printf("\n\n=== STM32N6 Thermal Camera System ===\n");
  printf("Resolution: %ux%u = %u pixels\n", HPIX, VPIX, N);
  printf("CPU: 600 MHz with ARM MVE (Helium)\n\n");

  // XSPI Flash Test
  printf("=== XSPI Flash Test ===\n");
  printf("GPION AFR[0]: 0x%08lX\n", GPION->AFR[0]);
  printf("GPION AFR[1]: 0x%08lX\n", GPION->AFR[1]);

  BSP_XSPI_NOR_Init_t xspi_init;
  xspi_init.InterfaceMode = BSP_XSPI_NOR_SPI_MODE;
  xspi_init.TransferRate  = BSP_XSPI_NOR_STR_TRANSFER;

  int32_t ret = BSP_XSPI_NOR_Init(0, &xspi_init);
  printf("BSP_XSPI_NOR_Init: %ld (%s)\n", ret, bsp_err_str(ret));

  uint8_t id[3] = {0};
  ret = BSP_XSPI_NOR_ReadID(0, id);
  printf("Flash ID: %02X %02X %02X (rc=%ld)\n", id[0], id[1], id[2], ret);

  if (id[0] == 0xC2 && id[1] == 0x85 && id[2] == 0x3B) {
      printf("✓ Flash OK: Macronix MX25UM51245G\n\n");
  } else {
      printf("✗ Flash detection failed\n\n");
  }

  // Initialize Planck LUT
  printf("Initializing Planck LUT...\n");
  for (uint32_t i = 0; i < 65536; i++) {
      planck_table[i] = (uint16_t)i;
  }

  // Initialize test pattern
  printf("Generating test pattern...\n");
  for (uint32_t i = 0; i < N; i++) {
      uint32_t x = i % HPIX;
      src_u16[i] = (uint16_t)((x * 16383) / HPIX);
      gain_u16[i] = gain_to_q15(1.0f);
  }

  // Benchmark
  printf("\n=== Performance Benchmark ===\n");
  __DSB();
  __ISB();
  DWT->CTRL |= 1;
  uint32_t t0 = DWT->CYCCNT;

  process_mve(src_u16, gain_u16, temp_u16, N, 0);
  uint32_t t1 = DWT->CYCCNT;

  planck_lut_mve(temp_u16, dst_u16, N);
  uint32_t t2 = DWT->CYCCNT;

  uint32_t cyc_gain = (t1 - t0);
  uint32_t cyc_planck = (t2 - t1);
  uint32_t cyc_total = (t2 - t0);

  printf("Gain/Offset: %u cycles (%u ms)\n", cyc_gain, cyc_gain / 600000);
  printf("Planck LUT:  %u cycles (%u ms)\n", cyc_planck, cyc_planck / 600000);
  printf("Total:       %u cycles (%u ms)\n", cyc_total, cyc_total / 600000);
  printf("FPS:         %u\n\n", 600000000 / cyc_total);

  // LED Test
  printf("Testing LED...\n");
  for (int i = 0; i < 3; i++) {
      LED1_SET();
      HAL_Delay(200);
      LED1_RESET();
      HAL_Delay(200);
  }
  printf("LED OK!\n\n");

  printf("Starting continuous processing...\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t led_state = 0;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    process_mve(src_u16, gain_u16, temp_u16, N, 0);
    planck_lut_mve(temp_u16, dst_u16, N);

    if (led_state) {
        LED1_RESET();
        led_state = 0;
    } else {
        LED1_SET();
        led_state = 1;
    }
  }
  /* USER CODE END 3 */
}

/* System Clock and Peripheral Init functions - CubeMX generated */
/* ... (der Rest bleibt gleich) ... */

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
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}
#endif
/* USER CODE END 4 */

/* Error Handler */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
