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

#define CYCLES_TO_MS(cycles) ((cycles) / 600000)
#define CYCLES_TO_US(cycles) ((cycles) / 600)
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
uint8_t aTxBuffer[BUFFERSIZE];

/* Buffer used for reception */
__IO uint8_t aRxBuffer[BUFFERSIZE];
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
static void XSPI_WriteEnable(XSPI_HandleTypeDef *hxspi);
static void XSPI_AutoPollingMemReady(XSPI_HandleTypeDef *hxspi);
static void XSPI_NOR_OctalDTRModeCfg(XSPI_HandleTypeDef *hxspi);
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

//static const char* bsp_err_str(int32_t e)
//{
//    switch(e){
//        case  0:  return "BSP_ERROR_NONE";
//        case -1:  return "BSP_ERROR_NO_INIT";
//        case -2:  return "BSP_ERROR_WRONG_PARAM";
//        case -3:  return "BSP_ERROR_BUSY";
//        case -4:  return "BSP_ERROR_PERIPH_FAILURE";
//        case -5:  return "BSP_ERROR_COMPONENT_FAILURE";
//        case -6:  return "BSP_ERROR_UNKNOWN_FAILURE";
//        case -7:  return "BSP_ERROR_FEATURE_NOT_SUPPORTED";
//        case -8:  return "BSP_ERROR_TIMEOUT";
//        default:  return "BSP_ERROR_OTHER";
//    }
//}
void XSPI_PerformanceTest(void)
{
    printf("\n================================================\n");
    printf("  XSPI NOR FLASH PERFORMANCE TESTS\n");
    printf("================================================\n\n");

    XSPI_RegularCmdTypeDef sCommand = {0};
    uint32_t cycles_start, cycles_end, cycles_elapsed;
    uint32_t test_sizes[] = {256, 512, 1024, 4096};
    int test_idx;

    printf("Clock: 600 MHz (1 cycle = 1.67ns)\n");
    printf("Buffer location: AXISRAM1\n\n");

    /* Test 1: Sequential Write Performance */
    printf("TEST 1: Sequential Write Performance\n");
    printf("------------------------------------\n");

    for (test_idx = 0; test_idx < 4; test_idx++) {
        uint32_t size = test_sizes[test_idx];

        /* Prepare data */
        for (uint32_t i = 0; i < size; i++) {
            aTxBuffer[i] = (uint8_t)(i & 0xFF);
        }

        /* Erase sector first */
        XSPI_WriteEnable(&hxspi2);
        sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
        sCommand.Instruction        = OCTAL_SECTOR_ERASE_CMD;
        sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
        sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
        sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
        sCommand.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
        sCommand.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
        sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
        sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
        sCommand.DataDTRMode        = HAL_XSPI_DATA_DTR_ENABLE;
        sCommand.DataMode           = HAL_XSPI_DATA_NONE;
        sCommand.Address            = 0;
        sCommand.DummyCycles        = 0;
        sCommand.DQSMode            = HAL_XSPI_DQS_ENABLE;
        HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
        XSPI_AutoPollingMemReady(&hxspi2);

        /* Write Enable */
        XSPI_WriteEnable(&hxspi2);

        /* Measure write time */
        sCommand.Instruction = OCTAL_PAGE_PROG_CMD;
        sCommand.DataMode    = HAL_XSPI_DATA_8_LINES;
        sCommand.DataLength  = size;
        sCommand.Address     = 0;

        HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);

        /* Start timer */
        DWT->CTRL |= 1;
        cycles_start = DWT->CYCCNT;

        HAL_XSPI_Transmit(&hxspi2, aTxBuffer, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);

        cycles_end = DWT->CYCCNT;
        cycles_elapsed = cycles_end - cycles_start;

        XSPI_AutoPollingMemReady(&hxspi2);

        uint32_t bandwidth_kbps = (size * 600000) / (cycles_elapsed / 1000);
        printf("  %u bytes:  %u cycles (%u ms) = %u KB/s\n",
               size, cycles_elapsed, CYCLES_TO_MS(cycles_elapsed), bandwidth_kbps / 1024);
    }

    printf("\n");

    /* Test 2: Sequential Read Performance */
    printf("TEST 2: Sequential Read Performance\n");
    printf("-----------------------------------\n");

    for (test_idx = 0; test_idx < 4; test_idx++) {
        uint32_t size = test_sizes[test_idx];

        sCommand.Instruction        = OCTAL_IO_DTR_READ_CMD;
        sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
        sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
        sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
        sCommand.Address            = 0;
        sCommand.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
        sCommand.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
        sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
        sCommand.DataMode           = HAL_XSPI_DATA_8_LINES;
        sCommand.DataDTRMode        = HAL_XSPI_DATA_DTR_ENABLE;
        sCommand.DataLength         = size;
        sCommand.DummyCycles        = DUMMY_CLOCK_CYCLES_READ_OCTAL;
        sCommand.DQSMode            = HAL_XSPI_DQS_ENABLE;

        HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);

        /* Start timer */
        cycles_start = DWT->CYCCNT;

        HAL_XSPI_Receive(&hxspi2, aRxBuffer, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);

        cycles_end = DWT->CYCCNT;
        cycles_elapsed = cycles_end - cycles_start;

        uint32_t bandwidth_kbps = (size * 600000) / (cycles_elapsed / 1000);
        printf("  %u bytes:  %u cycles (%u ms) = %u KB/s\n",
               size, cycles_elapsed, CYCLES_TO_MS(cycles_elapsed), bandwidth_kbps / 1024);
    }

    printf("\n");

    /* Test 3: Verify data integrity */
    printf("TEST 3: Data Integrity Verification\n");
    printf("------------------------------------\n");

    int errors = 0;
    for (uint32_t i = 0; i < BUFFERSIZE; i++) {
        if (aRxBuffer[i] != aTxBuffer[i]) {
            errors++;
        }
    }

    if (errors == 0) {
        printf("  All %u bytes verified: PASS\n", BUFFERSIZE);

    } else {
        printf("  FAIL: %d byte errors\n", errors);

    }

    printf("\n================================================\n");
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
    XSPI_RegularCmdTypeDef sCommand = {0};
  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/
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
  printf("XSPI Debug startet...\r\n");
    /* USER CODE BEGIN 2 */

    /* Initialize Transmission and Reception buffer ----------------------------- */
    for (int i = 0; i < BUFFERSIZE; i++)
    {
      aTxBuffer[i] = i;
      aRxBuffer[i] = 0;
    }
  XSPI_NOR_OctalDTRModeCfg(&hxspi2);

    /* Erasing Sequence --------------------------------------------------------- */
    XSPI_WriteEnable(&hxspi2);

    sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
    sCommand.Instruction        = OCTAL_SECTOR_ERASE_CMD;
    sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
    sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
    sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
    sCommand.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
    sCommand.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
    sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
    sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    sCommand.DataDTRMode        = HAL_XSPI_DATA_DTR_ENABLE;
    sCommand.DataMode           = HAL_XSPI_DATA_NONE;
    sCommand.Address            = 0;
    sCommand.DummyCycles        = 0;
    sCommand.DQSMode            = HAL_XSPI_DQS_ENABLE;

    if (HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      Error_Handler();
    }

    /* Configure automatic polling mode to wait for end of erase ---------------- */
    XSPI_AutoPollingMemReady(&hxspi2);

    /* Writing Sequence --------------------------------------------------------- */
    XSPI_WriteEnable(&hxspi2);

    sCommand.Instruction = OCTAL_PAGE_PROG_CMD;
    sCommand.DataMode    = HAL_XSPI_DATA_8_LINES;
    sCommand.DataLength  = BUFFERSIZE;
    sCommand.Address     = 0;

    if (HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      Error_Handler();
    }

    if (HAL_XSPI_Transmit(&hxspi2, aTxBuffer, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      Error_Handler();
    }

    /* Configure automatic polling mode to wait for end of program -------------- */
    XSPI_AutoPollingMemReady(&hxspi2);

    /* Reading Sequence --------------------------------------------------------- */
    sCommand.Instruction        = OCTAL_IO_DTR_READ_CMD;
    sCommand.DummyCycles        = DUMMY_CLOCK_CYCLES_READ_OCTAL;
    sCommand.DataLength         = BUFFERSIZE;

    if (HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      Error_Handler();
    }

    if (HAL_XSPI_Receive(&hxspi2, (uint8_t *)aRxBuffer, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      Error_Handler();
    }

    /* Result comparison -------------------------------------------------------- */
      for (int i = 0; i < BUFFERSIZE; i++)
      {
        if (aRxBuffer[i] != aTxBuffer[i])
        {
          /* Turn LED_RED on */
          //BSP_LED_On(LED_RED);
          //errorBuffer++;
        }
      }
      XSPI_PerformanceTest();
    /* DeInit XSPI */
    HAL_XSPI_DeInit(&hxspi2);


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
/* USER CODE BEGIN CLK 1 */
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV4;
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
  * @brief XSPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_XSPI2_Init(void)
{

  /* USER CODE BEGIN XSPI2_Init 0 */

  /* USER CODE END XSPI2_Init 0 */

  XSPIM_CfgTypeDef sXspiManagerCfg = {0};

  /* USER CODE BEGIN XSPI2_Init 1 */

  /* USER CODE END XSPI2_Init 1 */
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
  hxspi2.Init.ClockPrescaler = 1;
  hxspi2.Init.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_NONE;
  hxspi2.Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_ENABLE;
  hxspi2.Init.ChipSelectBoundary = HAL_XSPI_BONDARYOF_NONE;
  hxspi2.Init.MaxTran = 0;
  hxspi2.Init.Refresh = 0;
  hxspi2.Init.MemorySelect = HAL_XSPI_CSSEL_NCS1;
  if (HAL_XSPI_Init(&hxspi2) != HAL_OK)
  {
    Error_Handler();
  }
  sXspiManagerCfg.nCSOverride = HAL_XSPI_CSSEL_OVR_NCS1;
  sXspiManagerCfg.IOPort = HAL_XSPIM_IOPORT_2;
  sXspiManagerCfg.Req2AckTime = 1;
  if (HAL_XSPIM_Config(&hxspi2, &sXspiManagerCfg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN XSPI2_Init 2 */

  /* USER CODE END XSPI2_Init 2 */

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
  __HAL_RCC_GPION_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

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
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}
#endif
static void XSPI_WriteEnable(XSPI_HandleTypeDef *hxspi)
{
  XSPI_RegularCmdTypeDef  sCommand = {0};
  XSPI_AutoPollingTypeDef sConfig  = {0};

  /* Enable write operations ------------------------------------------ */
  sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommand.Instruction        = OCTAL_WRITE_ENABLE_CMD;
  sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
  sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
  sCommand.AddressMode        = HAL_XSPI_ADDRESS_NONE;
  sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommand.DataMode           = HAL_XSPI_DATA_NONE;
  sCommand.DummyCycles        = 0;
  sCommand.DQSMode            = HAL_XSPI_DQS_DISABLE;

  if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure automatic polling mode to wait for write enabling ---- */
  sCommand.Instruction        = OCTAL_READ_STATUS_REG_CMD;
  sCommand.Address            = 0x0;
  sCommand.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
  sCommand.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
  sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
  sCommand.DataMode           = HAL_XSPI_DATA_8_LINES;
  sCommand.DataDTRMode        = HAL_XSPI_DATA_DTR_ENABLE;
  sCommand.DataLength         = 2;
  sCommand.DummyCycles        = DUMMY_CLOCK_CYCLES_READ_OCTAL;
  sCommand.DQSMode            = HAL_XSPI_DQS_ENABLE;

  if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.MatchMode           = HAL_XSPI_MATCH_MODE_AND;
  sConfig.AutomaticStop       = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
  sConfig.IntervalTime        = AUTO_POLLING_INTERVAL;
  sConfig.MatchMask           = WRITE_ENABLE_MASK_VALUE;
  sConfig.MatchValue          = WRITE_ENABLE_MATCH_VALUE;

  if (HAL_XSPI_AutoPolling(hxspi, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
}

static void XSPI_AutoPollingMemReady(XSPI_HandleTypeDef *hxspi)
{
  XSPI_RegularCmdTypeDef  sCommand = {0};
  XSPI_AutoPollingTypeDef sConfig  = {0};

  /* Configure automatic polling mode to wait for memory ready ------ */
  sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommand.Instruction        = OCTAL_READ_STATUS_REG_CMD;
  sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
  sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
  sCommand.Address            = 0x0;
  sCommand.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
  sCommand.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
  sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
  sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommand.DataMode           = HAL_XSPI_DATA_8_LINES;
  sCommand.DataDTRMode        = HAL_XSPI_DATA_DTR_ENABLE;
  sCommand.DataLength         = 2;
  sCommand.DummyCycles        = DUMMY_CLOCK_CYCLES_READ_OCTAL;
  sCommand.DQSMode            = HAL_XSPI_DQS_ENABLE;

  if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.MatchMode           = HAL_XSPI_MATCH_MODE_AND;
  sConfig.AutomaticStop       = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
  sConfig.IntervalTime        = AUTO_POLLING_INTERVAL;
  sConfig.MatchMask           = MEMORY_READY_MASK_VALUE;
  sConfig.MatchValue          = MEMORY_READY_MATCH_VALUE;

  if (HAL_XSPI_AutoPolling(hxspi, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
* @brief  This function configure the memory in Octal DTR mode.
* @param  hxspi: XSPI handle
* @retval None
*/
static void XSPI_NOR_OctalDTRModeCfg(XSPI_HandleTypeDef *hxspi)
{
  uint8_t reg = 0;
  XSPI_RegularCmdTypeDef  sCommand = {0};
  XSPI_AutoPollingTypeDef sConfig  = {0};

  sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_1_LINE;
  sCommand.InstructionWidth    = HAL_XSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_DISABLE;
  sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommand.DataDTRMode        = HAL_XSPI_DATA_DTR_DISABLE;
  sCommand.DummyCycles        = 0;
  sCommand.DQSMode            = HAL_XSPI_DQS_DISABLE;
  sConfig.MatchMode           = HAL_XSPI_MATCH_MODE_AND;
  sConfig.AutomaticStop       = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
  sConfig.IntervalTime        = 0x10;


  /* Enable write operations */
  sCommand.Instruction = WRITE_ENABLE_CMD;
  sCommand.DataMode    = HAL_XSPI_DATA_NONE;
  sCommand.AddressMode = HAL_XSPI_ADDRESS_NONE;

  if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Reconfigure XSPI to automatic polling mode to wait for write enabling */
  sConfig.MatchMask           = 0x02;
  sConfig.MatchValue          = 0x02;

  sCommand.Instruction    = READ_STATUS_REG_CMD;
  sCommand.DataMode       = HAL_XSPI_DATA_1_LINE;
  sCommand.DataLength         = 1;

  if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_XSPI_AutoPolling(hxspi, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Write Configuration register 2 (with Octal I/O SPI protocol) */
  sCommand.Instruction = WRITE_CFG_REG_2_CMD;
  sCommand.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
  sCommand.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
  sCommand.Address = 0;
  reg = 0x2;


  if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_XSPI_Transmit(hxspi, &reg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  sCommand.Instruction    = READ_STATUS_REG_CMD;
  sCommand.DataMode       = HAL_XSPI_DATA_1_LINE;
  sCommand.DataLength     = 1;

  if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_XSPI_AutoPolling(hxspi, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

}


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
#ifdef USE_FULL_ASSERT
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
