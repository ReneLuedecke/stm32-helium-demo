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
#define SENSOR_WIDTH  HPIX
#define SENSOR_HEIGHT VPIX

#define LED1_SET()    (GPIOG->BSRR = LED1_Pin)
#define LED1_RESET()  (GPIOG->BSRR = (LED1_Pin << 16))
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOG

#define CYCLES_TO_MS(cycles) ((cycles) / 600000)
#define CYCLES_TO_US(cycles) ((cycles) / 600)

#define XSPI1_MMAP_BASE   0x90000000UL
#define XSPI2_MMAP_BASE   0x70000000UL
#define XSPI_MMAP_SIZE    0x10000000UL  // 256 MB

#define TIM2_PRESCALER  9999
#define TIM2_PERIOD     799

// Forward declarations
extern TIM_HandleTypeDef htim2;

void thermal_vsync_init(void);
void thermal_vsync_start(void);
void thermal_frame_process(void);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart1;

XSPI_HandleTypeDef hxspi2;

/* USER CODE BEGIN PV */

// ═══════════════════════════════════════════════════════
// DTCM (128 KB) - Planck LUT
// ═══════════════════════════════════════════════════════

//__attribute__((section(".dtcm")))
//uint16_t planck_table[65536];  // 128 KB
__attribute__((aligned(16), section(".axisram1")))
uint16_t planck_table[65536];  // ← Move to AXISRAM! (128 KB)
// ═══════════════════════════════════════════════════════
// AXISRAM1 (1 MB) - Line processing buffers
// ═══════════════════════════════════════════════════════

__attribute__((aligned(16), section(".axisram1")))
uint16_t gain_line[HPIX];      // 1.3 KB

__attribute__((aligned(16), section(".axisram1")))
uint16_t dark_line[HPIX];      // 1.3 KB

__attribute__((aligned(16), section(".axisram1")))
uint16_t temp_line[HPIX];      // 1.3 KB

__attribute__((aligned(16), section(".axisram1")))
uint16_t line_output[HPIX];    // 1.3 KB

// Total in AXISRAM1: ~5 KB ✓

// ═══════════════════════════════════════════════════════
// Main RAM (2 MB) - Frame buffers (dual purpose!)
// ═══════════════════════════════════════════════════════

// Frame buffers for DCMIPP (noncacheable for DMA)
__attribute__((section(".noncacheable")))
uint16_t frame_buffer_A[VPIX][HPIX];  // 614 KB

__attribute__((section(".noncacheable")))
uint16_t frame_buffer_B[VPIX][HPIX];  // 614 KB

// Total: 1228 KB ✓ Fits perfectly!

// ═══════════════════════════════════════════════════════
// Aliases for Phase 1 Tests - Reuse frame buffers!
// ═══════════════════════════════════════════════════════

// Smart pointer aliasing - NO extra memory used!
#define src_u16   ((uint16_t*)frame_buffer_A)    // 614 KB
#define dst_u16   ((uint16_t*)frame_buffer_B)    // 614 KB
#define temp_u16  ((uint16_t*)frame_buffer_A)    // Alias (can share with src)

// Small gain array for full-frame benchmark (NOT frame-sized!)
__attribute__((aligned(16)))
uint16_t gain_u16[HPIX];  // Just 1.3 KB! (replicate for each line)

// Small offset array
__attribute__((aligned(16)))
uint16_t off_u16[HPIX];   // Just 1.3 KB!

// Legacy XSPI test buffers (small)
uint8_t aTxBuffer[BUFFERSIZE];
__IO uint8_t aRxBuffer[BUFFERSIZE];

uint32_t xspi_mmap_base = XSPI2_MMAP_BASE;



// Timer handle
TIM_HandleTypeDef htim2;

// Thermal pipeline variables
volatile uint32_t vsync_count = 0;
volatile uint32_t frames_processed = 0;
volatile uint8_t frame_ready = 0;

// XSPI Calibration Data Pointers
#define XSPI_BASE          0x70000000
#define XSPI_DARK_OFFSET   0x000000
#define XSPI_GAIN_OFFSET   0x096000
#define XSPI_OFFSET_OFFSET 0x12C000

volatile uint16_t (*xspi_dark)[HPIX]   = (void*)(XSPI_BASE + XSPI_DARK_OFFSET);
volatile uint16_t (*xspi_gain)[HPIX]   = (void*)(XSPI_BASE + XSPI_GAIN_OFFSET);
volatile uint16_t (*xspi_offset)[HPIX] = (void*)(XSPI_BASE + XSPI_OFFSET_OFFSET);

// DCMIPP variables
volatile uint8_t dcmipp_active_buffer = 0;
volatile uint8_t frame_ready_for_processing = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_XSPI2_Init(void);
/* USER CODE BEGIN PFP */

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

void thermal_vsync_init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    // Enable TIM2 clock
    __HAL_RCC_TIM2_CLK_ENABLE();

    // Configure Timer2
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = TIM2_PRESCALER;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = TIM2_PERIOD;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        printf("ERROR: Timer2 init failed!\n");
        Error_Handler();
    }

    // Clock source configuration
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    // Master configuration
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    // Enable Timer2 interrupt
    HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    printf("✓ Timer2 initialized: 50 Hz VSYNC\n");
}

void thermal_vsync_start(void) {
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        printf("ERROR: Timer2 start failed!\n");
        Error_Handler();
    }
    printf("✓ Timer2 started\n");
}

void thermal_frame_process(void) {
    // Placeholder - will be filled later
    frames_processed++;

    // Toggle LED to show activity
    static uint8_t led_state = 0;
    if (led_state) {
        LED1_RESET();
        led_state = 0;
    } else {
        LED1_SET();
        led_state = 1;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    vsync_count++;
    frame_ready = 1;

    // Process frame
    thermal_frame_process();
  }
}
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
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

/* USER CODE BEGIN 4 */
static inline void DWT_CycleCounter_Init(void)
{
    // Trace/DWT einschalten
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Manche MCUs haben ein verriegeltes DWT -> via LAR unlocken
    #if defined(DWT_LAR)
    DWT->LAR = 0xC5ACCE55;   // <-- richtiges Register (nicht LSR)
    #endif

    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline void process_thermal_line_fastest(
    const uint16_t * __restrict__ sensor_data,
    const uint16_t * __restrict__ dark,
    const uint16_t * __restrict__ gain,
    const uint16_t * __restrict__ offset,
    const uint16_t * __restrict__ planck_lut,
    uint16_t * __restrict__ output,
    uint32_t width)
{
	uint16_t last_valid = 0;  // <-- Carry zwischen 8er-Blöcken
    for (uint32_t x = 0; x < width; x += 8)
    {
        // Load
        uint16x8_t adc = vld1q_u16(&sensor_data[x]);
        uint16x8_t dark_val = vld1q_u16(&dark[x]);
        uint16x8_t gain_val = vld1q_u16(&gain[x]);
        uint16x8_t offset_val = vld1q_u16(&offset[x]);

        // Dark subtract
        uint16x8_t corrected = vqsubq_u16(adc, dark_val);

        // ⭐ Gain (Q15) - Reinterpret as signed, multiply, reinterpret back
        int16x8_t corrected_s = vreinterpretq_s16_u16(corrected);
        int16x8_t gain_s = vreinterpretq_s16_u16(gain_val);
        int16x8_t result_s = vqrdmulhq_s16(corrected_s, gain_s);
        uint16x8_t result = vreinterpretq_u16_s16(result_s);

        uint16_t last_valid = 0;
        // Add offset
//        result = vqaddq_u16(result, offset_val);
//        // -------- Primitive BPC (gain==0 -> vorheriges gültiges Pixel) --------
//		// Lane-Operationen mit *konstanten* Indizes (keine Loop-Variable!)
//		{
//			uint16_t l = last_valid;
//
//			uint16_t g0 = vgetq_lane_u16(gain_val, 0); uint16_t r0 = vgetq_lane_u16(result, 0); if (g0 == 0) r0 = l; else l = r0; result = vsetq_lane_u16(r0, result, 0);
//			uint16_t g1 = vgetq_lane_u16(gain_val, 1); uint16_t r1 = vgetq_lane_u16(result, 1); if (g1 == 0) r1 = l; else l = r1; result = vsetq_lane_u16(r1, result, 1);
//			uint16_t g2 = vgetq_lane_u16(gain_val, 2); uint16_t r2 = vgetq_lane_u16(result, 2); if (g2 == 0) r2 = l; else l = r2; result = vsetq_lane_u16(r2, result, 2);
//			uint16_t g3 = vgetq_lane_u16(gain_val, 3); uint16_t r3 = vgetq_lane_u16(result, 3); if (g3 == 0) r3 = l; else l = r3; result = vsetq_lane_u16(r3, result, 3);
//			uint16_t g4 = vgetq_lane_u16(gain_val, 4); uint16_t r4 = vgetq_lane_u16(result, 4); if (g4 == 0) r4 = l; else l = r4; result = vsetq_lane_u16(r4, result, 4);
//			uint16_t g5 = vgetq_lane_u16(gain_val, 5); uint16_t r5 = vgetq_lane_u16(result, 5); if (g5 == 0) r5 = l; else l = r5; result = vsetq_lane_u16(r5, result, 5);
//			uint16_t g6 = vgetq_lane_u16(gain_val, 6); uint16_t r6 = vgetq_lane_u16(result, 6); if (g6 == 0) r6 = l; else l = r6; result = vsetq_lane_u16(r6, result, 6);
//			uint16_t g7 = vgetq_lane_u16(gain_val, 7); uint16_t r7 = vgetq_lane_u16(result, 7); if (g7 == 0) r7 = l; else l = r7; result = vsetq_lane_u16(r7, result, 7);
//
//			last_valid = l;
//		}
		// ----------------------------------------------------------------------

        // Planck LUT
        result = vldrhq_gather_shifted_offset_u16(planck_lut, result);

        // Store
        vst1q_u16(&output[x], result);
    }
}

/* USER CODE END 4 */
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
  /* USER CODE BEGIN 2 */

    // ═══════════════════════════════════════════════════════════════════
    // STM32N6 Thermal Imaging System - Production Code
    // 66 FPS Processing Pipeline (31 us/line)
    // ═══════════════════════════════════════════════════════════════════

    printf("\n");
    printf("╔════════════════════════════════════════════════╗\n");
    printf("║  STM32N6 Thermal Imaging System               ║\n");
    printf("║  Production Code - 66 FPS Pipeline            ║\n");
    printf("╚════════════════════════════════════════════════╝\n");
    printf("\n");

    // ────────────────────────────────────────────────────────────────────
    // Initialize XSPI Memory-Mapped Mode
    // ────────────────────────────────────────────────────────────────────
    printf("Initializing XSPI Memory-Mapped Mode...\n");

    XSPI_NOR_OctalDTRModeCfg(&hxspi2);

    // READ configuration
    XSPI_RegularCmdTypeDef sRead = {0};
    sRead.OperationType        = HAL_XSPI_OPTYPE_READ_CFG;
    sRead.Instruction          = OCTAL_IO_DTR_READ_CMD;
    sRead.InstructionMode      = HAL_XSPI_INSTRUCTION_8_LINES;
    sRead.InstructionWidth     = HAL_XSPI_INSTRUCTION_16_BITS;
    sRead.InstructionDTRMode   = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
    sRead.AddressMode          = HAL_XSPI_ADDRESS_8_LINES;
    sRead.AddressWidth         = HAL_XSPI_ADDRESS_32_BITS;
    sRead.AddressDTRMode       = HAL_XSPI_ADDRESS_DTR_ENABLE;
    sRead.DataMode             = HAL_XSPI_DATA_8_LINES;
    sRead.DataDTRMode          = HAL_XSPI_DATA_DTR_ENABLE;
    sRead.DummyCycles          = DUMMY_CLOCK_CYCLES_READ_OCTAL;
    sRead.DQSMode              = HAL_XSPI_DQS_ENABLE;
    sRead.AlternateBytesMode   = HAL_XSPI_ALT_BYTES_NONE;

    if (HAL_XSPI_Command(&hxspi2, &sRead, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        printf("✗ XSPI READ-CFG failed\n");
        Error_Handler();
    }

    // WRITE configuration
    XSPI_RegularCmdTypeDef sWrite = {0};
    sWrite.OperationType       = HAL_XSPI_OPTYPE_WRITE_CFG;
    sWrite.Instruction         = OCTAL_PAGE_PROG_CMD;
    sWrite.InstructionMode     = HAL_XSPI_INSTRUCTION_8_LINES;
    sWrite.InstructionWidth    = HAL_XSPI_INSTRUCTION_16_BITS;
    sWrite.InstructionDTRMode  = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
    sWrite.AddressMode         = HAL_XSPI_ADDRESS_8_LINES;
    sWrite.AddressWidth        = HAL_XSPI_ADDRESS_32_BITS;
    sWrite.AddressDTRMode      = HAL_XSPI_ADDRESS_DTR_ENABLE;
    sWrite.DataMode            = HAL_XSPI_DATA_8_LINES;
    sWrite.DataDTRMode         = HAL_XSPI_DATA_DTR_ENABLE;
    sWrite.DummyCycles         = 0;
    sWrite.DQSMode             = HAL_XSPI_DQS_DISABLE;
    sWrite.AlternateBytesMode  = HAL_XSPI_ALT_BYTES_NONE;

    if (HAL_XSPI_Command(&hxspi2, &sWrite, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        printf("✗ XSPI WRITE-CFG failed\n");
        Error_Handler();
    }

    // Enter Memory-Mapped mode
    XSPI_MemoryMappedTypeDef sMMAP = {0};
    sMMAP.TimeOutActivation  = HAL_XSPI_TIMEOUT_COUNTER_ENABLE;
    sMMAP.TimeoutPeriodClock = 0x50;

    if (HAL_XSPI_MemoryMapped(&hxspi2, &sMMAP) != HAL_OK) {
        printf("✗ XSPI Memory-Mapped failed\n");
        Error_Handler();
    }

    // Configure MPU for XSPI
    xspi_mmap_base = XSPI2_MMAP_BASE;
    uint32_t xspi_mmap_end = xspi_mmap_base + XSPI_MMAP_SIZE - 1;

    ARM_MPU_Region_t mpu_table[] = {
      {
        .RBAR = ARM_MPU_RBAR(xspi_mmap_base, ARM_MPU_SH_NON, 0, 1, 0),
        .RLAR = ARM_MPU_RLAR(xspi_mmap_end, 0)
      }
    };

    ARM_MPU_Disable();
    ARM_MPU_SetMemAttr(0, ARM_MPU_ATTR(
      ARM_MPU_ATTR_MEMORY_(1,0,1,0),  // Outer: WB/WA
      ARM_MPU_ATTR_MEMORY_(1,0,1,0)   // Inner: WB/WA
    ));
    ARM_MPU_Load(0, mpu_table, 1);
    ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);
    __DSB(); __ISB();

    printf("✓ XSPI Memory-Mapped Mode ready\n");

    // ────────────────────────────────────────────────────────────────────
    // Initialize Thermal Pipeline
    // ────────────────────────────────────────────────────────────────────
    printf("\nInitializing thermal pipeline...\n");

    // Initialize DWT cycle counter
    DWT_CycleCounter_Init();

    // Initialize Planck LUT (dummy for now)
    printf("  • Planck LUT...");
    for (uint32_t i = 0; i < 65536; i++) {
        planck_table[i] = (uint16_t)i;  // TODO: Real Planck function
    }
    printf(" ✓\n");

    // Initialize test data (dummy sensor frame)
    printf("  • Test pattern...");
    for (uint32_t y = 0; y < VPIX; y++) {
        for (uint32_t x = 0; x < HPIX; x++) {
            frame_buffer_A[y][x] = 8000 + (x % 100);  // Dummy ADC values
        }
    }
    printf(" ✓\n");

    // Initialize calibration data (dummy)
    printf("  • Calibration data...");
    for (uint32_t i = 0; i < HPIX; i++) {
        dark_line[i] = 100 + (i % 10);       // Dummy dark frame
        gain_line[i] = gain_to_q15(1.0f);    // Unity gain
        // offset in temp_line reused
    }
    printf(" ✓\n");

    // ────────────────────────────────────────────────────────────────────
    // Initialize 50 Hz Timer (VSYNC simulation)
    // ────────────────────────────────────────────────────────────────────
    printf("\nInitializing 50 Hz VSYNC timer...\n");
    thermal_vsync_init();
    thermal_vsync_start();

    printf("\n");
    printf("╔════════════════════════════════════════════════╗\n");
    printf("║  System Ready!                                 ║\n");
    printf("║  Processing @ 66 FPS (31 us/line)             ║\n");
    printf("║  LED toggle @ 66 Hz (visible on oscilloscope) ║\n");
    printf("╚════════════════════════════════════════════════╝\n");
    printf("\n");

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    uint32_t frame_count = 0;
    uint32_t last_perf_check = HAL_GetTick();

    while (1)
    {
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */

      // ═══════════════════════════════════════════════════════════════════
      // Main Processing Loop - 66 FPS
      // ═══════════════════════════════════════════════════════════════════

      // Process all lines with fastest pipeline
      for (int line = 0; line < VPIX; line++) {
          process_thermal_line_fastest(
              frame_buffer_A[line],    // Raw sensor (dummy)
              dark_line,               // Dark frame
              gain_line,               // Gain coefficients (Q15)
              temp_line,               // Offset (reused buffer)
              planck_table,            // Planck LUT
              frame_buffer_B[line],    // Output
              HPIX
          );
      }

      // Toggle LED @ 66 Hz (visible on oscilloscope!)
      static uint8_t led_state = 0;
      if (led_state) {
          LED1_RESET();
          led_state = 0;
      } else {
          LED1_SET();
          led_state = 1;
      }

      frame_count++;

      // Performance stats every 5 seconds
      if (HAL_GetTick() - last_perf_check >= 5000) {
          uint32_t fps = frame_count / 5;

          printf("═══════════════════════════════════════════\n");
          printf("  Frames processed: %lu\n", frame_count);
          printf("  Average FPS: %lu\n", fps);
          printf("  VSYNC count: %lu\n", vsync_count);
          printf("═══════════════════════════════════════════\n");

          frame_count = 0;
          last_perf_check = HAL_GetTick();
      }

    }
    /* USER CODE END 3 */
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

/* === XSPI helpers (protected) ============================================ */
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
  sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
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
  sCommand.DataLength     = 1;

  if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_XSPI_AutoPolling(hxspi, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Write Configuration register 2 (with Octal I/O SPI protocol) */
  sCommand.Instruction  = WRITE_CFG_REG_2_CMD;
  sCommand.AddressMode  = HAL_XSPI_ADDRESS_1_LINE;
  sCommand.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
  sCommand.Address      = 0;
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
