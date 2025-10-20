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



#define TEST_LINES 10u
#define LINE_BYTES (HPIX * sizeof(uint16_t))

// A) Realistic-Test: getrennte Arrays in SRAM (AXISRAM1)
__attribute__((aligned(32), section(".axisram1")))
uint16_t sensor_data[TEST_LINES * HPIX];

__attribute__((aligned(32), section(".axisram1")))
uint16_t dark_frame[TEST_LINES * HPIX];

__attribute__((aligned(32), section(".axisram1")))
uint16_t gain_coeff[TEST_LINES * HPIX];

__attribute__((aligned(32), section(".axisram1")))
uint16_t offset_val[TEST_LINES * HPIX];

// B) Thomas-Test: 4+4 interleaved in SRAM (dark+gain) und XSPI (offset+emi)
__attribute__((aligned(32), section(".axisram1")))
uint16_t dark_gain_4x4[TEST_LINES * HPIX * 2u];   // pro 8 Pixel 16 Halfwords -> *2

// XSPI MMAP: als Pointer definieren, kein RAM nötig
const uint16_t *offset_emi_4x4 = (const uint16_t*)XSPI2_MMAP_BASE;

// C) LUTs: Planck und inverse Planck
// planck_table existiert bei dir bereits in .axisram1 (128 KB)
__attribute__((aligned(16), section(".axisram1")))
uint16_t inv_planck_table[65536];  // zweite LUT (ebenfalls 128 KB)

// ===== DWT-Zähler einmalig vorbereiten =====


// ===== Testdaten-Init (Dummy-Inhalte) =====
static void init_test_data(void)
{
    // simple Muster
    for (uint32_t l = 0; l < TEST_LINES; ++l) {
        uint16_t *s = &sensor_data[l*HPIX];
        uint16_t *d = &dark_frame[l*HPIX];
        uint16_t *g = &gain_coeff[l*HPIX];
        uint16_t *o = &offset_val[l*HPIX];

        for (uint32_t x = 0; x < HPIX; ++x) {
            s[x] = (uint16_t)((x + l) & 0x0FFF);
            d[x] = (uint16_t)((x & 0x00FF) >> 1);
            g[x] = 0x6000;            // ≈0.75 in Q15
            o[x] = 32;                // kleiner Offset
        }
    }

    // inverse/Planck LUTs dummy (echte Werte später!)
    for (uint32_t i = 0; i < 65536; ++i) {
        inv_planck_table[i] = (uint16_t)i;
        // planck_table[] hast du bereits definiert; zur Not:
        // planck_table[i] = (uint16_t)i;
    }

    // Thomas‘ 4+4-Layout aus dark_frame/gain_coeff für die 10 Zeilen bauen:
    // pro 8 Pixel: [D0..D3, G0..G3, D4..D7, G4..G7] => 16 Halfwords
    for (uint32_t l = 0; l < TEST_LINES; ++l) {
        const uint16_t *d = &dark_frame[l*HPIX];
        const uint16_t *g = &gain_coeff[l*HPIX];
        uint16_t *dg = &dark_gain_4x4[l*HPIX*2u];

        for (uint32_t x = 0; x < HPIX; x += 8) {
            // Block 0: D0..D3, G0..G3
            dg[0] = d[x+0]; dg[1] = d[x+1]; dg[2] = d[x+2]; dg[3] = d[x+3];
            dg[4] = g[x+0]; dg[5] = g[x+1]; dg[6] = g[x+2]; dg[7] = g[x+3];
            // Block 1: D4..D7, G4..G7
            dg[8]  = d[x+4]; dg[9]  = d[x+5]; dg[10] = d[x+6]; dg[11] = d[x+7];
            dg[12] = g[x+4]; dg[13] = g[x+5]; dg[14] = g[x+6]; dg[15] = g[x+7];

            dg += 16;
        }
    }
}

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

/* Debug / Helper prototypes (kept inside USER CODE blocks) */
void XSPI_PerformanceTest(void);
void test_memory_mapped_detailed(void);
void check_xspi_clock(void);
void xspi_quick_status(void);
void xspi_print_all_registers(void);
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

// ... existing thermal functions ...

void thermal_check_timer_clock(void) {
    // Get actual APB1 timer clock
    uint32_t apb1_clock = HAL_RCC_GetPCLK1Freq();
    uint32_t tim_clock = apb1_clock;

    // Check if APB1 prescaler > 1, then timer clock = 2 × APB1
    RCC_ClkInitTypeDef clk_config;
    uint32_t flash_latency;
    HAL_RCC_GetClockConfig(&clk_config);

    if (clk_config.APB1CLKDivider != RCC_APB1_DIV1) {
        tim_clock = apb1_clock * 2;
    }

    printf("\n=== Timer Clock Debug ===\n");
    printf("  APB1 Clock: %lu Hz (%lu MHz)\n", apb1_clock, apb1_clock / 1000000);
    printf("  Timer Clock: %lu Hz (%lu MHz)\n", tim_clock, tim_clock / 1000000);
    printf("  Current Prescaler: %lu\n", TIM2_PRESCALER);
    printf("  Current Period: %lu\n", TIM2_PERIOD);

    // Calculate actual frequency
    uint32_t timer_freq = tim_clock / (TIM2_PRESCALER + 1) / (TIM2_PERIOD + 1);
    printf("  Calculated Timer Freq: %lu Hz\n", timer_freq);
    printf("  Target: 50 Hz\n");

    // Calculate correct prescaler for 50 Hz
    uint32_t correct_prescaler = (tim_clock / 10000) - 1;  // 10 kHz intermediate
    printf("\n  RECOMMENDED Prescaler: %lu\n", correct_prescaler);
    printf("  RECOMMENDED Period: 199\n");
    printf("  This will give: %lu Hz\n", tim_clock / (correct_prescaler + 1) / 200);
}

/* USER CODE END 0 */
void XSPI_PerformanceTest(void)
{
    // Nutze temp_u16 als Test-Buffer (ist schon im RAM!)
    uint8_t *test_tx = (uint8_t*)temp_u16;
    uint8_t *test_rx = (uint8_t*)dst_u16;

    printf("\n================================================\n");
    printf("  XSPI NOR FLASH PERFORMANCE TESTS\n");
    printf("================================================\n\n");

    XSPI_RegularCmdTypeDef sCommand = {0};
    uint32_t cycles_start, cycles_end, cycles_elapsed;
    uint32_t test_sizes[] = {256, 512, 1024, 4096};
    uint32_t test_addresses[] = {0x0000, 0x1000, 0x2000, 0x3000};
    int test_idx;

    printf("Clock: 600 MHz\n");
    printf("Buffer: Reusing temp_u16/dst_u16\n");
    printf("Mode: OPI DTR (8-line DDR)\n\n");

    /* ==================== TEST 1: WRITE ==================== */
    printf("TEST 1: Write Performance\n");
    printf("-------------------------\n");

    for (test_idx = 0; test_idx < 4; test_idx++) {
        printf("[DEBUG] Test %d starting...\n", test_idx);

        uint32_t size = test_sizes[test_idx];
        uint32_t addr = test_addresses[test_idx];

        printf("[DEBUG] Preparing %u bytes...\n", size);
        for (uint32_t i = 0; i < size; i++) {
            test_tx[i] = (uint8_t)(i & 0xFF);
        }

        printf("[DEBUG] Write Enable...\n");
        XSPI_WriteEnable(&hxspi2);

        printf("[DEBUG] Erasing sector @ 0x%04X...\n", addr);
        sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
        sCommand.Instruction        = OCTAL_SECTOR_ERASE_CMD;
        sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
        sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
        sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
        sCommand.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
        sCommand.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
        sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
        sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
        sCommand.DataMode           = HAL_XSPI_DATA_NONE;
        sCommand.DataDTRMode        = HAL_XSPI_DATA_DTR_ENABLE;
        sCommand.Address            = addr;
        sCommand.DummyCycles        = 0;
        sCommand.DQSMode            = HAL_XSPI_DQS_ENABLE;

        if (HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            printf("[ERROR] Erase command failed!\n");
            return;
        }

        printf("[DEBUG] Waiting for erase...\n");
        XSPI_AutoPollingMemReady(&hxspi2);

        // Multi-Page Programming (256 bytes per page)
        printf("[DEBUG] Programming %u bytes in pages...\n", size);

        DWT->CTRL |= 1;
        cycles_start = DWT->CYCCNT;

        uint32_t bytes_written = 0;
        while (bytes_written < size) {
            uint32_t page_size = (size - bytes_written) > 256 ? 256 : (size - bytes_written);
            uint32_t page_addr = addr + bytes_written;

            XSPI_WriteEnable(&hxspi2);

            sCommand.Instruction = OCTAL_PAGE_PROG_CMD;
            sCommand.DataMode    = HAL_XSPI_DATA_8_LINES;
            sCommand.DataLength  = page_size;
            sCommand.Address     = page_addr;

            if (HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
                printf("[ERROR] Program command failed at 0x%04X!\n", page_addr);
                return;
            }

            if (HAL_XSPI_Transmit(&hxspi2, &test_tx[bytes_written], HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
                printf("[ERROR] Transmit failed at 0x%04X!\n", page_addr);
                return;
            }

            XSPI_AutoPollingMemReady(&hxspi2);

            bytes_written += page_size;
        }

        cycles_end = DWT->CYCCNT;
        cycles_elapsed = cycles_end - cycles_start;

        uint32_t MBps = (uint64_t)size * 600000000ULL / cycles_elapsed / 1048576ULL;
        printf("  %4u bytes: %6u cycles (%4u us) = %lu MB/s\n",
               size, cycles_elapsed, CYCLES_TO_US(cycles_elapsed), MBps);

    }

    printf("\n[DEBUG] Write tests complete\n");

    /* ==================== TEST 2: READ ==================== */
    printf("\nTEST 2: Read Performance\n");
    printf("------------------------\n");

    for (test_idx = 0; test_idx < 4; test_idx++) {
        printf("[DEBUG] Test %d starting...\n", test_idx);

        uint32_t size = test_sizes[test_idx];
        uint32_t addr = test_addresses[test_idx];

        printf("[DEBUG] Reading %u bytes from 0x%04X...\n", size, addr);

        sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
        sCommand.Instruction        = OCTAL_IO_DTR_READ_CMD;
        sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
        sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
        sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
        sCommand.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
        sCommand.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
        sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
        sCommand.DataMode           = HAL_XSPI_DATA_8_LINES;
        sCommand.DataDTRMode        = HAL_XSPI_DATA_DTR_ENABLE;
        sCommand.Address            = addr;
        sCommand.DataLength         = size;
        sCommand.DummyCycles        = DUMMY_CLOCK_CYCLES_READ_OCTAL;
        sCommand.DQSMode            = HAL_XSPI_DQS_ENABLE;
        sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;

        if (HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            printf("[ERROR] Read command failed!\n");
            return;
        }

        cycles_start = DWT->CYCCNT;

        if (HAL_XSPI_Receive(&hxspi2, test_rx, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            printf("[ERROR] Receive failed!\n");
            return;
        }

        cycles_end = DWT->CYCCNT;
        cycles_elapsed = cycles_end - cycles_start;

        uint32_t MBps = (uint64_t)size * 600000000ULL / cycles_elapsed / 1048576ULL;
        printf("  %4u bytes: %6u cycles (%4u us) = %lu MB/s\n",
               size, cycles_elapsed, CYCLES_TO_US(cycles_elapsed), MBps);

    }

    printf("\n[DEBUG] Read tests complete\n");

    /* ==================== TEST 3: VERIFY ==================== */
    printf("\nTEST 3: Data Integrity\n");
    printf("----------------------\n");

    uint32_t total_errors = 0;

    for (test_idx = 0; test_idx < 4; test_idx++) {
        uint32_t size = test_sizes[test_idx];
        uint32_t addr = test_addresses[test_idx];
        uint32_t errors = 0;

        printf("[DEBUG] Verifying %u bytes from 0x%04X...\n", size, addr);

        sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
        sCommand.Instruction        = OCTAL_IO_DTR_READ_CMD;
        sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
        sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
        sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
        sCommand.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
        sCommand.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
        sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
        sCommand.DataMode           = HAL_XSPI_DATA_8_LINES;
        sCommand.DataDTRMode        = HAL_XSPI_DATA_DTR_ENABLE;
        sCommand.Address            = addr;
        sCommand.DataLength         = size;
        sCommand.DummyCycles        = DUMMY_CLOCK_CYCLES_READ_OCTAL;
        sCommand.DQSMode            = HAL_XSPI_DQS_ENABLE;
        sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;

        HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
        HAL_XSPI_Receive(&hxspi2, test_rx, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);

        for (uint32_t i = 0; i < size; i++) {
            if (test_rx[i] != (uint8_t)(i & 0xFF)) {
                errors++;
                if (errors <= 3) {
                    printf("  [ERROR] Byte %u: expected 0x%02X, got 0x%02X\n",
                           i, (uint8_t)(i & 0xFF), test_rx[i]);
                }
            }
        }

        total_errors += errors;
        printf("  %4u bytes: %s\n", size, errors ? "FAIL" : "PASS");
    }

    printf("\n");

    if (total_errors == 0) {
        printf("  *** ALL TESTS PASSED ***\n");
    } else {
        printf("  *** TESTS FAILED: %u errors ***\n", total_errors);
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
static inline void process_thermal_line_ultra_optimized(
    const uint16_t * __restrict__ sensor_data,
    const uint16_t * __restrict__ dark,
    const uint16_t * __restrict__ gain,
    const uint16_t * __restrict__ offset,
    const uint16_t * __restrict__ planck_lut,
    uint16_t * __restrict__ output,
    uint32_t width)
{
    const int32x4_t negShift15 = vdupq_n_s32(-15);

    // Hint compiler about alignment
    sensor_data = (const uint16_t*)__builtin_assume_aligned(sensor_data, 32);
    dark = (const uint16_t*)__builtin_assume_aligned(dark, 32);
    gain = (const uint16_t*)__builtin_assume_aligned(gain, 32);
    offset = (const uint16_t*)__builtin_assume_aligned(offset, 32);
    output = (uint16_t*)__builtin_assume_aligned(output, 32);

    // Main loop - process 16 pixels per iteration
    for (uint32_t x = 0; x < width; x += 16)
    {
        // ==================== FIRST 8 PIXELS ====================

        // Load
        uint16x8_t adc1 = vld1q_u16(&sensor_data[x]);
        uint16x8_t dark1 = vld1q_u16(&dark[x]);
        uint16x8_t gain1 = vld1q_u16(&gain[x]);
        uint16x8_t offset1 = vld1q_u16(&offset[x]);

        // Dark subtract
        uint16x8_t corrected1 = vqsubq_u16(adc1, dark1);

        // Gain (Q15)
        uint32x4_t lo1 = vmullbq_int_u16(corrected1, gain1);
        uint32x4_t hi1 = vmulltq_int_u16(corrected1, gain1);
        lo1 = vqrshlq_u32(lo1, negShift15);
        hi1 = vqrshlq_u32(hi1, negShift15);

        // ==================== SECOND 8 PIXELS (PARALLEL) ====================

        // Load (while waiting for first batch)
        uint16x8_t adc2 = vld1q_u16(&sensor_data[x + 8]);
        uint16x8_t dark2 = vld1q_u16(&dark[x + 8]);
        uint16x8_t gain2 = vld1q_u16(&gain[x + 8]);
        uint16x8_t offset2 = vld1q_u16(&offset[x + 8]);

        // Dark subtract
        uint16x8_t corrected2 = vqsubq_u16(adc2, dark2);

        // Gain (Q15)
        uint32x4_t lo2 = vmullbq_int_u16(corrected2, gain2);
        uint32x4_t hi2 = vmulltq_int_u16(corrected2, gain2);
        lo2 = vqrshlq_u32(lo2, negShift15);
        hi2 = vqrshlq_u32(hi2, negShift15);

        // ==================== FINISH FIRST 8 ====================

        // Narrow to 16-bit
        uint16x8_t result1 = vdupq_n_u16(0);
        result1 = vqmovnbq_u32(result1, lo1);
        result1 = vqmovntq_u32(result1, hi1);

        // Add offset
        result1 = vqaddq_u16(result1, offset1);

        // Planck LUT
        result1 = vldrhq_gather_shifted_offset_u16(planck_lut, result1);

        // ==================== FINISH SECOND 8 ====================

        // Narrow to 16-bit
        uint16x8_t result2 = vdupq_n_u16(0);
        result2 = vqmovnbq_u32(result2, lo2);
        result2 = vqmovntq_u32(result2, hi2);

        // Add offset
        result2 = vqaddq_u16(result2, offset2);

        // Planck LUT
        result2 = vldrhq_gather_shifted_offset_u16(planck_lut, result2);

        // ==================== STORE BOTH ====================

        vst1q_u16(&output[x], result1);
        vst1q_u16(&output[x + 8], result2);
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


void xspi_memory_mapped_speed_test(void) {
    printf("\n");
    printf("╔════════════════════════════════════════════════╗\n");
    printf("║ XSPI Memory-Mapped Performance Test           ║\n");
    printf("╚════════════════════════════════════════════════╝\n");

    volatile uint8_t *flash_base = (uint8_t*)xspi_mmap_base;
    uint32_t test_sizes[] = {256, 1024, 4096, 16384, 65536};  // ← Reduced!
    const int num_tests = 5;

    printf("\nSequential Read Performance:\n");
    printf("Size      | Cycles    | Time (us) | Speed (MB/s)\n");
    printf("----------|-----------|-----------|-------------\n");

    for (int t = 0; t < num_tests; t++) {
        uint32_t size = test_sizes[t];
        volatile uint32_t dummy = 0;

        // Warmup (prime cache)
        for (uint32_t i = 0; i < size; i += 4) {
            dummy += flash_base[i];
        }

        // Actual measurement
        __DSB();
        __ISB();

        uint32_t start = DWT->CYCCNT;

        // Read test - byte-wise
        for (uint32_t i = 0; i < size; i++) {
            dummy += flash_base[i];
        }

        __DSB();
        __ISB();
        uint32_t cycles = DWT->CYCCNT - start;

        uint32_t us = cycles / 600;
        uint32_t mbps = (uint64_t)size * 600000000ULL / cycles / 1048576ULL;

        printf("%7lu B | %9lu | %9lu | %11lu\n",
               size, cycles, us, mbps);

        // Prevent optimization
        if (dummy == 0xFFFFFFFF) printf(".");
    }

    printf("\n32-bit Word Read Performance:\n");
    printf("Size      | Cycles    | Time (us) | Speed (MB/s)\n");
    printf("----------|-----------|-----------|-------------\n");

    for (int t = 0; t < num_tests; t++) {
        uint32_t size = test_sizes[t];
        if (size < 4) continue;

        volatile uint32_t *flash_32 = (uint32_t*)xspi_mmap_base;
        uint32_t words = size / 4;
        volatile uint32_t dummy = 0;

        // Warmup
        for (uint32_t i = 0; i < words; i++) {
            dummy += flash_32[i];
        }

        __DSB();
        __ISB();

        uint32_t start = DWT->CYCCNT;

        // Read test - word-wise
        for (uint32_t i = 0; i < words; i++) {
            dummy += flash_32[i];
        }

        __DSB();
        __ISB();
        uint32_t cycles = DWT->CYCCNT - start;

        uint32_t us = cycles / 600;
        uint32_t mbps = (uint64_t)size * 600000000ULL / cycles / 1048576ULL;

        printf("%7lu B | %9lu | %9lu | %11lu\n",
               size, cycles, us, mbps);

        if (dummy == 0xFFFFFFFF) printf(".");
    }

    printf("\nCache Effect Test (repeated reads):\n");
    printf("Pass | Cycles    | Time (us) | Speed (MB/s)\n");
    printf("-----|-----------|-----------|-------------\n");

    const uint32_t test_size = 16384;  // ← Reduced to 16 KB
    volatile uint32_t *flash_32 = (uint32_t*)xspi_mmap_base;
    uint32_t words = test_size / 4;

    for (int pass = 0; pass < 5; pass++) {
        volatile uint32_t dummy = 0;

        __DSB();
        __ISB();

        uint32_t start = DWT->CYCCNT;

        for (uint32_t i = 0; i < words; i++) {
            dummy += flash_32[i];
        }

        __DSB();
        __ISB();
        uint32_t cycles = DWT->CYCCNT - start;

        uint32_t us = cycles / 600;
        uint32_t mbps = (uint64_t)test_size * 600000000ULL / cycles / 1048576ULL;

        printf("  %d  | %9lu | %9lu | %11lu\n",
               pass + 1, cycles, us, mbps);

        if (dummy == 0xFFFFFFFF) printf(".");
    }

    printf("\nRandom Access Pattern Test:\n");
    printf("Accesses | Cycles    | Time (us) | Avg cycles/access\n");
    printf("---------|-----------|-----------|------------------\n");

    uint32_t access_counts[] = {100, 1000, 10000};

    for (int t = 0; t < 3; t++) {
        uint32_t accesses = access_counts[t];
        volatile uint32_t dummy = 0;

        __DSB();
        __ISB();

        uint32_t start = DWT->CYCCNT;

        // Pseudo-random access pattern (within 64KB to avoid huge range)
        uint32_t idx = 0;
        for (uint32_t i = 0; i < accesses; i++) {
            idx = (idx * 1103515245 + 12345) & 0xFFFF;  // 0-65535 range
            dummy += flash_base[idx];
        }

        __DSB();
        __ISB();
        uint32_t cycles = DWT->CYCCNT - start;

        uint32_t us = cycles / 600;
        uint32_t cycles_per_access = cycles / accesses;

        printf("%8lu | %9lu | %9lu | %16lu\n",
               accesses, cycles, us, cycles_per_access);

        if (dummy == 0xFFFFFFFF) printf(".");
    }

    printf("\nBurst Read Test (using existing buffers):\n");
    printf("Size      | Cycles    | Time (us) | Speed (MB/s)\n");
    printf("----------|-----------|-----------|-------------\n");

    // Use temp_u16 buffer that's already allocated
    uint8_t *target = (uint8_t*)temp_u16;  // Reuse existing buffer!
    uint32_t max_size = sizeof(temp_u16);  // 640*480*2 = 614KB

    uint32_t burst_sizes[] = {1024, 4096, 16384, 65536};

    for (int t = 0; t < 4; t++) {
        uint32_t size = burst_sizes[t];
        if (size > max_size) continue;

        __DSB();
        __ISB();

        uint32_t start = DWT->CYCCNT;

        // Actual memcpy
        for (uint32_t i = 0; i < size; i++) {
            target[i] = flash_base[i];
        }

        __DSB();
        __ISB();
        uint32_t cycles = DWT->CYCCNT - start;

        uint32_t us = cycles / 600;
        uint32_t mbps = (uint64_t)size * 600000000ULL / cycles / 1048576ULL;

        printf("%7lu B | %9lu | %9lu | %11lu\n",
               size, cycles, us, mbps);
    }

    printf("\nCalibration Data Load Simulation:\n");
    printf("Simulating 640x480 camera calibration load from XSPI...\n");

    // Simulate loading one line of calibration (interleaved)
    // 640 pixels × 4 values (dark, gain, offset, emis) × 2 bytes = 5120 bytes/line
    const uint32_t line_size = 640 * 4 * 2;
    const uint32_t lines = 480;

    printf("\nPer-line load (5120 bytes):\n");

    uint32_t total_cycles = 0;
    for (int line = 0; line < 10; line++) {  // Test 10 lines
        uint32_t offset = line * line_size;
        volatile uint32_t dummy = 0;

        __DSB();
        __ISB();
        uint32_t start = DWT->CYCCNT;

        // Load one line of calibration
        for (uint32_t i = 0; i < line_size; i += 4) {
            dummy += flash_base[offset + i];
        }

        __DSB();
        __ISB();
        uint32_t cycles = DWT->CYCCNT - start;
        total_cycles += cycles;

        if (dummy == 0xFFFFFFFF) printf(".");
    }

    uint32_t avg_cycles = total_cycles / 10;
    uint32_t avg_us = avg_cycles / 600;
    uint32_t mbps = (uint64_t)line_size * 600000000ULL / avg_cycles / 1048576ULL;

    printf("  Avg per line: %lu cycles (%lu us) = %lu MB/s\n",
           avg_cycles, avg_us, mbps);

    uint32_t total_frame_cycles = avg_cycles * lines;
    uint32_t total_frame_ms = total_frame_cycles / 600000;

    printf("  Full frame (480 lines): %lu cycles (%lu ms)\n",
           total_frame_cycles, total_frame_ms);
    printf("  Theoretical max FPS if only loading: %lu\n",
           600000000 / total_frame_cycles);

    printf("\n");
    printf("╔════════════════════════════════════════════════╗\n");
    printf("║ Memory-Mapped Speed Test Complete!            ║\n");
    printf("╚════════════════════════════════════════════════╝\n");
}

static inline void process_thermal_line_realistic(
    const uint16_t *sensor_data,      // Raw sensor ADC values
    const uint16_t *dark,             // Dark frame
    const uint16_t *gain,             // Gain coefficients (Q15)
    const uint16_t *offset,           // Offset values (u16)
    const uint16_t *planck_lut,       // Planck LUT (u16)
    uint16_t *output,                 // Output buffer
    uint32_t width)                   // 640, vielfaches von 8
{
    const int32x4_t negShift15 = vdupq_n_s32(-15);

    for (uint32_t x = 0; x < width; x += 8)
    {
        // 1) ADC & Dark laden
        uint16x8_t adc      = vld1q_u16(&sensor_data[x]);
        uint16x8_t dark_val = vld1q_u16(&dark[x]);

        // 2) Dark abziehen (saturierend, um Underflow zu vermeiden)
        uint16x8_t corrected = vqsubq_u16(adc, dark_val);

        // 3) Gain laden
        uint16x8_t gain_val = vld1q_u16(&gain[x]);

        // 4) 16x16 -> 32 (low/high), runden & >>15 in einem Schritt
        uint32x4_t lo32 = vmullbq_int_u16(corrected, gain_val);
        uint32x4_t hi32 = vmulltq_int_u16(corrected, gain_val);
        lo32 = vqrshlq_u32(lo32, negShift15);   // rundend >>15
        hi32 = vqrshlq_u32(hi32, negShift15);

        // 5) Offset laden und auf 32 Bit erweitern
        uint16x8_t offset_val = vld1q_u16(&offset[x]);
        uint32x4_t lo_off = vmovlbq_u16(offset_val); // widen low 4x u16 -> u32
        uint32x4_t hi_off = vmovltq_u16(offset_val); // widen high 4x u16 -> u32

        // 6) Offset addieren
        lo32 = vaddq_u32(lo32, lo_off);
        hi32 = vaddq_u32(hi32, hi_off);

        // 7) Packen 32->16 (MVE-Variante, nicht NEONs vqmovn_u32!)
        uint16x8_t result16 = vdupq_n_u16(0);
        result16 = vqmovnbq_u32(result16, lo32);  // lower half
        result16 = vqmovntq_u32(result16, hi32);  // upper half

        // 8) Planck-LUT (gather, 16-bit Indizes)
        result16 = vldrhq_gather_shifted_offset_u16(planck_lut, result16);

        // 9) Store
        vst1q_u16(&output[x], result16);
    }
}
static inline void process_thermal_line_optimized(
    const uint16_t *sensor_data,
    const uint16_t *dark,
    const uint16_t *gain,
    const uint16_t *offset,
    const uint16_t *planck_lut,
    uint16_t *output,
    uint32_t width)
{
    const int32x4_t negShift15 = vdupq_n_s32(-15);

    for (uint32_t x = 0; x < width; x += 8)
    {
        // Load
        uint16x8_t adc = vld1q_u16(&sensor_data[x]);
        uint16x8_t dark_val = vld1q_u16(&dark[x]);
        uint16x8_t gain_val = vld1q_u16(&gain[x]);
        uint16x8_t offset_val = vld1q_u16(&offset[x]);

        // Dark subtract
        uint16x8_t corrected = vqsubq_u16(adc, dark_val);

        // Gain (Q15)
        uint32x4_t lo32 = vmullbq_int_u16(corrected, gain_val);
        uint32x4_t hi32 = vmulltq_int_u16(corrected, gain_val);
        lo32 = vqrshlq_u32(lo32, negShift15);  // Round & shift
        hi32 = vqrshlq_u32(hi32, negShift15);

        // Narrow to 16-bit
        uint16x8_t result16 = vdupq_n_u16(0);
        result16 = vqmovnbq_u32(result16, lo32);
        result16 = vqmovntq_u32(result16, hi32);

        // Add offset (16-bit, much faster!)
        result16 = vqaddq_u16(result16, offset_val);

        // Planck LUT
        result16 = vldrhq_gather_shifted_offset_u16(planck_lut, result16);

        // Store
        vst1q_u16(&output[x], result16);
    }
}
// dark_gain_4x4: [D0..D3, G0..G3, D4..D7, G4..G7, ...]  (u16, SRAM)
// offset_emi_4x4: [Off0..Off3, Emi0..Emi3, Off4..Off7, Emi4..Emi7, ...] (u16, XSPI-MMAP)
// planck_inv/planck: u16 LUTs (ideal in DTCM), output: u16
static inline void process_thermal_line_thomas(
    const uint16_t *adc_sram,          // 640 u16 in SRAM
    const uint16_t *dark_gain_4x4,     // 4+4 interleaved in SRAM
    const uint16_t *offset_emi_4x4,    // 4+4 interleaved in XSPI (MMAP)
    const uint16_t *planck_inv_lut,    // inverse Planck (u16)
    const uint16_t *planck_lut,        // Planck (u16)
    uint16_t *out,                     // u16
    uint32_t width)                    // 640 (Vielfaches von 8)
{
    const int32x4_t negShift15 = vdupq_n_s32(-15);
    const uint32x4_t mask16    = vdupq_n_u32(0xFFFFu);
    uint16_t last_valid = 0;

    for (uint32_t x = 0; x < width; x += 8)
    {
        // --- ADC laden (8) ---
        uint16x8_t adc8 = vld1q_u16(&adc_sram[x]);
        uint32x4_t adc_lo = vmovlbq_u16(adc8);
        uint32x4_t adc_hi = vmovltq_u16(adc8);

        // --- DARK/GAIN (4+4) aus SRAM ---
        // index = 2*x  (pro 8 Pixel sind es 16 Halfwords: 8 für [D0..D3,G0..G3], 8 für [D4..D7,G4..G7])
        const uint16_t *dg_ptr = &dark_gain_4x4[2u * x];
        uint16x8_t dg0 = vld1q_u16(&dg_ptr[0]);  // D0..D3, G0..G3
        uint16x8_t dg1 = vld1q_u16(&dg_ptr[8]);  // D4..D7, G4..G7
        uint32x4_t dark_lo = vmovlbq_u16(dg0);
        uint32x4_t gain_lo = vmovltq_u16(dg0);
        uint32x4_t dark_hi = vmovlbq_u16(dg1);
        uint32x4_t gain_hi = vmovltq_u16(dg1);

        // corrected = max(adc - dark, 0)  (saturierend)
        uint32x4_t corr_lo = vqsubq_u32(adc_lo, dark_lo);
        uint32x4_t corr_hi = vqsubq_u32(adc_hi, dark_hi);

        // Gain (Q15) anwenden: (corr*gain + 0.5)>>15  (u32* u32, gain <= 65535)
        uint32x4_t prod_lo = vmulq_u32(corr_lo, gain_lo);
        uint32x4_t prod_hi = vmulq_u32(corr_hi, gain_hi);
        prod_lo = vqrshlq_u32(prod_lo, negShift15);
        prod_hi = vqrshlq_u32(prod_hi, negShift15);

        // --- OFFSET/EMI (4+4) aus XSPI per 32-bit laden ---
        const uint16_t *oe16 = &offset_emi_4x4[2u * x];   // x in 8er-Schritten → 32B aligned

        // Block0: Off0..Off3, Emi0..Emi3   (8 Halfwords)
        uint16x8_t oe0_16 = vld1q_u16(&oe16[0]);
        // Block1: Off4..Off7, Emi4..Emi7   (8 Halfwords)
        uint16x8_t oe1_16 = vld1q_u16(&oe16[8]);

        // u16 → u32 aufspalten
        uint32x4_t off_lo = vmovlbq_u16(oe0_16);          // Off0..Off3
        uint32x4_t emi_lo = vmovltq_u16(oe0_16);          // Emi0..Emi3
        uint32x4_t off_hi = vmovlbq_u16(oe1_16);          // Off4..Off7
        uint32x4_t emi_hi = vmovltq_u16(oe1_16);          // Emi4..Emi7

        // Offset addieren
        prod_lo = vaddq_u32(prod_lo, off_lo);
        prod_hi = vaddq_u32(prod_hi, off_hi);

//        // --- primitive BPC (scalar, robust & selten) ---
//        // Gewinne als u16 holen (nur für die Maske)
//        uint16x8_t gain16 = vdupq_n_u16(0);
//        gain16 = vqmovnbq_u32(gain16, gain_lo);
//        gain16 = vqmovntq_u32(gain16, gain_hi);
//
//        // Zwischenresultat u16 (vor LUT), für BPC/prev
//        uint16x8_t pix16 = vdupq_n_u16(0);
//        pix16 = vqmovnbq_u32(pix16, prod_lo);
//        pix16 = vqmovntq_u32(pix16, prod_hi);
//
//        // scalar BPC: wenn gain==0 → ersetze mit last_valid (vorheriges Pixel)
//        uint16_t tmp[8];
//        vst1q_u16(tmp, pix16);
//        uint16_t gtmp[8];
//        vst1q_u16(gtmp, gain16);
//        for (int i = 0; i < 8; ++i) {
//            if (gtmp[i] == 0) tmp[i] = last_valid;
//            else              last_valid = tmp[i];
//        }
//        pix16 = vld1q_u16(tmp);


        uint16x8_t pix16 = vdupq_n_u16(0);
        pix16 = vqmovnbq_u32(pix16, prod_lo);
        pix16 = vqmovntq_u32(pix16, prod_hi);


        // --- inverse Planck ---
        pix16 = vldrhq_gather_shifted_offset_u16(planck_inv_lut, pix16);

        // --- Emissivität anwenden: (pix * emi + 0.5)>>15 ---
        uint32x4_t p_lo = vmovlbq_u16(pix16);
        uint32x4_t p_hi = vmovltq_u16(pix16);
        p_lo = vmulq_u32(p_lo, emi_lo);
        p_hi = vmulq_u32(p_hi, emi_hi);
        p_lo = vqrshlq_u32(p_lo, negShift15);
        p_hi = vqrshlq_u32(p_hi, negShift15);

        uint16x8_t after_emi = vdupq_n_u16(0);
        after_emi = vqmovnbq_u32(after_emi, p_lo);
        after_emi = vqmovntq_u32(after_emi, p_hi);

        // --- Planck ---
        after_emi = vldrhq_gather_shifted_offset_u16(planck_lut, after_emi);

        // Store
        vst1q_u16(&out[x], after_emi);
    }
}
// Hilfsfunktion für saubere Zeit-Ausgabe
static void print_line_stats(const char* name, uint32_t total_cycles, uint32_t lines)
{
    const uint32_t cpu_hz = SystemCoreClock;             // z.B. 600000000
    uint32_t per_line     = total_cycles / lines;
    uint32_t full_frame   = per_line * 480u;

    uint32_t total_us     = (uint32_t)((uint64_t)total_cycles * 1000000ull / cpu_hz);
    uint32_t per_line_us  = (uint32_t)((uint64_t)per_line     * 1000000ull / cpu_hz);
    uint32_t frame_ms     = (uint32_t)((uint64_t)full_frame   * 1000ull    / cpu_hz);
    uint32_t fps          = (cpu_hz / (full_frame ? full_frame : 1u));

    printf("\n[%s] Lines: %lu\n", name, (unsigned long)lines);
    printf("  Total:    %lu cycles  (%lu us)\n", (unsigned long)total_cycles, (unsigned long)total_us);
    printf("  Per line: %lu cycles  (%lu us)\n", (unsigned long)per_line,     (unsigned long)per_line_us);

    printf("\n[Extrapolation to 480 lines]\n");
    printf("  Full frame: %lu cycles  (%lu ms)\n", (unsigned long)full_frame, (unsigned long)frame_ms);
    printf("  FPS: %lu\n", (unsigned long)fps);

    if (per_line_us <= 41u) {
        printf("  ✓ Can sustain 50 FPS! Margin: %lu us\n", (unsigned long)(41u - per_line_us));
    } else {
        printf("  ⚠ Need %lu us faster for 50 FPS\n", (unsigned long)(per_line_us - 41u));
    }
    printf("──────────────────────────────────────────────\n");
}

// Prototypen deiner Kernels
void process_thermal_line_realistic(
    const uint16_t *sensor_data,
    const uint16_t *dark,
    const uint16_t *gain,
    const uint16_t *offset,
    const uint16_t *planck_lut,
    uint16_t *output,
    uint32_t width);

void process_thermal_line_thomas(
    const uint16_t *adc_sram,          // 640 u16 in SRAM
    const uint16_t *dark_gain_4x4,     // [D0..D3,G0..G3,D4..D7,G4..G7,...] in SRAM
    const uint16_t *offset_emi_4x4,    // [Off0..Off3,Emi0..Emi3,...] in XSPI (MMAP)
    const uint16_t *planck_inv_lut,    // u16 (ideal DTCM)
    const uint16_t *planck_lut,        // u16 (ideal DTCM)
    uint16_t *out,
    uint32_t width);

// === Beispiel-Test ===
// Hinweis: Achte auf Ausrichtung! Für beste Performance:
//   - sensor_data / dark_gain_4x4 in SRAM, 32-Byte aligned
//   - offset_emi_4x4 in XSPI-MMAP (0x7000_0000...), 32-Byte aligned
//   - LUTs in DTCM (CPU-only)
void run_test_thermal_pipelines(void)
{
    const uint32_t test_lines = 10;

    // ---- Eingabepuffer (hier nur als Platzhalter/Beispiel) ----
    extern uint16_t sensor_data[];       // [HPIX*test_lines] SRAM
    extern uint16_t dark_frame[];        // [HPIX*test_lines] SRAM
    extern uint16_t gain_coeff[];        // [HPIX*test_lines] SRAM
    extern uint16_t offset_val[];        // [HPIX*test_lines] SRAM (nur für "realistic")

    extern uint16_t dark_gain_4x4[];     // [HPIX*test_lines] * 1 (weil 4+4 in 8 halfwords) SRAM
    //extern uint16_t offset_emi_4x4[];    // [HPIX*test_lines] * 1 (4+4) XSPI-MMAP

    extern uint16_t planck_table[];      // u16 LUT (DTCM empfohlen)
    extern uint16_t inv_planck_table[];  // u16 LUT (DTCM empfohlen)

    static uint16_t line_output[HPIX] __attribute__((aligned(32)));

    // DWT-Zähler an
    DWT_CycleCounter_Init();

    // -------- Test A: Realistic (getrennte Arrays) --------
    {
        printf("\n[Test A] Realistic Thermal Pipeline...\n");
        volatile uint32_t t0, t1;
        DWT->CYCCNT = 0;
        t0 = DWT->CYCCNT;

        for (uint32_t line = 0; line < test_lines; ++line) {
            const uint16_t *adc    = &sensor_data[line * HPIX];
            const uint16_t *dark   = &dark_frame[line * HPIX];
            const uint16_t *gain   = &gain_coeff[line * HPIX];
            const uint16_t *offset = &offset_val[line * HPIX];

            process_thermal_line_realistic(
                adc, dark, gain, offset,
                planck_table,
                line_output,
                HPIX
            );
        }
        t1 = DWT->CYCCNT;
        print_line_stats("Test A (realistic)", t1 - t0, test_lines);
    }

    // -------- Test B: Thomas (4+4 interleaved, Offset/Emi in XSPI) --------
    {
        printf("\n[Test B] Thomas 4+4 Interleaved (XSPI offset/emissivity)...\n");
        volatile uint32_t t0, t1;
        DWT->CYCCNT = 0;
        t0 = DWT->CYCCNT;
        assert((((uintptr_t)dark_gain_4x4)   & 0x1F) == 0);
        assert((((uintptr_t)offset_emi_4x4)  & 0x1F) == 0);

        // Stride-Konsistenz
        static_assert((HPIX % 8) == 0, "HPIX must be multiple of 8");
        for (uint32_t line = 0; line < test_lines; ++line) {
            const uint16_t *adc    = &sensor_data[line * HPIX];          // SRAM
            //const uint16_t *dg44   = &dark_gain_4x4[line * HPIX];        // SRAM (D0..D3,G0..G3, ...)
            const uint16_t *dg44 = &dark_gain_4x4[line * (HPIX * 2u)];
                 // XSPI (MMAP, 4+4)
            const uint16_t *oe44 = &offset_emi_4x4[line * (HPIX * 2u)];
            process_thermal_line_thomas(
                adc,
                dg44,
                oe44,
                inv_planck_table,
                planck_table,
                line_output,
                HPIX
            );
        }
        t1 = DWT->CYCCNT;
        print_line_stats("Test B (thomas 4+4)", t1 - t0, test_lines);
    }

    printf("\n═══════════════════════════════════\n");
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
  printf("XSPI Debug startet...\r\n");

  /* Initialize buffers */
  for (int i = 0; i < BUFFERSIZE; i++) {
      aTxBuffer[i] = i;
      aRxBuffer[i] = 0;
  }

  /* Switch to OPI DTR Mode */
  XSPI_NOR_OctalDTRModeCfg(&hxspi2);

  /* Optional: Basic Write/Read Test (erase/write/read via commands) */
  XSPI_WriteEnable(&hxspi2);

  /* Performance Test */
  XSPI_PerformanceTest();

  /* ================= Memory-Mapped Mode Activation ================= */
  printf("\n╔════════════════════════════════════════════════╗\n");
  printf("║ XSPI Memory-Mapped Mode Initialization        ║\n");
  printf("╚════════════════════════════════════════════════╝\n");

  /* READ configuration (Octal IO DTR) */
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
  sRead.DummyCycles          = DUMMY_CLOCK_CYCLES_READ_OCTAL;   // passend zum Flash @200MHz
  sRead.DQSMode              = HAL_XSPI_DQS_ENABLE;
  sRead.AlternateBytesMode   = HAL_XSPI_ALT_BYTES_NONE;

  if (HAL_XSPI_Command(&hxspi2, &sRead, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
      printf("✗ READ-CFG failed (err=0x%08lX)\r\n", hxspi2.ErrorCode);
      Error_Handler();
  }
  printf("✓ READ-CFG set\r\n");

  /* WRITE configuration (Page Program) — auch setzen, selbst wenn MMAP nur Lesen nutzt */
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
      printf("✗ WRITE-CFG failed (err=0x%08lX)\r\n", hxspi2.ErrorCode);
      Error_Handler();
  }
  printf("✓ WRITE-CFG set\r\n");

  /* Enter Memory-Mapped mode (mit Timeout-Zähler) */
  XSPI_MemoryMappedTypeDef sMMAP = {0};
  sMMAP.TimeOutActivation  = HAL_XSPI_TIMEOUT_COUNTER_ENABLE;
  sMMAP.TimeoutPeriodClock = 0x50; // unkritisch; bei Bedarf feiner einstellen

  if (HAL_XSPI_MemoryMapped(&hxspi2, &sMMAP) != HAL_OK) {
      printf("✗ HAL_XSPI_MemoryMapped failed (state=%d, err=0x%08lX)\r\n",
             hxspi2.State, hxspi2.ErrorCode);
      Error_Handler();
  }
  printf("✓ XSPI entered Memory-Mapped mode (state=%d)\r\n", hxspi2.State);

  xspi_mmap_base =
     (hxspi2.Instance == XSPI2) ? XSPI2_MMAP_BASE : XSPI1_MMAP_BASE;
  uint32_t xspi_mmap_end  = xspi_mmap_base + XSPI_MMAP_SIZE - 1;
  printf("MMAP base=0x%08lX..0x%08lX\r\n", (unsigned)xspi_mmap_base, (unsigned)xspi_mmap_end);


  ARM_MPU_Region_t mpu_table[] = {
    {
      .RBAR = ARM_MPU_RBAR(xspi_mmap_base, ARM_MPU_SH_NON, 0, 1, 0), // AttrIdx=0, XN=0 (exec erlaubt)
      .RLAR = ARM_MPU_RLAR(xspi_mmap_end, 0) // enable region with AttrIdx 0
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
  printf("✓ MPU configured for XSPI @0x%08lX..0x%08lX\r\n", (unsigned)xspi_mmap_base, (unsigned)xspi_mmap_end);


  printf("\r\n=== Simple Memory Test (Direct) ===\r\n");
  volatile uint8_t *flash_ptr = (uint8_t*)xspi_mmap_base;
  printf("Reading first byte... ");
  uint8_t first = flash_ptr[0];
  printf("0x%02X\r\n", first);

  printf("First 16 bytes: ");
  for (int i = 0; i < 16; i++) printf("%02X ", flash_ptr[i]);
  printf("\r\n");
  printf("Simple Memory Test complete.\r\n");

  /* Clock / diagnostics */
  check_xspi_clock();
  test_memory_mapped_detailed();
  xspi_quick_status();

  printf("\n╔════════════════════════════════════════════════╗\n");
  printf("║ Memory-Mapped Mode Ready!                     ║\n");
  printf("╚════════════════════════════════════════════════╝\n");
  printf("\n");

  /* ======== Helium Benchmarks Setup ======== */
  printf("Initializing Planck LUT...\n");
  for (uint32_t i = 0; i < 65536; i++) {
      planck_table[i] = (uint16_t)i;
  }

  // Initialize test pattern using frame_buffer_A (via src_u16 alias)
  printf("Generating test pattern in frame buffer...\n");
  for (uint32_t i = 0; i < N; i++) {
      uint32_t x = i % HPIX;
      src_u16[i] = (uint16_t)((x * 16383) / HPIX);
  }

  // Initialize small gain array (will be replicated per line)
  printf("Initializing gain table...\n");
  for (uint32_t i = 0; i < HPIX; i++) {
      gain_u16[i] = gain_to_q15(1.0f);
  }

  // XSPI speed test
  xspi_memory_mapped_speed_test();

  printf("\n╔════════════════════════════════════════════════╗\n");
  printf("║ Memory-Mapped Mode Ready!                     ║\n");
  printf("╚════════════════════════════════════════════════╝\n");

  printf("\n=== Performance Benchmark (Optimized) ===\n");
  printf("Testing with memcpy optimization...\n");

  // ════════════════════════════════════════════════
  // Test 1: memcpy test
  // ════════════════════════════════════════════════
  printf("\n[Test 1] memcpy test...\n");
  memcpy(temp_line, gain_u16, HPIX * sizeof(uint16_t));
  printf("  ✓ memcpy OK\n");

  // ════════════════════════════════════════════════
  // Test 2: MVE Gain
  // ════════════════════════════════════════════════
  printf("[Test 2] MVE Gain test...\n");
  process_mve(temp_line, gain_u16, temp_line, HPIX, 0);
  printf("  ✓ MVE Gain OK\n");

  // ════════════════════════════════════════════════
  // Test 3: Planck LUT (now in AXISRAM)
  // ════════════════════════════════════════════════
  printf("[Test 3] Planck LUT test...\n");
  planck_lut_mve(temp_line, line_output, HPIX);
  printf("  ✓ Planck LUT OK\n");

  // ════════════════════════════════════════════════
  // Test 4: Full pipeline with memcpy
  // ════════════════════════════════════════════════
  printf("\n[Test 4] Full pipeline (1 line)...\n");

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  uint32_t t_start = DWT->CYCCNT;

  // Fast memcpy
  memcpy(temp_line, gain_u16, HPIX * sizeof(uint16_t));

  // Gain
  process_mve(temp_line, gain_u16, temp_line, HPIX, 0);

  // Planck
  planck_lut_mve(temp_line, line_output, HPIX);

  uint32_t t_end = DWT->CYCCNT;
  uint32_t cycles = t_end - t_start;

  printf("  ✓ Pipeline OK\n");
  printf("  Time: %lu cycles (%lu us)\n", cycles, cycles / 600);

  // ════════════════════════════════════════════════
  // Test 5: Loop 10 lines (optimized)
  // ════════════════════════════════════════════════
  printf("\n[Test 5] Processing 10 lines (optimized)...\n");

  DWT->CYCCNT = 0;
  uint32_t t0 = DWT->CYCCNT;

  for (int line = 0; line < 10; line++) {
      // Fast memcpy
      memcpy(temp_line, gain_u16, HPIX * sizeof(uint16_t));

      // Gain
      process_mve(temp_line, gain_u16, temp_line, HPIX, 0);

      // Planck
      planck_lut_mve(temp_line, line_output, HPIX);
  }

  uint32_t t_total = DWT->CYCCNT - t0;

  printf("  ✓ 10 lines OK\n");
  printf("  Total: %lu cycles (%lu ms)\n", t_total, t_total / 600000);
  printf("  Per line: %lu cycles (%lu us)\n", t_total / 10, (t_total / 10) / 600);

  // Extrapolate
  uint32_t full_frame = (t_total / 10) * 480;
  printf("\n[Extrapolation to 480 lines]\n");
  printf("  Full frame: %lu cycles (%lu ms)\n", full_frame, full_frame / 600000);
  printf("  FPS: %lu\n", 600000000 / full_frame);

  uint32_t line_us = (t_total / 10) / 600;
  printf("\nPerformance vs Target:\n");
  printf("  Current: %lu us per line\n", line_us);
  printf("  Budget @ 50 FPS: 41 us per line\n");

  if (line_us < 41) {
      printf("  ✓ Can sustain 50 FPS! Margin: %lu us\n", 41 - line_us);
  } else {
      printf("  ⚠ Too slow for 50 FPS! Need %lu us faster\n", line_us - 41);
  }
  printf("\n[Test 6] Detailed Timing Breakdown...\n");

  // Test memcpy alone
  DWT->CYCCNT = 0;
  uint32_t t_memcpy_start = DWT->CYCCNT;
  for (int i = 0; i < 100; i++) {
      memcpy(temp_line, gain_u16, HPIX * sizeof(uint16_t));
  }
  uint32_t t_memcpy = (DWT->CYCCNT - t_memcpy_start) / 100;

  // Test MVE Gain alone
  DWT->CYCCNT = 0;
  uint32_t t_gain_start = DWT->CYCCNT;
  for (int i = 0; i < 100; i++) {
      process_mve(temp_line, gain_u16, temp_line, HPIX, 0);
  }
  uint32_t t_gain = (DWT->CYCCNT - t_gain_start) / 100;

  // Test Planck alone
  DWT->CYCCNT = 0;
  uint32_t t_planck_start = DWT->CYCCNT;
  for (int i = 0; i < 100; i++) {
      planck_lut_mve(temp_line, line_output, HPIX);
  }
  uint32_t t_planck = (DWT->CYCCNT - t_planck_start) / 100;

  printf("  memcpy:  %lu cycles (%lu us)\n", t_memcpy, t_memcpy / 600);
  printf("  MVE Gain: %lu cycles (%lu us)\n", t_gain, t_gain / 600);
  printf("  Planck:   %lu cycles (%lu us)\n", t_planck, t_planck / 600);
  printf("  TOTAL:    %lu cycles (%lu us)\n",
         t_memcpy + t_gain + t_planck,
         (t_memcpy + t_gain + t_planck) / 600);

  printf("\n[Test 8] NO COPY - Direct processing...\n");

  DWT->CYCCNT = 0;
  t0 = DWT->CYCCNT;

  for (int line = 0; line < 10; line++) {
      // NO memcpy! Process directly
	  process_mve(temp_line, gain_u16, temp_line, HPIX, 0);
      planck_lut_mve(temp_line, line_output, HPIX);
  }

  uint32_t total = DWT->CYCCNT - t0;

  printf("  Total: %lu cycles (%lu ms)\n", t_total, t_total / 600000);
  printf("  Per line: %lu cycles (%lu us)\n", t_total / 10, (t_total / 10) / 600);

  full_frame = (t_total / 10) * 480;
  printf("  Full frame: %lu cycles (%lu ms)\n", full_frame, full_frame / 600000);
  printf("  FPS: %lu\n", 600000000 / full_frame);

  line_us = (t_total / 10) / 600;
  if (line_us < 41) {
      printf("  ✓ Can sustain 50 FPS! Margin: %lu us\n", 41 - line_us);
  } else {
      printf("  ⚠ Still need %lu us faster\n", line_us - 41);
  }


  printf("\n=== REALISTIC THERMAL PIPELINE TEST ===\n");

  // Create realistic test data
  static uint16_t sensor_data[HPIX];
  static uint16_t dark_frame[HPIX];
  static uint16_t gain_coeff[HPIX];
  static uint16_t offset_val[HPIX];

  // Fill with realistic values
  printf("Generating realistic test data...\n");
  for (int i = 0; i < HPIX; i++) {
      sensor_data[i] = 8000 + (i % 100);  // Simulated ADC values
      dark_frame[i] = 100 + (i % 10);     // Dark current
      gain_coeff[i] = gain_to_q15(1.0f);  // Gain = 1.0
      offset_val[i] = 500;                // Offset
  }

  printf("\n[Test 10] Realistic Thermal Pipeline...\n");

  DWT->CYCCNT = 0;
   t0 = DWT->CYCCNT;

  for (int line = 0; line < 10; line++) {
      process_thermal_line_realistic(
          sensor_data,
          dark_frame,
          gain_coeff,
          offset_val,
          planck_table,
          line_output,
          HPIX
      );
  }

  t_total = DWT->CYCCNT - t0;

  printf("  Total: %lu cycles (%lu ms)\n", t_total, t_total / 600000);
  printf("  Per line: %lu cycles (%lu us)\n", t_total / 10, (t_total / 10) / 600);

   full_frame = (t_total / 10) * 480;
  printf("\n[Extrapolation]\n");
  printf("  Full frame: %lu cycles (%lu ms)\n", full_frame, full_frame / 600000);
  printf("  FPS: %lu\n", 600000000 / full_frame);

   line_us = (t_total / 10) / 600;
  if (line_us < 41) {
      printf("  ✓ Can sustain 50 FPS! Margin: %lu us\n", 41 - line_us);
  } else {
      printf("  ⚠ Need %lu us faster for 50 FPS\n", line_us - 41);
  }

  printf("\n═══════════════════════════════════\n");

  printf("\n[Test 14] Realistic Thermal Pipeline...\n");

    DWT->CYCCNT = 0;
     t0 = DWT->CYCCNT;

    for (int line = 0; line < 10; line++) {
        process_thermal_line_optimized(
            sensor_data,
            dark_frame,
            gain_coeff,
            offset_val,
            planck_table,
            line_output,
            HPIX
        );
    }

    t_total = DWT->CYCCNT - t0;

    printf("  Total: %lu cycles (%lu ms)\n", t_total, t_total / 600000);
    printf("  Per line: %lu cycles (%lu us)\n", t_total / 10, (t_total / 10) / 600);

     full_frame = (t_total / 10) * 480;
    printf("\n[Extrapolation]\n");
    printf("  Full frame: %lu cycles (%lu ms)\n", full_frame, full_frame / 600000);
    printf("  FPS: %lu\n", 600000000 / full_frame);

     line_us = (t_total / 10) / 600;
    if (line_us < 41) {
        printf("  ✓ Can sustain 50 FPS! Margin: %lu us\n", 41 - line_us);
    } else {
        printf("  ⚠ Need %lu us faster for 50 FPS\n", line_us - 41);
    }

    printf("\n═══════════════════════════════════\n");
    printf("\n[Test 15] ULTRA OPTIMIZED (2x unroll)...\n");

    DWT->CYCCNT = 0;
    t0 = DWT->CYCCNT;

    for (int line = 0; line < 10; line++) {
        process_thermal_line_ultra_optimized(
            sensor_data,
            dark_frame,
            gain_coeff,
            offset_val,
            planck_table,
            line_output,
            HPIX
        );
    }

    t_total = DWT->CYCCNT - t0;

    printf("  Total: %lu cycles (%lu ms)\n", t_total, t_total / 600000);
    printf("  Per line: %lu cycles (%lu us)\n", t_total / 10, (t_total / 10) / 600);

    full_frame = (t_total / 10) * 480;
    printf("\n[Extrapolation]\n");
    printf("  Full frame: %lu cycles (%lu ms)\n", full_frame, full_frame / 600000);
    printf("  FPS: %lu\n", 600000000 / full_frame);

    line_us = (t_total / 10) / 600;
    if (line_us <= 41) {
        printf("  ✓✓✓ CAN SUSTAIN 50 FPS! Margin: %lu us\n", 41 - line_us);
    } else {
        printf("  ⚠ Need %lu us faster for 50 FPS\n", line_us - 41);
    }

    printf("═══════════════════════════════════════\n");

    printf("\n[Test 16] FASTEST (vqrdmulhq)...\n");

    DWT->CYCCNT = 0;
    t0 = DWT->CYCCNT;

    for (int line = 0; line < 10; line++) {
        process_thermal_line_fastest(
            sensor_data,
            dark_frame,
            gain_coeff,
            offset_val,
            planck_table,
            line_output,
            HPIX
        );
    }

    t_total = DWT->CYCCNT - t0;

    printf("  Total: %lu cycles (%lu ms)\n", t_total, t_total / 600000);
    printf("  Per line: %lu cycles (%lu us)\n", t_total / 10, (t_total / 10) / 600);

    full_frame = (t_total / 10) * 480;
    printf("\n[Extrapolation]\n");
    printf("  Full frame: %lu cycles (%lu ms)\n", full_frame, full_frame / 600000);
    printf("  FPS: %lu\n", 600000000 / full_frame);

    line_us = (t_total / 10) / 600;
    if (line_us <= 41) {
        printf("  🎉🎉🎉 CAN SUSTAIN 50 FPS! Margin: %lu us 🎉🎉🎉\n", 41 - line_us);
    } else {
        printf("  ⚠ Need %lu us faster for 50 FPS\n", line_us - 41);
    }

    printf("═══════════════════════════════════════\n");

  init_test_data();
  run_test_thermal_pipelines();

  printf("\n═══ Benchmark complete! ═══\n\n");




  printf("╔════════════════════════════════════════════════╗\n");
  printf("║ Initializing 50 Hz Thermal Pipeline           ║\n");
  printf("╚════════════════════════════════════════════════╝\n");

  thermal_check_timer_clock();
  thermal_vsync_init();
  thermal_vsync_start();

  printf("Pipeline ready! LED should blink at 50 Hz\n");
  printf("\n");

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
  printf("NOTE: Frame buffers now reserved for DCMIPP!\n");
  printf("      (Benchmark data will be overwritten)\n\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t led_state = 0;
  uint32_t last_stats = HAL_GetTick();
  uint32_t last_frame_count = 0;
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
    if (HAL_GetTick() - last_stats >= 5000) {
          uint32_t frames = frames_processed - last_frame_count;
          uint32_t fps = frames / 5;

          printf("═══════════════════════════════════════\n");
          printf("VSYNC: %lu | Frames: %lu | FPS: ~%lu\n",
                 vsync_count, frames_processed, fps);
          printf("═══════════════════════════════════════\n");

          last_stats = HAL_GetTick();
          last_frame_count = frames_processed;
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

/* === Debug helpers (protected) =========================================== */
void test_memory_mapped_detailed(void) {
    printf("\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║ Memory-Mapped Access Detailed Debug   ║\n");
    printf("╚════════════════════════════════════════╝\n");

    // [1] XSPI Handle Status
    printf("\n[1] XSPI2 HAL Handle Status:\n");
    printf("  Handle State: ");
    switch(hxspi2.State) {
        case HAL_XSPI_STATE_RESET:
            printf("RESET\n"); break;
        case HAL_XSPI_STATE_READY:
            printf("READY\n"); break;
        case HAL_XSPI_STATE_BUSY_CMD:
            printf("BUSY_CMD\n"); break;
        case HAL_XSPI_STATE_BUSY_TX:
            printf("BUSY_TX\n"); break;
        case HAL_XSPI_STATE_BUSY_RX:
            printf("BUSY_RX\n"); break;
        case HAL_XSPI_STATE_BUSY_AUTO_POLLING:
            printf("BUSY_AUTO_POLLING\n"); break;
        case HAL_XSPI_STATE_BUSY_MEM_MAPPED:
            printf("BUSY_MEM_MAPPED ✓\n"); break;
        case HAL_XSPI_STATE_ABORT:
            printf("ABORT\n"); break;
        case HAL_XSPI_STATE_ERROR:
            printf("ERROR ✗\n"); break;
        default:
            printf("UNKNOWN (%d)\n", hxspi2.State); break;
    }

    printf("  Error Code: 0x%08lX", hxspi2.ErrorCode);
    if (hxspi2.ErrorCode == HAL_XSPI_ERROR_NONE) {
        printf(" (No Error)\n");
    } else {
        printf("\n");
        if (hxspi2.ErrorCode & HAL_XSPI_ERROR_TIMEOUT)  printf("    - TIMEOUT\n");
        if (hxspi2.ErrorCode & HAL_XSPI_ERROR_TRANSFER) printf("    - TRANSFER ERROR\n");
        if (hxspi2.ErrorCode & HAL_XSPI_ERROR_DMA)      printf("    - DMA ERROR\n");
    }

    // [2] Configuration Summary
    printf("\n[2] XSPI Configuration:\n");
    printf("  Memory Type: ");
    switch(hxspi2.Init.MemoryType) {
        case HAL_XSPI_MEMTYPE_MICRON:   printf("Micron\n"); break;
        case HAL_XSPI_MEMTYPE_MACRONIX: printf("Macronix ✓\n"); break;
        case HAL_XSPI_MEMTYPE_APMEM:    printf("AP Memory\n"); break;
        default:                        printf("Unknown\n"); break;
    }

    printf("  Memory Size: %lu\n", hxspi2.Init.MemorySize);
    printf("  Clock Prescaler: %lu (÷%lu)\n",
           hxspi2.Init.ClockPrescaler,
           hxspi2.Init.ClockPrescaler + 1);
    printf("  Chip Select High Time: %lu cycles\n",
           hxspi2.Init.ChipSelectHighTimeCycle);
    printf("  Free Running Clock: %s\n",
           hxspi2.Init.FreeRunningClock == HAL_XSPI_FREERUNCLK_ENABLE ? "YES" : "NO");

    // [3] Try Memory Access
    printf("\n[3] Attempting Memory Access:\n");
    printf("  Base Address: 0x70000000\n");

    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk;

    volatile uint32_t *test_addr = (uint32_t*)xspi_mmap_base;
    volatile uint32_t dummy = 0;

    printf("  Reading first word... ");
    __disable_irq();
    dummy = *test_addr;
    __enable_irq();
    printf("0x%08lX\n", dummy);

    printf("  Reading first 16 bytes:\n    ");
    for (int i = 0; i < 4; i++) {
        printf("0x%08lX ", test_addr[i]);
    }
    printf("\n");

    // [4] Data Pattern Analysis
    printf("\n[4] Data Pattern Analysis (first 256 bytes):\n");

    uint8_t all_ff = 1, all_00 = 1;
    volatile uint8_t *byte_ptr = (uint8_t*)xspi_mmap_base;
    uint32_t unique_bytes = 0;
    uint8_t byte_count[256] = {0};

    for (int i = 0; i < 256; i++) {
        uint8_t val = byte_ptr[i];
        if (val != 0xFF) all_ff = 0;
        if (val != 0x00) all_00 = 0;

        if (byte_count[val] == 0) unique_bytes++;
        byte_count[val]++;
    }

    if (all_ff) {
        printf("  Status: All 0xFF (Flash empty/erased)\n");
    } else if (all_00) {
        printf("  Status: All 0x00 (Likely read error)\n");
    } else {
        printf("  Status: Contains data pattern\n");
        printf("  Unique bytes: %lu / 256\n", unique_bytes);

        printf("  First values: ");
        int shown = 0;
        for (int i = 0; i < 32 && shown < 8; i++) {
            if (byte_ptr[i] != 0x00 && byte_ptr[i] != 0xFF) {
                printf("0x%02X ", byte_ptr[i]);
                shown++;
            }
        }
        printf("\n");
    }

    // [5] MPU Status
    printf("\n[5] MPU Status:\n");
    if (MPU->CTRL & MPU_CTRL_ENABLE_Msk) {
        printf("  MPU is ENABLED\n");
        printf("  Checking regions:\n");
        for (int region = 0; region < 8; region++) {
            MPU->RNR = region;
            uint32_t rbar = MPU->RBAR;
            uint32_t rlar = MPU->RLAR;

            if (rlar & MPU_RLAR_EN_Msk) {
                uint32_t base = rbar & 0xFFFFFFE0;
                printf("    Region %d: Base=0x%08lX ", region, base);
                if (base == xspi_mmap_base) {
                    printf("✓ (Covers XSPI)\n");
                } else {
                    printf("\n");
                }
            }
        }
    } else {
        printf("  MPU is DISABLED\n");
    }

    // [6] Cache Status
    printf("\n[6] Cache Status:\n");
    if (SCB->CCR & SCB_CCR_IC_Msk) {
        printf("  I-Cache: ENABLED\n");
    } else {
        printf("  I-Cache: DISABLED\n");
    }
    if (SCB->CCR & SCB_CCR_DC_Msk) {
        printf("  D-Cache: ENABLED\n");
    } else {
        printf("  D-Cache: DISABLED\n");
    }

    printf("\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║ Debug Analysis Complete                ║\n");
    printf("╚════════════════════════════════════════╝\n");
}

void check_xspi_clock(void) {
    printf("\n=== XSPI Clock Configuration ===\n");

    uint32_t xspi_clk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_XSPI2);
    uint32_t xspi_clk_mhz = xspi_clk / 1000000;
    uint32_t xspi_clk_khz = (xspi_clk / 1000) % 1000;

    printf("  XSPI2 Kernel Clock: %lu Hz (%lu.%03lu MHz)\n",
           xspi_clk, xspi_clk_mhz, xspi_clk_khz);

    if (xspi_clk < 190000000 || xspi_clk > 210000000) {
        printf("  WARNING: Clock not optimal for OPI DTR!\n");
        printf("    Current: %lu MHz\n", xspi_clk_mhz);
        printf("    Recommended: 200 MHz\n");
    } else {
        printf("  ✓ Clock in optimal range (190-210 MHz)\n");
    }

    uint32_t prescaler = hxspi2.Init.ClockPrescaler;
    printf("  Prescaler: %lu (Divider: %lu)\n", prescaler, prescaler + 1);

    uint32_t flash_clk = xspi_clk / (prescaler + 1);
    uint32_t flash_clk_mhz = flash_clk / 1000000;
    uint32_t flash_clk_khz = (flash_clk / 1000) % 1000;

    printf("  Effective Flash Clock: %lu.%03lu MHz\n",
           flash_clk_mhz, flash_clk_khz);

    uint32_t theoretical_bandwidth = (flash_clk_mhz * 16);
    printf("  Theoretical Bandwidth: %lu MB/s (OPI DTR)\n", theoretical_bandwidth);
    printf("  (Practical: ~50%% = %lu MB/s due to overhead)\n", theoretical_bandwidth / 2);
}

void xspi_quick_status(void) {
    printf("\n=== XSPI2 Quick Status ===\n");

    printf("HAL State: ");
    if (hxspi2.State == HAL_XSPI_STATE_BUSY_MEM_MAPPED) {
        printf("Memory-Mapped Mode Active ✓\n");
    } else {
        printf("NOT in Memory-Mapped Mode ✗ (State=%d)\n", hxspi2.State);
    }

    if (hxspi2.ErrorCode != HAL_XSPI_ERROR_NONE) {
        printf("ERROR: 0x%08lX\n", hxspi2.ErrorCode);
    }

    printf("\nMemory Test: ");
    volatile uint32_t *test = (uint32_t*)xspi_mmap_base;
    uint32_t val = *test;
    printf("0x%08lX", val);

    if (val == 0xFFFFFFFF) {
        printf(" (Empty Flash)\n");
    } else if (val == 0x00000000) {
        printf(" (Read Error?)\n");
    } else {
        printf(" (Has Data) ✓\n");
    }

    uint32_t clk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_XSPI2);
    printf("\nClock: %lu MHz\n", clk / 1000000);
}

void xspi_print_all_registers(void) {
    printf("\n╔════════════════════════════════════════╗\n");
    printf("║ XSPI2 Register Dump                    ║\n");
    printf("╚════════════════════════════════════════╝\n");

    XSPI_TypeDef *xspi = hxspi2.Instance;

    if (xspi == NULL) {
        printf("ERROR: XSPI2 instance is NULL!\n");
        return;
    }

    printf("\nControl Registers:\n");
    printf("  CR   = 0x%08lX\n", xspi->CR);
    printf("  DCR1 = 0x%08lX\n", xspi->DCR1);
    printf("  DCR2 = 0x%08lX\n", xspi->DCR2);
    printf("  DCR3 = 0x%08lX\n", xspi->DCR3);

    printf("\nStatus:\n");
    printf("  SR   = 0x%08lX\n", xspi->SR);
    printf("  Flags: ");
    if (xspi->SR & XSPI_SR_BUSY) printf("[BUSY] ");
    if (xspi->SR & XSPI_SR_TOF)  printf("[TIMEOUT] ");
    if (xspi->SR & XSPI_SR_TEF)  printf("[ERROR] ");
    if (xspi->SR & XSPI_SR_TCF)  printf("[COMPLETE] ");
    if (xspi->SR == 0) printf("(idle)");
    printf("\n");

    printf("\nCommand Configuration:\n");
    printf("  CCR  = 0x%08lX\n", xspi->CCR);
    printf("  TCR  = 0x%08lX\n", xspi->TCR);
    printf("  IR   = 0x%08lX\n", xspi->IR);

    printf("\nDecoded Configuration:\n");
    uint32_t devsize = (xspi->DCR1 & XSPI_DCR1_DEVSIZE) >> XSPI_DCR1_DEVSIZE_Pos;
    uint32_t mem_size_mb = (1UL << (devsize + 1)) / (1024 * 1024);
    printf("  Device Size: 2^%lu = %lu MB\n", devsize + 1, mem_size_mb);

    uint32_t prescaler = (xspi->DCR2 & XSPI_DCR2_PRESCALER) >> XSPI_DCR2_PRESCALER_Pos;
    printf("  Prescaler: %lu (Divider: %lu)\n", prescaler, prescaler + 1);

    uint32_t mtyp = (xspi->DCR1 & XSPI_DCR1_MTYP) >> XSPI_DCR1_MTYP_Pos;
    printf("  Memory Type: ");
    switch(mtyp) {
        case 0: printf("Micron\n"); break;
        case 1: printf("Macronix\n"); break;
        case 2: printf("Standard\n"); break;
        case 3: printf("AP Memory\n"); break;
        default: printf("Unknown\n"); break;
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
