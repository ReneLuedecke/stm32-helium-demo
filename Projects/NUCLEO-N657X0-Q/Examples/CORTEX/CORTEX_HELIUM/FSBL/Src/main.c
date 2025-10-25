/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32N6 Thermal Imaging Pipeline
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_mve.h"
#include "stdio.h"
#include <stdint.h>
#include <string.h>
#include "xspi_nor.h"
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

// Planck constants
#define PLANCK_C1 1.191042e8f
#define PLANCK_C2 1.4387752e4f

// Temperature format
#define TEMP_SCALE 100.0f
#define TEMP_OFFSET 10000.0f

// Sensor parameters
#define SENSOR_WAVELENGTH 10.0f
#define ADC_TO_RADIANCE_SCALE 1e-11f
#define FLAG_TEMP_CELSIUS 25.0f

// LED1 Definition
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOG
#define LED1_SET()    (GPIOG->BSRR = LED1_Pin)
#define LED1_RESET()  (GPIOG->BSRR = (LED1_Pin << 16))

#define CYCLES_TO_MS(cycles) ((cycles) / 600000)
#define CYCLES_TO_US(cycles) ((cycles) / 600)

#define XSPI1_MMAP_BASE   0x90000000UL
#define XSPI2_MMAP_BASE   0x70000000UL
#define XSPI_MMAP_SIZE    0x10000000UL  // 256 MB
#define OCTAL_IO_DTR_READ_CMD           0xEE11
#define OCTAL_PAGE_PROG_CMD             0x12ED
#define OCTAL_READ_STATUS_REG_CMD       0x05FA
#define OCTAL_WRITE_ENABLE_CMD          0x06F9
#define OCTAL_BLOCK_ERASE_64K_CMD       0xDC23
#define OCTAL_SECTOR_ERASE_4K_CMD   	0x21DE

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

// ═══════════════════════════════════════════════════════════════════
// MEMORY LAYOUT
// ═══════════════════════════════════════════════════════════════════

// AXISRAM1 (1 MB) - Critical high-speed data
__attribute__((aligned(64), section(".axisram1")))
uint16_t planck_table[65536];  // 128 KB

__attribute__((aligned(16), section(".axisram1")))
uint16_t gain_frame[VPIX][HPIX];  // 614 KB - FASTEST ACCESS!

__attribute__((aligned(16), section(".axisram1")))
uint16_t offset_line[HPIX];  // 1.3 KB

__attribute__((aligned(16), section(".axisram1")))
static uint16_t g_unity_q15_line[HPIX];

// Main RAM (2 MB) - Frame buffers
__attribute__((section(".noncacheable")))
uint16_t frame_buffer_A[VPIX][HPIX];  // 614 KB

__attribute__((section(".noncacheable")))
uint16_t frame_buffer_B[VPIX][HPIX];  // 614 KB

__attribute__((section(".noncacheable")))
uint16_t dark_frame[VPIX][HPIX];  // 614 KB

// Timer handle
TIM_HandleTypeDef htim2;

// Thermal pipeline variables
volatile uint32_t vsync_count = 0;
volatile uint32_t frames_processed = 0;
volatile uint8_t frame_ready = 0;

// XSPI Calibration Data Pointers (Memory-Mapped)
// Memory Map:
//   0x000000 - 0x095FFF: Dark frame      (600 KB)
//   0x096000 - 0x12BFFF: Gain frame      (600 KB)
//   0x12C000 - 0x1C1FFF: Offset frame    (600 KB)
//   0x1C2000 - 0x257FFF: Emissivity frame (600 KB) [NEW]
//   0x258000+          : Available for future calibration data
#define XSPI_BASE              0x70000000
#define XSPI_DARK_OFFSET       0x000000
#define XSPI_GAIN_OFFSET       0x096000
#define XSPI_OFFSET_OFFSET     0x12C000
#define XSPI_EMISSIVITY_OFFSET 0x1C2000  // 640×480×2 = 600 KB

volatile uint16_t (*xspi_dark)[HPIX]       = (void*)(XSPI_BASE + XSPI_DARK_OFFSET);
volatile uint16_t (*xspi_gain)[HPIX]       = (void*)(XSPI_BASE + XSPI_GAIN_OFFSET);
volatile uint16_t (*xspi_offset)[HPIX]     = (void*)(XSPI_BASE + XSPI_OFFSET_OFFSET);
volatile uint16_t (*xspi_emissivity)[HPIX] = (void*)(XSPI_BASE + XSPI_EMISSIVITY_OFFSET);

// Calibration state machine
typedef enum {
    CALIB_IDLE,
    CALIB_WAIT_STABLE,
    CALIB_COLLECTING,
    CALIB_COMPLETE
} CalibState_t;

volatile CalibState_t calib_state = CALIB_IDLE;
volatile uint32_t calib_frame_count = 0;
volatile uint32_t calib_samples_collected = 0;

#define CALIB_SKIP_FRAMES 4
#define CALIB_TOTAL_SAMPLES 12

// Flag & Temperature parameters
volatile uint16_t g_flag_adc = 0;
volatile float g_last_flag_temp_c = 30.0f;

// Chip temperature & compensation
static float T_chip_c  = 40.0f;
static float T0_chip_c = 25.0f;
static float alpha     = 0.0025f;
static float last_T_chip_c = 40.0f;

// Bad Pixel Correction
#define MAX_BAD_PIXELS ((N * 15) / 1000)  // 1.5%

typedef struct {
    uint16_t x;
    uint16_t rep_x;
    int8_t   rep_dy;
} BadPixelInLine_t;

typedef struct {
    uint16_t count;
    uint16_t offset;
} LineInfo_t;

__attribute__((aligned(16), section(".axisram1")))
LineInfo_t bad_pixel_line_info[VPIX];

__attribute__((aligned(16), section(".axisram1")))
BadPixelInLine_t bad_pixels_sorted[MAX_BAD_PIXELS];

typedef struct {
    uint32_t dst;
    uint32_t src;
} PixPatch;

__attribute__((aligned(64), section(".axisram1")))
static PixPatch g_patches[MAX_BAD_PIXELS];

__attribute__((aligned(16), section(".axisram1")))
static uint16_t g_patch_values[MAX_BAD_PIXELS];

static uint32_t g_num_patches = 0;
uint16_t total_bad_pixels = 0;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t rep_x;
    uint16_t rep_y;
} BadPixelFactory_t;

// Test bad pixels
const BadPixelFactory_t factory_bad_pixels[] = {
    {100, 50, 101, 50},
    {200, 100, 199, 100},
    {320, 240, 320, 239},
    {400, 300, 401, 300},
    {500, 400, 500, 401},
    {600, 450, 599, 450},
    {150, 200, 150, 199},
    {250, 350, 251, 350},
    {350, 150, 350, 151},
    {450, 250, 449, 250},
    {550, 350, 550, 349},
    {120, 220, 120, 221},
    {220, 320, 221, 320},
    {320, 420, 320, 419},
    {420, 120, 419, 120},
    {520, 220, 520, 221},
    {620, 320, 621, 320},
    {130, 230, 130, 229},
    {230, 330, 231, 330},
};

const uint16_t factory_bad_pixel_count =
    sizeof(factory_bad_pixels) / sizeof(factory_bad_pixels[0]);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_XSPI2_Init(void);

/* USER CODE BEGIN PFP */

// Planck LUT
void generate_linear_temp_lut(uint16_t *planck_table, float temp_min_c,
                              float temp_max_c, float flag_temp_c);
void init_planck_lut(void);
float decode_temperature(uint16_t value);

// Q15 Helpers
static inline uint16_t q15_from_float(float x);
static inline uint16_t q15_mul_scalar_u16(uint16_t a_q15, uint16_t b_q15);

// Calibration
void generate_dummy_gain_frame(uint16_t gain[VPIX][HPIX]);
void generate_dummy_emissivity_frame(uint16_t emissivity[VPIX][HPIX]);
void init_gain_frame(void);
void start_dark_frame_calibration(void);
uint8_t process_calibration_frame(const uint16_t sensor_frame[VPIX][HPIX]);
uint8_t is_calibrating(void);
static void XSPI_WriteEnable(XSPI_HandleTypeDef *hxspi);
static void XSPI_AutoPollingMemReady(XSPI_HandleTypeDef *hxspi);
static void XSPI_NOR_OctalDTRModeCfg(XSPI_HandleTypeDef *hxspi);
static HAL_StatusTypeDef XSPI_WriteGainFrame(const uint16_t *gain_frame, uint32_t flash_offset);
static HAL_StatusTypeDef XSPI_WriteEmissivityFrame(const uint16_t *emissivity_frame, uint32_t flash_offset);
static HAL_StatusTypeDef XSPI_EraseSector(uint32_t address);
static HAL_StatusTypeDef XSPI_ProgramPage(uint32_t address, const uint8_t *data, uint32_t size);

// Processing
static inline void DWT_CycleCounter_Init(void);
static inline void process_thermal_line_fastest(
    const uint16_t * __restrict__ sensor_data,
    const uint16_t * __restrict__ dark,
    const uint16_t * __restrict__ gain,
    const uint16_t * __restrict__ offset,
    const uint16_t * __restrict__ emissivity,
    const uint16_t * __restrict__ planck_lut,
    uint16_t * __restrict__ output,
    uint32_t width);

// Bad Pixel
void load_bad_pixel_map(void);
void build_bad_pixel_patches(void);
static inline void apply_bad_pixel_patches(uint16_t * __restrict__ frame, uint32_t count);

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

// ═══════════════════════════════════════════════════════════════════
// TIMER & VSYNC
// ═══════════════════════════════════════════════════════════════════
void XSPI_PerformanceTest(void)
{
    // Nutze temp_u16 als Test-Buffer (ist schon im RAM!)
    uint8_t *test_tx = (uint8_t*)frame_buffer_A;
    uint8_t *test_rx = (uint8_t*)frame_buffer_B;

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
        uint8_t use4k = ((addr & 0xFFFF) != 0); // use 4K unless 64K-aligned

        sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
        sCommand.Instruction = use4k ? OCTAL_SECTOR_ERASE_4K_CMD
                                     : OCTAL_BLOCK_ERASE_64K_CMD;
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
        SCB_InvalidateDCache_by_Addr((uint32_t*)test_rx, size);
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
        SCB_InvalidateDCache_by_Addr((uint32_t*)test_rx, size);
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
void thermal_vsync_init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();

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

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

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
    frames_processed++;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        vsync_count++;
        frame_ready = 1;
        thermal_frame_process();
    }
}

// ═══════════════════════════════════════════════════════════════════
// PLANCK LUT
// ═══════════════════════════════════════════════════════════════════

void generate_linear_temp_lut(uint16_t *planck_table, float temp_min_c,
                              float temp_max_c, float flag_temp_c)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════╗\n");
    printf("║  Generating LINEAR Temperature LUT                        ║\n");
    printf("╚═══════════════════════════════════════════════════════════╝\n");
    printf("\n");

    int temp_min_int = (int)temp_min_c;
    int temp_max_int = (int)temp_max_c;
    int flag_temp_int = (int)flag_temp_c;

    printf("Parameters:\n");
    printf("  • Min temp:      %d°C\n", temp_min_int);
    printf("  • Max temp:      %d°C\n", temp_max_int);
    printf("  • Flag temp:     %d°C\n", flag_temp_int);
    printf("  • Resolution:    0.01°C\n");
    printf("  • Output format: (T × 100) + 10000\n");
    printf("\n");

    printf("Generating 65536 entries...\n");

    for (uint32_t adc = 0; adc < 65536; adc++) {
        float temp_celsius = temp_min_c + ((float)adc / 65535.0f) * (temp_max_c - temp_min_c);
        float temp_relative = temp_celsius - flag_temp_c;
        int32_t temp_output = (int32_t)((temp_relative * TEMP_SCALE) + TEMP_OFFSET);

        if (temp_output < 0) temp_output = 0;
        if (temp_output > 65535) temp_output = 65535;

        planck_table[adc] = (uint16_t)temp_output;
    }

    printf("✓ Linear LUT generated!\n\n");

    printf("Sample LUT Values:\n");
    printf("  ADC      -> Output   -> Temperature\n");
    printf("  ────────────────────────────────────\n");

    uint32_t samples[] = {0, 8192, 16384, 32768, 49152, 57344, 65535};
    for (int i = 0; i < 7; i++) {
        uint32_t adc = samples[i];
        uint16_t output = planck_table[adc];

        int32_t temp_centi = (int32_t)output - 10000;
        int32_t temp_abs = (temp_centi < 0) ? -temp_centi : temp_centi;
        char sign = (temp_centi < 0) ? '-' : '+';
        uint32_t temp_int = temp_abs / 100;
        uint32_t temp_frac = temp_abs % 100;

        printf("  %-8lu -> %-7u -> %c%lu.%02lu°C\n",
               adc, output, sign, temp_int, temp_frac);
    }

    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════╗\n");
    printf("║  Linear LUT Ready!                                        ║\n");
    printf("╚═══════════════════════════════════════════════════════════╝\n");
    printf("\n");
}

void init_planck_lut(void) {
    generate_linear_temp_lut(planck_table, -20.0f, 150.0f, FLAG_TEMP_CELSIUS);
}

float decode_temperature(uint16_t value) {
    return ((float)value - TEMP_OFFSET) / TEMP_SCALE;
}

// ═══════════════════════════════════════════════════════════════════
// Q15 FIXED-POINT HELPERS
// ═══════════════════════════════════════════════════════════════════

static inline uint16_t q15_from_float(float x) {
    if (x < -0.999969f) x = -0.999969f;
    if (x >  0.999969f) x =  0.999969f;
    int32_t v = (int32_t)lrintf(x * 32768.0f);
    if (v < -32768) v = -32768;
    if (v >  32767) v =  32767;
    return (uint16_t)(int16_t)v;
}

static inline uint16_t q15_mul_scalar_u16(uint16_t a_q15, uint16_t b_q15) {
    int16_t a = (int16_t)a_q15, b = (int16_t)b_q15;
    int32_t p = (int32_t)a * (int32_t)b;
    p = (p + (1<<14)) >> 15;
    if (p < -32768) p = -32768;
    if (p >  32767) p =  32767;
    return (uint16_t)(int16_t)p;
}

// ═══════════════════════════════════════════════════════════════════
// DUMMY CALIBRATION GENERATION
// ═══════════════════════════════════════════════════════════════════

void generate_dummy_gain_frame(uint16_t gain[VPIX][HPIX]) {
    printf("Generating dummy gain frame (Q15)...\n");

    const float BASE_GAIN = 1.0f;
    const float NUC_VARIATION = 0.05f;  // ±5%

    const float cx = (float)HPIX / 2.0f;
    const float cy = (float)VPIX / 2.0f;
    const float max_radius = sqrtf(cx*cx + cy*cy);

    for (uint32_t y = 0; y < VPIX; y++) {
        for (uint32_t x = 0; x < HPIX; x++) {
            float gain_val = BASE_GAIN;

            // Pixel response variation
            uint32_t seed = (y * HPIX + x) * 1103515245 + 12345;
            float nuc = ((float)((seed >> 16) % 1000) / 1000.0f - 0.5f) * 2.0f * NUC_VARIATION;
            gain_val += nuc;

            // Vignetting (cos^4)
            float dx = (float)x - cx;
            float dy = (float)y - cy;
            float radius = sqrtf(dx*dx + dy*dy);
            float theta = radius / max_radius;

            if (theta < 1.0f) {
                float cos_theta = sqrtf(1.0f - theta*theta);
                float cos4 = cos_theta * cos_theta * cos_theta * cos_theta;
                float vignetting = 0.90f + 0.10f * cos4;
                gain_val *= vignetting;
            }

            gain[y][x] = q15_from_float(gain_val);
        }
    }

    // Mark bad pixels
    for (uint16_t i = 0; i < factory_bad_pixel_count; i++) {
        uint16_t x = factory_bad_pixels[i].x;
        uint16_t y = factory_bad_pixels[i].y;
        if (x < HPIX && y < VPIX) {
            gain[y][x] = 0;
        }
    }

    printf("  ✓ Gain frame generated (center: 0x%04X = %.3f)\n",
           gain[VPIX/2][HPIX/2],
           (float)(int16_t)gain[VPIX/2][HPIX/2] / 32768.0f);
}

void init_gain_frame(void) {
    printf("Initializing gain frame...\n");

    for (uint32_t y = 0; y < VPIX; y++) {
        for (uint32_t x = 0; x < HPIX; x++) {
            gain_frame[y][x] = 0x7FFF;  // Unity gain
        }
    }

    last_T_chip_c = T_chip_c;
    printf("✓ Gain frame initialized\n");
}

/**
 * @brief Generate dummy emissivity frame for thermal imaging
 * @param emissivity Output emissivity frame [VPIX][HPIX] in Q15 format
 *
 * Generates per-pixel emissivity values in Q15 fixed-point format.
 * - 0x0000 = 0.0 (perfect reflector)
 * - 0x8000 = 1.0 (perfect blackbody, 100% emissivity)
 * - 0x7FFF ≈ 0.999969 (maximum positive Q15)
 *
 * Default: Unity emissivity (0x8000) representing ideal blackbody behavior.
 * Optional material variations can be added (e.g., metals ~0.3, plastics ~0.95).
 */
void generate_dummy_emissivity_frame(uint16_t emissivity[VPIX][HPIX]) {
    printf("Generating dummy emissivity frame (Q15)...\n");

    // Default: Unity emissivity (100% = perfect blackbody)
    const float BASE_EMISSIVITY = 1.0f;

    for (uint32_t y = 0; y < VPIX; y++) {
        for (uint32_t x = 0; x < HPIX; x++) {
            // Unity emissivity for all pixels (0x8000 in Q15)
            emissivity[y][x] = q15_from_float(BASE_EMISSIVITY);

            // Optional: Add material-based spatial variation
            // Example zones for different materials:
            // if (x < HPIX/3) {
            //     emissivity[y][x] = q15_from_float(0.3f);  // Metal zone
            // } else if (x < 2*HPIX/3) {
            //     emissivity[y][x] = q15_from_float(0.95f); // Plastic zone
            // } else {
            //     emissivity[y][x] = q15_from_float(1.0f);  // Blackbody zone
            // }
        }
    }

    printf("  ✓ Emissivity frame generated (unity: 0x%04X = %.3f)\n",
           emissivity[VPIX/2][HPIX/2],
           (float)(int16_t)emissivity[VPIX/2][HPIX/2] / 32768.0f);
}

// ═══════════════════════════════════════════════════════════════════
// DARK FRAME CALIBRATION
// ═══════════════════════════════════════════════════════════════════

void start_dark_frame_calibration(void) {
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════╗\n");
    printf("║  Starting Dark Frame Calibration                         ║\n");
    printf("╚═══════════════════════════════════════════════════════════╝\n");
    printf("\n");

    printf("Parameters:\n");
    printf("  • Skip frames:    %d (settling time)\n", CALIB_SKIP_FRAMES);
    printf("  • Sample frames:  %d\n", CALIB_TOTAL_SAMPLES);
    printf("  • Total time:     ~%d ms\n",
           (CALIB_SKIP_FRAMES + CALIB_TOTAL_SAMPLES) * 20);
    printf("\n");

    printf("Initializing dark frame buffer...\n");
    for (uint32_t y = 0; y < VPIX; y++) {
        for (uint32_t x = 0; x < HPIX; x++) {
            frame_buffer_A[y][x] = 0;
        }
    }

    calib_state = CALIB_WAIT_STABLE;
    calib_frame_count = 0;
    calib_samples_collected = 0;

    printf("✓ Ready to capture\n");
    printf("\nWaiting for shutter to settle...\n");
}

uint8_t process_calibration_frame(const uint16_t sensor_frame[VPIX][HPIX]) {
    if (calib_state == CALIB_IDLE || calib_state == CALIB_COMPLETE) {
        return 1;
    }

    calib_frame_count++;

    if (calib_state == CALIB_WAIT_STABLE) {
        if (calib_frame_count >= CALIB_SKIP_FRAMES) {
            calib_state = CALIB_COLLECTING;
            printf("Shutter stable. Collecting samples...\n");
        }
        return 0;
    }

    if (calib_state == CALIB_COLLECTING) {
        calib_samples_collected++;

        printf("  Sample %lu/%d...\n", calib_samples_collected, CALIB_TOTAL_SAMPLES);

        // Incremental averaging
        for (uint32_t y = 0; y < VPIX; y++) {
            for (uint32_t x = 0; x < HPIX; x++) {
                uint16_t current_avg = frame_buffer_A[y][x];
                uint16_t new_sample = sensor_frame[y][x];

                int32_t diff = (int32_t)new_sample - (int32_t)current_avg;
                int32_t delta = diff / (int32_t)calib_samples_collected;
                int32_t new_avg = (int32_t)current_avg + delta;

                if (new_avg < 0) new_avg = 0;
                if (new_avg > 65535) new_avg = 65535;

                frame_buffer_A[y][x] = (uint16_t)new_avg;
            }
        }

        if (calib_samples_collected >= CALIB_TOTAL_SAMPLES) {
            calib_state = CALIB_COMPLETE;

            printf("\n✓ Calibration complete!\n");
            printf("  Total frames: %lu\n", calib_frame_count);
            printf("  Samples used: %lu\n", calib_samples_collected);
            printf("\n");

            printf("Copying to dark_frame array...\n");
            for (uint32_t y = 0; y < VPIX; y++) {
                for (uint32_t x = 0; x < HPIX; x++) {
                    dark_frame[y][x] = frame_buffer_A[y][x];
                }
            }
            printf("✓ Dark frame stored\n");
            printf("\n");

            unsigned enc = (unsigned)((g_last_flag_temp_c - FLAG_TEMP_CELSIUS) * TEMP_SCALE + TEMP_OFFSET);
            printf("✓ Flag ADC set: %u (T_flag_enc=%u)\n", g_flag_adc, enc);

            return 1;
        }

        return 0;
    }

    return 1;
}

uint8_t is_calibrating(void) {
    return (calib_state != CALIB_IDLE && calib_state != CALIB_COMPLETE);
}

// ═══════════════════════════════════════════════════════════════════
// THERMAL PROCESSING
// ═══════════════════════════════════════════════════════════════════

static inline void DWT_CycleCounter_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    #if defined(DWT_LAR)
    DWT->LAR = 0xC5ACCE55;
    #endif

    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline void process_thermal_line_fastest(
    const uint16_t * __restrict__ sensor_data,
    const uint16_t * __restrict__ dark,
    const uint16_t * __restrict__ gain,
    const uint16_t * __restrict__ offset,
    const uint16_t * __restrict__ emissivity,
    const uint16_t * __restrict__ planck_lut,
    uint16_t * __restrict__ output,
    uint32_t width)
{
    const uint16x8_t flag_vec = vdupq_n_u16(g_flag_adc);

    for (uint32_t x = 0; x < width; x += 8)
    {
        // ═══════════════════════════════════════════════════════════
        // Load input data (8 pixels at once)
        // ═══════════════════════════════════════════════════════════
        uint16x8_t adc   = vld1q_u16(&sensor_data[x]);
        uint16x8_t darkv = vld1q_u16(&dark[x]);
        uint16x8_t gainv = vld1q_u16(&gain[x]);
        uint16x8_t offv  = vld1q_u16(&offset[x]);
        uint16x8_t emiss = vld1q_u16(&emissivity[x]);

        // ═══════════════════════════════════════════════════════════
        // Dark frame correction
        // ═══════════════════════════════════════════════════════════
        uint16x8_t corr = vqsubq_u16(adc, darkv);

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
        val = vqaddq_u16(val, flag_vec);
        val = vqaddq_u16(val, offv);

        // ═══════════════════════════════════════════════════════════
        // Planck LUT lookup
        // ═══════════════════════════════════════════════════════════
        uint16x8_t out = vldrhq_gather_shifted_offset_u16(planck_lut, val);

        // ═══════════════════════════════════════════════════════════
        // Store result
        // ═══════════════════════════════════════════════════════════
        vst1q_u16(&output[x], out);
    }
}

// ═══════════════════════════════════════════════════════════════════
// BAD PIXEL CORRECTION
// ═══════════════════════════════════════════════════════════════════

void load_bad_pixel_map(void) {
    printf("Loading bad pixel map...\n");

    for (uint16_t y = 0; y < VPIX; y++) {
        bad_pixel_line_info[y].count = 0;
        bad_pixel_line_info[y].offset = 0;
    }

    uint16_t current_offset = 0;

    for (uint16_t y = 0; y < VPIX; y++) {
        bad_pixel_line_info[y].offset = current_offset;

        for (uint16_t i = 0; i < factory_bad_pixel_count; i++) {
            if (factory_bad_pixels[i].y != y) continue;

            uint16_t x  = factory_bad_pixels[i].x;
            uint16_t rx = factory_bad_pixels[i].rep_x;
            uint16_t ry = factory_bad_pixels[i].rep_y;

            if (x >= HPIX || y >= VPIX || rx >= HPIX || ry >= VPIX) {
                continue;
            }

            if (current_offset >= MAX_BAD_PIXELS) {
                printf("BPR: Max bad pixels reached!\n");
                break;
            }

            bad_pixels_sorted[current_offset].x      = x;
            bad_pixels_sorted[current_offset].rep_x  = rx;
            bad_pixels_sorted[current_offset].rep_dy = (int8_t)(ry - y);

            gain_frame[y][x] = 0;

            current_offset++;
            bad_pixel_line_info[y].count++;
        }
    }

    total_bad_pixels = current_offset;

    printf("✓ Loaded %u bad pixels\n", total_bad_pixels);
    printf("✓ Loaded %u bad pixels\n", total_bad_pixels);

        if (total_bad_pixels > 0) {
            build_bad_pixel_patches();
        } else {
            printf("BPR: No bad pixels to process\n");
            g_num_patches = 0;
        }
    }

    void build_bad_pixel_patches(void)
    {
        g_num_patches = 0;
        uint32_t dropped = 0;

        if (total_bad_pixels == 0) {
            printf("BPR: No bad pixels to build patches from\n");
            return;
        }

        for (uint16_t y = 0; y < VPIX; ++y) {
            const uint16_t cnt  = bad_pixel_line_info[y].count;
            const uint16_t off  = bad_pixel_line_info[y].offset;

            if (off >= MAX_BAD_PIXELS) {
                printf("BPR: Invalid offset %u at line %u\n", off, y);
                continue;
            }

            if (cnt == 0) continue;

            if (off + cnt > MAX_BAD_PIXELS) {
                printf("BPR: Overflow at line %u (off=%u cnt=%u)\n", y, off, cnt);
                continue;
            }

            const BadPixelInLine_t *px = &bad_pixels_sorted[off];

            for (uint16_t i = 0; i < cnt; ++i) {
                int16_t ry = (int16_t)y + (int16_t)px[i].rep_dy;
                uint16_t x  = px[i].x;
                uint16_t rx = px[i].rep_x;

                if (ry < 0 || ry >= (int16_t)VPIX || x >= HPIX || rx >= HPIX) {
                    dropped++;
                    continue;
                }

                if (g_num_patches >= MAX_BAD_PIXELS) {
                    printf("BPR: Reached max patches limit!\n");
                    break;
                }

                g_patches[g_num_patches].dst = (uint32_t)y  * HPIX + x;
                g_patches[g_num_patches].src = (uint32_t)ry * HPIX + rx;
                g_num_patches++;
            }

            if (g_num_patches >= MAX_BAD_PIXELS) break;
        }

        if (dropped > 0) {
            printf("BPR: dropped %lu out-of-range entries\n", (unsigned long)dropped);
        }
        printf("BPR: %lu valid patches\n", (unsigned long)g_num_patches);
    }

    static inline void apply_bad_pixel_patches(uint16_t * __restrict__ frame, uint32_t count)
    {
        // Phase 1: Buffer source values
        for (uint32_t i = 0; i < count; ++i)
            g_patch_values[i] = frame[g_patches[i].src];

        // Phase 2: Write to destinations
        for (uint32_t i = 0; i < count; ++i)
            frame[g_patches[i].dst] = g_patch_values[i];
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

        /* Enable the CPU Cache */
        SCB_EnableICache();
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

        printf("\n");
        printf("╔═══════════════════════════════════════════════════════════╗\n");
        printf("║  STM32N6 Thermal Imaging Pipeline v1.0                   ║\n");
        printf("╚═══════════════════════════════════════════════════════════╝\n");
        printf("\n");


        XSPI_NOR_OctalDTRModeCfg(&hxspi2);

        /* Optional: Basic Write/Read Test (erase/write/read via commands) */
        XSPI_WriteEnable(&hxspi2);

        /* Performance Test */
        XSPI_PerformanceTest();
        // ═══════════════════════════════════════════════════════════════
        // Initialize Planck LUT
        // ═══════════════════════════════════════════════════════════════
        init_planck_lut();

        for (uint32_t i = 0; i < HPIX; ++i) {
            g_unity_q15_line[i] = 0x7FFF;
        }

        // ═══════════════════════════════════════════════════════════════
        // Initialize XSPI - USE OLD WORKING METHOD!
        // ═══════════════════════════════════════════════════════════════
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

        // ═══════════════════════════════════════════════════════════════
        // CRITICAL: Configure MPU for XSPI (THIS WAS MISSING!)
        // ═══════════════════════════════════════════════════════════════
        uint32_t xspi_mmap_base = XSPI2_MMAP_BASE;
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
        __DSB();
        __ISB();

        printf("✓ XSPI Memory-Mapped Mode ready\n");


        //complex gain frame to XSPI
        generate_dummy_gain_frame(gain_frame);
        //simple gain frame to XSPI to debug!
        init_gain_frame();


        // Test: Check gain_frame values BEFORE writing
        printf("\nDEBUG: Checking gain_frame before write:\n");
        printf("  [0][0]:       0x%04X\n", gain_frame[0][0]);
        printf("  [240][320]:   0x%04X\n", gain_frame[240][320]);
        printf("  [479][639]:   0x%04X\n", gain_frame[479][639]);

        // Simple test: Fill with known pattern
        printf("\nDEBUG: Filling with test pattern...\n");
        for (uint32_t y = 0; y < 10; y++) {
            for (uint32_t x = 0; x < 10; x++) {
                gain_frame[y][x] = 0x7FFF;  // Unity gain
            }
        }
        printf("  [0][0] after fill: 0x%04X\n", gain_frame[0][0]);
		printf("  ✓ Test pattern written to gain_frame\n");

        if (XSPI_WriteGainFrame(&gain_frame[0][0], XSPI_GAIN_OFFSET) != HAL_OK) {
            printf("✗ Failed to write gain frame!\n");
            Error_Handler();
        }

        // ═══════════════════════════════════════════════════════════════
        // NOW we can safely read from XSPI!
        // ═══════════════════════════════════════════════════════════════
        printf("\nLoading gain frame from XSPI...\n");
        printf("  Copying from 0x%08lX to AXISRAM1...\n", (uint32_t)XSPI_GAIN_OFFSET);

        for (uint32_t y = 0; y < VPIX; y++) {
            if (y % 48 == 0) {
                printf("  Progress: %lu%%...\n", (y * 100) / VPIX);
                fflush(stdout);
            }

            for (uint32_t x = 0; x < HPIX; x++) {
                gain_frame[y][x] = xspi_gain[y][x];
            }
        }

        printf("\n  ✓ Gain frame loaded to AXISRAM1\n");
        printf("  Sample value [240][320]: 0x%04X (%.3f)\n",
               gain_frame[240][320],
               (float)(int16_t)gain_frame[240][320] / 32768.0f);

        // ═══════════════════════════════════════════════════════════════
        // Generate and Write Emissivity Frame to XSPI
        // ═══════════════════════════════════════════════════════════════
        printf("\n");
        generate_dummy_emissivity_frame(gain_frame);  // Reuse gain_frame buffer

        if (XSPI_WriteEmissivityFrame(&gain_frame[0][0], XSPI_EMISSIVITY_OFFSET) != HAL_OK) {
            printf("✗ Failed to write emissivity frame!\n");
            Error_Handler();
        }

        printf("\nEmissivity frame verification from XSPI...\n");
        printf("  Memory-mapped address: 0x%08lX\n", (uint32_t)(XSPI_BASE + XSPI_EMISSIVITY_OFFSET));
        printf("  Sample value [240][320]: 0x%04X (%.3f emissivity)\n",
               xspi_emissivity[240][320],
               (float)xspi_emissivity[240][320] / 32768.0f);

        // ═══════════════════════════════════════════════════════════════
        // Restore gain frame from XSPI (we reused the buffer)
        // ═══════════════════════════════════════════════════════════════
        printf("\nRestoring gain frame to AXISRAM1...\n");
        for (uint32_t y = 0; y < VPIX; y++) {
            for (uint32_t x = 0; x < HPIX; x++) {
                gain_frame[y][x] = xspi_gain[y][x];
            }
        }
        printf("  ✓ Gain frame restored\n");

        // Rest of initialization...
        printf("\nInitializing thermal pipeline...\n");
        DWT_CycleCounter_Init();

        // Initialize test data (dummy sensor frame)
        printf("  • Test pattern...");
        for (uint32_t y = 0; y < VPIX; y++) {
            for (uint32_t x = 0; x < HPIX; x++) {
                frame_buffer_A[y][x] = 8000 + (x % 100);
            }
        }
        printf(" ✓\n");

        // Initialize calibration data
        printf("  • Calibration data...");

        // Initialize dark frame (will be replaced by real calibration)
        for (uint32_t y = 0; y < VPIX; y++) {
            for (uint32_t x = 0; x < HPIX; x++) {
                dark_frame[y][x] = 100 + (x % 10);
            }
        }

        // Initialize offset line (zero for now)
        for (uint32_t i = 0; i < HPIX; i++) {
            offset_line[i] = 0;
        }

        printf(" ✓\n");

        // Initialize 50 Hz VSYNC timer (optional)
        printf("\n");
        printf("Initializing 50 Hz VSYNC timer...\n");
        thermal_vsync_init();
        thermal_vsync_start();

        /* USER CODE END 2 */

        /* Infinite loop */
        /* USER CODE BEGIN WHILE */

        // Load bad pixel map
        printf("\n");
        load_bad_pixel_map();

        // Initialize gain frame (already loaded from XSPI above)
        // init_gain_frame();  // Don't overwrite XSPI data!

        printf("\n");
        printf("╔═══════════════════════════════════════════════════════════╗\n");
        printf("║  System Ready - Starting Processing Loop                 ║\n");
        printf("╚═══════════════════════════════════════════════════════════╝\n");
        printf("\n");

        static int count = 0;

        while (1)
        {
            /* USER CODE END WHILE */

            /* USER CODE BEGIN 3 */

            DWT->CYCCNT = 0;

            // ═══════════════════════════════════════════════════════════
            // Calibration Mode
            // ═══════════════════════════════════════════════════════════
            if (is_calibrating()) {
                if (process_calibration_frame(frame_buffer_B)) {
                    printf("Returning to normal processing...\n\n");
                }
                HAL_Delay(20);
                continue;
            }

            // ═══════════════════════════════════════════════════════════
            // Temperature Compensation (on-demand)
            // ═══════════════════════════════════════════════════════════
            float temp_diff = fabsf(T_chip_c - last_T_chip_c);

            if (temp_diff > 0.5f) {
                // Update gain frame for temperature change
                float dT = (T_chip_c - T0_chip_c);
                float temp_gain_f = 1.0f / (1.0f + alpha * dT);
                uint16_t temp_gain_q15 = q15_from_float(temp_gain_f);

                for (uint32_t y = 0; y < VPIX; y++) {
                    for (uint32_t x = 0; x < HPIX; x++) {
                        gain_frame[y][x] = q15_mul_scalar_u16(0x7FFF, temp_gain_q15);
                    }
                }

                last_T_chip_c = T_chip_c;
            }

            // ═══════════════════════════════════════════════════════════
            // Main Thermal Processing Pipeline
            // ═══════════════════════════════════════════════════════════
            for (int line = 0; line < VPIX; ++line)
            {
                process_thermal_line_fastest(
                    frame_buffer_A[line],
                    dark_frame[line],
                    gain_frame[line],           // From AXISRAM1
                    offset_line,
					(const uint16_t *)xspi_emissivity[line],
                    planck_table,
                    frame_buffer_B[line],
                    HPIX
                );
            }

            // ═══════════════════════════════════════════════════════════
            // Bad Pixel Correction
            // ═══════════════════════════════════════════════════════════
            if (g_num_patches > 0) {
                apply_bad_pixel_patches(&frame_buffer_B[0][0], g_num_patches);
            }

            // ═══════════════════════════════════════════════════════════
            // Output Verification (prevents over-optimization)
            // ═══════════════════════════════════════════════════════════
            volatile uint32_t checksum = 0;
            for (int y = 0; y < VPIX; y += 10) {
                checksum += frame_buffer_B[y][0];
            }

            // ═══════════════════════════════════════════════════════════
            // Performance Monitoring
            // ═══════════════════════════════════════════════════════════
            uint32_t cycles = DWT->CYCCNT;

            if (++count >= 100) {
                printf("Frame: %lu cycles (%lu ms) - CS: %lu - BP: %lu\n",
                       cycles, cycles / 600000, checksum, (unsigned long)g_num_patches);
                count = 0;

                // Optional: Trigger calibration every 100 frames
                // start_dark_frame_calibration();
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

        /** Configure the System Power Supply */
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

        /** Get current CPU/System buses clocks configuration */
        HAL_RCC_GetClockConfig(&RCC_ClkInitStruct);
        if ((RCC_ClkInitStruct.CPUCLKSource == RCC_CPUCLKSOURCE_IC1) ||
            (RCC_ClkInitStruct.SYSCLKSource == RCC_SYSCLKSOURCE_IC2_IC6_IC11))
        {
            RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK);
            RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_HSI;
            RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
            if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
            {
                Error_Handler();
            }
        }

        /** Initializes the RCC Oscillators */
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

        /** Initializes the CPU, AHB and APB buses clocks */
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
        GPIO_InitTypeDef GPIO_InitStruct = {0};

        /* USER CODE BEGIN MX_GPIO_Init_1 */
        /* USER CODE END MX_GPIO_Init_1 */

        /* GPIO Ports Clock Enable */
        __HAL_RCC_GPIOG_CLK_ENABLE();

        /*Configure GPIO pin Output Level */
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

        /*Configure GPIO pin : LED1_Pin */
        GPIO_InitStruct.Pin = LED1_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

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

    /* USER CODE END 4 */

    /**
      * @brief  This function is executed in case of error occurrence.
      * @retval None
      */
    void Error_Handler(void)
    {
        /* USER CODE BEGIN Error_Handler_Debug */
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
    /* === XSPI helpers (protected) ============================================ */
    static void XSPI_WriteEnable(XSPI_HandleTypeDef *hxspi)
    {
        XSPI_RegularCmdTypeDef sCommand = {0};
        XSPI_AutoPollingTypeDef sConfig = {0};

        // Write Enable command
        sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
        sCommand.Instruction        = OCTAL_WRITE_ENABLE_CMD;
        sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
        sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
        sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
        sCommand.AddressMode        = HAL_XSPI_ADDRESS_NONE;
        sCommand.DataMode           = HAL_XSPI_DATA_NONE;
        sCommand.DummyCycles        = 0;
        sCommand.DQSMode            = HAL_XSPI_DQS_DISABLE;
        sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;

        if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            Error_Handler();
        }

        // Auto-polling to wait for write enable
        sCommand.Instruction    = OCTAL_READ_STATUS_REG_CMD;
        sCommand.Address        = 0x0;
        sCommand.AddressMode    = HAL_XSPI_ADDRESS_8_LINES;
        sCommand.AddressWidth   = HAL_XSPI_ADDRESS_32_BITS;
        sCommand.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_ENABLE;
        sCommand.DataMode       = HAL_XSPI_DATA_8_LINES;
        sCommand.DataDTRMode    = HAL_XSPI_DATA_DTR_ENABLE;
        sCommand.DataLength     = 2;
        sCommand.DummyCycles    = DUMMY_CLOCK_CYCLES_READ_OCTAL;
        sCommand.DQSMode        = HAL_XSPI_DQS_ENABLE;

        if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            Error_Handler();
        }

        sConfig.MatchMode      = HAL_XSPI_MATCH_MODE_AND;
        sConfig.AutomaticStop  = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
        sConfig.IntervalTime   = 0x10;
        sConfig.MatchMask  = 0x0202; // WEL in beiden Bytes
        sConfig.MatchValue = 0x0202;

        if (HAL_XSPI_AutoPolling(hxspi, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            Error_Handler();
        }
    }

    static void XSPI_AutoPollingMemReady(XSPI_HandleTypeDef *hxspi)
    {
        XSPI_RegularCmdTypeDef sCommand = {0};
        XSPI_AutoPollingTypeDef sConfig = {0};

        sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
        sCommand.Instruction        = OCTAL_READ_STATUS_REG_CMD;
        sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
        sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
        sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
        sCommand.Address            = 0x0;
        sCommand.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
        sCommand.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
        sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
        sCommand.DataMode           = HAL_XSPI_DATA_8_LINES;
        sCommand.DataDTRMode        = HAL_XSPI_DATA_DTR_ENABLE;
        sCommand.DataLength         = 2;
        sCommand.DummyCycles        = DUMMY_CLOCK_CYCLES_READ_OCTAL;
        sCommand.DQSMode            = HAL_XSPI_DQS_ENABLE;
        sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;

        if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            Error_Handler();
        }

        sConfig.MatchMode      = HAL_XSPI_MATCH_MODE_AND;
        sConfig.AutomaticStop  = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
        sConfig.IntervalTime   = 0x10;
        sConfig.MatchMask  = 0x0101; // WIP in beiden Bytes
        sConfig.MatchValue = 0x0000; // ready

        if (HAL_XSPI_AutoPolling(hxspi, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
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
    HAL_StatusTypeDef XSPI_WriteGainFrame(const uint16_t *gain_frame, uint32_t flash_offset)
    {
        const uint32_t GAIN_FRAME_SIZE = 640 * 480 * 2;  // 614,400 bytes
        const uint32_t BLOCK_SIZE = 64 * 1024;            // 64 KB
        const uint32_t PAGE_SIZE = 256;                   // 256 bytes

        printf("\n");
        printf("╔═══════════════════════════════════════════════════════════╗\n");
        printf("║  Writing Gain Frame to XSPI Flash                        ║\n");
        printf("╚═══════════════════════════════════════════════════════════╝\n");
        printf("\n");

        printf("Target address: 0x%08lX\n", XSPI_GAIN_OFFSET);
        printf("Size:           %lu bytes (%lu KB)\n", GAIN_FRAME_SIZE, GAIN_FRAME_SIZE / 1024);

        // ═══════════════════════════════════════════════════════════════
        // 1. Exit Memory-Mapped Mode
        // ═══════════════════════════════════════════════════════════════
        printf("\n[1/4] Exiting memory-mapped mode...\n");

        if (HAL_XSPI_Abort(&hxspi2) != HAL_OK) {
            printf("  ✗ Failed to exit memory-mapped mode!\n");
            return HAL_ERROR;
        }

        printf("  ✓ Memory-mapped mode disabled\n");

        // ═══════════════════════════════════════════════════════════════
        // 2. Erase Required Blocks (64KB each)
        // ═══════════════════════════════════════════════════════════════
        uint32_t num_blocks = (GAIN_FRAME_SIZE + BLOCK_SIZE - 1) / BLOCK_SIZE;

        printf("\n[2/4] Erasing %lu blocks (64KB each)...\n", num_blocks);

        uint32_t erase_start = DWT->CYCCNT;

        for (uint32_t i = 0; i < num_blocks; i++) {
            uint32_t block_addr = flash_offset + (i * BLOCK_SIZE);

            printf("  Erasing block %lu/%lu @ 0x%08lX...\n", i + 1, num_blocks, block_addr);
            fflush(stdout);

            if (XSPI_EraseSector(block_addr) != HAL_OK) {
                printf("\n  ✗ Erase failed at block %lu!\n", i);
                return HAL_ERROR;
            }
        }

        uint32_t erase_cycles = DWT->CYCCNT - erase_start;
        printf("\n  ✓ Erase complete (%lu ms)\n", erase_cycles / 600000);

        // ═══════════════════════════════════════════════════════════════
        // 3. Program Data (256 bytes per page) - 1 MB/s!
        // ═══════════════════════════════════════════════════════════════
        const uint8_t *src = (const uint8_t *)gain_frame;
        uint32_t num_pages = (GAIN_FRAME_SIZE + PAGE_SIZE - 1) / PAGE_SIZE;

        printf("\n[3/4] Programming %lu pages (256 bytes each)...\n", num_pages);

        uint32_t prog_start = DWT->CYCCNT;

        for (uint32_t i = 0; i < num_pages; i++) {
            uint32_t page_addr = flash_offset + (i * PAGE_SIZE);
            uint32_t page_size = PAGE_SIZE;

            // Last page might be smaller
            if (i == num_pages - 1) {
                uint32_t remainder = GAIN_FRAME_SIZE % PAGE_SIZE;
                if (remainder != 0) {
                    page_size = remainder;
                }
            }

            // Progress indicator every 100 pages
            if (i % 100 == 0 || i == num_pages - 1) {
                uint32_t percent = (i * 100) / num_pages;
                printf("  Programming: %lu%% (%lu/%lu pages)...\n",
                       percent, i + 1, num_pages);
                fflush(stdout);
            }

            if (XSPI_ProgramPage(page_addr, &src[i * PAGE_SIZE], page_size) != HAL_OK) {
                printf("\n  ✗ Program failed at page %lu (addr: 0x%08lX)!\n",
                       i, page_addr);
                return HAL_ERROR;
            }
        }

        uint32_t prog_cycles = DWT->CYCCNT - prog_start;
        uint32_t prog_us = prog_cycles / 600;
        uint32_t speed_kbps = (GAIN_FRAME_SIZE * 1000) / prog_us;  // KB/s

        printf("\n  ✓ Programming complete\n");
        printf("    Time: %lu ms\n", prog_cycles / 600000);
        printf("    Speed: %lu KB/s\n", speed_kbps);

        // ═══════════════════════════════════════════════════════════════
        // 4. Re-enter Memory-Mapped Mode
        // ═══════════════════════════════════════════════════════════════
        printf("\n[4/4] Re-entering memory-mapped mode...\n");

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
            printf("  ✗ READ-CFG failed\n");
            return HAL_ERROR;
        }
        // WRITE configuration (add this block before HAL_XSPI_MemoryMapped)
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
            printf("  ✗ Write-CFG failed\n");
            return HAL_ERROR;
        }


        // Enter Memory-Mapped mode
        XSPI_MemoryMappedTypeDef sMMAP = {0};
        sMMAP.TimeOutActivation  = HAL_XSPI_TIMEOUT_COUNTER_ENABLE;
        sMMAP.TimeoutPeriodClock = 0x50;

        if (HAL_XSPI_MemoryMapped(&hxspi2, &sMMAP) != HAL_OK) {
            printf("  ✗ Memory-Mapped re-entry failed\n");
            return HAL_ERROR;
        }

        printf("  ✓ Memory-mapped mode enabled\n");

        // ═══════════════════════════════════════════════════════════════
        // Success!
        // ═══════════════════════════════════════════════════════════════
        printf("\n");
        printf("╔═══════════════════════════════════════════════════════════╗\n");
        printf("║  ✓ Gain Frame written successfully!                      ║\n");
        printf("╚═══════════════════════════════════════════════════════════╝\n");
        printf("\n");

        return HAL_OK;
    }

    /**
     * @brief Write emissivity frame to XSPI NOR Flash
     * @param emissivity_frame Pointer to emissivity data [VPIX][HPIX] in Q15 format
     * @param flash_offset Flash offset address (e.g., XSPI_EMISSIVITY_OFFSET)
     * @return HAL status
     *
     * Writes 640×480×2 = 600 KB emissivity calibration data to external flash.
     * Automatically handles:
     * - Memory-mapped mode exit/re-entry
     * - Block erase (64 KB blocks)
     * - Page programming (256 byte pages)
     * - Performance measurement
     */
    HAL_StatusTypeDef XSPI_WriteEmissivityFrame(const uint16_t *emissivity_frame, uint32_t flash_offset)
    {
        const uint32_t EMISSIVITY_FRAME_SIZE = 640 * 480 * 2;  // 614,400 bytes
        const uint32_t BLOCK_SIZE = 64 * 1024;                  // 64 KB
        const uint32_t PAGE_SIZE = 256;                         // 256 bytes

        printf("\n");
        printf("╔═══════════════════════════════════════════════════════════╗\n");
        printf("║  Writing Emissivity Frame to XSPI Flash                  ║\n");
        printf("╚═══════════════════════════════════════════════════════════╝\n");
        printf("\n");

        printf("Target address: 0x%08lX\n", flash_offset);
        printf("Size:           %lu bytes (%lu KB)\n", EMISSIVITY_FRAME_SIZE, EMISSIVITY_FRAME_SIZE / 1024);

        // ═══════════════════════════════════════════════════════════════
        // 1. Exit Memory-Mapped Mode
        // ═══════════════════════════════════════════════════════════════
        printf("\n[1/4] Exiting memory-mapped mode...\n");

        if (HAL_XSPI_Abort(&hxspi2) != HAL_OK) {
            printf("  ✗ Failed to exit memory-mapped mode!\n");
            return HAL_ERROR;
        }

        printf("  ✓ Memory-mapped mode disabled\n");

        // ═══════════════════════════════════════════════════════════════
        // 2. Erase Required Blocks (64KB each)
        // ═══════════════════════════════════════════════════════════════
        uint32_t num_blocks = (EMISSIVITY_FRAME_SIZE + BLOCK_SIZE - 1) / BLOCK_SIZE;

        printf("\n[2/4] Erasing %lu blocks (64KB each)...\n", num_blocks);

        uint32_t erase_start = DWT->CYCCNT;

        for (uint32_t i = 0; i < num_blocks; i++) {
            uint32_t block_addr = flash_offset + (i * BLOCK_SIZE);

            printf("  Erasing block %lu/%lu @ 0x%08lX...\n", i + 1, num_blocks, block_addr);
            fflush(stdout);

            if (XSPI_EraseSector(block_addr) != HAL_OK) {
                printf("\n  ✗ Erase failed at block %lu!\n", i);
                return HAL_ERROR;
            }
        }

        uint32_t erase_cycles = DWT->CYCCNT - erase_start;
        printf("\n  ✓ Erase complete (%lu ms)\n", erase_cycles / 600000);

        // ═══════════════════════════════════════════════════════════════
        // 3. Program Data (256 bytes per page)
        // ═══════════════════════════════════════════════════════════════
        const uint8_t *src = (const uint8_t *)emissivity_frame;
        uint32_t num_pages = (EMISSIVITY_FRAME_SIZE + PAGE_SIZE - 1) / PAGE_SIZE;

        printf("\n[3/4] Programming %lu pages (256 bytes each)...\n", num_pages);

        uint32_t prog_start = DWT->CYCCNT;

        for (uint32_t i = 0; i < num_pages; i++) {
            uint32_t page_addr = flash_offset + (i * PAGE_SIZE);
            uint32_t page_size = PAGE_SIZE;

            // Last page might be smaller
            if (i == num_pages - 1) {
                uint32_t remainder = EMISSIVITY_FRAME_SIZE % PAGE_SIZE;
                if (remainder != 0) {
                    page_size = remainder;
                }
            }

            // Progress indicator every 100 pages
            if (i % 100 == 0 || i == num_pages - 1) {
                uint32_t percent = (i * 100) / num_pages;
                printf("  Programming: %lu%% (%lu/%lu pages)...\n",
                       percent, i + 1, num_pages);
                fflush(stdout);
            }

            if (XSPI_ProgramPage(page_addr, &src[i * PAGE_SIZE], page_size) != HAL_OK) {
                printf("\n  ✗ Program failed at page %lu (addr: 0x%08lX)!\n",
                       i, page_addr);
                return HAL_ERROR;
            }
        }

        uint32_t prog_cycles = DWT->CYCCNT - prog_start;
        uint32_t prog_us = prog_cycles / 600;
        uint32_t speed_kbps = (EMISSIVITY_FRAME_SIZE * 1000) / prog_us;  // KB/s

        printf("\n  ✓ Programming complete\n");
        printf("    Time: %lu ms\n", prog_cycles / 600000);
        printf("    Speed: %lu KB/s\n", speed_kbps);

        // ═══════════════════════════════════════════════════════════════
        // 4. Re-enter Memory-Mapped Mode
        // ═══════════════════════════════════════════════════════════════
        printf("\n[4/4] Re-entering memory-mapped mode...\n");

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
            printf("  ✗ READ-CFG failed\n");
            return HAL_ERROR;
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
            printf("  ✗ Write-CFG failed\n");
            return HAL_ERROR;
        }

        // Enter Memory-Mapped mode
        XSPI_MemoryMappedTypeDef sMMAP = {0};
        sMMAP.TimeOutActivation  = HAL_XSPI_TIMEOUT_COUNTER_ENABLE;
        sMMAP.TimeoutPeriodClock = 0x50;

        if (HAL_XSPI_MemoryMapped(&hxspi2, &sMMAP) != HAL_OK) {
            printf("  ✗ Memory-Mapped re-entry failed\n");
            return HAL_ERROR;
        }

        printf("  ✓ Memory-mapped mode enabled\n");

        // ═══════════════════════════════════════════════════════════════
        // Success!
        // ═══════════════════════════════════════════════════════════════
        printf("\n");
        printf("╔═══════════════════════════════════════════════════════════╗\n");
        printf("║  ✓ Emissivity Frame written successfully!                ║\n");
        printf("╚═══════════════════════════════════════════════════════════╝\n");
        printf("\n");

        return HAL_OK;
    }

    static HAL_StatusTypeDef XSPI_EraseSector(uint32_t address)
    {
        XSPI_RegularCmdTypeDef sCommand = {0};

        // Strip memory-mapped base if present
        if (address >= 0x70000000) {
            printf("  [DEBUG] Stripping MMAP base: 0x%08lX → ", address);
            address &= 0x0FFFFFFF;  // Keep only offset
            printf("0x%08lX\n", address);
        }

        // Write enable
        XSPI_WriteEnable(&hxspi2);

        // Erase command
        sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
        sCommand.Instruction        = OCTAL_BLOCK_ERASE_64K_CMD;
        sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
        sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
        sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
        sCommand.Address            = address;  // ← Now correct flash offset
        sCommand.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
        sCommand.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
        sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
        sCommand.DataMode           = HAL_XSPI_DATA_NONE;
        sCommand.DummyCycles        = 0;
        sCommand.DQSMode            = HAL_XSPI_DQS_DISABLE;
        sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;

        if (HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            return HAL_ERROR;
        }

        XSPI_AutoPollingMemReady(&hxspi2);
        return HAL_OK;
    }

    static HAL_StatusTypeDef XSPI_ProgramPage(uint32_t address, const uint8_t *data, uint32_t size)
    {
        XSPI_RegularCmdTypeDef sCommand = {0};

        if (size > 256) return HAL_ERROR;

        // Strip memory-mapped base if present
        if (address >= 0x70000000) {
            address &= 0x0FFFFFFF;
        }

        XSPI_WriteEnable(&hxspi2);

        sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
        sCommand.Instruction        = OCTAL_PAGE_PROG_CMD;
        sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
        sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
        sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
        sCommand.Address            = address;
        sCommand.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
        sCommand.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
        sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
        sCommand.DataMode           = HAL_XSPI_DATA_8_LINES;
        sCommand.DataDTRMode        = HAL_XSPI_DATA_DTR_ENABLE;
        sCommand.DataLength         = size;
        sCommand.DummyCycles        = 0;
        sCommand.DQSMode            = HAL_XSPI_DQS_DISABLE;
        sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;

        if (HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            return HAL_ERROR;
        }

        if (HAL_XSPI_Transmit(&hxspi2, (uint8_t*)data, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            return HAL_ERROR;
        }

        XSPI_AutoPollingMemReady(&hxspi2);
        return HAL_OK;
    }

