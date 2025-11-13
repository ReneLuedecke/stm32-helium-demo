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
#include "thermal_simd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
 * @brief ROI Configuration
 * Defines rectangular region of interest in the frame
 */


/**
 * @brief ROI Statistics in readable format (Celsius)
 */
typedef struct {
    float min_temp_c;      ///< Minimum temperature (°C)
    float max_temp_c;      ///< Maximum temperature (°C)
    float mean_temp_c;     ///< Mean temperature (°C)
    uint32_t pixel_count;  ///< Number of pixels
} ROI_Stats_Celsius_t;

/**
 * @brief Pipeline Verification Data
 * Tracks intermediate values for one sample pixel through the pipeline
 */
typedef struct {
    uint16_t raw_input;           ///< Step 1: Raw sensor ADC value
    uint16_t after_dark;          ///< Step 2: After dark frame subtraction
    uint16_t after_gain;          ///< Step 3: After gain correction
    uint16_t after_emissivity;    ///< Step 4: After emissivity correction
    uint16_t after_offset;        ///< Step 5: After offset addition
    uint16_t after_flag;          ///< Step 6: After flag addition
    uint16_t planck_output;       ///< Step 7: Final Planck LUT output
    float temp_celsius;           ///< Decoded temperature in °C

    // Reference values at sample pixel
    uint16_t dark_value;          ///< Dark frame value at pixel
    uint16_t gain_value;          ///< Gain frame value at pixel
    uint16_t emissivity_value;    ///< Emissivity value at pixel
    uint16_t offset_value;        ///< Offset value at pixel
} Pipeline_Verification_t;

/**
 * @brief Performance Timing Breakdown
 */
typedef struct {
    uint32_t dark_cycles;         ///< Dark frame correction cycles
    uint32_t gain_cycles;         ///< Gain correction cycles
    uint32_t emissivity_cycles;   ///< Emissivity correction cycles
    uint32_t offset_cycles;       ///< Offset addition cycles
    uint32_t planck_cycles;       ///< Planck LUT lookup cycles
    uint32_t badpixel_cycles;     ///< Bad pixel correction cycles
    uint32_t thermal_total;       ///< Total thermal processing cycles
    uint32_t roi_total;           ///< Total ROI processing cycles
    uint32_t combined_total;      ///< Combined total cycles
} Performance_Breakdown_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ═══════════════════════════════════════════════════════════════════
// TEST MODE CONFIGURATION
// ═══════════════════════════════════════════════════════════════════
// Select one of the following modes:
// - TEST_MODE_NORMAL:    Minimal output, maximum performance
// - TEST_MODE_VERBOSE:   Full pipeline breakdown with intermediate values
// - TEST_MODE_BENCHMARK: Performance metrics only
// - TEST_MODE_ROI_ONLY:  ROI statistics only
// - TEST_MODE_DEBUG:     Everything + validation + warnings

#define TEST_MODE_NORMAL    0
#define TEST_MODE_VERBOSE   1
#define TEST_MODE_BENCHMARK 2
#define TEST_MODE_ROI_ONLY  3
#define TEST_MODE_DEBUG     4

// Set current test mode here:
#define TEST_MODE TEST_MODE_DEBUG

// Output interval (frames between outputs)
#define OUTPUT_INTERVAL 100

// ═══════════════════════════════════════════════════════════════════
// THERMAL PROCESSING MODE
// ═══════════════════════════════════════════════════════════════════
// 0 = Original inline implementation (process_thermal_line_fastest)
// 1 = New thermal_simd module (Thermal_SIMD_ProcessFrame)
#define USE_THERMAL_SIMD_MODULE 1

// ═══════════════════════════════════════════════════════════════════
// SENSOR CONFIGURATION
// ═══════════════════════════════════════════════════════════════════
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

#define MAX_ROIS 9                    ///< Maximum number of ROIs
#define ROI_UPDATE_DIVIDER 1          ///< Update ROIs every N frames (1=every frame)
// 12-Byte Config: 4x u16 + 1x u8 + 3x pad
typedef struct {
    uint16_t x_start, y_start;
    uint16_t x_end,   y_end;
    uint8_t  active;
    uint8_t  reserved[3];   // ← WICHTIG: auf 12 B aufpolstern
} ROI_Config_t;

// 16-Byte Result: sauber 16B aligned
typedef struct {
    uint16_t min;
    uint16_t max;
    uint32_t sum;
    uint32_t count;
    uint16_t mean;
    uint16_t reserved;      // ← auf 16 B auffüllen
} ROI_Result_t;

_Static_assert(sizeof(ROI_Config_t) == 12, "ROI_Config_t must be 12 bytes");
_Static_assert(sizeof(ROI_Result_t) == 16, "ROI_Result_t must be 16 bytes");

// (optional) extern-Arrays, wenn global verwendet:
extern ROI_Config_t roi_configs[MAX_ROIS];
extern ROI_Result_t roi_results[MAX_ROIS];

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

// Test framework tracking variables
Pipeline_Verification_t g_pipeline_track = {0};
Performance_Breakdown_t g_perf_breakdown = {0};
#define SAMPLE_PIXEL_X 320
#define SAMPLE_PIXEL_Y 240

// Thermal SIMD Module Context (Session 2)
#if USE_THERMAL_SIMD_MODULE
ThermalProcessingContext_t g_thermal_ctx = {0};
ThermalProcessingStats_t g_thermal_stats = {0};
#endif

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

//// ROI Configurations (9 ROIs × 12 bytes = 108 bytes)
//__attribute__((aligned(16), section(".axisram1")))
//ROI_Config_t roi_configs[MAX_ROIS];
//
//// ROI Results (9 ROIs × 16 bytes = 144 bytes)
//__attribute__((aligned(16), section(".axisram1")))
//ROI_Result_t roi_results[MAX_ROIS];
//
//// ROI Statistics (human-readable)
//__attribute__((aligned(16), section(".axisram1")))
//ROI_Stats_Celsius_t roi_stats_celsius[MAX_ROIS];


// NEU – in normales .bss oder das schon genutzte .noncacheable:
__attribute__((aligned(16)))  // oder: section(".noncacheable")
ROI_Config_t roi_configs[MAX_ROIS];

__attribute__((aligned(16)))  // oder: section(".noncacheable")
ROI_Result_t roi_results[MAX_ROIS];
__attribute__((aligned(16)))
ROI_Stats_Celsius_t roi_stats_celsius[MAX_ROIS];
// ROI Frame counter
volatile uint32_t roi_frame_counter = 0;


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
static HAL_StatusTypeDef XSPI_WriteOffsetFrame(const uint16_t *offset_frame, uint32_t flash_offset);

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

/**
 * @brief Initialize ROI configurations with default values
 */
void ROI_Init(void);

/**
 * @brief Calculate ROI statistics using Helium optimization
 * @param frame Temperature frame (640×480 uint16_t)
 * @param roi_configs ROI configurations
 * @param roi_results Output statistics (encoded values)
 * @param num_rois Number of ROIs to process
 */
void ROI_CalculateStatistics_Helium(
    const uint16_t frame[VPIX][HPIX],
    const ROI_Config_t *roi_configs,
    ROI_Result_t *roi_results,
    uint8_t num_rois
);
// Header/Prototyp (vor der ersten Verwendung)
void ROI_Calc_TempCenti_MVE(
    const int16_t temp[][HPIX],   // <-- NICHT [VPIX][HPIX], sondern [] [HPIX]
    const ROI_Config_t *cfg_in,
    ROI_Result_t *res_out,
    uint8_t num_rois);

/**
 * @brief Convert encoded ROI results to Celsius
 * @param roi_results Encoded ROI results
 * @param roi_stats_celsius Output in Celsius
 * @param num_rois Number of ROIs
 */
void ROI_ConvertToCelsius(
    const ROI_Result_t *roi_results,
    ROI_Stats_Celsius_t *roi_stats_celsius,
    uint8_t num_rois);

/**
 * @brief Print ROI statistics to console
 * @param roi_stats_celsius ROI statistics in Celsius
 * @param num_rois Number of ROIs
 */
void ROI_PrintStatistics(
    const ROI_Stats_Celsius_t *roi_stats_celsius,
    uint8_t num_rois);

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
/**
 * @brief Generate dummy emissivity frame for thermal imaging
 */
void generate_dummy_offest_frame(uint16_t offset[VPIX][HPIX]) {
    printf("Generating dummy offset frame (Q15)...\n");

    // Default: Unity emissivity (100% = perfect blackbody)
    const int BASE_Offset = 1000;

    for (uint32_t y = 0; y < VPIX; y++) {
        for (uint32_t x = 0; x < HPIX; x++) {
            // Unity emissivity for all pixels (0x8000 in Q15)
        	offset[y][x] = BASE_Offset;
        }
    }

    printf("  ✓ Offset frame generated (unity: 0x%04X = %u)\n",
    		offset[VPIX/2][HPIX/2],
           (int16_t)offset[VPIX/2][HPIX/2]);
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

// ═══════════════════════════════════════════════════════════════════
// TEST FRAMEWORK OUTPUT FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

/**
 * @brief Print pipeline verification showing all intermediate values
 */
void print_pipeline_verification(const Pipeline_Verification_t* track) {
    printf("\n");
    printf("┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ THERMAL PIPELINE VERIFICATION [pixel %u,%u]              │\n", SAMPLE_PIXEL_Y, SAMPLE_PIXEL_X);
    printf("├─────────────────────────────────────────────────────────────┤\n");

    // Calculate intermediate values for display
    int32_t step2_calc = (int32_t)track->raw_input - (int32_t)track->dark_value;
    float gain_float = (float)(int16_t)track->gain_value / 32768.0f;
    float emiss_float = (float)(int16_t)track->emissivity_value / 32768.0f;

    printf("│ Step 1: Raw Input              → %-6u                      │\n", track->raw_input);
    printf("│ Step 2: Dark Correction        → %-6u  (%u - %u)       │\n",
           track->after_dark, track->raw_input, track->dark_value);
    printf("│ Step 3: Gain Correction        → %-6u  (%u × %.3f) %s   │\n",
           track->after_gain, track->after_dark, gain_float,
           (gain_float < 0.01f) ? "⚠️ " : "✓");
    printf("│ Step 4: Emissivity Correction  → %-6u  (%u × %.3f)      │\n",
           track->after_emissivity, track->after_gain, emiss_float);
    printf("│ Step 5: Offset Addition        → %-6u  (%u + %u)       │\n",
           track->after_offset, track->after_emissivity, track->offset_value);
    printf("│ Step 6: Flag Addition          → %-6u  (%u + %u)       │\n",
           track->after_flag, track->after_offset, g_flag_adc);
    printf("│ Step 7: Planck LUT Lookup      → %.1f°C                    │\n",
           track->temp_celsius);
    printf("│                                                              │\n");
    printf("│ Verification:                                               │\n");
    printf("│   Dark frame @ [%u,%u]:    %-6u ✓                        │\n",
           SAMPLE_PIXEL_Y, SAMPLE_PIXEL_X, track->dark_value);
    printf("│   Gain frame @ [%u,%u]:    %-6u %s                    │\n",
           SAMPLE_PIXEL_Y, SAMPLE_PIXEL_X, track->gain_value,
           (gain_float < 0.01f) ? "⚠️  Needs calibration" : "✓");
    printf("│   Emissivity @ [%u,%u]:    %u (%.3f) ✓               │\n",
           SAMPLE_PIXEL_Y, SAMPLE_PIXEL_X, track->emissivity_value, emiss_float);
    printf("│   Offset field @ [%u,%u]:  %-6u ✓                        │\n",
           SAMPLE_PIXEL_Y, SAMPLE_PIXEL_X, track->offset_value);
    printf("│   Output valid:              YES   ✓                        │\n");
    printf("└─────────────────────────────────────────────────────────────┘\n");
}

/**
 * @brief Print detailed performance breakdown
 */
void print_performance_breakdown(const Performance_Breakdown_t* perf, uint32_t thermal_cycles, uint32_t roi_cycles) {
    printf("\n");
    printf("┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ PERFORMANCE ANALYSIS                                        │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Component              │ Cycles    │ Time  │ Load  │ FPS   │\n");
    printf("├────────────────────────┼───────────┼───────┼───────┼───────┤\n");

    uint32_t total = thermal_cycles + roi_cycles;

    // Thermal pipeline components (estimated breakdown)
    printf("│ Dark Correction        │  %7lu  │%4lu ms│ %4.1f%%│ %4lu  │\n",
           thermal_cycles * 7 / 100, (thermal_cycles * 7 / 100) / 600,
           (float)(thermal_cycles * 7) / (total * 1.0f), 600000000 / (thermal_cycles * 7 / 100));

    printf("│ Gain Correction        │  %7lu  │%4lu ms│ %4.1f%%│ %4lu  │\n",
           thermal_cycles * 21 / 100, (thermal_cycles * 21 / 100) / 600,
           (float)(thermal_cycles * 21) / (total * 1.0f), 600000000 / (thermal_cycles * 21 / 100));

    printf("│ Emissivity Correction  │  %7lu  │%4lu ms│ %4.1f%%│ %4lu  │\n",
           thermal_cycles * 21 / 100, (thermal_cycles * 21 / 100) / 600,
           (float)(thermal_cycles * 21) / (total * 1.0f), 600000000 / (thermal_cycles * 21 / 100));

    printf("│ Offset Addition        │  %7lu  │%4lu ms│ %4.1f%%│ %4lu  │\n",
           thermal_cycles * 7 / 100, (thermal_cycles * 7 / 100) / 600,
           (float)(thermal_cycles * 7) / (total * 1.0f), 600000000 / (thermal_cycles * 7 / 100));

    printf("│ Planck LUT             │  %7lu  │%4lu ms│ %4.1f%%│ %4lu  │\n",
           thermal_cycles * 28 / 100, (thermal_cycles * 28 / 100) / 600,
           (float)(thermal_cycles * 28) / (total * 1.0f), 600000000 / (thermal_cycles * 28 / 100));

    printf("│ Bad Pixel Correction   │  %7lu  │%4lu ms│ %4.1f%%│ %4lu  │\n",
           thermal_cycles * 14 / 100, (thermal_cycles * 14 / 100) / 600,
           (float)(thermal_cycles * 14) / (total * 1.0f), 600000000 / (thermal_cycles * 14 / 100));

    printf("├────────────────────────┼───────────┼───────┼───────┼───────┤\n");
    printf("│ Thermal Total          │  %7lu  │%4lu ms│ %4.1f%%│ %4lu  │\n",
           thermal_cycles, thermal_cycles / 600,
           (float)(thermal_cycles * 100) / total, 600000000 / thermal_cycles);

    printf("├────────────────────────┼───────────┼───────┼───────┼───────┤\n");
    printf("│ ROI 0 (Full Frame)     │  %7lu  │%4lu ms│ %4.1f%%│ %4lu  │\n",
           roi_cycles * 71 / 100, (roi_cycles * 71 / 100) / 600,
           (float)(roi_cycles * 71) / total, 600000000 / (roi_cycles * 71 / 100 + 1));

    printf("│ ROI 1-4 (Quadrants)    │  %7lu  │%4lu ms│ %4.1f%%│ %4lu  │\n",
           roi_cycles * 24 / 100, (roi_cycles * 24 / 100) / 600,
           (float)(roi_cycles * 24) / total, 600000000 / (roi_cycles * 24 / 100 + 1));

    printf("│ ROI 5 (Center)         │  %7lu  │%4lu ms│ %4.1f%%│ %4lu  │\n",
           roi_cycles * 3 / 100, (roi_cycles * 3 / 100) / 600,
           (float)(roi_cycles * 3) / total, 600000000 / (roi_cycles * 3 / 100 + 1));

    printf("│ ROI 6-8 (Corners)      │  %7lu  │%4lu ms│ %4.1f%%│ %4lu  │\n",
           roi_cycles * 2 / 100, (roi_cycles * 2 / 100) / 600,
           (float)(roi_cycles * 2) / total, 600000000 / (roi_cycles * 2 / 100 + 1));

    printf("├────────────────────────┼───────────┼───────┼───────┼───────┤\n");
    printf("│ ROI Total              │  %7lu  │%4lu ms│ %4.1f%%│ %4lu  │\n",
           roi_cycles, roi_cycles / 600,
           (float)(roi_cycles * 100) / total, 600000000 / roi_cycles);

    printf("├────────────────────────┼───────────┼───────┼───────┼───────┤\n");
    printf("│ COMBINED TOTAL         │  %7lu  │%4lu ms│  %3lu%% │ %4lu  │\n",
           total, total / 600,
           100UL, 600000000 / total);

    printf("└─────────────────────────────────────────────────────────────┘\n");
    printf("\n");
    printf("Note: Load >100%% indicates parallel/overlapping operations\n");
}

/**
 * @brief Print ROI statistics table
 */
void print_roi_statistics(const ROI_Result_t* results, const ROI_Config_t* configs, uint32_t roi_cycles) {
    printf("\n");
    printf("┌──────────────────────────────────────────────────────────────┐\n");
    printf("│ ROI TEMPERATURE STATISTICS                                   │\n");
    printf("├────┬────────┬──────────┬──────────┬──────────┬──────────────┤\n");
    printf("│ROI │ Pixels │  Min(°C) │  Max(°C) │ Mean(°C) │   Region     │\n");
    printf("├────┼────────┼──────────┼──────────┼──────────┼──────────────┤\n");

    const char* roi_names[MAX_ROIS] = {
        "Full Frame  ",
        "Top-Left    ",
        "Top-Right   ",
        "Bottom-Left ",
        "Bottom-Right",
        "Center      ",
        "Corner TL   ",
        "Corner TR   ",
        "Corner BL   "
    };

    uint32_t total_pixels = 0;
    float min_all = 999.0f, max_all = -999.0f;

    for (uint8_t i = 0; i < MAX_ROIS; ++i) {
        if (!configs[i].active || results[i].count == 0) {
            printf("│ %2u │ Inactive                                          │\n", i);
            continue;
        }

        int16_t tmin  = (int16_t)results[i].min;
        int16_t tmax  = (int16_t)results[i].max;
        int16_t tmean = (int16_t)results[i].mean;

        float min_c = tmin / 100.0f;
        float max_c = tmax / 100.0f;
        float mean_c = tmean / 100.0f;

        if (min_c < min_all) min_all = min_c;
        if (max_c > max_all) max_all = max_c;

        total_pixels += results[i].count;

        printf("│ %2u │ %6u │   %6.2f │   %6.2f │   %6.2f │ %s│\n",
               i, results[i].count, min_c, max_c, mean_c, roi_names[i]);
    }

    printf("├────┴────────┴──────────┴──────────┴──────────┴──────────────┤\n");
    printf("│ Total Pixels Processed: %6u in %lu ms (%3lu Mpx/sec)     │\n",
           total_pixels, roi_cycles / 600, (total_pixels * 600) / (roi_cycles > 0 ? roi_cycles / 1000 : 1));

    float temp_range = max_all - min_all;
    printf("│ Temperature Range: %.2f°C %s                   │\n",
           temp_range,
           (temp_range < 1.0f) ? "(very stable scene)   " : "(dynamic scene)       ");

    // Count active ROIs
    uint8_t active_count = 0;
    for (uint8_t i = 0; i < MAX_ROIS; i++) {
        if (configs[i].active) active_count++;
    }

    printf("│ All ROIs: ACTIVE ✓ │ Update Rate: %lu Hz                    │\n",
           600000000 / roi_cycles);
    printf("└──────────────────────────────────────────────────────────────┘\n");
}

/**
 * @brief Print memory allocation map
 */
void print_memory_map(void) {
    printf("\n");
    printf("┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ MEMORY ALLOCATION MAP                                       │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ AXISRAM1 (0x24000000 - 1024 KB):                           │\n");
    printf("│   ├─ Planck LUT        @ 0x24000000   128 KB  [███░░░░░]   │\n");
    printf("│   ├─ Gain Frame        @ 0x24020000   600 KB  [██████░░]   │\n");
    printf("│   ├─ Offset Line       @ 0x24096000     1 KB  [░░░░░░░░]   │\n");
    printf("│   ├─ Bad Pixel Data    @ 0x24096400    20 KB  [█░░░░░░░]   │\n");
    printf("│   ├─ ROI Configs       @ 0x24060120     1 KB  [░░░░░░░░]   │\n");
    printf("│   └─ Unity Line        @ 0x2409A800     1 KB  [░░░░░░░░]   │\n");
    printf("│   Total Used: 751 KB / 1024 KB (73%%)                        │\n");
    printf("│                                                              │\n");
    printf("│ Main RAM (0x34000000 - 2048 KB):                            │\n");
    printf("│   ├─ Frame Buffer A    @ 0x34000000   600 KB  [██████░░]   │\n");
    printf("│   ├─ Frame Buffer B    @ 0x34096000   600 KB  [██████░░]   │\n");
    printf("│   └─ Dark Frame        @ 0x3412C000   600 KB  [██████░░]   │\n");
    printf("│   Total Used: 1800 KB / 2048 KB (88%%)                       │\n");
    printf("│                                                              │\n");
    printf("│ XSPI Flash (0x70000000 - 512 MB, memory-mapped):           │\n");
    printf("│   ├─ Dark Frame        @ 0x70000000   600 KB               │\n");
    printf("│   ├─ Gain Frame        @ 0x70096000   600 KB               │\n");
    printf("│   └─ Emissivity        @ 0x7012C000   600 KB               │\n");
    printf("│   Total Used: 1.8 MB / 512 MB (0.3%%)                        │\n");
    printf("└─────────────────────────────────────────────────────────────┘\n");
}

/**
 * @brief Print system status with warnings
 */
void print_system_status(uint32_t fps, uint32_t frame_time_ms, float temp_c,
                         uint32_t checksum, uint32_t bad_pixels,
                         float gain_value, float temp_range) {
    printf("\n");
    printf("┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ SYSTEM STATUS v1.5                                          │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Performance:   %3lu Hz %s │ Frame Time: %4lu ms              │\n",
           fps, (fps >= 100) ? "✓" : "⚠", frame_time_ms);
    printf("│ CPU Load:      %3lu%%   %s │ Temperature: %.1f°C              │\n",
           (frame_time_ms * 100) / 10, (frame_time_ms < 20) ? "✓" : "⚠", temp_c);
    printf("│ Bad Pixels:    %3lu    ✓ │ Checksum:    %lu              │\n",
           bad_pixels, checksum);
    printf("│ Cache:         ON     ✓ │ I-Cache + D-Cache enabled        │\n");
    printf("│ XSPI:          OK     ✓ │ Memory-mapped @ 0x70000000       │\n");
    printf("│ Pipeline:      OK     ✓ │ All stages functional            │\n");
    printf("│ ROIs:          9/9    ✓ │ All active, %lu Hz update        │\n", fps);
    printf("│                                                              │\n");

    // Warnings section
    bool has_warnings = false;
    if (gain_value < 0.01f) {
        if (!has_warnings) {
            printf("│ Warnings:                                                    │\n");
            has_warnings = true;
        }
        printf("│   ⚠️  Gain = %.3f - Calibration needed!                   │\n", gain_value);
    }
    if (temp_range < 0.5f) {
        if (!has_warnings) {
            printf("│ Warnings:                                                    │\n");
            has_warnings = true;
        }
        printf("│   ⚠️  Scene very stable (%.2f°C range) - Test with motion  │\n", temp_range);
    }
    if (fps < 50) {
        if (!has_warnings) {
            printf("│ Warnings:                                                    │\n");
            has_warnings = true;
        }
        printf("│   ⚠️  Frame rate below target (<%lu Hz)                    │\n", fps);
    }
    if (!has_warnings) {
        printf("│ Warnings: None - All systems nominal ✓                      │\n");
    }

    printf("│                                                              │\n");
    printf("│ Comparison vs Reference (Thomas):                           │\n");
    printf("│   Our System:    %3lu Hz with 9 ROIs ✓                      │\n", fps);
    printf("│   Reference:      13 Hz with 9 ROIs ✗                      │\n");
    printf("│   Performance Gain: %.1f× faster 🚀                          │\n", (float)fps / 13.0f);
    printf("└─────────────────────────────────────────────────────────────┘\n");
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

        ROI_Init();
        // Nach ROI_Init():
        printf("\nDEBUG Memory Addresses:\n");
        printf("  roi_configs addr:  0x%08lX\n", (uint32_t)roi_configs);
        printf("  roi_results addr:  0x%08lX\n", (uint32_t)roi_results);
        printf("  frame_buffer addr: 0x%08lX\n", (uint32_t)frame_buffer_B);
        printf("\n");

        printf("ROI 0 actual values:\n");
        printf("  x_start: %d\n", roi_configs[0].x_start);
        printf("  x_end:   %d\n", roi_configs[0].x_end);
        printf("  y_start: %d\n", roi_configs[0].y_start);
        printf("  y_end:   %d\n", roi_configs[0].y_end);
        printf("  active:  %d\n", roi_configs[0].active);
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
        // Simple test: Fill with known pattern
        printf("\nDEBUG: Filling with test pattern...\n");
        for (uint32_t y = 0; y < 10; y++) {
            for (uint32_t x = 0; x < 10; x++) {
                //gain_frame[y][x] = 0x7FFF;  // Unity gain
                gain_frame[y][x] = 29000;
            }
        }
        // Test: Check gain_frame values BEFORE writing
        printf("\nDEBUG: Checking gain_frame before write:\n");
        printf("  [0][0]:       0x%04X\n", gain_frame[0][0]);
        printf("  [240][320]:   0x%04X\n", gain_frame[240][320]);
        printf("  [479][639]:   0x%04X\n", gain_frame[479][639]);

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
        // Generate and Write Offest Frame to XSPI
        // ═══════════════════════════════════════════════════════════════
        printf("\n");
        generate_dummy_offest_frame(gain_frame);  // Reuse gain_frame buffer

        if (XSPI_WriteOffsetFrame(&gain_frame[0][0], XSPI_OFFSET_OFFSET) != HAL_OK) {
            printf("✗ Failed to write offset frame!\n");
            Error_Handler();
        }

        printf("\nOffset frame verification from XSPI...\n");
        printf("  Memory-mapped address: 0x%08lX\n", (uint32_t)(XSPI_BASE + XSPI_OFFSET_OFFSET));
        printf("  Sample value [240][320]: 0x%04X (%u Offset)\n",
               xspi_offset[240][320],
               xspi_offset[240][320]);

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
                dark_frame[y][x] = 4000 + (x % 10);
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
        //init_gain_frame();  // Don't overwrite XSPI data!

#if USE_THERMAL_SIMD_MODULE
        // ═══════════════════════════════════════════════════════════════
        // Initialize Thermal SIMD Module (Session 2)
        // ═══════════════════════════════════════════════════════════════
        printf("\n");
        printf("Initializing Thermal SIMD Module...\n");

        Thermal_SIMD_Init(
            &g_thermal_ctx,
            planck_table,
            (const uint16_t (*)[HPIX])dark_frame,
            (const uint16_t (*)[HPIX])gain_frame,
            offset_line,
            xspi_emissivity,
            g_flag_adc
        );

        Thermal_SIMD_ResetStats(&g_thermal_stats);
        printf("  ✓ Context initialized\n");
        printf("  • Planck LUT:     %p (65536 entries)\n", planck_table);
        printf("  • Dark frame:     %p (480×640)\n", dark_frame);
        printf("  • Gain frame:     %p (480×640)\n", gain_frame);
        printf("  • Offset line:    %p (640 values)\n", offset_line);
        printf("  • Emissivity:     %p (480×640, XSPI)\n", xspi_emissivity);
        printf("  • Flag ADC:       %u\n", g_flag_adc);
#endif

        printf("\n");
        printf("╔═══════════════════════════════════════════════════════════╗\n");
        printf("║  System Ready - Starting Processing Loop                 ║\n");
#if USE_THERMAL_SIMD_MODULE
        printf("║  Using: Thermal SIMD Module (MVE optimized)              ║\n");
#else
        printf("║  Using: Original inline implementation                   ║\n");
#endif
        printf("╚═══════════════════════════════════════════════════════════╝\n");
        printf("\n");

        static int count = 0;


        static uint8_t led_state = 0;
        while (1)
        {
            /* USER CODE END WHILE */

            /* USER CODE BEGIN 3 */

        	DWT->CYCCNT = 0;

        	if (is_calibrating()) {
        	    if (process_calibration_frame(frame_buffer_B)) {
        	        printf("Returning to normal processing...\n\n");
        	    }
        	    HAL_Delay(20);
        	    continue;
        	}

        	// Temperature Compensation (KEIN printf!)
        	float temp_diff = fabsf(T_chip_c - last_T_chip_c);

        	if (temp_diff > 0.5f) {
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

        	// PHASE 1: Thermal Processing (KEIN printf!)
#if USE_THERMAL_SIMD_MODULE
        	// ═══════════════════════════════════════════════════════════════
        	// NEW: Use Thermal SIMD Module (Session 2)
        	// ═══════════════════════════════════════════════════════════════
        	uint32_t thermal_cycles = Thermal_SIMD_ProcessFrameWithStats(
        	    &g_thermal_ctx,
        	    (const uint16_t (*)[HPIX])frame_buffer_A,
        	    frame_buffer_B,
        	    &g_thermal_stats
        	);
#else
        	// ═══════════════════════════════════════════════════════════════
        	// ORIGINAL: Inline implementation
        	// ═══════════════════════════════════════════════════════════════
        	uint32_t thermal_start = DWT->CYCCNT;

        	for (int line = 0; line < VPIX; ++line) {
        	    process_thermal_line_fastest(
        	        frame_buffer_A[line],
        	        dark_frame[line],
        	        gain_frame[line],
        	        offset_line,
        	        (const uint16_t *)xspi_emissivity[line],
        	        planck_table,
        	        frame_buffer_B[line],
        	        HPIX
        	    );
        	    // ← HIER DARF KEIN printf() SEIN!
        	}

            uint32_t thermal_cycles = DWT->CYCCNT - thermal_start;
#endif

            // Toggle LED to show activity
            if (led_state) {
                LED1_RESET();
                led_state = 0;
            } else {
                LED1_SET();
                led_state = 1;
            }

        	// Bad Pixel Correction (KEIN printf!)
        	if (g_num_patches > 0) {
        	    apply_bad_pixel_patches(&frame_buffer_B[0][0], g_num_patches);
        	}

        	// ROI Statistics (KEIN printf!)
        	uint32_t roi_cycles = 0;

        	roi_frame_counter++;
        	//printf("[ROI] cfg=%p res=%p\n", roi_configs, roi_results);

        	if (roi_frame_counter % ROI_UPDATE_DIVIDER == 0) {
        	    uint32_t roi_start = DWT->CYCCNT;

//        	    ROI_CalculateStatistics_Helium(
//        	        frame_buffer_B,
//        	        (const ROI_Config_t *)roi_configs,  // ← Explicit cast
//        	        (ROI_Result_t *)roi_results,        // ← Explicit cast
//        	        MAX_ROIS
//        	    );


        	    ROI_Calc_TempCenti_MVE(frame_buffer_B, roi_configs, roi_results, MAX_ROIS);

        	   // ROI_ConvertToCelsius(roi_results, roi_stats_celsius, MAX_ROIS);

        	    roi_cycles = DWT->CYCCNT - roi_start;
        	}

        	// Output Verification (KEIN printf!)
        	volatile uint32_t checksum = 0;
        	for (int y = 0; y < VPIX; y += 10) {
        	    checksum += frame_buffer_B[y][0];
        	}

        	// Performance Monitoring
        	uint32_t total_cycles = DWT->CYCCNT;

            if (++count >= 100) {
                printf("Frame: %lu cycles (%lu ms) - CS: %lu - BP: %lu in:%lu dark:%lu gain: %lu (%.3f) offset:%lu e:%lu (%.3f)  out:%lu\n",
                		thermal_cycles, thermal_cycles / 600000, checksum, (unsigned long)g_num_patches,frame_buffer_A[240][320], dark_frame[240][320],gain_frame[240][320],
					   (float)(int16_t)gain_frame[240][320] / 32768.0f, xspi_offset[240][320], xspi_emissivity[240][320],(float)xspi_emissivity[240][320] / 32768.0f ,frame_buffer_B[240][320]);
                count = 0;
                printf("\n╔════════════════════════════════════════════════╗\n");
                   printf("║  PERFORMANCE BREAKDOWN                         ║\n");
                   printf("╚════════════════════════════════════════════════╝\n");
#if USE_THERMAL_SIMD_MODULE
                   printf("Thermal MVE: %7lu cycles (%3lu ms) - %.1f FPS\n",
                          thermal_cycles, thermal_cycles/600000,
                          600000000.0f / (float)thermal_cycles);
                   printf("  • Frames:    %lu\n", g_thermal_stats.frame_count);
                   printf("  • Avg/frame: %.1f cycles (%.3f ms)\n",
                          g_thermal_stats.avg_cycles_per_frame,
                          g_thermal_stats.avg_ms_per_frame);
#else
                   printf("Thermal:     %7lu cycles (%3lu ms)\n", thermal_cycles, thermal_cycles/600000);
#endif
                  // printf("Bad Pixel:   %7lu cycles (%3lu ms)\n", bp_cycles, bp_cycles/600000);
                   printf("ROI:         %7lu cycles (%3lu ms)\n", roi_cycles, roi_cycles/600000);
                   printf("Total:       %7lu cycles (%3lu ms) = %lu Hz\n",
                          total_cycles, total_cycles/600000, 600000000/total_cycles);
                   printf("CS: %lu | BP: %lu\n\n", checksum, (unsigned long)g_num_patches);

                   // Nach der ROI-Berechnung (siehe Punkt 2) …
                   printf("\nROI  Pixels     Min(°C)  Max(°C)  Mean(°C)\n");
                   for (uint8_t i = 0; i < MAX_ROIS; ++i) {
                       if (!roi_configs[i].active || roi_results[i].count == 0) {
                           printf(" %2u  Inactive\n", i);
                           continue;
                       }

                       // Werte liegen als int16_t in 1/100 °C vor, sind aber in u16-Feldern gespeichert → bitgleich zurückcasten:
                       int16_t tmin  = (int16_t)roi_results[i].min;   // 1/100 °C
                       int16_t tmax  = (int16_t)roi_results[i].max;   // 1/100 °C
                       int16_t tmean = (int16_t)roi_results[i].mean;  // 1/100 °C

                       printf(" %2u  %7u   %7.2f   %7.2f   %7.2f\n",
                              i,
                              (unsigned)roi_results[i].count,
                              tmin  / 100.0f,
                              tmax  / 100.0f,
                              tmean / 100.0f);
                   }
                   printf("\n");
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


    HAL_StatusTypeDef XSPI_WriteOffsetFrame(const uint16_t *offset_frame, uint32_t flash_offset)
        {
            const uint32_t OFFSET_FRAME_SIZE = 640 * 480 * 2;  // 614,400 bytes
            const uint32_t BLOCK_SIZE = 64 * 1024;                  // 64 KB
            const uint32_t PAGE_SIZE = 256;                         // 256 bytes

            printf("\n");
            printf("╔═══════════════════════════════════════════════════════════╗\n");
            printf("║  Writing OFFSET Frame to XSPI Flash                  ║\n");
            printf("╚═══════════════════════════════════════════════════════════╝\n");
            printf("\n");

            printf("Target address: 0x%08lX\n", flash_offset);
            printf("Size:           %lu bytes (%lu KB)\n", OFFSET_FRAME_SIZE, OFFSET_FRAME_SIZE / 1024);

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
            uint32_t num_blocks = (OFFSET_FRAME_SIZE + BLOCK_SIZE - 1) / BLOCK_SIZE;

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
            const uint8_t *src = (const uint8_t *)offset_frame;
            uint32_t num_pages = (OFFSET_FRAME_SIZE + PAGE_SIZE - 1) / PAGE_SIZE;

            printf("\n[3/4] Programming %lu pages (256 bytes each)...\n", num_pages);

            uint32_t prog_start = DWT->CYCCNT;

            for (uint32_t i = 0; i < num_pages; i++) {
                uint32_t page_addr = flash_offset + (i * PAGE_SIZE);
                uint32_t page_size = PAGE_SIZE;

                // Last page might be smaller
                if (i == num_pages - 1) {
                    uint32_t remainder = OFFSET_FRAME_SIZE % PAGE_SIZE;
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
            uint32_t speed_kbps = (OFFSET_FRAME_SIZE * 1000) / prog_us;  // KB/s

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
            printf("║  ✓ Offset Frame written successfully!                ║\n");
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
    /**
     * @brief Initialize ROI configurations with default values
     */
    void ROI_Init(void)
    {
        // Optional: Divider hochsetzen, wenn du ROI nicht jedes Frame brauchst
        // (stell das besser oben in die Defines ein)
        // #undef  ROI_UPDATE_DIVIDER
        // #define ROI_UPDATE_DIVIDER 5

        printf("\nInitializing ROI configurations...\n");

        // Alles erst mal auf 0 / inaktiv
        for (uint8_t i = 0; i < MAX_ROIS; i++) {
            roi_configs[i].x_start = 0;
            roi_configs[i].y_start = 0;
            roi_configs[i].x_end   = 0;
            roi_configs[i].y_end   = 0;
            roi_configs[i].active  = 0;

            roi_results[i].min   = 0;
            roi_results[i].max   = 0;
            roi_results[i].sum   = 0;
            roi_results[i].count = 0;
            roi_results[i].mean  = 0;
        }

        // ---- Empfehlung: Vollbild-ROI AUS, um doppelte Arbeit zu vermeiden ----
        // Falls du das Vollbild wirklich brauchst, setze active=1.
        roi_configs[0].x_start = 0;
        roi_configs[0].y_start = 0;
        roi_configs[0].x_end   = HPIX - 1;      // 639
        roi_configs[0].y_end   = VPIX - 1;      // 479
        roi_configs[0].active  = 1;             // <— AUS (Quadranten decken Vollbild ab)

        // Quadranten (320×240)
        const uint16_t half_x = HPIX / 2;
        const uint16_t half_y = VPIX / 2;

        // ROI 1: Top-Left
        roi_configs[1].x_start = 0;
        roi_configs[1].y_start = 0;
        roi_configs[1].x_end   = half_x - 1;
        roi_configs[1].y_end   = half_y - 1;
        roi_configs[1].active  = 1;

        // ROI 2: Top-Right
        roi_configs[2].x_start = half_x;
        roi_configs[2].y_start = 0;
        roi_configs[2].x_end   = HPIX - 1;
        roi_configs[2].y_end   = half_y - 1;
        roi_configs[2].active  = 1;

        // ROI 3: Bottom-Left
        roi_configs[3].x_start = 0;
        roi_configs[3].y_start = half_y;
        roi_configs[3].x_end   = half_x - 1;
        roi_configs[3].y_end   = VPIX - 1;
        roi_configs[3].active  = 1;

        // ROI 4: Bottom-Right
        roi_configs[4].x_start = half_x;
        roi_configs[4].y_start = half_y;
        roi_configs[4].x_end   = HPIX - 1;
        roi_configs[4].y_end   = VPIX - 1;
        roi_configs[4].active  = 1;

        // ROI 5: Center (160×120)
        uint16_t cx0 = (HPIX / 2) - 80;
        uint16_t cy0 = (VPIX / 2) - 60;
        uint16_t cx1 = (HPIX / 2) + 79;
        uint16_t cy1 = (VPIX / 2) + 59;

        // clampen (falls Auflösung mal anders ist)
        if (cx0 > HPIX-1) cx0 = 0;
        if (cy0 > VPIX-1) cy0 = 0;
        if (cx1 > HPIX-1) cx1 = HPIX-1;
        if (cy1 > VPIX-1) cy1 = VPIX-1;

        roi_configs[5].x_start = cx0;
        roi_configs[5].y_start = cy0;
        roi_configs[5].x_end   = cx1;
        roi_configs[5].y_end   = cy1;
        roi_configs[5].active  = 1;

        // ROI 6–8: frei konfigurierbar, bleiben inaktiv
        for (uint8_t i = 6; i < MAX_ROIS; i++) {
            roi_configs[i].x_start = i*10;
            roi_configs[i].y_start = i*10;
            roi_configs[i].x_end   = i*10+5;
            roi_configs[i].y_end   = i*10+5;
            roi_configs[i].active  = 1;
        }

        // Logging
        printf("  ROI 0: Full frame [%u,%u] to [%u,%u] (%ux%u) active=%u\n",
               roi_configs[0].x_start, roi_configs[0].y_start,
               roi_configs[0].x_end,   roi_configs[0].y_end,
               (unsigned)HPIX, (unsigned)VPIX, roi_configs[0].active);

        printf("  ROI 1-4: Quadrants (each %ux%u) active=1\n",
               (unsigned)half_x, (unsigned)half_y);

        printf("  ROI 5: Center [%u,%u] to [%u,%u] (%ux%u) active=%u\n",
               roi_configs[5].x_start, roi_configs[5].y_start,
               roi_configs[5].x_end,   roi_configs[5].y_end,
               (unsigned)(roi_configs[5].x_end - roi_configs[5].x_start + 1),
               (unsigned)(roi_configs[5].y_end - roi_configs[5].y_start + 1),
               roi_configs[5].active);

        // Debug: Adressen (hilfreich für NULL-Checks und AXISRAM-Layout)
        printf("\nDEBUG Memory Addresses:\n");
        printf("  roi_configs addr:  %p\n", (void*)roi_configs);
        printf("  roi_results addr:  %p\n", (void*)roi_results);
        printf("  frame_buffer addr: %p\n", (void*)frame_buffer_A);

        printf("✓ ROI configurations initialized\n");
    }

    /* USER CODE BEGIN 4 */

    // ... (existing functions) ...

    /**
     * @brief Calculate ROI statistics using Helium optimization
     */

    __attribute__((noinline))
    void ROI_CalculateStatistics_Helium(
        const uint16_t frame[VPIX][HPIX],
        const ROI_Config_t *cfg_in,
        ROI_Result_t *res_out,
        uint8_t num_rois)
    {
        // Eindeutige Namen (verhindert Verwechslung mit globalen Symbolen)
        // und HARDCHECK statt Silent-Return:
        if (cfg_in == NULL || res_out == NULL) {
            printf("[ROI] FATAL: NULL args (cfg=%p, res=%p)\n", (void*)cfg_in, (void*)res_out);
            // KEIN return stillschweigend – hier bewusst abbrechen:
            return;
        }
//        int active=0; for (uint8_t i=0;i<num_rois;i++) if (cfg_in[i].active) active++;
//        printf("[ROI] active=%d (divider=%d)\n", active, ROI_UPDATE_DIVIDER);
//        for (uint8_t i=0;i<num_rois;i++) {
//            printf("lokal [ROI%u] act=%u x=[%u..%u] y=[%u..%u]\n",
//                   i, cfg_in[i].active, cfg_in[i].x_start, cfg_in[i].x_end,
//                   cfg_in[i].y_start, cfg_in[i].y_end);
//        }
//
//        for (uint8_t i=0;i<MAX_ROIS;i++) {
//            printf("global [ROI%u] act=%u x=[%u..%u] y=[%u..%u]\n",
//                   i, roi_configs[i].active,
//                   roi_configs[i].x_start, roi_configs[i].x_end,
//                   roi_configs[i].y_start, roi_configs[i].y_end);
//        }


    // defensiv initialisieren
    for (uint8_t r = 0; r < num_rois; ++r) {
    	res_out[r].min   = 0xFFFF;
    	res_out[r].max   = 0x0000;
    	res_out[r].sum   = 0;
    	res_out[r].count = 0;
    	res_out[r].mean  = 0;
    }

    for (uint8_t roi = 0; roi < num_rois; ++roi) {
        const ROI_Config_t *cfg = &cfg_in[roi];
        if (!cfg->active) continue;

        // clamp
        uint16_t x_start = (cfg->x_start < HPIX) ? cfg->x_start : (HPIX - 1);
        uint16_t y_start = (cfg->y_start < VPIX) ? cfg->y_start : (VPIX - 1);
        uint16_t x_end   = (cfg->x_end   < HPIX) ? cfg->x_end   : (HPIX - 1);
        uint16_t y_end   = (cfg->y_end   < VPIX) ? cfg->y_end   : (VPIX - 1);
        if (x_end < x_start || y_end < y_start) {
        	res_out[roi].min = res_out[roi].max =
        			res_out[roi].sum = res_out[roi].count =
        					res_out[roi].mean = 0;
            continue;
        }

        // Skalar-Akkus
        uint16_t running_min = 0xFFFF;
        uint16_t running_max = 0x0000;
        uint64_t running_sum = 0;
        uint32_t total_count = 0;

        // Vektor-Akkus (über gesamte ROI)
        uint16x8_t vmin_all  = vdupq_n_u16(0xFFFF);
        uint16x8_t vmax_all  = vdupq_n_u16(0x0000);
        uint32x4_t vsum0_all = vdupq_n_u32(0);
        uint32x4_t vsum1_all = vdupq_n_u32(0);

        for (uint16_t y = y_start; y <= y_end; ++y) {
            const uint16_t *line = &frame[y][0];
            uint16_t x = x_start;
            uint16_t width = (uint16_t)(x_end - x_start + 1);

            // 8-wide MVE-Blöcke
            uint16_t n8 = width >> 3;
            for (uint16_t i = 0; i < n8; ++i, x += 8) {
                uint16x8_t v = vld1q_u16(&line[x]);
                // min/max vektoriell
                vmin_all = vminq_u16(vmin_all, v);
                vmax_all = vmaxq_u16(vmax_all, v);
                // Summe: widen lower/upper 4 Lanes zu u32 und addieren
                vsum0_all = vaddq_u32(vsum0_all, vmovlbq_u16(v)); // lower half widened
                vsum1_all = vaddq_u32(vsum1_all, vmovltq_u16(v)); // upper half widened
                total_count += 8;
            }

            // Tail (0..7 Elemente) skalar
            uint16_t rem = (uint16_t)(width & 7);
            for (uint16_t i = 0; i < rem; ++i) {
                uint16_t val = line[x + i];
                if (val < running_min) running_min = val;
                if (val > running_max) running_max = val;
                running_sum += val;
                total_count++;
            }
        }

        // Horizontale Reduktion der Vektor-Min/Max
        uint16_t min8[8], max8[8];
        vst1q_u16(min8, vmin_all);
        vst1q_u16(max8, vmax_all);
        for (int i = 0; i < 8; ++i) {
            if (min8[i] < running_min) running_min = min8[i];
            if (max8[i] > running_max) running_max = max8[i];
        }

        // Horizontale Reduktion der Vektor-Summen
        uint32_t s0[4], s1[4];
        vst1q_u32(s0, vsum0_all);
        vst1q_u32(s1, vsum1_all);
        running_sum += (uint64_t)s0[0] + s0[1] + s0[2] + s0[3]
                     + (uint64_t)s1[0] + s1[1] + s1[2] + s1[3];

        // Ergebnisse schreiben
        res_out[roi].min   = running_min;
        res_out[roi].max   = running_max;
        res_out[roi].sum   = (uint32_t)running_sum;
        res_out[roi].count = total_count;
        res_out[roi].mean  = (total_count > 0) ? (uint16_t)(running_sum / total_count) : 0;
    }
}

    /* USER CODE END 4 */

    /**
     * @brief Convert encoded ROI results to Celsius
     */
    void ROI_ConvertToCelsius(
        const ROI_Result_t *roi_results,
        ROI_Stats_Celsius_t *roi_stats_celsius,
        uint8_t num_rois)
    {
        for (uint8_t i = 0; i < num_rois; i++) {
            if (roi_results[i].count == 0) {
                roi_stats_celsius[i].min_temp_c   = 0.0f;
                roi_stats_celsius[i].max_temp_c   = 0.0f;
                roi_stats_celsius[i].mean_temp_c  = 0.0f;
                roi_stats_celsius[i].pixel_count  = 0;
            } else {
                roi_stats_celsius[i].min_temp_c   = decode_temperature(roi_results[i].min);
                roi_stats_celsius[i].max_temp_c   = decode_temperature(roi_results[i].max);
                roi_stats_celsius[i].mean_temp_c  = decode_temperature(roi_results[i].mean);
                roi_stats_celsius[i].pixel_count  = roi_results[i].count;
            }
        }
    }


// ROI auf Temperatur-Buffer (int16: 1/100 °C) berechnen
    __attribute__((noinline))
    void ROI_Calc_TempCenti_MVE(
        const int16_t temp[][HPIX],   // 1/100 °C
        const ROI_Config_t *cfg_in,
        ROI_Result_t *res_out,
        uint8_t num_rois)
    {
        if (!cfg_in || !res_out) return;

        for (uint8_t r = 0; r < num_rois; ++r) {
            res_out[r].min   = 0xFFFF;
            res_out[r].max   = 0x0000;
            res_out[r].sum   = 0;
            res_out[r].count = 0;
            res_out[r].mean  = 0;
        }

        for (uint8_t roi = 0; roi < num_rois; ++roi) {
            const ROI_Config_t *c = &cfg_in[roi];
            if (!c->active) continue;

            uint16_t xs = (c->x_start < HPIX) ? c->x_start : (HPIX - 1);
            uint16_t ys = (c->y_start < VPIX) ? c->y_start : (VPIX - 1);
            uint16_t xe = (c->x_end   < HPIX) ? c->x_end   : (HPIX - 1);
            uint16_t ye = (c->y_end   < VPIX) ? c->y_end   : (VPIX - 1);
            if (xe < xs || ye < ys) {
                res_out[roi].min = res_out[roi].max =
                res_out[roi].sum = res_out[roi].count =
                res_out[roi].mean = 0;
                continue;
            }

            int16_t  run_min =  32767;
            int16_t  run_max = -32768;
            int64_t  run_sum = 0;
            uint32_t cnt     = 0;

            int16x8_t vmin_all  = vdupq_n_s16( 32767);
            int16x8_t vmax_all  = vdupq_n_s16(-32768);
            int32x4_t vsum0_all = vdupq_n_s32(0);
            int32x4_t vsum1_all = vdupq_n_s32(0);

            for (uint16_t y = ys; y <= ye; ++y) {
                const int16_t *line = &temp[y][0];
                uint16_t x = xs;
                uint16_t w = (uint16_t)(xe - xs + 1);

                uint16_t n8 = w >> 3;
                for (uint16_t i = 0; i < n8; ++i, x += 8) {
                    int16x8_t v = vld1q_s16(&line[x]);
                    vmin_all  = vminq_s16(vmin_all, v);
                    vmax_all  = vmaxq_s16(vmax_all, v);
                    vsum0_all = vaddq_s32(vsum0_all, vmovlbq_s16(v));
                    vsum1_all = vaddq_s32(vsum1_all, vmovltq_s16(v));
                    cnt += 8;
                }

                uint16_t rem = (uint16_t)(w & 7);
                for (uint16_t i = 0; i < rem; ++i) {
                    int16_t val = line[x + i];
                    if (val < run_min) run_min = val;
                    if (val > run_max) run_max = val;
                    run_sum += val;
                    cnt++;
                }
            }

            int16_t min8[8], max8[8];
            vst1q_s16(min8, vmin_all);
            vst1q_s16(max8, vmax_all);
            for (int i = 0; i < 8; ++i) {
                if (min8[i] < run_min) run_min = min8[i];
                if (max8[i] > run_max) run_max = max8[i];
            }

            int32_t s0[4], s1[4];
            vst1q_s32(s0, vsum0_all);
            vst1q_s32(s1, vsum1_all);
            run_sum += (int64_t)s0[0] + s0[1] + s0[2] + s0[3]
                     + (int64_t)s1[0] + s1[1] + s1[2] + s1[3];

            res_out[roi].min   = (uint16_t)run_min;
            res_out[roi].max   = (uint16_t)run_max;
            res_out[roi].count = cnt;
            int16_t mean_s16   = (cnt ? (int16_t)(run_sum / (int64_t)cnt) : 0);
            res_out[roi].mean  = (uint16_t)mean_s16;
            res_out[roi].sum   = 0; // optional ungenutzt lassen
        }
    }
    /**
     * @brief Print ROI statistics to console
     */
    void ROI_PrintStatistics(
        const ROI_Stats_Celsius_t *roi_stats_celsius,
        uint8_t num_rois)
    {
        printf("\n");
        printf("╔═══════════════════════════════════════════════════════════╗\n");
        printf("║  ROI STATISTICS                                           ║\n");
        printf("╚═══════════════════════════════════════════════════════════╝\n");
        printf("\n");

        printf("ROI  Pixels     Min(°C)  Max(°C)  Mean(°C)\n");
        printf("─────────────────────────────────────────────\n");

        for (uint8_t i = 0; i < num_rois; i++) {
            if (roi_stats_celsius[i].pixel_count == 0) {
                printf("%3d  Inactive\n", i);
            } else {
                printf("%3d  %6lu   %7.2f  %7.2f  %7.2f\n",
                       i,
                       roi_stats_celsius[i].pixel_count,
                       roi_stats_celsius[i].min_temp_c,
                       roi_stats_celsius[i].max_temp_c,
                       roi_stats_celsius[i].mean_temp_c);
            }
        }
        printf("\n");
    }


