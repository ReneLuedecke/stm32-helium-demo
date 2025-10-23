/* USER CODE BEGIN Header */
/** 
 * @file main.c
 * @brief Entry-Point und Initialisierung für STM32N6 NUCLEO-Projekt.
 * @details
 *  - Takt-/Cache-Init, GPIO/USART/XSPI Setup
 *  - Temperatur-LUT-Erzeugung (linear/Planck)
 *  - Zeilenweise Thermal-Pipeline (Dark, Gain, Offset, LUT, BPR) – Hotpath per MVE/Helium
 *  - Timing/Profiling via DWT, Frame-Loop via TIM-IRQ (VSYNC ~50 Hz)
 *
 * @note Diese Datei wurde automatisch mit Doxygen-Kommentaren angereichert.
 *       Bitte die Beschreibungen bei Bedarf konkretisieren.
 */

/**
 * @defgroup thermal_pipeline Thermal-Pipeline
 * @brief Dark/Gain/Offset/LUT/BPR – zeilenweise Verarbeitung mit Helium/MVE.
 * @{
 * @}
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

// Planck constants
#define PLANCK_C1 1.191042e8f
#define PLANCK_C2 1.4387752e4f

// Temperature format
#define TEMP_SCALE 100.0f
#define TEMP_OFFSET 10000.0f

// Sensor parameters (ADJUST!)
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
__attribute__((aligned(64), section(".axisram1")))
uint16_t planck_table[65536];

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

__attribute__((section(".noncacheable")))
uint16_t dark_frame[VPIX][HPIX];  // 614 KB ← Main RAM!
//__attribute__((aligned(16)))
//uint16_t dark_frame[VPIX][HPIX];  // 614 KB ← Main RAM!

//__attribute__((aligned(16)))
//uint16_t gain_frame[VPIX][HPIX];  // 614 KB
__attribute__((aligned(16), section(".axisram1")))
uint16_t gain_frame[VPIX][HPIX];  // 614 KB ✓ FASTEST ACCESS!
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

typedef enum {
    CALIB_IDLE,              // Not calibrating
    CALIB_WAIT_STABLE,       // Waiting for shutter to settle (skip 4 frames)
    CALIB_COLLECTING,        // Collecting samples (frames 5-16)
    CALIB_COMPLETE           // Calibration finished
} CalibState_t;

volatile CalibState_t calib_state = CALIB_IDLE;
volatile uint32_t calib_frame_count = 0;
volatile uint32_t calib_samples_collected = 0;

#define CALIB_SKIP_FRAMES 4      // Skip first 4 frames (settling time)
#define CALIB_TOTAL_SAMPLES 12   // Average 12 frames (frames 5-16)

/* USER CODE BEGIN PV */


// ==== Flag & LUT helpers (Prototypen) ====
static inline uint16_t encode_temp_to_lut(float t_c, float flag_ref_c);
static uint16_t inv_lut_find_adc_by_temp(const uint16_t *lut, float t_flag_c, float flag_ref_c);

// ==== Globale Flag-/Temp-Parameter ====
// Wird bei Dark-Cal gesetzt; initial ein sinnvoller Default:
volatile uint16_t g_flag_adc = 0;
volatile float    g_last_flag_temp_c = 30.0f; // °C, Beispielwert

// ==== Chip-Temp/Alpha Fallbacks ====
// TODO: Später aus echter Telemetrie befüllen!
static float T_chip_c  = 40.0f;   // aktuelle Chiptemp °C
static float T0_chip_c = 25.0f;   // Referenz °C
static float alpha     = 0.0025f; // Gain-Tempkoeffizient 1/°C

// ==== Unity Q15 Linie für Fallbacks ====
__attribute__((aligned(16), section(".axisram1")))
static uint16_t g_unity_q15_line[HPIX]; // wird mit 0x7FFF gefüllt

// Falls du Pixel-Maps aus XSPI hast, kannst du diese Pointer setzen.
// Bleiben sie NULL, verwenden wir Unity (1.0 in Q15).
static const uint16_t (*sensor_gain_q15)[HPIX] = NULL;    // optional extern liefern
static const uint16_t (*vignetting_q15)[HPIX]  = NULL;    // optional extern liefern



__attribute__((aligned(16), section(".axisram1")))
static uint16_t offset_line[HPIX]= {0};             // Arbeitsbuffer

static float last_T_chip_c = 0.0f;  // Track temperature changes


//BadPixel
#define MAX_BAD_PIXELS ((N * 15) / 1000)  // 1.5% = 4608 pixels

typedef struct {
    uint16_t x;          // Bad pixel X coordinate in this line
    uint16_t rep_x;      // Replacement pixel X coordinate
    int8_t   rep_dy;     // Replacement line offset (-1, 0, +1)
} BadPixelInLine_t;  // 5 bytes

typedef struct {
    uint16_t count;      // Number of bad pixels in this line
    uint16_t offset;     // Offset into bad_pixels_sorted array
} LineInfo_t;  // 4 bytes

__attribute__((aligned(16), section(".axisram1")))
LineInfo_t bad_pixel_line_info[VPIX];  // 480 × 4 = 1,920 bytes

__attribute__((aligned(16), section(".axisram1")))
BadPixelInLine_t bad_pixels_sorted[MAX_BAD_PIXELS];  // 4608 × 5 = 23 KB


typedef struct {
    uint32_t dst;  // y*HPIX + x
    uint32_t src;  // (y+dy)*HPIX + rx
} PixPatch;

__attribute__((aligned(64), section(".axisram1")))
static PixPatch g_patches[MAX_BAD_PIXELS];

__attribute__((aligned(16), section(".axisram1")))
static uint16_t g_patch_values[MAX_BAD_PIXELS];   // 1. Phase: Quellen puffern

static uint32_t g_num_patches = 0;


uint16_t total_bad_pixels = 0;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t rep_x;
    uint16_t rep_y;
} BadPixelFactory_t;
//test data
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
	{650, 400, 651, 400},
	{120, 220, 120, 221},
	{220, 320, 221, 320},
	{320, 420, 320, 419},
	{420, 120, 419, 120},
	{520, 220, 520, 221},
	{620, 320, 621, 320},
	{130, 230, 130, 229},
	{230, 330, 231, 330},
    // ... up to 4608 entries (1.5%)
};

const uint16_t factory_bad_pixel_count =
    sizeof(factory_bad_pixels) / sizeof(factory_bad_pixels[0]);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_XSPI2_Init(void);
void generate_planck_lut(uint16_t *planck_table, float wavelength_um,
                         float radiance_scale, float flag_temp_c);
float decode_temperature(uint16_t value);
void init_planck_lut(void);


/* USER CODE BEGIN PFP */

static inline uint16_t gain_to_q15(float gain);
void process_mve(const uint16_t *src, const uint16_t *gain, uint16_t *dst, int n, int16_t offset);
void planck_lut_mve(const uint16_t *src, uint16_t *dst, int n);
//static void XSPI_WriteEnable(XSPI_HandleTypeDef *hxspi);
//static void XSPI_AutoPollingMemReady(XSPI_HandleTypeDef *hxspi);
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

//    // Toggle LED to show activity
//    static uint8_t led_state = 0;
//    if (led_state) {
//        LED1_RESET();
//        led_state = 0;
//    } else {
//        LED1_SET();
//        led_state = 1;
//    }
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
// ═══════════════════════════════════════════════════════════════════
// PLANCK LUT GENERATION - NO FLOAT PRINTF!
// ═══════════════════════════════════════════════════════════════════

/**
 * @brief Erzeugt Planck-basierte LUT (nicht aktiv in Demo).
 * @param planck_table [uint16_t*] LUT-Zeiger
 * @param emissivity [float] Emissionsgrad
 * @param temp_min [float] Minimaltemperatur [°C]
 * @param temp_max [float] Maximaltemperatur [°C]
 * @return void
 */
void generate_planck_lut(
    uint16_t *planck_table,
    float wavelength_um,
    float radiance_scale,
    float flag_temp_c)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════╗\n");
    printf("║  Generating Planck LUT (0.01°C resolution)   ║\n");
    printf("╚═══════════════════════════════════════════════╝\n");
    printf("\n");

    printf("Parameters:\n");

    // Convert float to int for printing
    int wavelength_int = (int)wavelength_um;
    printf("  • Wavelength:     %d μm\n", wavelength_int);

    int flag_temp_int = (int)flag_temp_c;
    printf("  • Flag temp:      %d°C\n", flag_temp_int);

    // Radiance scale as fixed-point (×1000000)
    int radiance_int = (int)(radiance_scale * 1000000.0f);
    printf("  • Radiance scale: %d (×1e-6)\n", radiance_int);

    printf("  • Resolution:     0.01°C\n");
    printf("  • Output format:  (T × 100) + 10000\n");
    printf("\n");

    const float lambda = wavelength_um;
    const float lambda5 = powf(lambda, 5.0f);
    const float lambda_c2 = lambda * PLANCK_C2;

    uint32_t valid_count = 0;
    uint32_t saturated_count = 0;
    uint32_t zero_count = 0;

    printf("Generating 65536 entries...\n");

    for (uint32_t adc = 0; adc < 65536; adc++) {
        float radiance = (float)adc * radiance_scale;

        if (radiance <= 0.0f) {
            planck_table[adc] = 0;
            zero_count++;
            continue;
        }

        float ratio = PLANCK_C1 / (radiance * lambda5);

        if (ratio < 1e-6f) {
            planck_table[adc] = 65535;
            saturated_count++;
            continue;
        }

        float temp_kelvin = lambda_c2 / logf(ratio + 1.0f);
        float temp_celsius = temp_kelvin - 273.15f;
        float temp_relative = temp_celsius - flag_temp_c;
        int32_t temp_output = (int32_t)((temp_relative * TEMP_SCALE) + TEMP_OFFSET);

        if (temp_output < 0) {
            planck_table[adc] = 0;
            zero_count++;
        } else if (temp_output > 65535) {
            planck_table[adc] = 65535;
            saturated_count++;
        } else {
            planck_table[adc] = (uint16_t)temp_output;
            valid_count++;
        }
    }

    printf("✓ LUT generation complete!\n\n");

    printf("Statistics:\n");

    // Calculate percentage as integer (×10 for one decimal)
    uint32_t valid_pct = (valid_count * 1000) / 65536;  // ×10 for 1 decimal
    uint32_t zero_pct = (zero_count * 1000) / 65536;
    uint32_t sat_pct = (saturated_count * 1000) / 65536;

    printf("  • Valid entries:     %5lu (%lu.%lu%%)\n",
           valid_count, valid_pct / 10, valid_pct % 10);
    printf("  • Zero entries:      %5lu (%lu.%lu%%)\n",
           zero_count, zero_pct / 10, zero_pct % 10);
    printf("  • Saturated entries: %5lu (%lu.%lu%%)\n",
           saturated_count, sat_pct / 10, sat_pct % 10);
    printf("\n");

    printf("Sample LUT Values:\n");
    printf("  ADC      -> Output   -> Temperature\n");
    printf("  ────────────────────────────────────\n");

    uint32_t samples[] = {100, 1000, 5000, 10000, 20000, 30000, 40000, 50000};
    for (int i = 0; i < 8; i++) {
        uint32_t adc = samples[i];
        if (adc < 65536) {
            uint16_t output = planck_table[adc];

            // Decode temperature as integer (centidegrees)
            int32_t temp_centi = (int32_t)output - 10000;  // Can be negative!
            int32_t temp_abs = (temp_centi < 0) ? -temp_centi : temp_centi;
            char sign = (temp_centi < 0) ? '-' : '+';

            // Split into integer and fractional parts
            uint32_t temp_int = temp_abs / 100;
            uint32_t temp_frac = temp_abs % 100;

            printf("  %-8lu -> %-7u -> %c%lu.%02lu°C\n",
                   adc, output, sign, temp_int, temp_frac);
        }
    }

    printf("\n");
    printf("╔═══════════════════════════════════════════════╗\n");
    printf("║  Planck LUT Ready in DTCM!                    ║\n");
    printf("╚═══════════════════════════════════════════════╝\n");
    printf("\n");
}

/**
 * @brief Decode temperature - Integer version (no float printf needed!)
 * @param value: LUT output value
 * @param temp_int: Output integer part (degrees)
 * @param temp_frac: Output fractional part (0-99 centidegrees)
 * @param is_negative: Output sign flag
 */
void decode_temperature_int(uint16_t value,
                           uint32_t *temp_int,
                           uint32_t *temp_frac,
                           uint8_t *is_negative)
{
    int32_t temp_centi = (int32_t)value - 10000;

    *is_negative = (temp_centi < 0) ? 1 : 0;

    int32_t temp_abs = (*is_negative) ? -temp_centi : temp_centi;

    *temp_int = temp_abs / 100;
    *temp_frac = temp_abs % 100;
}

float decode_temperature(uint16_t value) {
    return ((float)value - TEMP_OFFSET) / TEMP_SCALE;
}

float calculate_radiance_scale(
    uint16_t adc_min,
    uint16_t adc_max,
    float temp_min,
    float temp_max)
{
    const float lambda = SENSOR_WAVELENGTH;
    const float lambda5 = powf(lambda, 5.0f);

    // Convert temperatures to Kelvin
    float T_min = temp_min + 273.15f;
    float T_max = temp_max + 273.15f;

    // Calculate expected radiance using Planck's law
    // R = C1 / (λ⁵ × (exp(C2/(λ×T)) - 1))
    float exp_min = expf(PLANCK_C2 / (lambda * T_min));
    float exp_max = expf(PLANCK_C2 / (lambda * T_max));

    float R_min = PLANCK_C1 / (lambda5 * (exp_min - 1.0f));
    float R_max = PLANCK_C1 / (lambda5 * (exp_max - 1.0f));

    // Calculate scale factor
    // ADC = Radiance / scale
    // → scale = Radiance / ADC
    float scale = (R_max - R_min) / (float)(adc_max - adc_min);

    return scale;
}


/* USER CODE BEGIN 0 */

// ... (existing Planck code) ...

// ═══════════════════════════════════════════════════════════════════
// LINEAR TEMPERATURE LUT (SIMPLIFIED - WORKING!)
// ═══════════════════════════════════════════════════════════════════
/**
 * @brief Generiert eine lineare Temperatur-LUT.
 * @param planck_table [uint16_t*] Zeiger auf 65536-Eintrag LUT
 * @param temp_min_c [float] Minimaltemperatur [°C]
 * @param temp_max_c [float] Maximaltemperatur [°C]
 * @param flag_temp_c [float] Referenztemperatur [°C]
 * @return void
 */
void generate_linear_temp_lut(
    uint16_t *planck_table,
    float temp_min_c,
    float temp_max_c,
    float flag_temp_c)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════╗\n");
    printf("║  Generating LINEAR Temperature LUT            ║\n");
    printf("╚═══════════════════════════════════════════════╝\n");
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
    printf("╔═══════════════════════════════════════════════╗\n");
    printf("║  Linear LUT Ready in DTCM!                   ║\n");
    printf("╚═══════════════════════════════════════════════╝\n");
    printf("\n");
}


// Usage in init:
void init_planck_lut(void) {
/*    // Define expected scene temperature range
    const uint16_t ADC_AT_MIN_TEMP = 324;   // ADC value at min temp
    const uint16_t ADC_AT_MAX_TEMP = 4754;  // ADC value at max temp
    const float MIN_SCENE_TEMP = 40.0f;       // Minimum scene temp (°C)
    const float MAX_SCENE_TEMP = 100.0f;     // Maximum scene temp (°C)

    // Calculate calibrated scale
    float radiance_scale = calculate_radiance_scale(
        ADC_AT_MIN_TEMP,
        ADC_AT_MAX_TEMP,
        MIN_SCENE_TEMP,
        MAX_SCENE_TEMP
    );

    printf("Calculated radiance scale: ");
    int scale_exp = (int)log10f(radiance_scale);
    int scale_mant = (int)(radiance_scale / powf(10.0f, (float)scale_exp) * 1000.0f);
    printf("%d.%03d e%d\n", scale_mant / 1000, scale_mant % 1000, scale_exp);

    // Generate LUT with calibrated scale
    generate_planck_lut(
        planck_table,
        SENSOR_WAVELENGTH,
        radiance_scale,
        FLAG_TEMP_CELSIUS
    );*/
    generate_linear_temp_lut(
        planck_table,
        -20.0f,     // Min expected scene temp (°C)
        150.0f,     // Max expected scene temp (°C)
        FLAG_TEMP_CELSIUS
    );
}


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
/**
 * @brief Verarbeitet eine Zeile der Thermal-Daten (Helium-optimiert).
 * @param sensor_data [uint16_t*] ADC-Rohdaten
 * @param dark [uint16_t*] Dark-Frame
 * @param gain [uint16_t*] Gain-Q15
 * @param offset [uint16_t*] Offset-Werte
 * @param planck_lut [uint16_t*] Temperatur-LUT
 * @param output [uint16_t*] Ausgabepuffer
 * @param width [uint32_t] Pixelbreite
 * @return void
 * @note Nutzt vqrdmulhq_s16 + Gather-Load; 16-Byte alignment empfohlen.
 */
static inline void process_thermal_line_fastest(
    const uint16_t * __restrict__ sensor_data,
    const uint16_t * __restrict__ dark,
    const uint16_t * __restrict__ gain,     // Q15 pro Pixel
    const uint16_t * __restrict__ offset,   // pixelbezogener Offset (ADC-Domain)
    const uint16_t * __restrict__ planck_lut,
    uint16_t * __restrict__ output,
    uint32_t width)
{
    const uint16x8_t flag_vec = vdupq_n_u16(g_flag_adc);

    for (uint32_t x = 0; x < width; x += 8)
    {
        // Load
        uint16x8_t adc   = vld1q_u16(&sensor_data[x]);
        uint16x8_t darkv = vld1q_u16(&dark[x]);
        uint16x8_t gainv = vld1q_u16(&gain[x]);
        uint16x8_t offv  = vld1q_u16(&offset[x]);

        // Dark subtract
        uint16x8_t corr  = vqsubq_u16(adc, darkv);

        // Gain Q15  (corr * gain >> 15)  — wie bisher
        int16x8_t  corr_s = vreinterpretq_s16_u16(corr);
        int16x8_t  gain_s = vreinterpretq_s16_u16(gainv);
        int16x8_t  mul_s  = vqrdmulhq_s16(corr_s, gain_s);
        uint16x8_t val    = vreinterpretq_u16_s16(mul_s);

        // + InvPlanck(T_flag) (Skalar)  + pixelbezogener Offset
        val = vqaddq_u16(val, flag_vec);
        val = vqaddq_u16(val, offv);

        // LUT
        uint16x8_t out = vldrhq_gather_shifted_offset_u16(planck_lut, val);

        // Store
        vst1q_u16(&output[x], out);
    }
}
/**
 * @brief Startet eine neue Dunkelbild-Kalibrierung.
 * @return void
 */
void start_dark_frame_calibration(void) {
    printf("\n");
    printf("Parameters:\n");
    printf("  • Skip frames:    %d (settling time)\n", CALIB_SKIP_FRAMES);
    printf("  • Sample frames:  %d\n", CALIB_TOTAL_SAMPLES);
    printf("  • Total time:     ~%d ms\n",
           (CALIB_SKIP_FRAMES + CALIB_TOTAL_SAMPLES) * 20);
    printf("\n");

    // Initialize dark frame to zero (will be built up incrementally)
    printf("Initializing dark frame buffer...\n");
    for (uint32_t y = 0; y < VPIX; y++) {
        for (uint32_t x = 0; x < HPIX; x++) {
            frame_buffer_A[y][x] = 0;  // Use frame_buffer_A as temp storage
        }
    }

    // Reset state
    calib_state = CALIB_WAIT_STABLE;
    calib_frame_count = 0;
    calib_samples_collected = 0;

    printf("✓ Ready to capture\n");
    printf("\nWaiting for shutter to settle...\n");
}

/**
 * @brief Process one frame during calibration (call from main loop or ISR)
 * @param sensor_frame: Pointer to current sensor frame
 * @return 1 if calibration complete, 0 if still running
 */
uint8_t process_calibration_frame(const uint16_t sensor_frame[VPIX][HPIX]) {
    if (calib_state == CALIB_IDLE || calib_state == CALIB_COMPLETE) {
        return 1;  // Not calibrating or already done
    }

    calib_frame_count++;

    // State: Waiting for shutter to settle
    if (calib_state == CALIB_WAIT_STABLE) {
        if (calib_frame_count >= CALIB_SKIP_FRAMES) {
            calib_state = CALIB_COLLECTING;
            printf("Shutter stable. Collecting samples...\n");
        }
        return 0;  // Still running
    }

    // State: Collecting samples
    if (calib_state == CALIB_COLLECTING) {
        calib_samples_collected++;

        printf("  Sample %lu/%d...\n",
               calib_samples_collected, CALIB_TOTAL_SAMPLES);

        // Incremental averaging (no accumulator needed!)
        // avg_new = avg_old + (sample - avg_old) / n

        for (uint32_t y = 0; y < VPIX; y++) {
            for (uint32_t x = 0; x < HPIX; x++) {
                uint16_t current_avg = frame_buffer_A[y][x];
                uint16_t new_sample = sensor_frame[y][x];

                // Incremental average
                int32_t diff = (int32_t)new_sample - (int32_t)current_avg;
                int32_t delta = diff / (int32_t)calib_samples_collected;
                int32_t new_avg = (int32_t)current_avg + delta;

                // Clamp to valid range
                if (new_avg < 0) new_avg = 0;
                if (new_avg > 65535) new_avg = 65535;

                frame_buffer_A[y][x] = (uint16_t)new_avg;
            }
        }

        // Check if done
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
            printf("✓ Dark frame stored in AXISRAM1\n");

            printf("Dark frame ready for use.\n");
            printf("\n");
            // Nach Abschluss der Kalibrierung:
            unsigned enc = (unsigned)((g_last_flag_temp_c - FLAG_TEMP_CELSIUS) * TEMP_SCALE + TEMP_OFFSET);
            printf("✓ Flag ADC set: %u (T_flag_enc=%u)\n", g_flag_adc, enc);

            return 1;  // Complete!
        }

        return 0;  // Still collecting
    }

    return 1;  // Should not reach here
}

/**
 * @brief Prüft, ob eine Kalibrierung aktiv ist.
 * @retval true wenn Kalibrierung läuft, sonst false
 */
uint8_t is_calibrating(void) {
    return (calib_state != CALIB_IDLE && calib_state != CALIB_COMPLETE);
}
/* USER CODE END 4 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

// Kodierung wie in generate_*_lut: (T_rel*100)+10000
static inline uint16_t encode_temp_to_lut(float t_c, float flag_ref_c)
{
    // gleiche Kodierung wie in deiner LUT-Generierung
    int32_t out = (int32_t)lrintf((t_c - flag_ref_c) * TEMP_SCALE) + (int32_t)TEMP_OFFSET;
    if (out < 0)      out = 0;
    if (out > 65535)  out = 65535;
    return (uint16_t)out;
}

static uint16_t inv_lut_find_adc_by_temp(const uint16_t *lut, float t_flag_c, float flag_ref_c)
{
    const uint16_t target = encode_temp_to_lut(t_flag_c, flag_ref_c);
    uint32_t lo = 0, hi = 65535;
    while (lo < hi) {
        uint32_t mid = (lo + hi) >> 1;
        uint16_t v = lut[mid];
        if (v < target) lo = mid + 1; else hi = mid;
    }
    return (uint16_t)lo;
}


static inline void mve_copy_u16(uint16_t *dst, const uint16_t *src, int n) {
    int i = 0;
    for (; i <= n - 16; i += 16) {
        uint16x8_t a = vld1q_u16(src + i + 0);
        uint16x8_t b = vld1q_u16(src + i + 8);
        vst1q_u16(dst + i + 0, a);
        vst1q_u16(dst + i + 8, b);
    }
    // Handle remaining
    for (; i < n; i++) {
        dst[i] = src[i];
    }
}


// ===== Q15 Helpers =====
// Q15: signed 1.15 (−1.0 .. +0.99997). Deine Gains sind >=0, also typ. 0..+0.999.
// Speicherung als uint16_t, Re-Interpretation als int16_t für MVE.

static inline uint16_t q15_from_float(float x) {
    // clamp + round to nearest
    if (x < -0.999969f) x = -0.999969f;
    if (x >  0.999969f) x =  0.999969f;
    int32_t v = (int32_t)lrintf(x * 32768.0f);  // 1<<15
    if (v < -32768) v = -32768;
    if (v >  32767) v =  32767;
    return (uint16_t)(int16_t)v;
}

static inline uint16_t q15_mul_scalar_u16(uint16_t a_q15, uint16_t b_q15) {
    // (a*b + 0.5 ulp) >> 15  with saturation to Q15 range
    int16_t a = (int16_t)a_q15, b = (int16_t)b_q15;
    int32_t p = (int32_t)a * (int32_t)b;       // Q30
    p = (p + (1<<14)) >> 15;                   // round to Q15
    if (p < -32768) p = -32768;
    if (p >  32767) p =  32767;
    return (uint16_t)(int16_t)p;
}

// ===== 1) Q15 line-wise multiply: dst = a ⊗ b (Q15) =====
static inline void q15_mul_u16_line(uint16_t * __restrict__ dst,
                                    const uint16_t * __restrict__ a,
                                    const uint16_t * __restrict__ b,
                                    uint32_t n)
{
    uint32_t i = 0;
    for (; i + 8 <= n; i += 8) {
        int16x8_t va = vreinterpretq_s16_u16(vld1q_u16(&a[i]));
        int16x8_t vb = vreinterpretq_s16_u16(vld1q_u16(&b[i]));
        // Q15*Q15 -> Q15 (rounded high mul with saturation)
        int16x8_t vm = vqrdmulhq_s16(va, vb);
        vst1q_u16(&dst[i], vreinterpretq_u16_s16(vm));
    }
    for (; i < n; ++i) {
        dst[i] = q15_mul_scalar_u16(a[i], b[i]);
    }
}
// ===== 2) build_gain_line: dst = sensor_gain ⊗ vignetting ⊗ temp_gain  =====
// temp_gain_q15 = q15_from_float( 1.0f / (1.0f + alpha * (T_chip - T0_chip)) * cos4_global_scale )
// Hinweis: vignetting ist bereits cos^4(theta) in Q15.
//          sensor_gain ist deine per-Pixel Gain-Map in Q15.
//          temp_gain ist ein *Skalar* in Q15 (pro Frame).

static inline void build_gain_line(uint16_t * __restrict__ dst,
                                   const uint16_t * __restrict__ sensor_gain_q15,
                                   const uint16_t * __restrict__ vignetting_q15,
                                   uint16_t temp_gain_q15,
                                   uint32_t n)
{
    int16x8_t vtemp = vdupq_n_s16((int16_t)temp_gain_q15);
    uint32_t i = 0;
    for (; i + 8 <= n; i += 8) {
        // load
        int16x8_t vsg = vreinterpretq_s16_u16(vld1q_u16(&sensor_gain_q15[i]));
        int16x8_t vvg = vreinterpretq_s16_u16(vld1q_u16(&vignetting_q15[i]));
        // mul1 = sg ⊗ vign
        int16x8_t vm1 = vqrdmulhq_s16(vsg, vvg);
        // mul2 = mul1 ⊗ temp_gain
        int16x8_t vm2 = vqrdmulhq_s16(vm1, vtemp);
        // store
        vst1q_u16(&dst[i], vreinterpretq_u16_s16(vm2));
    }
    for (; i < n; ++i) {
        uint16_t m1 = q15_mul_scalar_u16(sensor_gain_q15[i], vignetting_q15[i]);
        dst[i] = q15_mul_scalar_u16(m1, temp_gain_q15);
    }
}

void init_gain_frame(void) {
    printf("Initializing gain frame...\n");

    // Unity gain for all pixels (for now)
    for (uint32_t y = 0; y < VPIX; y++) {
        for (uint32_t x = 0; x < HPIX; x++) {
            gain_frame[y][x] = 0x7FFF;  // Unity gain (Q15)
        }
    }

    last_T_chip_c = T_chip_c;
    printf("✓ Gain frame initialized\n");
}

static inline void correct_bad_pixels_line_fast(
    uint16_t * __restrict__ output_line,
    const uint16_t * __restrict__ prev_line,
    const uint16_t * __restrict__ next_line,
    uint16_t y)
{
    const uint16_t count = bad_pixel_line_info[y].count;
    if (count == 0) return;  // No bad pixels in this line!

    const uint16_t offset = bad_pixel_line_info[y].offset;
    const BadPixelInLine_t *pixels = &bad_pixels_sorted[offset];

    // Process all bad pixels in this line
    for (uint16_t i = 0; i < count; i++) {
        const uint16_t x = pixels[i].x;
        const uint16_t rx = pixels[i].rep_x;
        const int8_t dy = pixels[i].rep_dy;

        // Select source based on line offset
        if (dy == 0) {
            // Same line (most common case!)
            output_line[x] = output_line[rx];
        }
        else if (dy == -1 && prev_line) {
            // Previous line
            output_line[x] = prev_line[rx];
        }
        else if (dy == 1 && next_line) {
            // Next line
            output_line[x] = next_line[rx];
        }
        // else: Edge case, skip
    }
}

void load_bad_pixel_map(void) {
    printf("Loading bad pixel map...\n");

    // ═══════════════════════════════════════════════════════════════
    // 1. Initialize line info
    // ═══════════════════════════════════════════════════════════════
    for (uint16_t y = 0; y < VPIX; y++) {
        bad_pixel_line_info[y].count = 0;
        bad_pixel_line_info[y].offset = 0;
    }

    // ═══════════════════════════════════════════════════════════════
    // 2. Load from factory calibration
    // ═══════════════════════════════════════════════════════════════
    extern const BadPixelFactory_t factory_bad_pixels[];
    extern const uint16_t factory_bad_pixel_count;

    uint16_t current_offset = 0;

    for (uint16_t y = 0; y < VPIX; y++) {
        bad_pixel_line_info[y].offset = current_offset;

        for (uint16_t i = 0; i < factory_bad_pixel_count; i++) {
            if (factory_bad_pixels[i].y != y) continue;

            uint16_t x  = factory_bad_pixels[i].x;
            uint16_t rx = factory_bad_pixels[i].rep_x;
            uint16_t ry = factory_bad_pixels[i].rep_y;

            // Bounds check
            if (x >= HPIX || y >= VPIX || rx >= HPIX || ry >= VPIX) {
                continue;
            }

            if (current_offset >= MAX_BAD_PIXELS) {
                printf("BPR: Max bad pixels reached!\n");
                break;
            }

            // Store in sorted list
            bad_pixels_sorted[current_offset].x      = x;
            bad_pixels_sorted[current_offset].rep_x  = rx;
            bad_pixels_sorted[current_offset].rep_dy = (int8_t)(ry - y);

            // Mark as bad in gain frame
            gain_frame[y][x] = 0;

            current_offset++;
            bad_pixel_line_info[y].count++;
        }
    }

    total_bad_pixels = current_offset;

    printf("✓ Loaded %u bad pixels\n", total_bad_pixels);

    // ═══════════════════════════════════════════════════════════════
    // 3. Build patches (ONLY if we have bad pixels!)
    // ═══════════════════════════════════════════════════════════════
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

    // ═══════════════════════════════════════════════════════════════
    // SAFETY: Check if bad pixel data is valid
    // ═══════════════════════════════════════════════════════════════
    if (total_bad_pixels == 0) {
        printf("BPR: No bad pixels to build patches from\n");
        return;
    }

    for (uint16_t y = 0; y < VPIX; ++y) {
        const uint16_t cnt  = bad_pixel_line_info[y].count;
        const uint16_t off  = bad_pixel_line_info[y].offset;

        // ═══════════════════════════════════════════════════════════
        // SAFETY: Bounds check on offset and count
        // ═══════════════════════════════════════════════════════════
        if (off >= MAX_BAD_PIXELS) {
            printf("BPR: Invalid offset %u at line %u\n", off, y);
            continue;
        }

        if (cnt == 0) continue;  // No bad pixels in this line

        if (off + cnt > MAX_BAD_PIXELS) {
            printf("BPR: Overflow at line %u (off=%u cnt=%u)\n", y, off, cnt);
            continue;
        }

        const BadPixelInLine_t *px = &bad_pixels_sorted[off];

        for (uint16_t i = 0; i < cnt; ++i) {
            int16_t ry = (int16_t)y + (int16_t)px[i].rep_dy;
            uint16_t x  = px[i].x;
            uint16_t rx = px[i].rep_x;

            // Bounds check
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
/**
 * @brief Anwenden der Bad-Pixel-Patches (zweiphasig).
 * @param frame [uint16_t*] Frame-Daten
 * @param count [uint32_t] Anzahl der Patches
 * @return void
 * @note Vermeidet Read-after-Write-Artefakte.
 */
static inline void apply_bad_pixel_patches(uint16_t * __restrict__ frame, uint32_t count)
{
    for (uint32_t i = 0; i < count; ++i)
        g_patch_values[i] = frame[g_patches[i].src];

    for (uint32_t i = 0; i < count; ++i)
        frame[g_patches[i].dst] = g_patch_values[i];
}
/**
 * @brief Korrigiert fehlerhafte Pixel (einfache Variante).
 * @param frame [uint16_t*] Eingabeframe
 * @param width [uint32_t] Breite
 * @param height [uint32_t] Höhe
 * @return void
 */
static inline void correct_bad_pixels_simple(uint16_t frame[VPIX][HPIX]) {
    for (uint16_t i = 0; i < total_bad_pixels; i++) {
        // Find line
        uint16_t y = 0;
        for (y = 0; y < VPIX; y++) {
            uint16_t offset = bad_pixel_line_info[y].offset;
            uint16_t count = bad_pixel_line_info[y].count;
            if (i >= offset && i < offset + count) break;
        }

        // Get pixel
        uint16_t idx = i - bad_pixel_line_info[y].offset;
        uint16_t x = bad_pixels_sorted[bad_pixel_line_info[y].offset + idx].x;
        uint16_t rx = bad_pixels_sorted[bad_pixel_line_info[y].offset + idx].rep_x;
        int8_t dy = bad_pixels_sorted[bad_pixel_line_info[y].offset + idx].rep_dy;

        // Replace
        if (dy == 0 && rx < HPIX) {
            frame[y][x] = frame[y][rx];
        }
    }
}
/**
 * @brief Hauptprogramm-Einstiegspunkt.
 * @return int (sollte nie erreicht werden)
 * @note Führt Initialisierung und Hauptschleife aus.
 */
int main(void)
{

  /* USER CODE BEGIN 1 */
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

    printf("\n");


    init_planck_lut();
    for (uint32_t i = 0; i < HPIX; ++i) g_unity_q15_line[i] = 0x7FFF; // ~+0.999 in Q15

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

    printf("\nInitializing thermal pipeline...\n");

    // Initialize DWT cycle counter
    DWT_CycleCounter_Init();

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


    printf("\nInitializing 50 Hz VSYNC timer...\n");
    thermal_vsync_init();
    thermal_vsync_start();


    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    //uint32_t frame_count = 0;
    //uint32_t last_perf_check = HAL_GetTick();
    load_bad_pixel_map();
    init_gain_frame();
    static uint8_t led_state = 0;
    static int count =1000;
    while (1)
    {
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */

      DWT->CYCCNT = 0;
      if (is_calibrating()) {
		  // During calibration: Just collect samples
		  // (Assuming sensor writes to frame_buffer_B)

		  if (process_calibration_frame(frame_buffer_B)) {
			  // Calibration complete!
			  printf("Returning to normal processing...\n\n");
		  }

		  // Skip normal processing during calibration
		  HAL_Delay(20);  // Wait for next frame (~50 Hz)
		  continue;
	  }

	  float temp_diff = fabsf(T_chip_c - last_T_chip_c);
		 //if (temp_diff > 0.5f) {  // Only update if >0.5°C change
			 // Recompute gain frame (do this OUTSIDE the line loop!)
			 float dT = (T_chip_c - T0_chip_c);
			 float temp_gain_f = 1.0f / (1.0f + alpha * dT);
			 uint16_t temp_gain_q15 = q15_from_float(temp_gain_f);

			 // Update all pixels (this takes ~5 ms, but only when needed!)
			 for (uint32_t y = 0; y < VPIX; y++) {
				 for (uint32_t x = 0; x < HPIX; x++) {
					 // gain_frame[y][x] = sensor_gain × vignetting × temp_gain
					 // For now: just apply temp_gain to unity
					 gain_frame[y][x] = q15_mul_scalar_u16(0x7FFF, temp_gain_q15);
				 }
			 }

			 last_T_chip_c = T_chip_c;
		// }

	 for (int line = 0; line < VPIX; ++line)
		 {
			 process_thermal_line_fastest(
				 frame_buffer_A[line],
				 dark_frame[line],
				 gain_frame[line],  // ✓ Pre-computed!
				 offset_line,       // ✓ Static for now
				 planck_table,
				 frame_buffer_B[line],
				 HPIX
			 );

		 }
//	 apply_bad_pixel_patches(&frame_buffer_B[0][0], g_num_patches);
	 correct_bad_pixels_simple(frame_buffer_B);
      //LED1_RESET();
	 volatile uint32_t checksum = 0;
	     for (int y = 0; y < VPIX; y += 10) {
	         checksum += frame_buffer_B[y][0];
	     }
      uint32_t cycles = DWT->CYCCNT;
      if (++count >= 1000) {
          printf("Frame time: %lu cycles (%lu ms)\n",
                 cycles, cycles / 600000);
          count = 0;
//          if(led_state) {
//			  LED1_RESET();
//			  led_state = 0;
//		  } else {
//			  LED1_SET();
//			  led_state = 1;
//		  }
         start_dark_frame_calibration();
      }
    }
    /* USER CODE END 3 */
  /* USER CODE END 3 */
}
/* USER CODE BEGIN CLK 1 */
/* USER CODE END CLK 1 */

/**
 * @brief Initialisiert den Systemtakt (PLL, Busse, IC-Domain).
 * @note Angepasst für 600 MHz Ziel-Frequenz.
 * @return void
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
 * @brief Initialisiert XSPI2 (Octal DTR MemoryMapped Mode).
 * @note Aktiviert MPU-Region WB/WA, 256MB @0x7000_0000.
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
 * @brief Initialisiert GPIOs (LEDs, Debugpins).
 * @return void
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();  // ✓ Correct for LED1 (PG8)

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

///* === XSPI helpers (protected) ============================================ */
//static void XSPI_WriteEnable(XSPI_HandleTypeDef *hxspi)
//{
//  XSPI_RegularCmdTypeDef  sCommand = {0};
//  XSPI_AutoPollingTypeDef sConfig  = {0};
//
//  /* Enable write operations ------------------------------------------ */
//  sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
//  sCommand.Instruction        = OCTAL_WRITE_ENABLE_CMD;
//  sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
//  sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
//  sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
//  sCommand.AddressMode        = HAL_XSPI_ADDRESS_NONE;
//  sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
//  sCommand.DataMode           = HAL_XSPI_DATA_NONE;
//  sCommand.DummyCycles        = 0;
//  sCommand.DQSMode            = HAL_XSPI_DQS_DISABLE;
//
//  if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* Configure automatic polling mode to wait for write enabling ---- */
//  sCommand.Instruction        = OCTAL_READ_STATUS_REG_CMD;
//  sCommand.Address            = 0x0;
//  sCommand.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
//  sCommand.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
//  sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
//  sCommand.DataMode           = HAL_XSPI_DATA_8_LINES;
//  sCommand.DataDTRMode        = HAL_XSPI_DATA_DTR_ENABLE;
//  sCommand.DataLength         = 2;
//  sCommand.DummyCycles        = DUMMY_CLOCK_CYCLES_READ_OCTAL;
//  sCommand.DQSMode            = HAL_XSPI_DQS_ENABLE;
//
//  if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  sConfig.MatchMode           = HAL_XSPI_MATCH_MODE_AND;
//  sConfig.AutomaticStop       = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
//  sConfig.IntervalTime        = AUTO_POLLING_INTERVAL;
//  sConfig.MatchMask           = WRITE_ENABLE_MASK_VALUE;
//  sConfig.MatchValue          = WRITE_ENABLE_MATCH_VALUE;
//
//  if (HAL_XSPI_AutoPolling(hxspi, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

//static void XSPI_AutoPollingMemReady(XSPI_HandleTypeDef *hxspi)
//{
//  XSPI_RegularCmdTypeDef  sCommand = {0};
//  XSPI_AutoPollingTypeDef sConfig  = {0};
//
//  /* Configure automatic polling mode to wait for memory ready ------ */
//  sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
//  sCommand.Instruction        = OCTAL_READ_STATUS_REG_CMD;
//  sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
//  sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
//  sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
//  sCommand.Address            = 0x0;
//  sCommand.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
//  sCommand.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
//  sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
//  sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
//  sCommand.DataMode           = HAL_XSPI_DATA_8_LINES;
//  sCommand.DataDTRMode        = HAL_XSPI_DATA_DTR_ENABLE;
//  sCommand.DataLength         = 2;
//  sCommand.DummyCycles        = DUMMY_CLOCK_CYCLES_READ_OCTAL;
//  sCommand.DQSMode            = HAL_XSPI_DQS_ENABLE;
//
//  if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  sConfig.MatchMode           = HAL_XSPI_MATCH_MODE_AND;
//  sConfig.AutomaticStop       = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
//  sConfig.IntervalTime        = AUTO_POLLING_INTERVAL;
//  sConfig.MatchMask           = MEMORY_READY_MASK_VALUE;
//  sConfig.MatchValue          = MEMORY_READY_MATCH_VALUE;
//
//  if (HAL_XSPI_AutoPolling(hxspi, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

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
