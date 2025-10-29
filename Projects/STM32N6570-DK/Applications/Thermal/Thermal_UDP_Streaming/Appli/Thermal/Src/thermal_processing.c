/**
 * @file    thermal_processing.c
 * @brief   Thermal image processing pipeline (Helium optimized)
 */

#include "thermal_processing.h"
#include "arm_mve.h"
#include "xspi_nor.h"
#include <string.h>
#include "app_thermal.h"
#include "thermal_processing.h"
#include "stm32n6570_discovery.h"
#include <stdio.h>

// LED Fallbacks hinzufügen:
#ifndef LED_ORANGE
  #define LED_ORANGE 0
#endif
#ifndef LED_GREEN
  #define LED_GREEN 1
#endif
#ifndef LED_RED
  #define LED_RED 2
#endif



/* Thermal buffers placed via linker script (see STM32N657XX_LRUN.ld) */
uint16_t raw[FRAME_SIZE]     __attribute__((section(".thermal_frames"))) __attribute__((aligned(8)));
uint16_t dark[FRAME_SIZE]    __attribute__((section(".thermal_frames"))) __attribute__((aligned(8)));
uint16_t gain[FRAME_SIZE]    __attribute__((section(".thermal_frames"))) __attribute__((aligned(8)));
uint16_t offset[FRAME_SIZE]  __attribute__((section(".thermal_frames"))) __attribute__((aligned(8)));
uint16_t planck[65536]       __attribute__((section(".planck_lut")))           __attribute__((aligned(8)));
volatile uint16_t res[FRAME_SIZE] __attribute__((section(".noncacheable"))) __attribute__((aligned(8)));

static uint16_t bad_pixel_coords[MAX_BAD_PIXELS][2] __attribute__((section(".bad_pixel_data")));
static uint16_t bad_pixel_replacement[MAX_BAD_PIXELS] __attribute__((section(".bad_pixel_data")));
static uint32_t bad_pixel_count = 0U;

static uint32_t frame_counter = 0U;
static uint32_t processing_cycles = 0U;

static void update_bad_pixel_count(void);

static void process_thermal_line_fastest(const uint16_t *raw,
                                        const uint16_t *dark,
                                        const uint16_t *gain,
                                        uint16_t *result,
                                        int len)
{
    int offset = 0;

    // Process 8 pixels at a time with SIMD
    for (; offset + 8 <= len; offset += 8)
    {
        // Load 8 pixels
        uint16x8_t v_raw   = vld1q_u16(&raw[offset]);
        uint16x8_t v_dark  = vld1q_u16(&dark[offset]);
        uint16x8_t v_gain  = vld1q_u16(&gain[offset]);

        // Subtract dark (saturating)
        uint16x8_t v_result = vqsubq_u16(v_raw, v_dark);

        // Apply gain (saturating multiply)
        int16x8_t v_result_signed = vreinterpretq_s16_u16(v_result);
        int16x8_t v_gain_signed = vreinterpretq_s16_u16(v_gain);
        v_result_signed = vqrdmulhq_s16(v_result_signed, v_gain_signed);
        v_result = vreinterpretq_u16_s16(v_result_signed);

        // Apply Planck LUT
        uint16x8_t v_temp;
        v_temp = vsetq_lane_u16(planck[vgetq_lane_u16(v_result, 0)], v_temp, 0);
        v_temp = vsetq_lane_u16(planck[vgetq_lane_u16(v_result, 1)], v_temp, 1);
        v_temp = vsetq_lane_u16(planck[vgetq_lane_u16(v_result, 2)], v_temp, 2);
        v_temp = vsetq_lane_u16(planck[vgetq_lane_u16(v_result, 3)], v_temp, 3);
        v_temp = vsetq_lane_u16(planck[vgetq_lane_u16(v_result, 4)], v_temp, 4);
        v_temp = vsetq_lane_u16(planck[vgetq_lane_u16(v_result, 5)], v_temp, 5);
        v_temp = vsetq_lane_u16(planck[vgetq_lane_u16(v_result, 6)], v_temp, 6);
        v_temp = vsetq_lane_u16(planck[vgetq_lane_u16(v_result, 7)], v_temp, 7);

        // Store result - direkt mit vst1q
        vst1q_u16(&result[offset], v_temp);
    }

    // Process remaining pixels (scalar)
    for (; offset < len; offset++)
    {
        uint32_t corrected = (raw[offset] > dark[offset])
                           ? (raw[offset] - dark[offset])
                           : 0;

        uint32_t gained = (corrected * gain[offset]) >> 15;

        if (gained > 65535) gained = 65535;

        result[offset] = planck[gained];
    }
}


uint32_t thermal_frame_process(void)
{
  uint32_t start_cycles = DWT->CYCCNT;

  for (uint32_t line = 0U; line < VPIX; line++)
  {
    uint32_t offset_line = line * HPIX;
    process_thermal_line_fastest(&raw[offset_line],
                                 &dark[offset_line],
                                 &gain[offset_line],
                                 &res[offset_line],
                                 640);
  }

  apply_bad_pixel_correction();

  processing_cycles = DWT->CYCCNT - start_cycles;
  frame_counter++;

  return processing_cycles;
}

void apply_bad_pixel_correction(void)
{
  for (uint32_t i = 0U; i < bad_pixel_count; i++)
  {
    uint16_t x = bad_pixel_coords[i][0];
    uint16_t y = bad_pixel_coords[i][1];

    if ((x == 0U) || (x >= (HPIX - 1U)) || (y == 0U) || (y >= (VPIX - 1U)))
    {
      continue;
    }

    uint32_t idx = (uint32_t)y * HPIX + x;
    uint32_t sum = res[idx - HPIX] + res[idx + HPIX] + res[idx - 1U] + res[idx + 1U];

    res[idx] = (uint16_t)(sum / 4U);
  }
}

void generate_planck_lut(float c1, float c2, float wavelength)
{
  printf("[Thermal] Generating Planck LUT...\n");

  for (uint32_t adc = 0U; adc < 65536U; adc++)
  {
    float temp_kelvin = 273.15f + ((float)adc / 65535.0f) * 100.0f;
    float exponent = c2 / (wavelength * temp_kelvin);
    float radiance = c1 / (powf(wavelength, 5.0f) * (expf(exponent) - 1.0f));

    (void)radiance; /* Radiance kept for potential logging/validation */

    planck[adc] = (uint16_t)(((temp_kelvin - 273.15f) * 100.0f) + 10000.0f);
  }

  printf("[Thermal] Planck LUT generated.\n");
}

HAL_StatusTypeDef load_calibration_data(void)
{
  printf("[Thermal] Loading calibration from XSPI Flash...\n");

  if (XSPI_NOR_Read((uint8_t *)dark, DARK_FRAME_ADDRESS, FRAME_SIZE_BYTES) != HAL_OK)
  {
    printf("[Thermal] ERROR: Dark frame load failed\n");
    return HAL_ERROR;
  }

  if (XSPI_NOR_Read((uint8_t *)gain, GAIN_FRAME_ADDRESS, FRAME_SIZE_BYTES) != HAL_OK)
  {
    printf("[Thermal] ERROR: Gain frame load failed\n");
    return HAL_ERROR;
  }

  if (XSPI_NOR_Read((uint8_t *)offset, OFFSET_FRAME_ADDRESS, FRAME_SIZE_BYTES) != HAL_OK)
  {
    printf("[Thermal] ERROR: Offset frame load failed\n");
    return HAL_ERROR;
  }

  if (XSPI_NOR_Read((uint8_t *)bad_pixel_coords, BAD_PIXEL_ADDRESS, sizeof(bad_pixel_coords)) != HAL_OK)
  {
    printf("[Thermal] WARNING: Bad pixel map load failed, continuing without corrections\n");
    bad_pixel_count = 0U;
  }
  else
  {
    update_bad_pixel_count();
  }

  printf("[Thermal] Calibration loaded successfully.\n");
  return HAL_OK;
}

void thermal_get_stats(ThermalStats_t *stats)
{
  if (stats == NULL)
  {
    return;
  }

  stats->frame_count = frame_counter;
  stats->last_processing_cycles = processing_cycles;
  stats->last_processing_ms = (float)processing_cycles / 600000.0f;
}

void enable_dwt_cycle_counter(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void update_bad_pixel_count(void)
{
  bad_pixel_count = 0U;

  for (uint32_t i = 0U; i < MAX_BAD_PIXELS; i++)
  {
    if ((bad_pixel_coords[i][0] == 0xFFFFU) && (bad_pixel_coords[i][1] == 0xFFFFU))
    {
      break;
    }

    bad_pixel_replacement[i] = 0U;
    bad_pixel_count++;
  }
}
