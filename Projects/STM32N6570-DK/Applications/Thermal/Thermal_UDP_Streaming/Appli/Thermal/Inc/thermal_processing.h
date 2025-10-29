/**
 * @file    thermal_processing.h
 * @brief   Thermal image processing API for UDP streaming demo
 */

#ifndef THERMAL_PROCESSING_H
#define THERMAL_PROCESSING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32n6xx_hal.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>

/* Thermal frame geometry (640 x 480, 16-bit samples) */
#define HPIX               640U
#define VPIX               480U
#define FRAME_SIZE         (HPIX * VPIX)
#define FRAME_SIZE_BYTES   (FRAME_SIZE * sizeof(uint16_t))

/* Planck constants for LUT generation */
#define PLANCK_C1          1.191042e8f
#define PLANCK_C2          1.4387752e4f
#define SENSOR_WAVELENGTH  10.0f

/* XSPI NOR flash memory map (see integration plan §3.3) */
#define DARK_FRAME_ADDRESS 0x70000000UL
#define GAIN_FRAME_ADDRESS 0x70100000UL
#define OFFSET_FRAME_ADDRESS 0x70200000UL
#define BAD_PIXEL_ADDRESS  0x70300000UL

/* Bad pixel support */
#define MAX_BAD_PIXELS     1000U

typedef struct
{
    uint32_t frame_count;
    uint32_t last_processing_cycles;
    float    last_processing_ms;
} ThermalStats_t;

extern uint16_t raw[FRAME_SIZE]     __attribute__((section(".thermal_frames")));
extern uint16_t dark[FRAME_SIZE]    __attribute__((section(".thermal_frames")));
extern uint16_t gain[FRAME_SIZE]    __attribute__((section(".thermal_frames")));
extern uint16_t offset[FRAME_SIZE]  __attribute__((section(".thermal_frames")));
extern uint16_t planck[65536] __attribute__((section(".planck_lut")));
extern volatile uint16_t res[FRAME_SIZE] __attribute__((section(".noncacheable")));

void generate_planck_lut(float c1, float c2, float wavelength);
uint32_t thermal_frame_process(void);
void thermal_get_stats(ThermalStats_t *stats);
void enable_dwt_cycle_counter(void);
HAL_StatusTypeDef load_calibration_data(void);
void apply_bad_pixel_correction(void);


#ifdef __cplusplus
}
#endif

#endif /* THERMAL_PROCESSING_H */
