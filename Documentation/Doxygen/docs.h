/** @file docs.h */

/** @page page_architektur Architektur & Datenfluss
 * @section arch_flow Datenfluss pro Zeile
 * @code
 * ADC[line] -> dark_sub -> mul_Q15(gain) -> +flag_adc -> +offset -> LUT[adc] -> temp_enc -> BPR
 * @endcode
 * @section arch_mem Speicher
 * - AXISRAM1: LUT/Hot-Data (64B aligned)
 * - .noncacheable: DMA/Frames
 * - XSPI2 MMAP: Kalibrierkacheln @ 0x70000000 (MPU WB/WA)
 */

/** @page page_timing Timing & ISR
 * - TIM2 ~50 Hz VSYNC -> Frame-Loop
 * - DWT-Cycles pro Frame loggen
 * - Keine blockierenden I/Os im ISR-Kontext
 */

/** @page page_pipeline Pipeline-Details
 * - Q15 Gain (Q1.15, saturierend mit Helium/MVE)
 * - 65536er LUT: centi-°C + Offset 10000
 * - BPR: simple vs. Patch (zweiphasig)
 */
