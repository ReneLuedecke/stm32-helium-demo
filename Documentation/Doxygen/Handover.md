# CORTEX_HELIUM FSBL – Handover

**Ziel**  
Thermal-Pipeline (640×480): Dark → Q15-Gain → Flag/Offset → LUT → BPR; XSPI (OPI-DTR, MMAP); DWT-Profiling.

**Architektur**
- `main.c`: Boot/Init (Clock, GPIO, UART, XSPI, TIM2, DWT) + Frame-Loop
- `thermal_pipeline.c/.h`: Zeilenverarbeitung (MVE/Q15) + LUT
- `calibration.c/.h`: Dark-FSM, inverse LUT (Flag→ADC)
- `bpr.c/.h`: Bad-Pixel-Korrektur
- `xspi_storage.c/.h`: XSPI2 MMAP + MPU
- `platform.c/.h`: Clock/UART/GPIO/DWT/TIM2

Mehr Details: @ref page_architektur, @ref page_timing, @ref page_pipeline
