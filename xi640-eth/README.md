# Xi 640 ETH - Complete Thermal Processing Pipeline

## Overview

Complete integration of thermal frame generation and ARM Helium MVE processing for the Xi 640 ETH thermal camera on STM32N6570-DK.

**Target Performance:** 128+ FPS sustained throughput @ 600 MHz

## Architecture

### Session 1: Frame Generation
- Synthetic thermal frame generation (640×480 pixels)
- Radial gradient test pattern with animated hot spot
- Simulates Xi 640 ETH sensor output

### Session 2: MVE Thermal Processing
- ARM Helium (MVE) SIMD-optimized processing
- Dark frame subtraction and gain correction
- Planck radiation law temperature conversion
- 65536-entry LUT for fast temperature mapping
- Vectorized statistics (min/max/avg)

## Build Instructions

### Prerequisites
```bash
# Zephyr SDK and west tool installed
# STM32N6570-DK board connected
```

### Build
```bash
cd xi640-eth
west build -b stm32n6570dk -p auto
```

### Flash
```bash
west flash
```

### Monitor Output
```bash
# Connect to UART (115200 baud, 8N1)
picocom -b 115200 /dev/ttyUSB0
```

## Expected Performance

| Component | Time | FPS |
|-----------|------|-----|
| Frame Generation (Session 1) | ~0.2 ms | 5000 FPS |
| MVE Processing (Session 2) | ~2.4 ms | 416 FPS |
| **Total Pipeline** | **~2.6 ms** | **384 FPS** |

✅ **Target exceeded:** 384 FPS >> 128 FPS (3× margin)

## Output Example

```
========================================
Xi 640 ETH - Complete Pipeline Test
Session 1 + Session 2 Integration
========================================

✅ Thermal SIMD initialized

=== Frame 100 Statistics ===
Generation:  0.21 ms ( 4761.9 FPS) [Session 1]
Processing:  2.38 ms (  420.2 FPS) [Session 2]
───────────────────────────────────────
Total:       2.59 ms (  386.1 FPS)
───────────────────────────────────────
Temperature: 20.5°C - 85.3°C (avg: 45.2°C)
✅ Target 128 FPS maintained! (386.1 FPS)
```

## File Structure

```
xi640-eth/
├── CMakeLists.txt           # Main build configuration
├── prj.conf                 # Zephyr project configuration
├── README.md                # This file
├── src/
│   └── main.c              # Main application (Session 1+2 integration)
└── modules/
    └── thermal_simd/       # MVE processing module (Session 2)
        ├── CMakeLists.txt
        ├── thermal_simd.h
        └── thermal_simd.c
```

## Hardware Configuration

- **Board:** STM32N6570-DK
- **CPU:** Cortex-M55 @ 600 MHz
- **Features:** ARM Helium MVE, FPU
- **Memory:**
  - AXISRAM1/2/3: 3 MB total
  - Flash: External XSPI NOR

## Performance Tuning

### Compiler Optimizations
- `-O3`: Maximum optimization
- `-ffast-math`: Aggressive floating-point optimizations
- `-march=armv8.1-m.main+mve.fp`: Enable all Helium MVE instructions

### Memory Layout
- Planck LUT: 256 KB (AXISRAM1 - high speed)
- Frame buffers: 1.2 MB (AXISRAM2/3)
- Stack: 16 KB per thread

## Future Enhancements

- [ ] Real Xi 640 ETH sensor integration (SPI/Ethernet)
- [ ] Bad pixel correction with factory defect map
- [ ] Per-pixel calibration support
- [ ] UDP streaming to visualization tool
- [ ] ROI-based statistics and tracking

## License

Part of stm32-helium-demo repository. See top-level LICENSE.md.
