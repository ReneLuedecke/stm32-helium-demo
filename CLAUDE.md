# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a fork of STMicroelectronics' STM32CubeN6 MCU Firmware Package, customized for thermal imaging pipeline development on the STM32N6 (NUCLEO-N657X0-Q board). The primary development focus is the CORTEX_HELIUM example, which demonstrates ARM Helium SIMD optimization for real-time thermal image processing.

**Key Features:**
- Thermal sensor data processing pipeline with dark frame/gain calibration
- ARM Helium (MVE) vectorized processing for 640×480 thermal frames
- XSPI NOR Flash integration for calibration data persistence
- Planck radiation law-based temperature conversion
- Bad pixel correction with factory-defined defect map

## Build Commands

### STM32CubeIDE (Primary IDE)

**Import and build from GUI:**
1. Import workspace: `File > Import > Existing Projects into Workspace`
2. Select `Projects/NUCLEO-N657X0-Q/Examples/CORTEX/CORTEX_HELIUM/STM32CubeIDE`
3. Build: `Project > Build Project` or Ctrl+B

**Headless build (Windows PowerShell):**
```powershell
STM32CubeIDE -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild `
  -data ide_ws -importAll Projects\NUCLEO-N657X0-Q\Examples\CORTEX\CORTEX_HELIUM\STM32CubeIDE `
  -build all
```

**Output location:** `Projects/NUCLEO-N657X0-Q/Examples/CORTEX/CORTEX_HELIUM/STM32CubeIDE/FSBL/Debug/`

### Flashing to Hardware

**Development mode (BOOT1 = 2-3):**
```powershell
STM32_Programmer_CLI -c port=SWD -w Projects\NUCLEO-N657X0-Q\Examples\CORTEX\CORTEX_HELIUM\STM32CubeIDE\FSBL\Debug\CORTEX_HELIUM_FSBL.hex
```

**Boot from external flash (production):**
1. Sign binary with header:
   ```powershell
   STM32_SigningTool_CLI.exe -bin CORTEX_HELIUM_FSBL.bin -nk -of 0x80000000 -t fsbl -o CORTEX_HELIUM_FSBL-trusted.bin -hv 2.3 -dump CORTEX_HELIUM_FSBL-trusted.bin
   ```
2. Flash to external memory at 0x70000000
3. Set BOOT0 = 1-2, BOOT1 = 1-2

### Documentation Generation

**Doxygen (from `Documentation/Doxygen/`):**
```bash
doxygen Doxyfile
```
Output: `build/html/index.html`

## Architecture Overview

### Memory Layout (STM32N6 @ 600 MHz)

The thermal processing pipeline uses carefully partitioned memory regions:

- **AXISRAM1 (1 MB):** High-speed data - Planck LUT (128 KB)
- **AXISRAM2 (1 MB):** Active processing buffers - gain frame, dark frame, offset map, current sensor frame
- **AXISRAM3 (1 MB):** Bad pixel correction - patch arrays (coordinates + replacement values)
- **XSPI2 NOR Flash (Memory-mapped @ 0x70000000):** Persistent calibration data storage

### Thermal Processing Pipeline

**Main data flow (Projects/NUCLEO-N657X0-Q/Examples/CORTEX/CORTEX_HELIUM/FSBL/Src/main.c):**

1. **Calibration Phase** (`start_dark_frame_calibration`, `process_calibration_frame`):
   - Averages 16 frames with closed shutter → dark frame
   - Stores calibration data to XSPI Flash via `XSPI_WriteGainFrame`

2. **Frame Processing** (`thermal_frame_process`, `process_thermal_line_fastest`):
   - Triggered by TIM2 timer interrupt (simulating 50Hz thermal sensor VSYNC)
   - Per-line Helium-optimized processing:
     - Dark frame subtraction
     - Gain/offset correction (Q15 fixed-point)
     - Planck LUT temperature conversion
   - Bad pixel correction via `apply_bad_pixel_patches`

3. **Temperature Encoding** (`decode_temperature`, `generate_linear_temp_lut`):
   - Planck radiation law: `B(λ,T) = C₁ / (λ⁵(e^(C₂/λT) - 1))`
   - 16-bit LUT maps ADC values → temperature (stored as `(T_celsius * 100 + 10000)`)

### XSPI Flash Interface (xspi_nor.c/h)

Abstracts Octal DTR operations for calibration persistence:
- `XSPI_NOR_Write` / `XSPI_NOR_Read`: Page-level I/O
- `XSPI_NOR_EnableMemoryMappedMode`: Zero-copy access to stored calibration frames
- `XSPI_WriteGainFrame`: High-level calibration storage with automatic sector erase

### ARM Helium Optimization

**Key function:** `process_thermal_line_fastest` (main.c:852)
- Uses `arm_mve.h` intrinsics for SIMD operations
- Processes 8 pixels per iteration vs. scalar loop
- Expected 4-8× speedup for full frame processing

## Key Files and Locations

### Primary Development Area
- **Main application:** `Projects/NUCLEO-N657X0-Q/Examples/CORTEX/CORTEX_HELIUM/FSBL/Src/main.c`
- **XSPI driver:** `Projects/NUCLEO-N657X0-Q/Examples/CORTEX/CORTEX_HELIUM/FSBL/Src/xspi_nor.c`
- **XSPI header:** `Projects/NUCLEO-N657X0-Q/Examples/CORTEX/CORTEX_HELIUM/FSBL/Inc/xspi_nor.h`
- **Linker script:** `Projects/NUCLEO-N657X0-Q/Examples/CORTEX/CORTEX_HELIUM/STM32CubeIDE/FSBL/STM32N657X0HXQ_AXISRAM2_fsbl.ld`

### STM32Cube Infrastructure
- **HAL drivers:** `Drivers/STM32N6xx_HAL_Driver/`
- **BSP (Board Support):** `Drivers/BSP/NUCLEO-N657X0/`
- **CMSIS core:** `Drivers/CMSIS/`
- **Middleware:** `Middlewares/ST/` (ThreadX, NetXDuo, FileX, USBX)

## Development Workflow

### Making Code Changes

1. **Edit source files** in `Projects/NUCLEO-N657X0-Q/Examples/CORTEX/CORTEX_HELIUM/FSBL/`
2. **Rebuild** via STM32CubeIDE (GUI or headless)
3. **Serial output:** Connect UART1 (115200 baud, 8N1) to monitor processing metrics
4. **Flash and test** on hardware using ST-LINK

### Hardware Configuration

- Set boot mode switches before power-on
- UART logs show cycle counts, calibration status, and frame processing timing
- LED1 (GPIOG Pin 8) indicates processing activity

### Submodule Management

This repo uses git submodules for STM32Cube components:
```bash
# After pulling changes
git pull
git submodule update --init --recursive
```

## Coding Conventions

- **Indentation:** 2 spaces (match existing HAL driver style)
- **Naming:**
  - Functions: `Module_Action` (e.g., `XSPI_NOR_Init`)
  - Macros: `UPPERCASE_WITH_UNDERSCORES`
  - Files: `lowercase_with_underscores.c`
- **Comments:** Doxygen format for public APIs (`/** @brief ... */`)
- **Fixed-point:** Q15 format (16-bit signed, 1 sign + 15 fractional bits)

## Testing

No automated test suite exists. Validation is hardware-based:
1. Build and flash to NUCLEO-N657X0-Q
2. Monitor UART output for:
   - Calibration completion messages
   - Frame processing cycle counts
   - Memory access timing (XSPI read/write performance)
3. Document binary checksum and observed behavior in commits

## Important Constants

From `main.c` (lines 30-57):
- `HPIX = 640`, `VPIX = 480`: Thermal sensor resolution
- `PLANCK_C1 = 1.191042e8`, `PLANCK_C2 = 1.4387752e4`: Radiation constants
- `XSPI1_MMAP_BASE = 0x90000000`, `XSPI2_MMAP_BASE = 0x70000000`: Memory-mapped flash regions
- `TIM2_PRESCALER = 9999`, `TIM2_PERIOD = 799`: 50Hz frame trigger timer

## Commit Message Style

Use imperative mood with optional conventional-commit prefixes:
```
feat: Add Helium optimization for bad pixel interpolation
fix: Correct XSPI sector erase alignment check
docs: Update thermal pipeline architecture diagram
```

Reference board/module in subject line when applicable. See AGENTS.md for detailed PR guidelines.
