# Xi 640 ETH - STM32N6 Thermal Camera Development

**Hardware:** STM32N6570-DK | **Sensor:** 640×480 @ 14-bit | **Target:** 125-160 FPS

## 🎯 Project Overview

Development of the Xi 640 ETH infrared camera system using the STM32N6570 MCU with Zephyr RTOS. Four parallel development sessions optimizing for maximum performance:

```
Current:  23 FPS @ 640×480
Target:  160 FPS @ 640×480
Speedup: ~7× performance gain
```

## 📋 Development Sessions

### ✅ Session 1: Zephyr Base + Synthetic Frame Generator
**Status:** COMPLETE | **Location:** `01-zephyr-base/`

Foundation layer with Zephyr RTOS 3.7 integration:
- ✅ Fresh Zephyr 3.7 LTS project structure
- ✅ DCMIPP configured for 640×480 @ 14-bit parallel ADC
- ✅ Memory layout: AXISRAM (DMA) + HyperRAM (framebuffer) + HyperFlash (calibration)
- ✅ Synthetic thermal frame generator with 5 test patterns
- ✅ Triple-buffered frame pool architecture
- ✅ Performance monitoring dashboard
- ✅ 50 FPS baseline (solid foundation)

**Key Deliverables:**
- Complete Zephyr project with West workspace
- Device tree overlay for STM32N6570-DK
- Synthetic frame generator (gradient, hotspot, checkerboard, noise, animated)
- Memory manager with AXISRAM/HyperRAM/HyperFlash support
- Main application with frame capture thread

**Build & Run:**
```bash
cd 01-zephyr-base
west init -l .
west update
west build -b stm32n6570_dk
west flash
```

---

### 🚧 Session 2: Helium MVE SIMD Thermal Processing ⚡
**Status:** PENDING | **Location:** `02-helium-processing/` | **Base:** `stm32-helium-demo`

ARM Helium (MVE) vectorized thermal image processing:
- [ ] Scale existing Helium demo from NUCLEO to STM32N6570-DK
- [ ] MVE vectorized temperature calculation (16 pixels parallel)
- [ ] Fixed-point Q15 arithmetic for speed
- [ ] Dark frame subtraction + gain/offset correction
- [ ] Bad pixel correction with factory defect map
- [ ] Planck LUT temperature conversion
- [ ] Target: 23 FPS → 160+ FPS (7× speedup)

**Expected Performance:**
```
Scalar:     ~43 ms/frame (23 FPS)
Helium MVE: ~6 ms/frame (160+ FPS)
Speedup:    7-8× with SIMD vectorization
```

---

### 🚧 Session 3: JPEG Hardware Encoder + RTSP Streaming
**Status:** PENDING | **Location:** `03-jpeg-rtsp/`

Real-time thermal video streaming:
- [ ] STM32N6 JPEG hardware codec integration
- [ ] LwIP UDP/TCP stack configuration
- [ ] RTSP server implementation
- [ ] ONVIF compatibility layer
- [ ] Ethernet PHY driver (STM32N6570-DK)
- [ ] Target: 640×480 @ 160 FPS → ~100 Mbit/s stream

**Network Architecture:**
```
[Thermal Sensor] → [Helium MVE] → [JPEG Encoder] → [RTSP/UDP]
    640×480            <6ms           Hardware        Ethernet
```

---

### 🚧 Session 4: System Integration
**Status:** PENDING | **Location:** `04-integration/`

Complete system integration and testing:
- [ ] USB + Ethernet parallel operation
- [ ] PIF Interface (Process Interface I/O)
- [ ] Performance monitoring dashboard
- [ ] End-to-end validation (sensor → stream)
- [ ] Power consumption optimization
- [ ] Final benchmarking

---

## 🏗️ System Architecture

### Memory Layout (STM32N6570)

| Region | Address | Size | Usage |
|--------|---------|------|-------|
| AXISRAM1 | 0x24000000 | 1 MB | DMA buffers (DCMIPP capture) |
| AXISRAM2 | 0x24100000 | 1 MB | Processing (gain, dark, offset) |
| AXISRAM3 | 0x24200000 | 1 MB | Bad pixel correction data |
| HyperRAM | 0x90000000 | 64 MB | Frame pool (triple buffering) |
| HyperFlash | 0x70000000 | 128 MB | Calibration data storage |

### Data Flow Pipeline

```
┌─────────────────┐
│ Thermal Sensor  │ 640×480 @ 14-bit
│   (Simulated)   │
└────────┬────────┘
         │ DCMIPP (Parallel)
         ▼
┌─────────────────┐
│   AXISRAM1 DMA  │ Raw frame capture
│   Buffer        │
└────────┬────────┘
         │ DMA Transfer
         ▼
┌─────────────────┐
│  Helium MVE     │ Vectorized processing
│  Processing     │ • Dark frame subtract
│  (AXISRAM2)     │ • Gain/offset correction
└────────┬────────┘ • Temperature conversion
         │
         ▼
┌─────────────────┐
│  JPEG Encoder   │ Hardware compression
│  (STM32N6)      │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ RTSP/UDP Stream │ Ethernet output
│  (LwIP)         │ 100 Mbit/s
└─────────────────┘
```

## 🔧 Hardware Configuration

### STM32N6570-DK
- **MCU:** STM32N6570 (ARM Cortex-M55 @ 150 MHz)
- **Features:** ARM Helium MVE, JPEG codec, DCMIPP
- **RAM:** 3× 1MB AXISRAM + 64MB HyperRAM
- **Flash:** 128MB HyperFlash
- **Debug:** ST-Link V3E built-in
- **Ethernet:** 10/100 Mbit/s PHY

### Boot Configuration
```
Development Mode (ST-Link):
  BOOT0 = 2-3
  BOOT1 = 2-3

Production Mode (External Flash):
  BOOT0 = 1-2
  BOOT1 = 1-2
```

## 📊 Performance Targets

| Metric | Baseline (Session 1) | Target (Sessions 2-4) |
|--------|---------------------|----------------------|
| **Frame Rate** | 50 FPS | 125-160 FPS |
| **Processing Time** | N/A (synthetic) | <6 ms/frame |
| **Memory Bandwidth** | 30 MB/s | 98-156 MB/s |
| **Stream Bitrate** | N/A | ~100 Mbit/s |
| **CPU Load** | <10% | <80% @ 160 FPS |

## 🚀 Getting Started

### Prerequisites
```bash
# Zephyr dependencies
sudo apt install --no-install-recommends git cmake ninja-build gperf \
  ccache dfu-util device-tree-compiler wget \
  python3-dev python3-pip python3-setuptools python3-tk python3-wheel xz-utils file \
  make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1

# Install Zephyr SDK
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.8/zephyr-sdk-0.16.8_linux-x86_64.tar.xz
tar xvf zephyr-sdk-0.16.8_linux-x86_64.tar.xz
cd zephyr-sdk-0.16.8
./setup.sh
```

### Quick Start: Session 1
```bash
# Clone repository
git clone <repo-url> xi640-eth
cd xi640-eth/01-zephyr-base

# Initialize workspace
west init -l .
west update

# Build and flash
west build -b stm32n6570_dk
west flash

# Monitor serial output (115200 baud)
minicom -D /dev/ttyACM0 -b 115200
```

## 📚 References

- [Zephyr RTOS Documentation](https://docs.zephyrproject.org/)
- [STM32N6 Series Documentation](https://www.st.com/en/microcontrollers-microprocessors/stm32n6-series.html)
- [STM32N6570-DK Board](https://www.st.com/en/evaluation-tools/stm32n6570-dk.html)
- [ARM Helium (MVE) Guide](https://developer.arm.com/documentation/102102/latest/)
- [STM32CubeN6 HAL](https://github.com/STMicroelectronics/STM32CubeN6)
- [Existing Helium Demo](https://github.com/ReneLuedecke/stm32-helium-demo)

## 🤝 Contributing

This is a multi-session development project. Each session builds on the previous:

1. **Session 1** (✅ Complete): Solid foundation with Zephyr + synthetic frames
2. **Session 2** (Next): Helium MVE performance optimization
3. **Session 3** (Future): Network streaming
4. **Session 4** (Future): System integration

## 📝 License

[License information to be added]

## 👥 Authors

- **René Lüdecke** - Optris GmbH
- **Project Lead:** Claude (Anthropic)

---

**Last Updated:** 2025-11-13
**Current Session:** 1/4 Complete ✅
**Next:** Session 2 - Helium MVE SIMD Processing
