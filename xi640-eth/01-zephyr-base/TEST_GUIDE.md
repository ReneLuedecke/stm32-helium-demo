# Xi 640 ETH - Test Guide

## 🎯 Testing Options

### Option 1: Native PC Simulation (NO Hardware Required) ✅

Test the application logic on your PC without STM32 hardware.

#### Prerequisites
```bash
# Install Zephyr SDK
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.8/zephyr-sdk-0.16.8_linux-x86_64.tar.xz
tar xvf zephyr-sdk-0.16.8_linux-x86_64.tar.xz
cd zephyr-sdk-0.16.8
./setup.sh -h

# Install west
pip3 install west
```

#### Build & Run Native Simulation
```bash
cd xi640-eth/01-zephyr-base

# Initialize workspace (first time only)
west init -l .
west update

# Build for native_sim (runs on your PC)
west build -b native_sim -p auto

# Run directly on your PC!
./build/zephyr/zephyr.exe

# Or with west:
west build -t run
```

**Expected Output:**
```
╔════════════════════════════════════════════════════════════╗
║           Xi 640 ETH Thermal Camera System                ║
║        SESSION 1: Zephyr Base + Frame Generator           ║
╚════════════════════════════════════════════════════════════╝

✓ Memory manager ready
✓ Thermal generator ready (pattern=0)
✓ Capture thread started

╔════════════════════════════════════════════════════════════╗
║ Xi 640 ETH - PERFORMANCE STATISTICS                       ║
╠════════════════════════════════════════════════════════════╣
║ Frames generated:            250                          ║
║ Current FPS:                  50 (target: 50)             ║
╚════════════════════════════════════════════════════════════╝
```

---

### Option 2: Docker Container (Isolated Environment)

Complete pre-configured Zephyr development environment.

#### Build Docker Image
```bash
cd xi640-eth/01-zephyr-base

# Create Dockerfile
cat > Dockerfile <<'EOF'
FROM ubuntu:22.04

# Install dependencies
RUN apt-get update && apt-get install -y \
    git cmake ninja-build gperf ccache dfu-util \
    device-tree-compiler wget python3-dev python3-pip \
    python3-setuptools python3-wheel xz-utils file make \
    gcc g++ libsdl2-dev && \
    rm -rf /var/lib/apt/lists/*

# Install west
RUN pip3 install west

# Install Zephyr SDK
WORKDIR /opt
RUN wget -q https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.8/zephyr-sdk-0.16.8_linux-x86_64.tar.xz && \
    tar xf zephyr-sdk-0.16.8_linux-x86_64.tar.xz && \
    rm zephyr-sdk-0.16.8_linux-x86_64.tar.xz && \
    cd zephyr-sdk-0.16.8 && \
    ./setup.sh -h -c

ENV ZEPHYR_SDK_INSTALL_DIR=/opt/zephyr-sdk-0.16.8

WORKDIR /workspace
EOF

# Build image
docker build -t xi640-zephyr .

# Run container
docker run -it --rm -v $(pwd):/workspace xi640-zephyr bash
```

#### Inside Container
```bash
# Initialize workspace
west init -l .
west update

# Build native sim
west build -b native_sim

# Run
./build/zephyr/zephyr.exe
```

---

### Option 3: Hardware Test (STM32N6570-DK Required)

If you have the actual hardware board.

#### Prerequisites
```bash
# Install STM32CubeProgrammer
# Download from: https://www.st.com/en/development-tools/stm32cubeprog.html

# OR use OpenOCD
sudo apt install openocd
```

#### Build & Flash
```bash
cd xi640-eth/01-zephyr-base

# Build for STM32N6570-DK
west build -b stm32n6570_dk -p auto

# Flash via ST-Link
west flash

# Monitor serial output (115200 baud)
minicom -D /dev/ttyACM0 -b 115200

# OR with screen
screen /dev/ttyACM0 115200
```

#### Hardware Setup
```
STM32N6570-DK Boot Configuration:
  BOOT0 = 2-3 (Development mode)
  BOOT1 = 2-3

Connect:
  - USB ST-Link for programming
  - USB UART for console (USART1: PA9=TX, PA10=RX)
```

---

### Option 4: QEMU Emulation (Advanced)

Emulate ARM Cortex-M55 without hardware.

```bash
# Build for QEMU
west build -b qemu_cortex_m55

# Run in QEMU
west build -t run

# Exit QEMU: Ctrl+A, then X
```

**Note:** STM32N6-specific peripherals (DCMIPP, XSPI) won't work in QEMU, but you can test application logic.

---

## 🔧 Troubleshooting

### Native Sim: "Cannot allocate memory"
```bash
# Reduce frame pool size in prj_native.conf
CONFIG_THERMAL_FRAME_POOL_COUNT=2
CONFIG_HEAP_MEM_POOL_SIZE=32768
```

### West: "Command not found"
```bash
# Add to PATH
export PATH=$HOME/.local/bin:$PATH

# Or reinstall
pip3 install --user west
```

### Build Errors: "Zephyr SDK not found"
```bash
# Set environment variable
export ZEPHYR_SDK_INSTALL_DIR=/path/to/zephyr-sdk-0.16.8

# Or source zephyr-env.sh
source zephyr/zephyr-env.sh
```

### Serial Port Access Denied
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER
# Logout and login again
```

---

## 🧪 Verification Tests

### Test 1: Frame Generation
Check that frames are generated at target rate.

**Expected:**
- FPS close to 50
- No "frame dropped" warnings
- Generation time < 5ms

### Test 2: Memory Management
Verify frame pool operates correctly.

**Check:**
- Memory map shows AXISRAM/HyperRAM allocation
- No "out of memory" errors
- Frame locks/unlocks work

### Test 3: Test Patterns
Change patterns and verify output.

```bash
# If shell is enabled
uart:~$ pattern 1    # Switch to hotspot
uart:~$ pattern 2    # Switch to checkerboard
uart:~$ stats        # Show statistics
```

### Test 4: Performance
Monitor CPU usage and timing.

**Targets:**
- Frame generation: <5ms
- CPU load: <10%
- No frame drops over 5 minutes

---

## 📊 Benchmark Results

Record your test results here:

| Platform | FPS | Gen Time (ms) | CPU % | Notes |
|----------|-----|---------------|-------|-------|
| Native PC (i7-10700) | ? | ? | ? | |
| Native PC (ARM M1) | ? | ? | ? | |
| Docker (Ubuntu 22.04) | ? | ? | ? | |
| STM32N6570-DK | ? | ? | ? | Actual HW |
| QEMU ARM Cortex-M55 | ? | ? | ? | |

---

## 🚀 Quick Start (TL;DR)

**Fastest way to test WITHOUT hardware:**

```bash
# 1. Install dependencies
pip3 install west

# 2. Navigate to project
cd xi640-eth/01-zephyr-base

# 3. Initialize workspace
west init -l .
west update

# 4. Build native sim
west build -b native_sim -p

# 5. Run!
./build/zephyr/zephyr.exe
```

**That's it!** You should see the thermal frame generator running on your PC.

---

## 📝 Notes

- Native simulation runs at host CPU speed (much faster than 150 MHz STM32)
- Actual frame timing will differ from hardware
- Memory addresses are simulated (not real hardware addresses)
- Perfect for testing application logic and algorithms
- Use hardware for accurate performance measurement

---

**Need help?** Check the main [README.md](README.md) or open an issue.
