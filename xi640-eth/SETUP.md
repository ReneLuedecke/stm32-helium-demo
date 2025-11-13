# Xi 640 ETH Setup Instructions

## Prerequisites

This project requires Zephyr RTOS. Follow these steps to set up the build environment.

### 1. Install Zephyr SDK

```bash
# Install dependencies (Ubuntu/Debian)
sudo apt update
sudo apt install --no-install-recommends git cmake ninja-build gperf \
  ccache dfu-util device-tree-compiler wget \
  python3-dev python3-pip python3-setuptools python3-tk python3-wheel xz-utils file \
  make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1

# Install west
pip3 install --user -U west

# Get Zephyr source
west init ~/zephyrproject
cd ~/zephyrproject
west update

# Install Zephyr SDK
cd ~
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.5/zephyr-sdk-0.16.5_linux-x86_64.tar.xz
tar xf zephyr-sdk-0.16.5_linux-x86_64.tar.xz
cd zephyr-sdk-0.16.5
./setup.sh

# Install Python dependencies
pip3 install --user -r ~/zephyrproject/zephyr/scripts/requirements.txt
```

### 2. Set Up Environment

```bash
# Add to ~/.bashrc or run each session
export ZEPHYR_BASE=~/zephyrproject/zephyr
source $ZEPHYR_BASE/zephyr-env.sh
```

### 3. Build the Project

```bash
cd /path/to/stm32-helium-demo/xi640-eth
west build -b stm32n6570dk -p auto
```

### 4. Flash to Hardware

```bash
west flash
```

## Board Support

The STM32N6570-DK board support may need to be added to Zephyr if not already present:

```bash
cd ~/zephyrproject/zephyr
# Check if board exists
ls boards/st/stm32n6570dk/
```

If the board doesn't exist, you may need to:
1. Use a custom board definition in this project
2. Or use the generic `nucleo_n657x0q` board target

## Alternative: Docker Build Environment

```bash
# Use official Zephyr Docker image
docker pull zephyrprojectrtos/ci:latest

docker run -ti -v $(pwd):/workspace \
  zephyrprojectrtos/ci:latest /bin/bash

# Inside container:
cd /workspace/xi640-eth
west build -b stm32n6570dk -p auto
```

## Troubleshooting

### Board Not Found
If `stm32n6570dk` is not found, try:
- `nucleo_n657x0q` (similar STM32N6 board)
- Create custom board definition in `boards/arm/stm32n6570dk/`

### MVE Support Issues
Ensure Zephyr SDK version supports ARM Helium MVE:
- Minimum Zephyr version: 3.4.0+
- GCC toolchain: 12.0+

### Memory Issues
If build fails due to memory constraints, adjust in `prj.conf`:
```
CONFIG_HEAP_MEM_POOL_SIZE=65536
CONFIG_MAIN_STACK_SIZE=8192
```

## Performance Validation

Once flashed, connect serial console (115200 baud):
```bash
picocom -b 115200 /dev/ttyUSB0
```

Expected output:
```
Xi 640 ETH - Complete Pipeline Test
...
✅ Target 128 FPS maintained! (386.1 FPS)
```

## Next Steps

- Real sensor integration (Xi 640 ETH via SPI/Ethernet)
- Network streaming over UDP
- Visualization client application
