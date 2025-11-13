#!/bin/bash
#
# Xi 640 ETH - Native PC Test Setup Script
# Automatically sets up and runs native simulation
#

set -e  # Exit on error

echo "╔════════════════════════════════════════════════════════════╗"
echo "║   Xi 640 ETH - Native PC Test Setup                       ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Check if west is installed
if ! command -v west &> /dev/null; then
    echo "❌ West not found. Installing..."
    pip3 install --user west
    export PATH=$HOME/.local/bin:$PATH
else
    echo "✓ West found: $(which west)"
fi

# Check if Zephyr SDK is installed
if [ -z "$ZEPHYR_SDK_INSTALL_DIR" ]; then
    echo ""
    echo "⚠️  Zephyr SDK not found in environment."
    echo ""
    echo "Please install Zephyr SDK:"
    echo "  1. Download from: https://github.com/zephyrproject-rtos/sdk-ng/releases"
    echo "  2. Extract: tar xvf zephyr-sdk-*.tar.xz"
    echo "  3. Setup: cd zephyr-sdk-* && ./setup.sh -h"
    echo "  4. Set env: export ZEPHYR_SDK_INSTALL_DIR=/path/to/zephyr-sdk"
    echo ""
    read -p "Press Enter to continue anyway (may fail) or Ctrl+C to exit..."
else
    echo "✓ Zephyr SDK: $ZEPHYR_SDK_INSTALL_DIR"
fi

# Initialize west workspace if not already done
if [ ! -d ".west" ]; then
    echo ""
    echo "[1/4] Initializing west workspace..."
    west init -l .

    echo "[2/4] Updating Zephyr and modules (this may take a while)..."
    west update
else
    echo ""
    echo "✓ West workspace already initialized"
fi

# Build native simulation
echo ""
echo "[3/4] Building for native_sim (PC simulation)..."
west build -b native_sim -p auto -- -DCONF_FILE=prj_native.conf

if [ $? -eq 0 ]; then
    echo ""
    echo "╔════════════════════════════════════════════════════════════╗"
    echo "║ ✅ BUILD SUCCESSFUL!                                       ║"
    echo "╚════════════════════════════════════════════════════════════╝"
    echo ""
    echo "Ready to run! Options:"
    echo ""
    echo "  1. Run now:           ./build/zephyr/zephyr.exe"
    echo "  2. Run with west:     west build -t run"
    echo "  3. Debug:             gdb ./build/zephyr/zephyr.exe"
    echo ""

    read -p "Run now? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo ""
        echo "[4/4] Starting native simulation..."
        echo ""
        echo "Press Ctrl+C to exit"
        echo "════════════════════════════════════════════════════════════"
        echo ""
        ./build/zephyr/zephyr.exe
    else
        echo ""
        echo "Build complete. Run manually when ready:"
        echo "  ./build/zephyr/zephyr.exe"
    fi
else
    echo ""
    echo "❌ Build failed. Check errors above."
    exit 1
fi
