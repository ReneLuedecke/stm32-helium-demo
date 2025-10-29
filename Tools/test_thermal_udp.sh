#!/bin/bash
# Thermal UDP streaming test helper (Integration Plan lines 2186-2210)

set -euo pipefail

if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
  SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
fi

LOG_FILE="$SCRIPT_DIR/thermal_test.log"

{
  echo "Starting Thermal UDP Test..."
  echo "1. Flash STM32N6570-DK"
  echo "2. Start Python receiver"
  echo "3. Monitor UART output"

  echo "[Test] Installing Python dependencies (numpy, matplotlib)..."
  cd "$SCRIPT_DIR"
  python3 -m pip install --user numpy matplotlib

  echo "[Test] Launching receiver in headless mode..."
  python3 thermal_udp_receiver.py --headless --save-frames --frames 100 --output-dir "$SCRIPT_DIR" &
  RECEIVER_PID=$!

  echo "[Test] Waiting for frames (30s)..."
  sleep 30

  echo "[Test] Stopping receiver (PID: $RECEIVER_PID)..."
  kill "$RECEIVER_PID" 2>/dev/null || true
  wait "$RECEIVER_PID" 2>/dev/null || true

  echo "Test complete. Check logs in $LOG_FILE"
} | tee -a "$LOG_FILE"
