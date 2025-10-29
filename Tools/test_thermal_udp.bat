@echo off
SETLOCAL ENABLEEXTENSIONS

set "SCRIPT_DIR=%~dp0"
if "%SCRIPT_DIR:~-1%"=="\" set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
set "LOG_FILE=%SCRIPT_DIR%\thermal_test.log"

echo Starting Thermal UDP Test...>>"%LOG_FILE%"
echo 1. Flash STM32N6570-DK>>"%LOG_FILE%"
echo 2. Start Python receiver>>"%LOG_FILE%"
echo 3. Monitor UART output>>"%LOG_FILE%"

pushd "%SCRIPT_DIR%"

echo [Test] Installing Python dependencies (numpy, matplotlib)...>>"%LOG_FILE%"
python -m pip install numpy matplotlib>>"%LOG_FILE%" 2>&1

echo [Test] Launching receiver in headless mode...>>"%LOG_FILE%"
start "Thermal UDP Receiver" /MIN cmd /c "python thermal_udp_receiver.py --headless --save-frames --frames 100 --output-dir "%SCRIPT_DIR%" >>"%LOG_FILE%" 2>&1"
set "RECEIVER_PID="
for /f "tokens=2" %%P in ('tasklist /FI "WINDOWTITLE eq Thermal UDP Receiver" ^| find "cmd.exe"') do set "RECEIVER_PID=%%P"

if not defined RECEIVER_PID (
    echo [Test] WARNING: Could not determine receiver PID.>>"%LOG_FILE%"
    timeout /T 30 /NOBREAK >NUL
) else (
    echo [Test] Waiting for frames (30s)...>>"%LOG_FILE%"
    timeout /T 30 /NOBREAK >NUL
    echo [Test] Stopping receiver (PID: %RECEIVER_PID%)...>>"%LOG_FILE%"
    taskkill /PID %RECEIVER_PID% /F>>"%LOG_FILE%" 2>&1
)

echo Test complete. Check logs in %LOG_FILE%>>"%LOG_FILE%"

echo Test complete. Check logs in %LOG_FILE%

popd
ENDLOCAL
