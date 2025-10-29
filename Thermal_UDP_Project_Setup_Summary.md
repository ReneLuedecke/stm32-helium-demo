# Thermal UDP Project Setup Summary

## 1. Newly Created Files
- Tools/thermal_udp_receiver.py
- Tools/test_thermal_udp.sh
- Tools/test_thermal_udp.bat
- Projects/STM32N6570-DK/Applications/Thermal/Thermal_UDP_Streaming/README.md

## 2. Updated / Existing Files
- Projects/STM32N6570-DK/Applications/Thermal/Thermal_UDP_Streaming/** (project sources, linker, ThreadX/NetX integration, thermal processing)
- Tools/ (added receiver + scripts)
- .gitignore (ignore build & Python artifacts)

## 3. Next Steps
1. Build project in STM32CubeIDE or via headless command (see README).
2. Flash Thermal_UDP_Streaming.hex onto STM32N6570-DK.
3. Run Python receiver (Tools/thermal_udp_receiver.py) or helper script to confirm ~50 FPS stream.

## 4. Manual Verification Checklist
- [ ] CubeIDE build succeeds without unresolved symbols.
- [ ] UART output shows TIM2, Thermal and UDP thread bring-up.
- [ ] Python receiver logs continuous frames with <1 % loss.
- [ ] XSPI calibration load succeeds ([Thermal] Calibration loaded successfully).
- [ ] TIM2 interrupt counts at 50 Hz (visible via LED orange blink / logs).

## 5. Known TODOs / Follow-Ups
- TIM2/XSPI pin mux still manual; update .ioc project when ready.
- Optional double-buffering (see Integration Plan §8.3) kept for future packet-loss mitigation.
- Integrate automated tests (CI) once hardware-in-the-loop pipeline defined.
