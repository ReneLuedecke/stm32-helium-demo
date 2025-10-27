# STM32CubeN6 MCU Firmware Package

![latest tag](https://img.shields.io/github/v/tag/STMicroelectronics/STM32CubeN6.svg?color=brightgreen)

> [!IMPORTANT]
> This repository has been created using the `git submodule` command. Please refer to the ["How to use"](README.md#how-to-use) section for more details.

## Overview

**STM32Cube** is an STMicroelectronics original initiative to ease the developers' life by reducing efforts, time and cost.

**STM32Cube** covers the overall STM32 products portfolio. It includes a comprehensive embedded software platform delivered for each STM32 series.
   * The CMSIS modules (core and device) corresponding to the Arm(tm) core implemented in this STM32 product.
   * The STM32 HAL-LL drivers, an abstraction layer offering a set of APIs ensuring maximized portability across the STM32 portfolio.
   * The BSP drivers of each evaluation, discovery or nucleo board provided for this STM32 series.
   * A consistent set of middleware libraries such as RTOS, USB, File System, Graphics, Touch Sensing library...
   * A full set of software projects (basic examples, applications, and demonstrations) for each board provided for this STM32 series.

The **STM32CubeN6 MCU Package** projects are directly running on the STM32N6 series boards. You can find in each Projects/*Board name* directories a set of software projects (Applications/Demonstration/Examples).

## Release note

Details about the content of this release are available in the release note [here](https://htmlpreview.github.io/?https://github.com/STMicroelectronics/STM32CubeN6/blob/main/Release_Notes.html).

## How to use

This repository has been created using the `git submodule` command. Please check the instructions below for proper use. Please check also **the notes at the end of this section** for further information.

1. To **clone** this repository along with the linked submodules, option `--recursive` has to be specified as shown below.

```bash
git clone --recursive https://github.com/STMicroelectronics/STM32CubeN6.git
```

2. To get the **latest updates**, in case this repository is **already** on your local machine, issue the following **two** commands (with this repository as the **current working directory**).

```bash
git pull
git submodule update --init --recursive
```

3. To use the **same firmware version** as the one available on [st.com](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html), issue the command below **after** specifying the targeted `vX.Y.Z` version. This should be done **after** the command(s) indicated in instruction (1) or in instruction (2) above have been successfully executed.

```bash
git checkout vX.Y.Z # Specify the targeted vX.Y.Z version
```

4. To **avoid** going through the above instructions and **directly** clone the same firmware version as the one available on [st.com](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html), issue the command below **after** specifying the targeted `vX.Y.Z` version.

```bash
git clone --recursive  --depth 1 --branch vX.Y.Z https://github.com/STMicroelectronics/STM32CubeN6.git
```

> [!NOTE]
> * The latest version of this firmware available on GitHub may be **ahead** of the one available on [st.com](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html) or via [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html). This is due to the **rolling release** process deployed on GitHub. Please refer to [this](https://github.com/STMicroelectronics/STM32Cube_MCU_Overall_Offer/discussions/21) post for more details.
> * Option `--depth 1` specified in instruction (4) above is **not** mandatory. It may be useful to skip downloading all previous commits up to the one corresponding to the targeted version.
> * If GitHub "Download ZIP" option is used instead of the `git clone` command, then the different submodules have to be collected and added **manually**.

## Boards available

  * STM32N6
    * [STM32N6570-DK](https://www.st.com/en/evaluation-tools/stm32n6570-dk.html)  
    * [NUCLEO-N657X0-Q](https://www.st.com/en/evaluation-tools/nucleo-n657x0-q.html)

## Troubleshooting

Please refer to the [CONTRIBUTING.md](CONTRIBUTING.md) guide.


✓ Professional UTF-8 Output formatieren
✓ Test Modes implementieren (Normal/Verbose/Benchmark/Debug)
✓ Pipeline Verification Output für Thomas
✓ Memory Map Visualization
✓ Performance Breakdown Tables
→ Macht den Code präsentierbar!
```

### **2. Gain Calibration Fix** ⚠️ WICHTIG
```
Problem: Gain = 0.173 konstant, sollte variieren
Todo:
  - Echte Gain-Kalibrierung durchführen
  - Per-Pixel Gain statt konstant
  - In XSPI Flash schreiben
  - Verifizieren mit verschiedenen Temperaturen
```

### **3. Testing & Validation**
```
✓ Verschiedene Szenen testen (nicht nur 56°C uniform)
✓ Bewegung im Bild testen
✓ Temperaturbereich validieren (0-100°C)
✓ Bad Pixel Correction verifizieren
✓ ROI Bounds Edge Cases testen
```

---

## 🚀 MITTELFRISTIG (Nächste 2 Wochen)

### **4. Real Sensor Integration** ⭐⭐ MAJOR
```
Aktuell: Dummy frame_buffer_A Daten
Todo:
  - DCMIPP Kamera Interface integrieren
  - DMA Setup für echte Sensor Daten
  - Frame Synchronisation
  - Test mit echter Thermal Kamera
  
Sensoren zur Auswahl:
  - FLIR Lepton 3.5
  - Seek Thermal Compact
  - MLX90640 (32×24, günstig für Test)
```

### **5. UART Control Interface**
```
Todo:
  - Commands für Runtime Config:
    • "ROI0 ON/OFF" - ROI aktivieren/deaktivieren
    • "ROI5 100,100,200,200" - ROI Bounds setzen
    • "MODE VERBOSE" - Test Mode wechseln
    • "CALIB DARK" - Kalibrierung starten
    • "CALIB GAIN" - Gain Kalibrierung
    • "STATUS" - System Status ausgeben
  
  - Binary Protocol für High-Speed:
    • Frame data streaming
    • ROI results streaming
    • Low latency control
```

### **6. Advanced ROI Features**
```
✓ Basis ROI Statistics (Min/Max/Mean) ✓
Todo:
  - Temperature Alarms (wenn ROI > Threshold)
  - ROI Histogramme (Temperaturverteilung)
  - ROI Tracking (Moving average over time)
  - ROI Change Detection (Delta zur letzten Frame)
  - Adaptive ROI (Auto-move zu hottest spot)
```

---

## 🎨 MITTELFRISTIG (Nächste 2 Wochen)

### **7. Display Output**
```
Todo:
  - HDMI/LCD Display Integration
  - False-color Temperature Mapping
  - ROI Overlay Visualization
  - Real-time FPS Counter
  - Crosshair / Grid Overlay
  
Libraries:
  - TouchGFX für GUI
  - LTDC für Display Controller
  - DMA2D für Beschleunigung
```

### **8. Recording & Logging**
```
Todo:
  - Frame Recording zu SD Card
  - CSV Export von ROI Daten
  - Temperature Time-Series Logging
  - Event Logging (Alarms, Calibrations)
  - Playback Mode für aufgezeichnete Daten
  
Format:
  - Raw frames (binary)
  - Compressed (H.264 für Video)
  - Metadata (timestamps, ROI configs)
```

### **9. Multi-Frame Processing**
```
Todo:
  - Temporal Averaging (Noise Reduction)
  - Frame Differencing (Motion Detection)
  - Bad Pixel Detection (automatisch)
  - Scene Auto-Calibration
  - Dynamic Range Extension
```

---

## 🔬 LANGFRISTIG (Nächste 4 Wochen)

### **10. Advanced Calibration**
```
Todo:
  - Non-Uniformity Correction (NUC)
  - Two-Point Calibration
  - Multi-Temperature Reference
  - Automatic Bad Pixel Detection
  - Temperature Drift Compensation
  - Lens Distortion Correction
```

### **11. Networking**
```
Todo:
  - Ethernet Integration
  - UDP Stream für Frames
  - TCP Control Protocol
  - Web Interface (embedded HTTP server)
  - REST API für Remote Control
  - MQTT für IoT Integration
```

### **12. AI/ML Features**
```
Todo:
  - Object Detection (hot spots)
  - Temperature Anomaly Detection
  - Predictive Maintenance Alerts
  - Pattern Recognition
  - Integration mit STM32Cube.AI
```

### **13. Power Optimization**
```
Todo:
  - Low-Power Modes
  - Dynamic Clock Scaling
  - ROI Update Rate Optimization
  - Sleep Mode zwischen Frames
  - Power Profiling & Optimization
```

---

## 🐛 BUGFIXES & OPTIMIZATIONS

### **14. Bekannte Issues**
```
⚠️ Gain Calibration = 0.173 konstant (sollte variieren)
⚠️ Alle Temperaturen = 56.82°C (sehr uniform, Test mit Bewegung)
⚠️ UTF-8 Box Characters zeigen falsch in manchen Terminals
⚠️ Keine Fehlerbehandlung bei XSPI Write Failure
⚠️ Kein Watchdog implementiert
```

### **15. Performance Tuning**
```
Aktuell: 100 Hz (9 ms)
Potential:
  - Helium weiter optimieren → 120 Hz?
  - ROI parallel processing → 150 Hz?
  - Assembly Hot Path → 200 Hz?
  - GPU/DMA Offloading → 300 Hz?
```

---

## 📚 DOKUMENTATION

### **16. Code Documentation**
```
Todo:
  - Doxygen Comments für alle Functions
  - README.md mit Setup Instructions
  - Architecture Diagrams (Pipeline, Memory Map)
  - API Documentation
  - Calibration Guide
  - Troubleshooting Guide
```

### **17. Performance Report**
```
Todo:
  - Detaillierter Performance Comparison mit Thomas
  - Memory Profiling Report
  - Power Consumption Analysis
  - Thermal Tests (CPU Temperature)
  - Long-term Stability Test (24h+)
```

---

## 🎓 LEARNING & RESEARCH

### **18. Optimierungen Research**
```
Research Topics:
  - Alternative ROI Algorithms
  - GPU Acceleration (LTDC/DMA2D)
  - SIMD weitere Optimierungen
  - Cache Optimization Techniques
  - Real-time OS Integration (FreeRTOS)
```

### **19. Standards & Compliance**
```
Research:
  - Thermal Imaging Standards
  - Temperature Accuracy Requirements
  - Safety Certifications
  - EMC Testing
  - Industrial Protocols (Modbus, etc.)
```

---

## 🎯 PRIORISIERTE ROADMAP

### **DIESE WOCHE:**
1. ✅ Code Cleanup mit Claude Code (Output Formatting)
2. ⚠️ Gain Calibration Fix
3. ✅ Testing mit verschiedenen Szenen

### **NÄCHSTE WOCHE:**
4. 🔴 Real Sensor Integration (MAJOR)
5. 🟡 UART Control Interface
6. 🟡 Advanced ROI Features (Alarms)

### **WOCHE 3-4:**
7. 🟢 Display Output
8. 🟢 Recording & Logging
9. 🟢 Multi-Frame Processing

### **MONAT 2:**
10. 🔵 Advanced Calibration
11. 🔵 Networking
12. 🔵 AI/ML Features

---

## ❓ WELCHE PRIORITÄT FÜR DICH?

**Optionen:**

**A) Profesionalisierung (Quick Wins):**
```
1. Code Cleanup mit Claude Code
2. UART Interface für Remote Control
3. Better Testing & Validation
→ Macht System sofort nutzbar!
```

**B) Hardware Integration (Real Data):**
```
1. Real Sensor Integration (DCMIPP)
2. Display Output
3. Recording zu SD Card
→ Komplettes System mit echter Kamera!
```

**C) Performance & Features (Advanced):**
```
1. Advanced ROI Features (Alarms, Tracking)
2. Multi-Frame Processing
3. AI/ML Integration
→ Cutting-edge Features!
```

**D) Production-Ready (Enterprise):**
```
1. Networking (Ethernet/Web Interface)
2. Advanced Calibration
3. Documentation & Compliance
→ Marktreifes Produkt!