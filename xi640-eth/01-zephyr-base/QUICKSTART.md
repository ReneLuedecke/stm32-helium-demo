# 🚀 QUICKSTART - In 5 Minuten loslegen!

Teste die Xi 640 ETH Thermal Camera **OHNE Hardware** auf deinem PC!

---

## ⚡ Option 1: Automatisches Setup (Empfohlen)

```bash
cd xi640-eth/01-zephyr-base
./setup_native_test.sh
```

**Das Script macht alles automatisch:**
- ✓ Prüft Dependencies
- ✓ Initialisiert West Workspace
- ✓ Lädt Zephyr herunter
- ✓ Baut native Simulation
- ✓ Startet die Anwendung

---

## ⚡ Option 2: Makefile (Noch schneller)

```bash
cd xi640-eth/01-zephyr-base

# Workspace initialisieren (nur beim ersten Mal)
make init

# Bauen und sofort starten
make run-native
```

**Fertig!** Die Thermal Camera läuft jetzt auf deinem PC!

---

## ⚡ Option 3: Docker (Komplett isoliert)

```bash
cd xi640-eth/01-zephyr-base

# Container bauen und starten
make docker-run

# ODER mit docker-compose
docker-compose run --rm zephyr-dev

# Im Container:
west init -l .
west update
west build -b native_sim
./build/zephyr/zephyr.exe
```

---

## ⚡ Option 4: Manuell (Volle Kontrolle)

```bash
# 1. West installieren
pip3 install --user west

# 2. Zum Projekt navigieren
cd xi640-eth/01-zephyr-base

# 3. Workspace initialisieren
west init -l .
west update

# 4. Native Simulation bauen
west build -b native_sim -p auto

# 5. Starten!
./build/zephyr/zephyr.exe
```

---

## 📺 Was du sehen solltest

```
╔════════════════════════════════════════════════════════════╗
║           Xi 640 ETH Thermal Camera System                ║
║        SESSION 1: Zephyr Base + Frame Generator           ║
║                                                            ║
║  Board:    STM32N6570-DK @ 150 MHz                        ║
║  RTOS:     Zephyr 3.7 LTS                                 ║
║  Sensor:   640×480 @ 14-bit (synthetic mode)              ║
║  Target:   50 FPS baseline                                ║
╚════════════════════════════════════════════════════════════╝

[1/3] Initializing memory manager...
Frame pool allocated: 3 frames × 614656 bytes = 1800 KB
✓ Memory manager ready

[2/3] Initializing thermal frame generator...
Thermal frame generator initialized (pattern=0)
✓ Thermal generator ready (pattern=0)

[3/3] Starting capture thread...
Capture thread started (target: 50 FPS)
✓ Capture thread started

╔════════════════════════════════════════════════════════════╗
║ SYSTEM READY - Generating thermal frames @ 50 FPS         ║
╚════════════════════════════════════════════════════════════╝

╔════════════════════════════════════════════════════════════╗
║ Xi 640 ETH - PERFORMANCE STATISTICS                       ║
╠════════════════════════════════════════════════════════════╣
║ Frames generated:            250                          ║
║ Current FPS:                  50 (target: 50)             ║
║                                                            ║
║ Frame Generation Time:                                     ║
║   Average:                  3250 µs                       ║
║   Min:                      3100 µs                       ║
║   Max:                      3500 µs                       ║
║                                                            ║
║ Thermal Data (ADC 14-bit):                                 ║
║   Min value:              3276 / 16383                    ║
║   Max value:             13107 / 16383                    ║
║   Average:                 8191                           ║
╚════════════════════════════════════════════════════════════╝

=== MEMORY MAP ===
Region       Base        Size      Used      Free      Usage
----------------------------------------------------------------
AXISRAM1     0x24000000  1048576        0  1048576    0%
AXISRAM2     0x24100000  1048576        0  1048576    0%
AXISRAM3     0x24200000  1048576        0  1048576    0%
HYPERRAM     0x90000000  67108864  1843968  65264896    2%

Frame Pool: 3 frames × 614656 bytes (HyperRAM)
  Capture idx: 1
  Process idx: 0
  Display idx: 0
  Locks: [0 1 0]
================================================================
```

---

## 🎨 Test Patterns ausprobieren

```bash
# Gradient (default)
make pattern-gradient

# Hot Spot
make pattern-hotspot

# Checkerboard
make pattern-checker
```

---

## 🔧 Troubleshooting

### "west: command not found"
```bash
pip3 install --user west
export PATH=$HOME/.local/bin:$PATH
```

### "Zephyr SDK not found"
Du brauchst das Zephyr SDK nur für Hardware-Builds. Für native Simulation reicht dein normaler GCC!

Falls du es trotzdem installieren willst:
```bash
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.8/zephyr-sdk-0.16.8_linux-x86_64.tar.xz
tar xvf zephyr-sdk-0.16.8_linux-x86_64.tar.xz
cd zephyr-sdk-0.16.8
./setup.sh -h
```

### "Build failed"
```bash
# Clean und neu bauen
make clean
make build-native
```

---

## 🎯 Was du getestet hast

✅ Zephyr RTOS läuft auf deinem PC
✅ Thermal Frame Generator funktioniert
✅ Memory Management (AXISRAM/HyperRAM)
✅ Triple-buffered Frame Pool
✅ Performance Monitoring
✅ 50 FPS @ 640×480 @ 14-bit

**Bereit für Session 2:** Helium MVE SIMD Processing (7× Speedup)! 🚀

---

## 📚 Mehr Infos

- [Vollständige Test-Anleitung](TEST_GUIDE.md)
- [Projekt README](README.md)
- [Zephyr Docs](https://docs.zephyrproject.org/)

---

**Fragen?** Schau in [TEST_GUIDE.md](TEST_GUIDE.md) oder das Haupt-[README.md](README.md)!
