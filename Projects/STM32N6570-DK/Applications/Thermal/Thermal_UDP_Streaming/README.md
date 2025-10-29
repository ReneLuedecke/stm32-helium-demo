# Thermal UDP Streaming (STM32N6570-DK)

Thermal_UDP_Streaming demonstriert eine 50 Hz-Thermalkamera-Pipeline auf dem STM32N6570-DK:
TIM2 triggert die Planck-kalibrierte Helium-Verarbeitung, das Ergebnis wird per UDP (439 Pakete à 1400 Bytes) ins LAN gestreamt. Vorlage war das `VENC_RTSP_Server` Beispiel – RTSP wurde entfernt, Thermal- und UDP-Threads ersetzen den Video-Encoder.

## Hardware-Anforderungen

- STM32N6570-DK mit XSPI-NOR-Kalibrierungsflash
- ST-LINK V3 (Onboard) + USB-C Versorgung
- Ethernet-Verbindung (DHCP-fähiges Netz empfohlen)
- Terminal/Serial-Adapter (UART1 @ 115200 Bd) für Debuglogs

## Software-Anforderungen

- STM32CubeIDE 1.15+ mit STM32N6 Firmware Package
- ARM GNU Toolchain (bereitgestellt durch CubeIDE)
- Python 3.9+ mit `numpy` und `matplotlib` (für den UDP Receiver)
- STLink CLI (`STM32_Programmer_CLI`) zum Flashen

## Build-Anleitung

### STM32CubeIDE (GUI)
1. `STM32CubeIDE` starten und Workspace `ide_ws/` öffnen.
2. Projekt importieren: `File > Import > Existing Projects into Workspace`, Ordner `Projects/STM32N6570-DK/Applications/Thermal/Thermal_UDP_Streaming/STM32CubeIDE` auswählen.
3. Projekt bauen (`Project > Build Project`). Artefakte erscheinen unter `STM32CubeIDE/Appli/Debug/`.

### Headless Build (Linux/Windows)
```bash
STM32CubeIDE -nosplash \
  -application org.eclipse.cdt.managedbuilder.core.headlessbuild \
  -data ide_ws \
  -importAll Projects/STM32N6570-DK/Applications/Thermal/Thermal_UDP_Streaming/STM32CubeIDE \
  -build all
```

## Flash-Anleitung

1. ST-LINK verbinden, Board im normalen Modus starten.
2. Generiertes Binary flashen:
   ```bash
   STM32_Programmer_CLI -c port=SWD \
     -w Projects/STM32N6570-DK/Applications/Thermal/Thermal_UDP_Streaming/STM32CubeIDE/Appli/Debug/Thermal_UDP_Streaming.hex
   ```
3. Reset auslösen, UART-Monitor öffnen (115200 Bd, 8N1).

## Test-Anleitung

1. Python-Abhängigkeiten installieren:
   ```bash
   cd Tools
   python3 -m pip install --user numpy matplotlib
   ```
2. Receiver starten:
   ```bash
   python3 thermal_udp_receiver.py
   # oder für automatisierte Tests:
   python3 thermal_udp_receiver.py --headless --frames 100 --save-frames --output-dir Tools
   ```
3. Alternativ die Hilfsskripte aus `Tools/` verwenden:
   - Linux/macOS: `./test_thermal_udp.sh`
   - Windows: `test_thermal_udp.bat`
4. Erwartete UART-Ausgabe:
   ```
   [UDP] Thread started (Prio 8)
   [UDP] Socket bound to port 6100
   [UDP] Frame size: 614400 bytes (439 packets @ 1400 bytes)
   [Thermal] Frame 50: 13.7 ms (8234567 cycles)
   ```
5. Python-Receiver sollte ~50 FPS und 0 % Packet-Loss melden.

## Troubleshooting

- **Keine Frames am PC:** Prüfen, ob Board via DHCP eine IP bekommen hat (`[NetX] Network ready`). Ggf. statische IP konfigurieren.
- **UDP-Drops > 0 %:** Netzwerklast reduzieren, Gigabit-Switch verwenden, optional Double-Buffering (siehe Integration Plan §8.3).
- **Kalibrierung fehlschlägt:** XSPI-Flash-Inhalt prüfen (`load_calibration_data()` gibt Fehler auf UART aus).
- **Matplotlib fehlt/kein Display:** `--headless` verwenden oder Backend `Agg` setzen (z. B. `MPLBACKEND=Agg`).
- **TIM2 Trigger läuft nicht:** TIM2_CLK und NVIC-Konfiguration in `stm32n6xx_hal_msp.c` sowie `HAL_TIM_Base_Start_IT` in `main.c` prüfen.

## Verzeichnisstruktur

- `Appli/Core` – ThreadX, main, Interrupts, TIM2
- `Appli/NetXDuo` – DHCP + UDP Streaming Thread
- `Appli/Thermal` – Helium-optimierter Thermal-Stack + XSPI
- `STM32CubeIDE` – Projektdateien, Linkerskript
- `Tools` – Python Receiver & Testskripte

Weitere Details befinden sich im `Thermal_UDP_Streaming_Integration_Plan.md`.
