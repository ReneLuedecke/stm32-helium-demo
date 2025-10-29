# STM32N6 Projekt-Analyse: Thermal Camera Demo & UDP Echo Server

**Erstellt:** 2025-10-28
**Analyseumfang:** Detaillierter Vergleich zweier STM32N6-Projekte zur Vorbereitung einer möglichen Integration

---

## Inhaltsverzeichnis

1. [Executive Summary](#executive-summary)
2. [Projekt 1: Thermal Camera Demo (CORTEX_HELIUM)](#projekt-1-thermal-camera-demo-cortex_helium)
3. [Projekt 2: UDP Echo Server (Nx_UDP_Echo_Server)](#projekt-2-udp-echo-server-nx_udp_echo_server)
4. [Vergleichsanalyse](#vergleichsanalyse)
5. [Kompatibilitätsbewertung](#kompatibilitätsbewertung)
6. [Integrationsstrategie](#integrationsstrategie)
7. [Empfehlungen](#empfehlungen)

---

## Executive Summary

### Kernbefunde

- **Projekt 1 (Thermal Camera):** Bare-metal Echtzeitanwendung mit ARM Helium SIMD-Optimierung für thermische Bildverarbeitung (640×480 @ 50Hz)
- **Projekt 2 (UDP Echo Server):** Azure RTOS-basierte Netzwerkanwendung mit ThreadX und NetX Duo Stack
- **Hardware-Kompatibilität:** Beide Projekte nutzen STM32N6-Familie, jedoch unterschiedliche Boards (NUCLEO-N657X0-Q vs. STM32N6570-DK)
- **Hauptkonflikt:** ThreadX RTOS vs. Bare-Metal Architektur

### Integrationskomplexität: **HOCH**

**Grund:** Fundamentale Architekturunterschiede zwischen RTOS-basiertem Netzwerk-Stack und zeitkritischer Bare-Metal Bildverarbeitung

---

## Projekt 1: Thermal Camera Demo (CORTEX_HELIUM)

### 1.1 Übersicht

| **Parameter** | **Details** |
|---------------|-------------|
| **Projektpfad** | `Projects/NUCLEO-N657X0-Q/Examples/CORTEX/CORTEX_HELIUM` |
| **Hauptboard** | NUCLEO-N657X0-Q |
| **MCU** | STM32N657X0HXQ @ 600 MHz |
| **Architektur** | Bare-Metal (ohne RTOS) |
| **Hauptfunktion** | Echtzeit-Thermal-Imaging-Pipeline mit Helium-Optimierung |
| **Build-System** | STM32CubeIDE |

### 1.2 Hauptfunktionen (main.c)

#### Kernfunktionalität

```c
// Hauptdateien
- Projects/NUCLEO-N657X0-Q/Examples/CORTEX/CORTEX_HELIUM/FSBL/Src/main.c (Zeilen 1-100+)
- Projects/NUCLEO-N657X0-Q/Examples/CORTEX/CORTEX_HELIUM/FSBL/Inc/main.h
```

**Thermal Processing Pipeline:**

1. **Kalibrierung:**
   - `start_dark_frame_calibration()` - Dunkelbildkalibrierung (16 Frames gemittelt)
   - `process_calibration_frame()` - Frame-Akkumulation
   - Speicherung in XSPI Flash via `XSPI_WriteGainFrame()`

2. **Frame-Verarbeitung (50 Hz):**
   - `thermal_frame_process()` - Hauptverarbeitungsloop
   - `process_thermal_line_fastest()` - Helium-optimierte Zeilenverarbeitung (8 Pixel/Iteration)
   - Dunkelbildsubtraktion, Gain/Offset-Korrektur (Q15 Fixed-Point)
   - Planck-LUT Temperaturkonvertierung

3. **Bad Pixel Correction:**
   - `apply_bad_pixel_patches()` - Ersetzt defekte Pixel durch Interpolation

#### Datenstrukturen

```c
// Wichtige Typen (main.c:34-76)
typedef struct {
    float min_temp_c;
    float max_temp_c;
    float mean_temp_c;
    uint32_t pixel_count;
} ROI_Stats_Celsius_t;

typedef struct {
    uint16_t raw_input;
    uint16_t after_dark;
    uint16_t after_gain;
    // ... weitere Pipelinestadien
    float temp_celsius;
} Pipeline_Verification_t;
```

### 1.3 Hardware-Konfiguration

#### Verwendete Peripherals

| **Peripheral** | **Funktion** | **Konfiguration** |
|----------------|--------------|-------------------|
| **TIM2** | Frame-Trigger (VSYNC-Simulation) | Prescaler: 9999, Period: 799 → 50 Hz |
| **UART1** | Debug-Ausgabe | 115200 baud, 8N1 |
| **XSPI2** | NOR Flash (Kalibrierungsdaten) | Octal DTR Mode, Memory-Mapped @ 0x70000000 |
| **GPIO** | LED1 (GPIOG Pin 8) | Prozessierungsindikator |
| **DWT** | Cycle Counter | Performance-Messung |

#### HAL-Module (stm32n6xx_hal_conf.h:82-96)

```c
#define HAL_TIM_MODULE_ENABLED     // Timer
#define HAL_UART_MODULE_ENABLED    // UART
#define HAL_XSPI_MODULE_ENABLED    // XSPI
#define HAL_GPIO_MODULE_ENABLED    // GPIO
#define HAL_DMA_MODULE_ENABLED     // DMA (nicht aktiv genutzt)
#define HAL_RCC_MODULE_ENABLED     // Clock-Konfiguration
#define HAL_PWR_MODULE_ENABLED     // Power-Management
#define HAL_CORTEX_MODULE_ENABLED  // NVIC, Systick
```

### 1.4 Speicherlayout

#### Linker-Script: `STM32N657X0HXQ_AXISRAM2_fsbl.ld`

```ld
MEMORY
{
  ROM      (xrw) : ORIGIN = 0x341d0400, LENGTH = 255K   // Code + RO Data
  RAM      (xrw) : ORIGIN = 0x34000000, LENGTH = 2048K  // Hauptspeicher (AXISRAM2)
  AXISRAM1 (xrw) : ORIGIN = 0x24060000, LENGTH = 1024K  // Planck LUT (128 KB)
  DTCM     (xrw) : ORIGIN = 0x20000000, LENGTH = 128K   // Stack/Heap
}

_Min_Heap_Size = 0x200;   // 512 Bytes
_Min_Stack_Size = 0x800;  // 2 KB
```

#### Speicheraufteilung (aus CLAUDE.md)

| **Region** | **Größe** | **Verwendung** |
|------------|-----------|----------------|
| **AXISRAM1** | 1 MB | Planck-LUT (128 KB, 16-bit × 65536 Einträge) |
| **AXISRAM2** | 1 MB | Gain Frame, Dark Frame, Offset Map, aktueller Sensor-Frame |
| **AXISRAM3** | 1 MB | Bad Pixel Correction Arrays (Koordinaten + Ersatzwerte) |
| **XSPI2 Flash** | 512 Mbit | Persistente Kalibrierungsdaten (Memory-Mapped @ 0x70000000) |

### 1.5 Build-Konfiguration

#### STM32CubeIDE Projekt

```
Projektstruktur:
Projects/NUCLEO-N657X0-Q/Examples/CORTEX/CORTEX_HELIUM/
├── STM32CubeIDE/
│   └── FSBL/
│       ├── .project
│       ├── .cproject
│       └── STM32N657X0HXQ_AXISRAM2_fsbl.ld
├── FSBL/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── stm32n6xx_hal_conf.h
│   │   ├── stm32n6xx_it.h
│   │   └── xspi_nor.h
│   └── Src/
│       ├── main.c
│       ├── stm32n6xx_it.c
│       ├── stm32n6xx_hal_msp.c
│       └── xspi_nor.c
└── CORTEX_HELIUM.ioc
```

#### Compiler-Flags (typisch)

```bash
-mcpu=cortex-m55
-mfpu=fpv5-sp-d16  # Single-Precision FPU
-mfloat-abi=hard
-march=armv8.1-m.main+mve.fp+fp.dp  # Helium + Double-Precision FP
-O2  # Optimierung
```

#### Output

```
STM32CubeIDE/FSBL/Debug/CORTEX_HELIUM_FSBL.elf
STM32CubeIDE/FSBL/Debug/CORTEX_HELIUM_FSBL.hex
```

### 1.6 Wichtige Konstanten

```c
// main.c:30-57
#define HPIX 640                    // Thermal-Sensor-Auflösung (horizontal)
#define VPIX 480                    // Thermal-Sensor-Auflösung (vertikal)
#define PLANCK_C1 1.191042e8        // Erste Planck-Konstante
#define PLANCK_C2 1.4387752e4       // Zweite Planck-Konstante
#define XSPI1_MMAP_BASE 0x90000000  // XSPI1 Memory-Mapped-Region
#define XSPI2_MMAP_BASE 0x70000000  // XSPI2 Memory-Mapped-Region
#define TIM2_PRESCALER 9999         // Timer-Prescaler (100 kHz)
#define TIM2_PERIOD 799             // Timer-Period (50 Hz Frame-Trigger)
```

### 1.7 Interrupt-Handler (stm32n6xx_it.c)

```c
// Hauptinterrupts:
void TIM2_IRQHandler(void)   // Frame-Trigger (50 Hz VSYNC)
void HardFault_Handler(void) // Fehlerbehandlung
void SysTick_Handler(void)   // HAL Timebase (1 ms)

// Externe Variablen für Thermal-Pipeline:
extern TIM_HandleTypeDef htim2;
extern volatile uint32_t vsync_count;
extern volatile uint32_t frames_processed;
extern volatile uint8_t frame_ready;
```

---

## Projekt 2: UDP Echo Server (Nx_UDP_Echo_Server)

### 2.1 Übersicht

| **Parameter** | **Details** |
|---------------|-------------|
| **Projektpfad** | `ide_ws_thomas/Nx_UDP_Echo_Server` |
| **Hauptboard** | STM32N6570-DK (Discovery Kit) |
| **MCU** | STM32N657X0HXQ @ 600 MHz |
| **Architektur** | Azure RTOS (ThreadX + NetX Duo) |
| **Hauptfunktion** | UDP Echo Server mit DHCP und Ethernet |
| **Build-System** | STM32CubeIDE |

### 2.2 ThreadX Konfiguration

#### tx_user.h (Zeilen 159-184)

```c
// Aktivierte Optimierungen:
#define TX_DISABLE_PREEMPTION_THRESHOLD  // Reduziert Overhead
#define TX_DISABLE_NOTIFY_CALLBACKS      // Spart Code-Size

// Standard-Einstellungen:
// TX_MAX_PRIORITIES                32 (Default)
// TX_TIMER_TICKS_PER_SECOND       100 (Default)
// TX_MINIMUM_STACK                200 Bytes (Default)
```

#### Thread-Struktur (app_netxduo.c:48-62)

```c
// Definierte Threads:
TX_THREAD      NxAppThread;       // Hauptnetzwerk-Thread
TX_THREAD      AppUDPThread;      // UDP Server Thread
TX_THREAD      AppLinkThread;     // Link-Status-Überwachung

// Synchronisationsobjekte:
TX_SEMAPHORE   DHCPSemaphore;     // DHCP-Synchronisation

// Netzwerk-Ressourcen:
NX_PACKET_POOL NxAppPool;         // Packet-Pool für NetX
NX_IP          NetXDuoEthIpInstance;  // IP-Instanz
NX_DHCP        DHCPClient;        // DHCP-Client
NX_UDP_SOCKET  UDPSocket;         // UDP-Socket
```

### 2.3 NetX Duo Konfiguration

#### app_netxduo.c - Initialisierung (Zeilen 80-198)

```c
UINT MX_NetXDuo_Init(VOID *memory_ptr)
{
    // 1. NetX System-Initialisierung
    nx_system_initialize();

    // 2. Packet-Pool erstellen
    nx_packet_pool_create(&NxAppPool, "NetXDuo App Pool",
                          DEFAULT_PAYLOAD_SIZE, pointer,
                          NX_APP_PACKET_POOL_SIZE);

    // 3. IP-Instanz erstellen
    nx_ip_create(&NetXDuoEthIpInstance, "NetX Ip instance",
                 NX_APP_DEFAULT_IP_ADDRESS,
                 NX_APP_DEFAULT_NET_MASK,
                 &NxAppPool, nx_stm32_eth_driver, ...);

    // 4. Protokolle aktivieren
    nx_arp_enable(&NetXDuoEthIpInstance, ...);   // ARP
    nx_icmp_enable(&NetXDuoEthIpInstance);       // ICMP (Ping)
    nx_tcp_enable(&NetXDuoEthIpInstance);        // TCP
    nx_udp_enable(&NetXDuoEthIpInstance);        // UDP

    // 5. UDP-Thread erstellen
    tx_thread_create(&AppUDPThread, "App UDP Thread",
                     App_UDP_Thread_Entry, 0, pointer,
                     Nx_IP_INSTANCE_THREAD_SIZE,
                     NX_APP_INSTANCE_PRIORITY, ...);
}
```

#### Speicher-Pools (app_azure_rtos.c:53-66)

```c
// ThreadX Byte Pool
static UCHAR tx_byte_pool_buffer[TX_APP_MEM_POOL_SIZE];
static TX_BYTE_POOL tx_app_byte_pool;

// NetX Byte Pool (spezielle Sektion)
#pragma location = ".NetXPoolSection"  // IAR
__attribute__((section(".NetXPoolSection")))  // GCC/AC6
static UCHAR nx_byte_pool_buffer[NX_APP_MEM_POOL_SIZE];
static TX_BYTE_POOL nx_app_byte_pool;
```

### 2.4 UDP Socket Implementation

#### App_UDP_Thread_Entry (app_netxduo.c:370+)

**Hauptablauf:**

```c
static VOID App_UDP_Thread_Entry(ULONG thread_input)
{
    // 1. UDP-Socket erstellen
    nx_udp_socket_create(&NetXDuoEthIpInstance, &UDPSocket,
                         "UDP Server Socket", NX_IP_NORMAL,
                         NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE,
                         QUEUE_MAX_SIZE);

    // 2. Socket an Port binden (typisch Port 6000)
    nx_udp_socket_bind(&UDPSocket, UDP_SERVER_PORT, TX_WAIT_FOREVER);

    // 3. Empfangs-Loop
    while(1)
    {
        // Packet empfangen
        ret = nx_udp_socket_receive(&UDPSocket, &data_packet,
                                     TX_WAIT_FOREVER);

        // Daten verarbeiten (Echo-Funktion)
        // ...

        // Packet zurücksenden
        nx_udp_socket_send(&UDPSocket, my_packet,
                          source_ip_address, source_port);
    }
}
```

**Besonderheit:** Das Projekt enthält auch thermische Bilddaten-Variablen:

```c
// app_netxduo.c:120-128 (in main.c)
extern uint16_t raw[dlen];        // Rohdaten
extern uint16_t dark[dlen];       // Dark Frame
extern uint16_t gain[dlen];       // Gain Frame
extern uint16_t planck[0x10000];  // Planck LUT
extern volatile uint16_t res[dlen];  // Ergebnis (Non-Cacheable)
```

→ **HINWEIS:** Dies deutet darauf hin, dass bereits eine partielle Integration versucht wurde!

### 2.5 Ethernet-Hardware-Konfiguration

#### Hauptdateien (main.c:72-89)

```c
// Ethernet-Handle
ETH_HandleTypeDef heth;

// DMA-Deskriptoren (spezielle Speicherplatzierung)
#pragma location=0x341F8000  // IAR
__attribute__((at(0x341F8000)))  // MDK-ARM
__attribute__((section(".RxDecripSection")))  // GCC
ETH_DMADescTypeDef DMARxDscrTab[ETH_DMA_RX_CH_CNT][ETH_RX_DESC_CNT];

__attribute__((section(".TxDecripSection")))
ETH_DMADescTypeDef DMATxDscrTab[ETH_DMA_TX_CH_CNT][ETH_TX_DESC_CNT];
```

#### PHY-Treiber

```
Unterstützte PHYs:
- LAN8742 (Drivers/BSP/Components/lan8742/)
- RTL8211 (Drivers/BSP/Components/rtl8211/)
```

### 2.6 HAL-Module (stm32n6xx_hal_conf.h:38-92)

```c
#define HAL_BSEC_MODULE_ENABLED      // Secure Storage
#define HAL_ETH_MODULE_ENABLED       // Ethernet
#define HAL_ICACHE_MODULE_ENABLED    // Instruction Cache
#define HAL_RAMCFG_MODULE_ENABLED    // RAM Konfiguration
#define HAL_RIF_MODULE_ENABLED       // Resource Isolation Framework
#define HAL_TIM_MODULE_ENABLED       // Timer
#define HAL_UART_MODULE_ENABLED      // UART
#define HAL_XSPI_MODULE_ENABLED      // XSPI
#define HAL_GPIO_MODULE_ENABLED      // GPIO
#define HAL_DMA_MODULE_ENABLED       // DMA
#define HAL_RCC_MODULE_ENABLED       // Clock
#define HAL_PWR_MODULE_ENABLED       // Power
#define HAL_CORTEX_MODULE_ENABLED    // NVIC
```

### 2.7 Speicherlayout

#### Linker-Script: `STM32N657X0HXQ_AXISRAM2_fsbl.ld`

```ld
MEMORY
{
  ROM      (xrw) : ORIGIN = 0x341d0400, LENGTH = 255K
  RAM      (xrw) : ORIGIN = 0x34000000, LENGTH = 2048K
  AXISRAM1 (xrw) : ORIGIN = 0x24060000, LENGTH = 1024K
  DTCM     (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
}

// Spezielle Sektionen:
.NetXPoolSection    // NetX Byte Pool
.RxDecripSection    // Ethernet RX Deskriptoren
.TxDecripSection    // Ethernet TX Deskriptoren
.noncacheable       // Non-Cacheable Data (für DMA)
.plnk               // Planck-LUT
```

---

## Vergleichsanalyse

### 3.1 Hardware-Vergleich

| **Komponente** | **Thermal Camera** | **UDP Echo Server** | **Kompatibilität** |
|----------------|-------------------|---------------------|-------------------|
| **MCU** | STM32N657X0HXQ | STM32N657X0HXQ | ✅ Identisch |
| **Board** | NUCLEO-N657X0-Q | STM32N6570-DK | ⚠️ Unterschiedlich |
| **CPU-Frequenz** | 600 MHz | 600 MHz | ✅ Identisch |
| **RAM** | AXISRAM (3 MB) | AXISRAM (3 MB) | ✅ Identisch |
| **Flash** | XSPI NOR Flash | XSPI NOR Flash | ✅ Kompatibel |

### 3.2 Software-Architektur

| **Aspekt** | **Thermal Camera** | **UDP Echo Server** |
|------------|-------------------|---------------------|
| **RTOS** | Keine (Bare-Metal) | Azure RTOS (ThreadX) |
| **Scheduler** | Super-Loop + Timer-Interrupts | Preemptive Multitasking |
| **Speicherverwaltung** | Statische Allokation | Dynamische Byte-Pools |
| **Netzwerk** | Keine | NetX Duo (TCP/UDP/DHCP) |
| **Zeitkritisch** | Ja (50 Hz Frame-Rate) | Nein (Best-Effort) |

### 3.3 Peripheral-Nutzung

| **Peripheral** | **Thermal Camera** | **UDP Echo Server** | **Konfliktpotential** |
|----------------|-------------------|---------------------|----------------------|
| **UART1** | ✅ Debug-Ausgabe | ✅ Debug-Ausgabe | ⚠️ Shared Resource |
| **TIM2** | ✅ Frame-Trigger (50 Hz) | ❌ Nicht genutzt | ✅ Kein Konflikt |
| **XSPI2** | ✅ Flash (Kalibrierung) | ✅ Flash (Bilddaten) | ⚠️ Shared Resource |
| **ETH** | ❌ Nicht genutzt | ✅ Netzwerk | ✅ Kein Konflikt |
| **DMA** | ❌ Nicht aktiv | ✅ Ethernet DMA | ⚠️ Möglicher Konflikt |
| **GPIO** | ✅ LED1 | ❌ Nicht spezifiziert | ⚠️ Pin-Mapping prüfen |

### 3.4 Speicheraufteilung

| **Region** | **Thermal Camera** | **UDP Echo Server** | **Überlappung** |
|------------|-------------------|---------------------|----------------|
| **AXISRAM1** | Planck-LUT (128 KB) | ThreadX/NetX Pools? | ⚠️ Mögliche Kollision |
| **AXISRAM2** | Thermal-Frames | Code + Data | ⚠️ Mögliche Kollision |
| **AXISRAM3** | Bad Pixel Arrays | - | ✅ Frei |
| **DTCM** | Stack/Heap (klein) | Stack/Heap | ⚠️ Größe prüfen |
| **XSPI Flash** | Kalibrierungsdaten | Bilddaten (raw/dark/gain) | ⚠️ Adressierung |

### 3.5 Interrupt-Prioritäten

| **Projekt** | **Höchste Priorität** | **Kritische ISRs** |
|-------------|----------------------|-------------------|
| **Thermal Camera** | TIM2_IRQHandler | Frame-Trigger @ 50 Hz |
| **UDP Echo Server** | ETH_IRQHandler (wahrscheinlich) | Ethernet RX/TX |

**Problem:** Ohne RTOS-Interrupt-Management können sich Ethernet- und Timer-ISRs gegenseitig blockieren.

---

## Kompatibilitätsbewertung

### 4.1 Chip-Varianten

| **Parameter** | **STM32N657X0** | **STM32N6570** | **Kommentar** |
|---------------|-----------------|----------------|---------------|
| **Serie** | STM32N6 | STM32N6 | Gleiche Familie |
| **Package** | TFBGA361 (X0HXQ) | TFBGA361 | Identisch |
| **Flash** | Extern (XSPI) | Extern (XSPI) | Identisch |
| **RAM** | 2 MB AXISRAM + 128 KB DTCM | Identisch | Identisch |
| **Peripherals** | Full Set | Full Set | Identisch |

**Fazit:** Die Chips sind funktional **identisch**. Der Unterschied liegt nur in den Boards (NUCLEO vs. Discovery Kit).

### 4.2 Board-Unterschiede

| **Feature** | **NUCLEO-N657X0-Q** | **STM32N6570-DK** |
|-------------|---------------------|-------------------|
| **Formfaktor** | Nucleo-144 | Discovery Kit |
| **Ethernet-PHY** | Onboard (über HSE) | Onboard (LAN8742 oder RTL8211) |
| **User-LEDs** | LED1 (GPIOG Pin 8) | Mehrere LEDs |
| **Debug** | ST-LINK V3 | ST-LINK V3 |
| **Power** | USB / VIN | USB / Extern |

**Problem:** Pin-Mappings können unterschiedlich sein (z.B. LED-GPIOs, Ethernet-Pins).

### 4.3 Middleware-Versionen

#### Thermal Camera Demo

```
- STM32CubeN6 HAL: v1.0.x (geschätzt, basierend auf Copyright 2023/2024)
- CMSIS Core: Standard ARM CMSIS v5.x
- Keine Middleware (Bare-Metal)
```

#### UDP Echo Server

```
- Azure RTOS ThreadX: v6.3.0 (basierend auf tx_user.h Header)
- Azure RTOS NetX Duo: v6.x (kompatibel mit ThreadX 6.3)
- STM32CubeN6 HAL: v1.0.x
- DHCP Client: NetX Duo Addon
```

**Kompatibilität:** HAL-Versionen sind ähnlich, aber Azure RTOS bindet das Projekt an eine zusätzliche Middleware-Schicht.

### 4.4 Pin-Konflikte

#### Potentielle Konflikte

| **Funktion** | **Thermal Camera** | **UDP Echo Server** | **Konflikt?** |
|--------------|-------------------|---------------------|--------------|
| **UART1_TX** | Debug (Pin?) | Debug (Pin?) | ❓ Prüfen |
| **UART1_RX** | Debug (Pin?) | Debug (Pin?) | ❓ Prüfen |
| **XSPI2_CLK** | Flash-Interface | Flash-Interface | ✅ Shared OK |
| **XSPI2_DQS** | Flash-Interface | Flash-Interface | ✅ Shared OK |
| **ETH_RMII_*** | - | Ethernet | ✅ Kein Konflikt |
| **TIM2_CH1** | Intern (Frame-Trigger) | - | ✅ Kein Konflikt |

**Empfehlung:** `.ioc`-Dateien vergleichen oder neu generieren mit STM32CubeMX.

---

## Integrationsstrategie

### 5.1 Zielarchitektur

**Annahme:** Ziel ist es, **thermische Bilddaten über UDP/Ethernet zu versenden**.

#### Option A: **ThreadX-basierte Integration** (EMPFOHLEN)

**Vorteile:**
- Saubere Thread-Trennung
- Netzwerk-Stack bereits vorhanden
- Bessere Wartbarkeit

**Architektur:**

```
ThreadX Kernel
├── Thermal Processing Thread (Priorität: HOCH)
│   └── Helium-optimierte Frame-Verarbeitung @ 50 Hz
├── UDP Transmission Thread (Priorität: MITTEL)
│   └── Sendet verarbeitete Frames via NetX Duo
├── DHCP/Link Monitor Thread (Priorität: NIEDRIG)
│   └── Netzwerkmanagement
└── Idle Thread (Priorität: NIEDRIGSTE)
```

**Änderungen:**

1. **Thermal Camera Code:**
   - TIM2_IRQHandler → TX_EVENT_FLAGS_GROUP (Frame-Ready-Signal)
   - Super-Loop → `tx_thread_entry()` Funktion
   - Statische Arrays beibehalten (AXISRAM-Sektionen)

2. **UDP Echo Server Code:**
   - `App_UDP_Thread_Entry()` erweitern um Frame-Versand
   - Packet-Pool vergrößern (640×480×2 Bytes = ~614 KB pro Frame!)
   - Fragmentierung implementieren (MTU typisch 1500 Bytes)

3. **Speicher:**
   - AXISRAM1: Planck-LUT (unverändert)
   - AXISRAM2: Thermal-Frames + ThreadX/NetX Pools
   - AXISRAM3: Bad Pixel Arrays

#### Option B: **Bare-Metal mit Polling-Ethernet** (NICHT EMPFOHLEN)

**Vorteile:**
- Keine RTOS-Abhängigkeit
- Deterministisches Timing für Thermal-Pipeline

**Nachteile:**
- Ethernet-Treiber ohne NetX Duo sehr komplex
- TCP/IP-Stack von Grund auf implementieren (LwIP als Alternative?)
- Schlechtere Code-Wiederverwendung

### 5.2 Schritt-für-Schritt-Plan

#### Phase 1: Vorbereitungen (1-2 Tage)

1. **Hardware verifizieren:**
   - Welches Board wird verwendet? (NUCLEO oder DK)
   - Pin-Mapping dokumentieren (ETH, UART, GPIO)
   - Stromversorgung prüfen (Ethernet PHY benötigt 3.3V)

2. **Projekt-Setup:**
   - Neues STM32CubeIDE-Projekt erstellen
   - `.ioc` konfigurieren:
     - XSPI2 (Octal DTR)
     - UART1 (115200 baud)
     - ETH (RMII, PHY LAN8742)
     - TIM2 (50 Hz)
   - Azure RTOS aktivieren (ThreadX + NetX Duo)

3. **Speicher-Layout planen:**
   - Linker-Script anpassen:
     ```ld
     MEMORY
     {
       ROM      (xrw) : ORIGIN = 0x341d0400, LENGTH = 255K
       RAM      (xrw) : ORIGIN = 0x34000000, LENGTH = 1024K  // Reduziert
       AXISRAM1 (xrw) : ORIGIN = 0x24060000, LENGTH = 1024K  // Planck LUT
       AXISRAM2 (xrw) : ORIGIN = 0x24160000, LENGTH = 1024K  // Thermal Frames
       DTCM     (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
     }

     .planck_lut : { *(.plnk) } > AXISRAM1
     .thermal_frames : { *(.thermal) } > AXISRAM2
     .NetXPoolSection : { *(.NetXPoolSection) } > RAM
     ```

#### Phase 2: Thermal-Thread implementieren (2-3 Tage)

1. **Thread erstellen:**
   ```c
   TX_THREAD ThermalThread;

   tx_thread_create(&ThermalThread, "Thermal Processing",
                    Thermal_Thread_Entry, 0, thermal_stack_ptr,
                    THERMAL_STACK_SIZE,
                    5,  // Hohe Priorität
                    5, TX_NO_TIME_SLICE, TX_AUTO_START);
   ```

2. **Timer-Interrupt anpassen:**
   ```c
   void TIM2_IRQHandler(void)
   {
       HAL_TIM_IRQHandler(&htim2);

       // Signal an Thread senden
       tx_event_flags_set(&thermal_event_flags, FRAME_TRIGGER, TX_OR);
   }
   ```

3. **Thread-Hauptloop:**
   ```c
   VOID Thermal_Thread_Entry(ULONG thread_input)
   {
       while(1)
       {
           // Warten auf Timer-Trigger
           tx_event_flags_get(&thermal_event_flags, FRAME_TRIGGER,
                              TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);

           // Frame verarbeiten (Original-Code aus main.c)
           thermal_frame_process();

           // Frame bereit → UDP-Thread signalisieren
           tx_semaphore_put(&frame_ready_semaphore);
       }
   }
   ```

#### Phase 3: UDP-Transmission implementieren (2-3 Tage)

1. **UDP-Thread erweitern:**
   ```c
   VOID UDP_Transmission_Thread_Entry(ULONG thread_input)
   {
       // Socket erstellen (wie im Original)
       nx_udp_socket_create(...);
       nx_udp_socket_bind(&ThermalSocket, 7000, TX_WAIT_FOREVER);

       while(1)
       {
           // Auf fertiges Frame warten
           tx_semaphore_get(&frame_ready_semaphore, TX_WAIT_FOREVER);

           // Frame fragmentieren und senden
           send_thermal_frame_fragmented();
       }
   }
   ```

2. **Fragmentierungslogik:**
   ```c
   void send_thermal_frame_fragmented(void)
   {
       const uint32_t FRAME_SIZE = HPIX * VPIX * 2;  // 614400 Bytes
       const uint32_t CHUNK_SIZE = 1400;  // MTU - UDP-Header

       for(uint32_t offset = 0; offset < FRAME_SIZE; offset += CHUNK_SIZE)
       {
           nx_packet_allocate(&NxAppPool, &packet, NX_UDP_PACKET, TX_WAIT_FOREVER);

           uint32_t len = (offset + CHUNK_SIZE > FRAME_SIZE) ?
                          (FRAME_SIZE - offset) : CHUNK_SIZE;

           nx_packet_data_append(packet, &res[offset/2], len, &NxAppPool, TX_WAIT_FOREVER);

           nx_udp_socket_send(&ThermalSocket, packet,
                             destination_ip, destination_port);
       }
   }
   ```

#### Phase 4: Integration & Testing (3-5 Tage)

1. **Build & Flash:**
   - Compiler-Warnungen beheben
   - Linker-Fehler (Speicher-Überlauf) beheben
   - Flash mit ST-LINK

2. **Funktionstest:**
   - ✅ UART-Ausgabe: DHCP-Erfolg, IP-Adresse
   - ✅ Thermal-Processing: Frame-Rate @ 50 Hz
   - ✅ UDP-Transmission: Wireshark-Capture
   - ✅ Datenintegrität: Checksums prüfen

3. **Performance-Optimierung:**
   - DWT Cycle Counter messen
   - Thread-Prioritäten anpassen
   - Cache-Optimierung (AXISRAM ist nicht cacheable)

---

## Empfehlungen

### 6.1 Sofortmaßnahmen

1. **Board-Auswahl klären:**
   - Wird NUCLEO-N657X0-Q oder STM32N6570-DK verwendet?
   - Falls beide vorhanden: Parallel-Test durchführen

2. **Pin-Mapping dokumentieren:**
   - `.ioc` beider Projekte in STM32CubeMX öffnen
   - Excel-Sheet erstellen mit allen verwendeten Pins
   - Konflikte markieren

3. **Speicher-Audit:**
   - Exakte Größen von `raw[]`, `dark[]`, `gain[]`, `planck[]` bestimmen
   - ThreadX/NetX Speicherbedarf berechnen:
     ```
     ThreadX: ~20 KB (Kernel + 3 Threads)
     NetX Duo: ~50 KB (Stack + Packet Pools)
     Thermal Data: ~2.4 MB (640×480×2 × 4 Arrays)
     Total: ~2.5 MB → Passt in 3 MB AXISRAM
     ```

### 6.2 Architektur-Entscheidungen

| **Kriterium** | **ThreadX-Integration** | **Bare-Metal** | **Empfehlung** |
|---------------|------------------------|----------------|----------------|
| **Entwicklungszeit** | Mittel (2 Wochen) | Lang (4+ Wochen) | ✅ ThreadX |
| **Code-Wartbarkeit** | Hoch | Niedrig | ✅ ThreadX |
| **Determinismus** | Hoch (mit Prioritäten) | Sehr hoch | ⚠️ Beide OK |
| **Speicher-Overhead** | ~70 KB | 0 KB | ⚠️ Akzeptabel |
| **Netzwerk-Performance** | Hoch (Stack-Optimiert) | Niedrig | ✅ ThreadX |

**Finale Empfehlung:** **ThreadX-basierte Integration (Option A)**

### 6.3 Potenzielle Probleme

| **Problem** | **Wahrscheinlichkeit** | **Mitigation** |
|-------------|----------------------|----------------|
| **Speicher-Überlauf** | Mittel | Linker-Script frühzeitig anpassen, MAP-Datei prüfen |
| **Interrupt-Latenz** | Hoch | ThermalThread höchste Priorität geben, ETH-IRQ nachladen |
| **Ethernet-Fragmentierung** | Hoch | UDP mit eigener Sequenznummer + Reassembly auf PC-Seite |
| **Cache-Kohärenz** | Niedrig | `res[]` als `__NON_CACHEABLE` deklarieren (bereits im UDP-Code) |
| **DHCP-Timeout** | Niedrig | Fallback auf statische IP implementieren |
| **Frame-Drops** | Mittel | Double-Buffering für `res[]` implementieren |

### 6.4 Alternative Ansätze

#### 1. **LwIP statt NetX Duo**

- Vorteile: Geringerer Speicherverbrauch, Open-Source
- Nachteile: Keine direkte ThreadX-Integration, höherer Portierungsaufwand
- **Bewertung:** Nur wenn Azure RTOS-Lizenz problematisch ist

#### 2. **JPEG-Kompression vor Versand**

- Vorteile: Reduziert Bandbreite (640×480×2 → ~50-100 KB)
- Nachteile: CPU-Last (JPEG-Encoder), Latenz
- **Bewertung:** Sinnvoll für langsamere Netzwerke (< 100 Mbit/s)

#### 3. **Dedizierte DMA für XSPI → Ethernet**

- Vorteile: Null-Copy-Transmission, minimale CPU-Last
- Nachteile: Komplexe Konfiguration, Planck-LUT-Verarbeitung verliert Kontrolle
- **Bewertung:** Nur für extreme Performance-Anforderungen

---

## Anhang

### A. Wichtige Konfigurationsdateien

#### Projekt 1: Thermal Camera Demo

```
CORTEX_HELIUM.ioc
STM32CubeIDE/FSBL/STM32N657X0HXQ_AXISRAM2_fsbl.ld
FSBL/Inc/stm32n6xx_hal_conf.h
FSBL/Inc/main.h
FSBL/Inc/xspi_nor.h
FSBL/Src/main.c
FSBL/Src/stm32n6xx_it.c
FSBL/Src/xspi_nor.c
```

#### Projekt 2: UDP Echo Server

```
(Keine .ioc gefunden - vermutlich manuell konfiguriert)
STM32CubeIDE/STM32N657X0HXQ_AXISRAM2_fsbl.ld
Core/Inc/stm32n6xx_hal_conf.h
Core/Inc/tx_user.h
Core/Inc/app_threadx.h
Core/Src/main.c
Core/Src/app_threadx.c
AZURE_RTOS/App/app_azure_rtos.c
AZURE_RTOS/App/app_azure_rtos_config.h
NetXDuo/App/app_netxduo.c
NetXDuo/App/app_netxduo.h
```

### B. Speicherverbrauch-Schätzungen

| **Komponente** | **Größe** | **Speichertyp** |
|----------------|-----------|-----------------|
| **Planck LUT** | 128 KB | AXISRAM1 (.plnk) |
| **Raw Frame** | 614 KB | AXISRAM2/Global |
| **Dark Frame** | 614 KB | AXISRAM2/Global |
| **Gain Frame** | 614 KB | AXISRAM2/Global |
| **Result Frame** | 614 KB | AXISRAM2/Non-Cacheable |
| **ThreadX Kernel** | ~10 KB | RAM |
| **Thread Stacks** | 3 × 16 KB = 48 KB | RAM |
| **NetX Packet Pool** | ~50 KB | RAM (NetXPoolSection) |
| **Code + RO Data** | ~200 KB | ROM (Flash) |
| **Gesamt RAM** | ~2.7 MB | 3 MB verfügbar → ✅ OK |

### C. Referenzen

- [STM32N6 Reference Manual (RM0493)](https://www.st.com/resource/en/reference_manual/rm0493-stm32n6-armbased-32bit-mcus-stmicroelectronics.pdf)
- [Azure RTOS ThreadX User Guide](https://docs.microsoft.com/en-us/azure/rtos/threadx/)
- [Azure RTOS NetX Duo User Guide](https://docs.microsoft.com/en-us/azure/rtos/netx-duo/)
- [STM32N6 HAL Driver User Manual](https://www.st.com/resource/en/user_manual/um3214-stm32cube-mcu-package-for-stm32n6-series-stmicroelectronics.pdf)

---

## Changelog

| **Datum** | **Version** | **Änderungen** |
|-----------|-------------|----------------|
| 2025-10-28 | 1.0 | Initiale Analyse erstellt |

---

**Kontakt für Fragen:** Siehe CLAUDE.md für Entwicklungsworkflow-Details.
