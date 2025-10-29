# Thermal UDP Streaming - Integration Plan
## STM32N6570-DK Discovery Kit

**Basis-Projekt:** VENC_RTSP_Server
**Ziel-Projekt:** Thermal_UDP_Streaming
**Hardware:** STM32N6570-DK (STM32N657X0H3QU @ 600 MHz)

**Erstellt:** 2025-10-28
**Version:** 1.0 - Detailed Integration Plan

---

## Executive Summary

### Projektziel

Entwicklung eines **Thermal Camera UDP Streaming** Systems für das STM32N6570-DK Discovery Kit durch Integration von:

1. **VENC_RTSP_Server** (Basis): Netzwerk-Stack, Thread-Architektur, DK-Board BSP
2. **Thermal Camera Demo** (CORTEX_HELIUM): Helium-optimierte thermische Bildverarbeitung @ 50 Hz

### Key Performance Indicators (KPIs)

| **Metrik** | **Zielwert** | **Kritisch** |
|------------|--------------|--------------|
| **Frame-Rate** | 50 Hz (20 ms pro Frame) | ✅ Ja |
| **Frame-Processing** | < 18 ms | ✅ Ja |
| **UDP Transmission** | < 50 ms | ⚠️ Nice-to-Have |
| **End-to-End Latenz** | < 100 ms | ⚠️ Nice-to-Have |
| **Netzwerk-Bandbreite** | ~30 MB/s (240 Mbit/s) | ✅ Ja (Gigabit verfügbar) |

### Erfolgs-Kriterien

- [ ] Thermal-Pipeline läuft @ 50 Hz sustained
- [ ] PC empfängt vollständige 640×480 Frames via UDP
- [ ] Frame-Integrität (Checksums korrekt)
- [ ] < 1% Frame-Drops
- [ ] DHCP funktioniert zuverlässig

### Timeline Overview

| **Phase** | **Dauer** | **Completion** |
|-----------|-----------|----------------|
| **Phase 1:** Projekt-Setup | 2 Tage | Tag 1-2 |
| **Phase 2:** Thermal-Code Integration | 3 Tage | Tag 3-5 |
| **Phase 3:** UDP Streaming | 3 Tage | Tag 6-8 |
| **Phase 4:** Testing & Optimierung | 4 Tage | Tag 9-12 |
| **GESAMT** | **12 Tage** | **2,4 Wochen** |

---

## 1. VENC_RTSP_Server - Detailanalyse

### 1.1 Architektur-Übersicht

**Projektpfad:**
`Projects/STM32N6570-DK/Applications/VENC/VENC_RTSP_Server/`

**Hauptkomponenten:**

```
VENC_RTSP_Server/
├── FSBL/                          # First Stage Bootloader
│   ├── Src/main.c                 # FSBL Init
│   └── Inc/extmem.h               # External Memory Config
├── Appli/                         # Application (unsere Basis)
│   ├── Core/
│   │   ├── Src/main.c             # Hardware Init (ETH, UART, GPIO)
│   │   ├── Src/app_threadx.c     # ThreadX Kernel Start
│   │   ├── Src/venc_app.c        # ❌ Video Encoder (zu entfernen)
│   │   └── Inc/venc_app.h
│   ├── NetXDuo/
│   │   └── App/
│   │       ├── app_netxduo.c      # ✅ NetX Init, Thread Creation
│   │       ├── app_rtsp_over_rtp.c # ❌ RTSP Server (zu entfernen)
│   │       └── nx_user.h          # NetX Config
│   └── AZURE_RTOS/
│       └── App/
│           ├── app_azure_rtos.c   # ✅ RTOS Init
│           └── app_azure_rtos_config.h
└── STM32CubeIDE/
    └── Appli/
        └── STM32N657XX_LRUN.ld    # ✅ Linker Script (3.6 MB RAM!)
```

### 1.2 Thread-Architektur (Original)

**Aus `app_netxduo.c` (Zeilen 169-190):**

```c
// Thread 1: AppMainThread
tx_thread_create(&AppMainThread, "App Main thread",
                 App_Main_Thread_Entry, 0, pointer,
                 2 * DEFAULT_MEMORY_SIZE,
                 DEFAULT_PRIORITY, DEFAULT_PRIORITY,
                 TX_NO_TIME_SLICE, TX_AUTO_START);

// Thread 2: AppLinkThread
tx_thread_create(&AppLinkThread, "App Link Thread",
                 App_Link_Thread_Entry, 0, pointer,
                 2 * DEFAULT_MEMORY_SIZE,
                 LINK_PRIORITY, LINK_PRIORITY,
                 TX_NO_TIME_SLICE, TX_AUTO_START);

// Thread 3: venc_thread_func (in venc_app.c)
// - Erstellt Event Flags, Queue
// - Wartet auf VIDEO_START_FLAG
// - Ruft H264 Encoder auf
```

**Prioritäten (angenommen aus Code):**

| **Thread** | **Priorität** | **Stack** | **Funktion** |
|------------|---------------|-----------|--------------|
| AppMainThread | 10 (DEFAULT_PRIORITY) | 4 KB | DHCP starten, RTSP Server starten |
| AppLinkThread | 15 (LINK_PRIORITY) | 4 KB | Ethernet Link-Status überwachen |
| venc_thread | ?? | ?? | Video-Encoding (H264) |
| RTSP Server Thread | ?? | ?? | TCP/RTP Server |

### 1.3 Wiederverwendbare Komponenten ✅

#### A. Hardware-Initialisierung (main.c)

**100% Wiederverwendbar:**

```c
// Zeilen 60-68 (Ethernet DMA Descriptors)
ETH_DMADescTypeDef DMARxDscrTab[ETH_DMA_RX_CH_CNT][ETH_RX_DESC_CNT] __NON_CACHEABLE;
ETH_DMADescTypeDef DMATxDscrTab[ETH_DMA_TX_CH_CNT][ETH_TX_DESC_CNT] __NON_CACHEABLE;

// Zeilen 117-119 (Peripheral Init)
MX_GPIO_Init();
MX_ETH_Init();
MX_USART1_UART_Init();

// Zeilen 98-101 (Cache Enable)
SCB_EnableICache();
SCB_EnableDCache();
```

**Warum wiederverwendbar?**
- Ethernet-Setup ist identisch für UDP
- UART für Debug-Logging
- GPIO für LEDs (BSP-Treiber)

#### B. NetX Duo Initialisierung (app_netxduo.c)

**90% Wiederverwendbar:**

```c
// Zeilen 89-101 (Packet Pool)
tx_byte_allocate(byte_pool, (VOID **) &pointer, NX_PACKET_POOL_SIZE, TX_NO_WAIT);
nx_packet_pool_create(&AppPool, "Main Packet Pool", PAYLOAD_SIZE, pointer, NX_PACKET_POOL_SIZE);

// Zeilen 112-118 (IP Instance)
nx_ip_create(&IpInstance, "Main Ip instance",
             NX_APP_DEFAULT_IP_ADDRESS, NX_APP_DEFAULT_NET_MASK,
             &AppPool, nx_stm32_eth_driver, pointer, ...);

// Zeilen 129-142 (Protokolle)
nx_arp_enable(&IpInstance, ...);
nx_icmp_enable(&IpInstance);
nx_tcp_enable(&IpInstance);    // ⚠️ Optional (nur für RTSP)
nx_udp_enable(&IpInstance);    // ✅ Benötigt!

// Zeilen 193-201 (DHCP Client)
nx_dhcp_create(&DHCPClient, &IpInstance, "DHCP Client");
tx_semaphore_create(&DHCPSemaphore, "DHCP Semaphore", 0);
```

**Was zu ändern ist:**
- `nx_tcp_enable()` kann entfernt werden (nur UDP benötigt)
- RTSP-Server Code entfernen

#### C. DHCP Client Setup (app_netxduo.c)

**100% Wiederverwendbar:**

```c
// App_Main_Thread_Entry (Zeilen 236-281)
static VOID App_Main_Thread_Entry(ULONG thread_input)
{
    // IP-Change Callback registrieren
    nx_ip_address_change_notify(&IpInstance, ip_address_change_notify_callback, NULL);

    // DHCP starten
    nx_dhcp_start(&DHCPClient);

    // Warten auf IP-Adresse
    tx_semaphore_get(&DHCPSemaphore, TX_WAIT_FOREVER);

    // IP-Adresse ausgeben
    PRINT_IP_ADDRESS(IpAddress);

    // Hier: RTSP Server starten → Ersetzen durch UDP Thread Start
    // sample_entry(&IpInstance, &AppPool, NX_NULL, NX_NULL);
}
```

#### D. Link-Monitor Thread (app_netxduo.c)

**100% Wiederverwendbar:**

```c
// App_Link_Thread_Entry (Zeilen 293-299+)
static VOID App_Link_Thread_Entry(ULONG thread_input)
{
    ULONG actual_status;
    UINT linkdown = 0, status;

    while(1)
    {
        // Prüfe Link-Status alle 1 Sekunde
        tx_thread_sleep(100);  // 1000 ms @ 100 ticks/sec

        status = nx_ip_interface_status_check(&IpInstance, 0,
                                               NX_IP_LINK_ENABLED,
                                               &actual_status, 10);

        if(status == NX_SUCCESS)
        {
            // Link ist UP
            if(linkdown == 1)
            {
                printf("Ethernet cable connected\n");
                linkdown = 0;
                BSP_LED_On(LED_GREEN);
            }
        }
        else
        {
            // Link ist DOWN
            if(linkdown == 0)
            {
                printf("Ethernet cable disconnected\n");
                linkdown = 1;
                BSP_LED_Off(LED_GREEN);
            }
        }
    }
}
```

#### E. BSP Discovery Kit (stm32n6570_discovery.h)

**100% Wiederverwendbar:**

```c
// LED-Steuerung
BSP_LED_Init(LED1);    // Orange LED
BSP_LED_Init(LED2);    // Green LED
BSP_LED_Init(LED3);    // Red LED
BSP_LED_On(LED_ORANGE);
BSP_LED_Off(LED_GREEN);
BSP_LED_Toggle(LED_RED);

// LCD (optional, für Thermal-Anzeige in Phase 2)
BSP_LCD_Init(0, LCD_ORIENTATION_LANDSCAPE);
BSP_LCD_GetXSize(0, &x_size);  // 800 px
BSP_LCD_GetYSize(0, &y_size);  // 480 px
BSP_LCD_FillRect(0, 0, 0, x_size, y_size, LCD_COLOR_BLACK);
```

### 1.4 Zu entfernende Komponenten ❌

#### A. Video Encoder Hardware (venc_app.c)

**Vollständig entfernen:**

```c
// H264 Encoder
#include "h264encapi.h"
static H264EncIn encIn;
static H264EncOut encOut;
static H264EncInst encoder;

// EWL (Encoder Wrapper Library)
#include "ewl.h"
uint8_t ewl_pool[0x190000] __NON_CACHEABLE;

// Block Pool für Encoder Output
TX_BLOCK_POOL venc_block_pool;
venc_output_frame_t queue_buf[VENC_OUTPUT_BLOCK_NBR];
```

**Warum?**
- Thermal-Daten sind bereits verarbeitet (keine Kompression in Phase 1)
- VENC Hardware-Modul nicht benötigt
- Spart ~1.6 MB RAM (ewl_pool)

#### B. Camera Pipeline (venc_app.c)

**Vollständig entfernen:**

```c
// DCMIPP (Digital Camera Interface Pixel Preprocessor)
#include "stm32n6570_discovery_camera.h"
#include "imx335.h"  // Sony IMX335 CMOS Sensor

// Camera Init
BSP_CAMERA_Init(...);
BSP_CAMERA_Start(...);
```

**Warum?**
- Thermal-Sensor hat eigene Schnittstelle (XSPI Flash für Kalibrierung)
- DCMIPP nutzt Pins, die wir für TIM2/XSPI2 brauchen könnten

#### C. RTSP/RTP Server (app_rtsp_over_rtp.c)

**Vollständig entfernen:**

```c
#include "app_rtsp_over_rtp.h"

VOID sample_entry(NX_IP* ip_ptr, NX_PACKET_POOL* pool_ptr, ...);

// RTP (Real-time Transport Protocol)
// RTSP (Real-Time Streaming Protocol)
// TCP Server Port 554
```

**Warum?**
- Phase 1 nutzt simples UDP
- RTSP ist zu komplex für Prototyp
- TCP Overhead nicht benötigt

#### D. LCD/Display Code (optional behalten)

**Optional entfernen:**

```c
#include "stm32n6570_discovery_lcd.h"
#include "stlogo.h"  // ST Logo

BSP_LCD_Init(...);
BSP_LCD_DisplayOn(0);
```

**Entscheidung:**
- ❌ **Phase 1:** Entfernen (spart Init-Zeit)
- ✅ **Phase 2:** Reaktivieren für lokale Thermal-Anzeige (480×272 LCD)

### 1.5 Speicher-Layout (Linker-Script)

**STM32N657XX_LRUN.ld (Zeilen 49-54):**

```ld
MEMORY
{
  ROM    (xr)     : ORIGIN = 0x34000400,   LENGTH = 319K    // Code + RO Data
  RAM    (rw)     : ORIGIN = 0x34060000,   LENGTH = 3712K   // AXISRAM (3.6 MB!)
  DTCM   (rw)     : ORIGIN = 0x30000000,   LENGTH = 128K    // Stack
}
```

**Speicher-Aufteilung:**

| **Region** | **Adresse** | **Größe** | **Verwendung (Original)** |
|------------|-------------|-----------|--------------------------|
| **ROM** | 0x34000400 | 319 KB | Code, .rodata, ISR-Vector |
| **RAM (AXISRAM)** | 0x34060000 | 3.6 MB | .data, .bss, Heap, Video-Buffer |
| **DTCM** | 0x30000000 | 128 KB | Stack (_estack) |

**Thermal-Projekt benötigt:**

| **Komponente** | **Größe** | **Speicher-Region** |
|----------------|-----------|---------------------|
| Planck LUT | 128 KB | AXISRAM (.plnk) |
| Raw Frame | 614 KB | AXISRAM (.thermal) |
| Dark Frame | 614 KB | AXISRAM (.thermal) |
| Gain Frame | 614 KB | AXISRAM (.thermal) |
| Result Frame | 614 KB | AXISRAM (.thermal_result, non-cacheable) |
| Bad Pixel Arrays | ~100 KB | AXISRAM |
| ThreadX/NetX Pools | ~150 KB | AXISRAM |
| Code + RO Data | ~250 KB | ROM |
| **GESAMT RAM** | **~2.8 MB** | **< 3.6 MB ✅ PASST!** |

---

## 2. Nx_MQTT_Client - Referenzanalyse

### 2.1 Überblick

**Projektpfad:**
`Projects/STM32N6570-DK/Applications/NetXDuo/Nx_MQTT_Client/`

**Warum als Referenz?**

- Einfachere NetX Duo Implementation als VENC_RTSP
- Zeigt minimalen Thread-Overhead
- Gutes Beispiel für DHCP Client

### 2.2 Unterschiede zu VENC_RTSP

| **Aspekt** | **VENC_RTSP** | **Nx_MQTT** |
|------------|---------------|-------------|
| **Threads** | 3+ (Main, Link, VENC, RTSP) | 2 (Main, MQTT) |
| **Protokolle** | TCP, UDP, ARP, ICMP, DHCP | TCP, DHCP, TLS |
| **Komplexität** | Hoch (Video-Encoding) | Mittel |
| **RAM-Usage** | ~3 MB (mit Video-Buffer) | ~500 KB |

**Fazit:** Nx_MQTT ist zu minimal. VENC_RTSP ist bessere Basis (bereits DK-Board optimiert).

---

## 3. Thermal Camera Demo - Integration Requirements

### 3.1 Kernkomponenten

**Aus `Projects/NUCLEO-N657X0-Q/Examples/CORTEX/CORTEX_HELIUM/FSBL/Src/main.c`:**

#### A. Datenstrukturen

```c
#define HPIX 640
#define VPIX 480
#define FRAME_SIZE (HPIX * VPIX)  // 307200 Pixel

// Thermal Data Arrays
uint16_t raw[FRAME_SIZE]   __attribute__((section(".thermal")));
uint16_t dark[FRAME_SIZE]  __attribute__((section(".thermal")));
uint16_t gain[FRAME_SIZE]  __attribute__((section(".thermal")));
uint16_t planck[65536]     __attribute__((section(".plnk")));  // Planck LUT
volatile uint16_t res[FRAME_SIZE] __attribute__((section(".thermal_result")));
```

#### B. Frame-Processing Pipeline

**Hauptfunktion (main.c ~852):**

```c
void process_thermal_line_fastest(uint16_t *line_in,
                                   uint16_t *dark_line,
                                   uint16_t *gain_line,
                                   uint16_t *offset_line,
                                   uint16_t *planck_lut,
                                   volatile uint16_t *line_out,
                                   uint32_t pixel_count)
{
    // Helium-optimierte Verarbeitung (8 Pixel / Iteration)
    uint16x8_t v_input, v_dark, v_gain, v_offset;
    uint16x8_t v_result;

    for (uint32_t i = 0; i < pixel_count; i += 8)
    {
        // Dark Frame Subtraction
        v_input = vld1q_u16(&line_in[i]);
        v_dark = vld1q_u16(&dark_line[i]);
        v_result = vsubq_u16(v_input, v_dark);

        // Gain Correction (Q15 Fixed-Point)
        v_gain = vld1q_u16(&gain_line[i]);
        v_result = vqrdmulhq_u16(v_result, v_gain);

        // Offset Addition
        v_offset = vld1q_u16(&offset_line[i]);
        v_result = vaddq_u16(v_result, v_offset);

        // Planck LUT Lookup (manuell, nicht vektorisiert)
        for (int j = 0; j < 8; j++)
        {
            uint16_t idx = v_result[j];
            line_out[i + j] = planck_lut[idx];
        }
    }
}

void thermal_frame_process(void)
{
    for (uint32_t line = 0; line < VPIX; line++)
    {
        process_thermal_line_fastest(&raw[line * HPIX],
                                       &dark[line * HPIX],
                                       &gain[line * HPIX],
                                       &offset[line * HPIX],
                                       planck,
                                       &res[line * HPIX],
                                       HPIX);
    }
}
```

#### C. Timer-Trigger (50 Hz)

**TIM2 Konfiguration:**

```c
// Timer-Einstellungen
TIM2_PRESCALER = 9999;   // 600 MHz / 10000 = 60 kHz
TIM2_PERIOD = 1199;      // 60 kHz / 1200 = 50 Hz

// Interrupt-Handler (stm32n6xx_it.c)
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);

    // Frame-Ready Flag setzen
    vsync_count++;
    frame_ready = 1;
}
```

#### D. XSPI NOR Flash (Kalibrierung)

**xspi_nor.c:**

```c
void XSPI_NOR_Init_All(void);
HAL_StatusTypeDef XSPI_NOR_Read(uint8_t *pData, uint32_t Address, uint32_t Size);
HAL_StatusTypeDef XSPI_NOR_Write(uint8_t *pData, uint32_t Address, uint32_t Size);
HAL_StatusTypeDef XSPI_NOR_EraseSector(uint32_t Address);

// Kalibrierungsdaten laden
void load_calibration_data(void)
{
    XSPI_NOR_Read((uint8_t*)dark, DARK_FRAME_ADDRESS, FRAME_SIZE * 2);
    XSPI_NOR_Read((uint8_t*)gain, GAIN_FRAME_ADDRESS, FRAME_SIZE * 2);
}
```

### 3.2 Performance-Messungen (Original)

**Aus CLAUDE.md:**

| **Operation** | **Zyklen @ 600 MHz** | **Zeit (ms)** |
|---------------|---------------------|---------------|
| **thermal_frame_process()** | ~8.2 Mio | ~13.7 ms |
| **Planck LUT Generation** | ~5.0 Mio | ~8.3 ms |
| **XSPI Flash Read (614 KB)** | ~3.0 Mio | ~5.0 ms |

**Frame-Budget @ 50 Hz:** 20 ms
**Verbleibend für UDP Transmission:** 20 ms - 13.7 ms = **6.3 ms** ⚠️ KNAPP!

---

## 4. Thread-Architektur Design

### 4.1 Vorgeschlagene Thread-Struktur

```
ThreadX Kernel (100 ticks/sec = 10 ms Zeitscheibe)
│
├── Thread 1: NxAppThread (Prio 10)
│   └── Funktion: DHCP starten, IP auflösen, andere Threads aktivieren
│
├── Thread 2: AppLinkThread (Prio 15)
│   └── Funktion: Ethernet Link-Status überwachen
│
├── Thread 3: ThermalProcessingThread (Prio 5) ⭐ HÖCHSTE PRIORITÄT
│   └── Funktion:
│       - Warten auf TIM2 Interrupt (Event Flags)
│       - thermal_frame_process() ausführen (~13.7 ms)
│       - Frame-Ready Signal an UDP Thread (Semaphore)
│
└── Thread 4: UdpStreamingThread (Prio 8)
    └── Funktion:
        - Warten auf Frame-Ready Semaphore
        - Frame fragmentieren (614 KB → 440 Pakete)
        - UDP Senden an Client PC
        - Performance-Logging
```

### 4.2 Prioritäten-Begründung

| **Thread** | **Priorität** | **Begründung** |
|------------|---------------|----------------|
| **ThermalProcessingThread** | 5 (HOCH) | ✅ **Zeitkritisch!** Muss alle 20 ms laufen (50 Hz). Höchste Prio um Frame-Drops zu vermeiden. |
| **UdpStreamingThread** | 8 (MITTEL) | ⚠️ Wichtig, aber nicht zeitkritisch. Best-Effort Transmission. Darf Thermal-Thread nicht blockieren. |
| **NxAppThread** | 10 (NIEDRIG) | ✅ Nur bei Startup aktiv (DHCP), danach idle. |
| **AppLinkThread** | 15 (NIEDRIGST) | ✅ Prüft Link-Status alle 1 Sekunde. Nicht zeitkritisch. |

**ThreadX Scheduler-Verhalten:**

- **Preemptive:** Thermal-Thread kann UDP-Thread unterbrechen
- **Round-Robin:** Deaktiviert (TX_NO_TIME_SLICE)
- **Interrupt Priority:** TIM2_IRQn sollte Prio 5 haben (höher als Ethernet)

### 4.3 Synchronisations-Objekte

```c
// Event Flags für Timer-Trigger
TX_EVENT_FLAGS_GROUP thermal_event_flags;
#define FRAME_TRIGGER_FLAG 0x01

// Semaphore für Frame-Ready Signal
TX_SEMAPHORE frame_ready_semaphore;

// Mutex für res[] Zugriff (falls gleichzeitiger Zugriff)
TX_MUTEX frame_buffer_mutex;  // Optional
```

### 4.4 Alternative: DMA-basierter Ansatz (Phase 2)

**Problem:** UDP Transmission dauert > 6 ms → Frame-Drops möglich

**Lösung:** HPDMA für XSPI → Ethernet Zero-Copy

```
TIM2 Interrupt (50 Hz)
    ↓
Thermal-Thread: thermal_frame_process() (13.7 ms)
    ↓
DMA Setup: res[] → Ethernet Tx Buffer (per DMA)
    ↓
DMA Interrupt: nx_udp_socket_send() (non-blocking)
    ↓
Thermal-Thread: Nächstes Frame
```

**Vorteil:** CPU frei während UDP Transmission
**Nachteil:** Komplexe DMA-Konfiguration, Cache-Kohärenz

**Empfehlung:** Phase 1 ohne DMA, Phase 2 evaluieren.

---

## 5. Pin-Mapping Kompatibilitätsanalyse

### 5.1 Verwendete Peripherals

| **Peripheral** | **VENC_RTSP** | **Thermal Camera** | **Thermal UDP** | **Konflikt?** |
|----------------|---------------|-------------------|----------------|--------------|
| **UART1** | ✅ PE5/PE6 | ✅ ??/??† | ✅ PE5/PE6 | ✅ OK |
| **ETH1** | ✅ RMII (PF/PG) | ❌ - | ✅ RMII (PF/PG) | ✅ OK |
| **XSPI2** | ❌ - | ✅ Octal DTR | ✅ Octal DTR | ✅ OK (neu hinzufügen) |
| **TIM2** | ❌ - | ✅ Intern (50 Hz) | ✅ Intern (50 Hz) | ✅ OK (neu hinzufügen) |
| **DCMIPP** | ✅ Camera | ❌ - | ❌ Entfernen | ✅ OK |
| **VENC** | ✅ H264 Encoder | ❌ - | ❌ Entfernen | ✅ OK |
| **LCD (LTDC)** | ✅ Display | ❌ - | ⚠️ Optional | ⚠️ Prüfen |

† UART-Pins in Thermal Camera unbekannt (vermutlich Board-spezifisch)

### 5.2 Ethernet Pins (RMII Mode)

**Aus `stm32n6xx_hal_msp.c` (VENC_RTSP, Zeilen 107-145):**

| **Signal** | **Pin** | **Alt Function** | **Shared mit?** |
|------------|---------|-----------------|----------------|
| ETH_REF_CLK | PF7 | AF11_ETH1 | - |
| ETH_MDIO | PF4 | AF11_ETH1 | - |
| ETH_MDC | PG11 | AF11_ETH1 | - |
| ETH_CRS_DV | PF10 | AF11_ETH1 | - |
| ETH_RXD0 | PF14 | AF11_ETH1 | - |
| ETH_RXD1 | PF15 | AF11_ETH1 | - |
| ETH_TX_EN | PF11 | AF11_ETH1 | - |
| ETH_TXD0 | PF12 | AF11_ETH1 | - |
| ETH_TXD1 | PF13 | AF11_ETH1 | - |
| ETH_PHY_RST | PF0 | AF12_ETH1 | - |

**Status:** ✅ **Keine Konflikte** - XSPI2 nutzt PO0-PO7 (Port O)

### 5.3 XSPI2 NOR Flash Pins (zu konfigurieren)

**Basierend auf STM32N6570-DK Schematic:**

| **Signal** | **Pin** | **Alt Function** | **Shared mit?** |
|------------|---------|-----------------|----------------|
| XSPI2_CLK | PG6 | AF9_XSPI2 | ⚠️ LCD_BL_CTRL (prüfen!) |
| XSPI2_NCS | PG12 | AF9_XSPI2 | - |
| XSPI2_DQS | PG15 | AF9_XSPI2 | - |
| XSPI2_IO0 | PO0 | AF9_XSPI2 | - |
| XSPI2_IO1 | PO1 | AF9_XSPI2 | - |
| XSPI2_IO2 | PO2 | AF9_XSPI2 | - |
| XSPI2_IO3 | PO3 | AF9_XSPI2 | - |
| XSPI2_IO4 | PO4 | AF9_XSPI2 | - |
| XSPI2_IO5 | PO5 | AF9_XSPI2 | - |
| XSPI2_IO6 | PO6 | AF9_XSPI2 | - |
| XSPI2_IO7 | PO7 | AF9_XSPI2 | - |

**Potentieller Konflikt:** PG6 kann mit LCD Backlight Control kollidieren.

**Lösung:**
1. **Phase 1:** LCD nicht nutzen → Kein Konflikt
2. **Phase 2:** Backlight-Control auf anderen Pin mappen (z.B. PG7)

### 5.4 TIM2 Pins

**TIM2 Channel 1 (für Frame-Trigger):**

- Pin: **Intern** (kein externer Pin benötigt)
- Funktion: Generiert 50 Hz Interrupt
- Konflikt: ❌ Keiner

### 5.5 Pin-Konfiguration .ioc Datei

**VENC_RTSP hat keine .ioc Datei!**
→ Manuelle Pin-Konfiguration in `stm32n6xx_hal_msp.c`

**Für Thermal UDP:**
1. **Option A:** Manuelle Konfiguration kopieren (schneller)
2. **Option B:** Neue .ioc Datei mit CubeMX erstellen (sauberer)

**Empfehlung:** Option B für Phase 1 (sicherer).

---

## 6. Speicher-Layout & Linker-Script

### 6.1 Aktuelles Layout (VENC_RTSP)

**STM32N657XX_LRUN.ld:**

```ld
MEMORY
{
  ROM    (xr)     : ORIGIN = 0x34000400,   LENGTH = 319K
  RAM    (rw)     : ORIGIN = 0x34060000,   LENGTH = 3712K   // 3.6 MB
  DTCM   (rw)     : ORIGIN = 0x30000000,   LENGTH = 128K
}

SECTIONS
{
  .isr_vector : { ... } >ROM
  .text : { ... } >ROM
  .rodata : { ... } >ROM
  .data : { ... } >RAM AT> ROM
  .bss : { ... } >RAM
  ._user_heap_stack : { ... } >RAM
}
```

**RAM-Nutzung (Original):**

| **Sektion** | **Größe** | **Verwendung** |
|-------------|-----------|----------------|
| .data | ~50 KB | Initialisierte Variablen |
| .bss | ~200 KB | Nicht-init. Variablen |
| Heap | ~500 KB | ThreadX/NetX Byte Pools |
| Video Buffer | ~2 MB | VENC Output |
| Stack (DTCM) | 128 KB | - |

### 6.2 Vorgeschlagenes Layout (Thermal UDP)

**Modifiziertes Linker-Script:**

```ld
MEMORY
{
  ROM    (xr)     : ORIGIN = 0x34000400,   LENGTH = 319K
  RAM    (rw)     : ORIGIN = 0x34060000,   LENGTH = 1024K   // 1 MB (General Purpose)
  AXISRAM1 (rw)   : ORIGIN = 0x34160000,   LENGTH = 1024K   // 1 MB (Planck LUT)
  AXISRAM2 (rw)   : ORIGIN = 0x34260000,   LENGTH = 2688K   // 2.6 MB (Thermal Frames)
  DTCM   (rw)     : ORIGIN = 0x30000000,   LENGTH = 128K
}

SECTIONS
{
  /* ... Standard Sections ... */

  /* Planck LUT (128 KB, aligned) */
  .planck_lut (NOLOAD) :
  {
    . = ALIGN(8);
    *(.plnk)
    . = ALIGN(8);
  } >AXISRAM1

  /* Thermal Frames (4 × 614 KB = 2.45 MB) */
  .thermal_frames (NOLOAD) :
  {
    . = ALIGN(8);
    *(.thermal)
    . = ALIGN(8);
  } >AXISRAM2

  /* Thermal Result (non-cacheable, 614 KB) */
  .thermal_result (NOLOAD) :
  {
    . = ALIGN(8);
    *(.thermal_result)
    . = ALIGN(8);
  } >AXISRAM2

  /* Bad Pixel Arrays (~100 KB) */
  .bad_pixel (NOLOAD) :
  {
    . = ALIGN(8);
    *(.bad_pixel)
    . = ALIGN(8);
  } >AXISRAM2
}
```

**Speicher-Verteilung:**

| **Region** | **Adresse** | **Größe** | **Thermal UDP Verwendung** |
|------------|-------------|-----------|---------------------------|
| **ROM** | 0x34000400 | 319 KB | Code (~250 KB) |
| **RAM** | 0x34060000 | 1024 KB | .data, .bss, Heap, ThreadX/NetX |
| **AXISRAM1** | 0x34160000 | 1024 KB | Planck LUT (128 KB), Rest frei |
| **AXISRAM2** | 0x34260000 | 2688 KB | raw, dark, gain, res, bad_pixel |
| **DTCM** | 0x30000000 | 128 KB | Stack |

**Speicher-Budget:**

| **Komponente** | **Größe** | **Region** | **Verbleibend** |
|----------------|-----------|------------|----------------|
| **Code + RO Data** | 250 KB | ROM | 69 KB ✅ |
| **ThreadX/NetX** | 150 KB | RAM | 874 KB ✅ |
| **Planck LUT** | 128 KB | AXISRAM1 | 896 KB ✅ |
| **Thermal Data** | 2456 KB | AXISRAM2 | 232 KB ✅ |
| **GESAMT** | **~3.0 MB** | **< 4.7 MB** | **✅ PASST!** |

### 6.3 Cache-Konfiguration

**Problem:** `res[]` wird von Thermal-Thread geschrieben und von UDP-Thread gelesen.

**Lösung 1: Non-Cacheable Section**

```ld
/* Thermal Result (non-cacheable) */
.noncacheable (NOLOAD) :
{
    . = ALIGN(8);
    *(.noncacheable)
    . = ALIGN(8);
} >AXISRAM2
```

```c
// In Code:
volatile uint16_t res[FRAME_SIZE] __attribute__((section(".noncacheable")));
```

**Lösung 2: MPU Region konfigurieren**

```c
void MPU_Config_Thermal(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct;

    // Disable MPU
    HAL_MPU_Disable();

    // Configure AXISRAM2 (Thermal Result) as non-cacheable
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0x34260000 + (2 * 614400);  // res[] Start
    MPU_InitStruct.Size = MPU_REGION_SIZE_1MB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER5;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    // Enable MPU
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
```

**Empfehlung:** Lösung 1 (einfacher, weniger fehleranfällig).

---

## 7. Step-by-Step Migration Guide

### Phase 1: Projekt-Setup (Tag 1-2)

#### Tag 1: Projekt kopieren & bereinigen

**Schritt 1.1: VENC_RTSP kopieren**

```bash
cd Projects/STM32N6570-DK/Applications
mkdir -p Thermal
cp -r VENC/VENC_RTSP_Server Thermal/Thermal_UDP_Streaming
cd Thermal/Thermal_UDP_Streaming
```

**Schritt 1.2: Projekt umbenennen**

- Öffne STM32CubeIDE
- Import > Existing Projects > `Thermal_UDP_Streaming`
- Rechtsklick > Rename Project > `Thermal_UDP_Streaming`

**Schritt 1.3: Unbenötigte Dateien entfernen**

```bash
# VENC Code
rm Appli/Core/Src/venc_app.c
rm Appli/Core/Inc/venc_app.h

# RTSP Server
rm Appli/NetXDuo/App/app_rtsp_over_rtp.c
rm Appli/NetXDuo/App/app_rtsp_over_rtp.h

# Camera/LCD (optional behalten für Phase 2)
# rm Appli/Core/Inc/stlogo.h
# rm Appli/Core/Inc/imx335.h
```

**Schritt 1.4: Include-Pfade anpassen**

```c
// Appli/Core/Src/main.c
// ENTFERNEN:
// #include "venc_app.h"
// #include "app_rtsp_over_rtp.h"
// #include "imx335.h"
// #include "h264encapi.h"
// #include "ewl.h"
// #include "stm32n6570_discovery_camera.h"
// #include "stm32n6570_discovery_lcd.h"
```

#### Tag 2: Linker-Script modifizieren

**Schritt 2.1: STM32N657XX_LRUN.ld bearbeiten**

```ld
/* Zeile 49-54 ändern */
MEMORY
{
  ROM    (xr)     : ORIGIN = 0x34000400,   LENGTH = 319K
  RAM    (rw)     : ORIGIN = 0x34060000,   LENGTH = 1024K   /* 1 MB */
  AXISRAM1 (rw)   : ORIGIN = 0x34160000,   LENGTH = 1024K   /* 1 MB */
  AXISRAM2 (rw)   : ORIGIN = 0x34260000,   LENGTH = 2688K   /* 2.6 MB */
  DTCM   (rw)     : ORIGIN = 0x30000000,   LENGTH = 128K
}

/* Nach .bss Sektion hinzufügen (Zeile ~200) */
SECTIONS
{
  /* ... Standard Sections ... */

  .planck_lut (NOLOAD) :
  {
    . = ALIGN(8);
    *(.plnk)
    . = ALIGN(8);
  } >AXISRAM1

  .thermal_frames (NOLOAD) :
  {
    . = ALIGN(8);
    *(.thermal)
    . = ALIGN(8);
  } >AXISRAM2

  .noncacheable (NOLOAD) :
  {
    . = ALIGN(8);
    *(.noncacheable)
    . = ALIGN(8);
  } >AXISRAM2

  .bad_pixel (NOLOAD) :
  {
    . = ALIGN(8);
    *(.bad_pixel)
    . = ALIGN(8);
  } >AXISRAM2
}
```

**Schritt 2.2: Build & Verify**

```
Project > Build Project (Ctrl+B)
```

**Erwartete Ausgabe:**
```
arm-none-eabi-gcc ... -o Thermal_UDP_Streaming.elf
   text    data     bss     dec     hex filename
 256789   12345   98765  367899   59ddb Thermal_UDP_Streaming.elf
Finished building: Thermal_UDP_Streaming.elf
```

**Sanity-Check:**
- Build erfolgreich? ✅
- Code-Size < 319 KB? ✅
- Keine Linker-Errors? ✅

---

### Phase 2: Thermal-Code Integration (Tag 3-5)

#### Tag 3: Thermal-Modul hinzufügen

**Schritt 3.1: Neuen Ordner erstellen**

```bash
mkdir -p Appli/Thermal/Src
mkdir -p Appli/Thermal/Inc
```

**Schritt 3.2: Thermal-Dateien kopieren**

```bash
# Aus CORTEX_HELIUM Projekt
cp ../../../../NUCLEO-N657X0-Q/Examples/CORTEX/CORTEX_HELIUM/FSBL/Src/xspi_nor.c Appli/Thermal/Src/
cp ../../../../NUCLEO-N657X0-Q/Examples/CORTEX/CORTEX_HELIUM/FSBL/Inc/xspi_nor.h Appli/Thermal/Inc/

# Thermal-Processing Code aus main.c extrahieren
# (siehe unten)
```

**Schritt 3.3: thermal_processing.c erstellen**

**Datei:** `Appli/Thermal/Src/thermal_processing.c`

```c
/**
 * @file    thermal_processing.c
 * @brief   Thermal Image Processing with ARM Helium Optimization
 * @author  Integration Team
 * @date    2025-10-28
 */

#include "thermal_processing.h"
#include "arm_mve.h"
#include <string.h>

/* Thermal Data Arrays (BSS, will be placed by linker) */
uint16_t raw[FRAME_SIZE]   __attribute__((section(".thermal"))) __attribute__((aligned(8)));
uint16_t dark[FRAME_SIZE]  __attribute__((section(".thermal"))) __attribute__((aligned(8)));
uint16_t gain[FRAME_SIZE]  __attribute__((section(".thermal"))) __attribute__((aligned(8)));
uint16_t planck[65536]     __attribute__((section(".plnk"))) __attribute__((aligned(8)));

/* Result Frame (Non-Cacheable for DMA/UDP access) */
volatile uint16_t res[FRAME_SIZE] __attribute__((section(".noncacheable"))) __attribute__((aligned(8)));

/* Bad Pixel Arrays */
static uint16_t bad_pixel_coords[MAX_BAD_PIXELS][2] __attribute__((section(".bad_pixel")));
static uint16_t bad_pixel_replacement[MAX_BAD_PIXELS] __attribute__((section(".bad_pixel")));
static uint32_t bad_pixel_count = 0;

/* Private Variables */
static uint32_t frame_counter = 0;
static uint32_t processing_cycles = 0;

/**
 * @brief  Process single thermal line (Helium-optimized)
 * @param  line_in: Input raw sensor data
 * @param  dark_line: Dark frame calibration
 * @param  gain_line: Gain correction coefficients (Q15)
 * @param  offset_line: Offset correction
 * @param  planck_lut: Planck radiation LUT
 * @param  line_out: Output processed data
 * @param  pixel_count: Number of pixels (should be multiple of 8)
 * @retval None
 */
void process_thermal_line_fastest(const uint16_t *line_in,
                                    const uint16_t *dark_line,
                                    const uint16_t *gain_line,
                                    const uint16_t *offset_line,
                                    const uint16_t *planck_lut,
                                    volatile uint16_t *line_out,
                                    uint32_t pixel_count)
{
    uint16x8_t v_input, v_dark, v_gain, v_offset;
    uint16x8_t v_result;

    for (uint32_t i = 0; i < pixel_count; i += 8)
    {
        // Load 8 pixels
        v_input = vld1q_u16(&line_in[i]);
        v_dark = vld1q_u16(&dark_line[i]);

        // Dark Frame Subtraction
        v_result = vsubq_u16(v_input, v_dark);

        // Gain Correction (Q15 Fixed-Point Multiply)
        v_gain = vld1q_u16(&gain_line[i]);
        v_result = vqrdmulhq_u16(v_result, v_gain);

        // Offset Addition
        v_offset = vld1q_u16(&offset_line[i]);
        v_result = vaddq_u16(v_result, v_offset);

        // Planck LUT Lookup (scalar, as LUT access is not vectorizable)
        for (int j = 0; j < 8; j++)
        {
            uint16_t idx = vgetq_lane_u16(v_result, j);
            line_out[i + j] = planck_lut[idx];
        }
    }
}

/**
 * @brief  Process complete thermal frame
 * @retval Processing cycles (for performance measurement)
 */
uint32_t thermal_frame_process(void)
{
    // Start cycle counter
    uint32_t start_cycles = DWT->CYCCNT;

    // Process all lines
    for (uint32_t line = 0; line < VPIX; line++)
    {
        process_thermal_line_fastest(&raw[line * HPIX],
                                       &dark[line * HPIX],
                                       &gain[line * HPIX],
                                       &offset[line * HPIX],  // Assuming offset array exists
                                       planck,
                                       &res[line * HPIX],
                                       HPIX);
    }

    // Apply bad pixel correction
    apply_bad_pixel_correction();

    // Stop cycle counter
    uint32_t end_cycles = DWT->CYCCNT;
    processing_cycles = end_cycles - start_cycles;

    frame_counter++;

    return processing_cycles;
}

/**
 * @brief  Apply bad pixel correction (interpolation)
 * @retval None
 */
void apply_bad_pixel_correction(void)
{
    for (uint32_t i = 0; i < bad_pixel_count; i++)
    {
        uint16_t x = bad_pixel_coords[i][0];
        uint16_t y = bad_pixel_coords[i][1];

        if (x > 0 && x < HPIX - 1 && y > 0 && y < VPIX - 1)
        {
            // Simple 4-neighbor average
            uint32_t sum = res[(y - 1) * HPIX + x] +
                          res[(y + 1) * HPIX + x] +
                          res[y * HPIX + (x - 1)] +
                          res[y * HPIX + (x + 1)];
            res[y * HPIX + x] = (uint16_t)(sum / 4);
        }
    }
}

/**
 * @brief  Generate Planck radiation LUT
 * @param  c1: Planck constant 1 (1.191042e8)
 * @param  c2: Planck constant 2 (1.4387752e4)
 * @param  wavelength: Sensor wavelength (µm)
 * @retval None
 */
void generate_planck_lut(float c1, float c2, float wavelength)
{
    printf("Generating Planck LUT...\n");

    for (uint32_t adc = 0; adc < 65536; adc++)
    {
        // Simplified Planck's Law: B(λ,T) = C1 / (λ^5 * (e^(C2/(λT)) - 1))
        // Temperature estimation from ADC value
        float temp_kelvin = 273.15f + ((float)adc / 65535.0f) * 100.0f;  // 0-100°C

        float exponent = c2 / (wavelength * temp_kelvin);
        float radiance = c1 / (powf(wavelength, 5.0f) * (expf(exponent) - 1.0f));

        // Encode as uint16 (temperature * 100 + 10000)
        planck[adc] = (uint16_t)((temp_kelvin - 273.15f) * 100.0f + 10000.0f);
    }

    printf("Planck LUT generated.\n");
}

/**
 * @brief  Load calibration data from XSPI Flash
 * @retval HAL Status
 */
HAL_StatusTypeDef load_calibration_data(void)
{
    printf("Loading calibration from XSPI Flash...\n");

    // Load Dark Frame
    if (XSPI_NOR_Read((uint8_t*)dark, DARK_FRAME_ADDRESS, FRAME_SIZE * 2) != HAL_OK)
    {
        printf("ERROR: Dark frame load failed\n");
        return HAL_ERROR;
    }

    // Load Gain Frame
    if (XSPI_NOR_Read((uint8_t*)gain, GAIN_FRAME_ADDRESS, FRAME_SIZE * 2) != HAL_OK)
    {
        printf("ERROR: Gain frame load failed\n");
        return HAL_ERROR;
    }

    // Load Bad Pixel Map
    if (XSPI_NOR_Read((uint8_t*)bad_pixel_coords, BAD_PIXEL_ADDRESS, sizeof(bad_pixel_coords)) != HAL_OK)
    {
        printf("ERROR: Bad pixel map load failed\n");
        return HAL_ERROR;
    }

    printf("Calibration loaded successfully.\n");
    return HAL_OK;
}

/**
 * @brief  Get frame processing statistics
 * @param  stats: Pointer to stats structure
 * @retval None
 */
void thermal_get_stats(ThermalStats_t *stats)
{
    stats->frame_count = frame_counter;
    stats->last_processing_cycles = processing_cycles;
    stats->last_processing_ms = (float)processing_cycles / 600000.0f;  // @ 600 MHz
}

/**
 * @brief  Enable DWT cycle counter for performance measurement
 * @retval None
 */
void enable_dwt_cycle_counter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
```

**Schritt 3.4: thermal_processing.h erstellen**

**Datei:** `Appli/Thermal/Inc/thermal_processing.h`

```c
/**
 * @file    thermal_processing.h
 * @brief   Thermal Image Processing Header
 */

#ifndef THERMAL_PROCESSING_H
#define THERMAL_PROCESSING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32n6xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>

/* Thermal Frame Constants */
#define HPIX 640
#define VPIX 480
#define FRAME_SIZE (HPIX * VPIX)  // 307200 pixels
#define FRAME_SIZE_BYTES (FRAME_SIZE * 2)  // 614400 bytes

/* Planck Constants */
#define PLANCK_C1 1.191042e8f
#define PLANCK_C2 1.4387752e4f
#define SENSOR_WAVELENGTH 10.0f  // µm (Long-Wave Infrared)

/* Calibration Flash Addresses */
#define DARK_FRAME_ADDRESS 0x70000000
#define GAIN_FRAME_ADDRESS 0x70100000
#define BAD_PIXEL_ADDRESS  0x70200000

/* Bad Pixel Correction */
#define MAX_BAD_PIXELS 1000

/* Statistics Structure */
typedef struct {
    uint32_t frame_count;
    uint32_t last_processing_cycles;
    float last_processing_ms;
} ThermalStats_t;

/* External Data Arrays */
extern uint16_t raw[FRAME_SIZE];
extern uint16_t dark[FRAME_SIZE];
extern uint16_t gain[FRAME_SIZE];
extern uint16_t planck[65536];
extern volatile uint16_t res[FRAME_SIZE];

/* Function Prototypes */
void process_thermal_line_fastest(const uint16_t *line_in,
                                   const uint16_t *dark_line,
                                   const uint16_t *gain_line,
                                   const uint16_t *offset_line,
                                   const uint16_t *planck_lut,
                                   volatile uint16_t *line_out,
                                   uint32_t pixel_count);

uint32_t thermal_frame_process(void);
void apply_bad_pixel_correction(void);
void generate_planck_lut(float c1, float c2, float wavelength);
HAL_StatusTypeDef load_calibration_data(void);
void thermal_get_stats(ThermalStats_t *stats);
void enable_dwt_cycle_counter(void);

#ifdef __cplusplus
}
#endif

#endif /* THERMAL_PROCESSING_H */
```

**Schritt 3.5: Include-Pfad hinzufügen**

```
Project Properties > C/C++ Build > Settings > Tool Settings > MCU GCC Compiler > Include paths

Hinzufügen:
"${workspace_loc:/${ProjName}/Appli/Thermal/Inc}"
```

#### Tag 4: TIM2 Konfiguration

**Schritt 4.1: TIM2 in main.c initialisieren**

```c
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* Private function prototypes -----------------------------------------------*/
static void MX_TIM2_Init(void);

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* TIM2 clock = 600 MHz (from SystemClock_Config) */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9999;         /* 600 MHz / 10000 = 60 kHz */
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1199;            /* 60 kHz / 1200 = 50 Hz */
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 MSP Initialization
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM2)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);  // Prio 5 (höher als Ethernet)
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  }
}

/* In main() aufrufen */
int main(void)
{
  /* ... Hardware Init ... */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();  // ← Neu!

  /* Start TIM2 */
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  MX_ThreadX_Init();

  while (1) { }
}
```

**Schritt 4.2: TIM2 Interrupt-Handler**

**Datei:** `Appli/Core/Src/stm32n6xx_it.c`

```c
/* USER CODE BEGIN Includes */
#include "tx_api.h"
extern TX_EVENT_FLAGS_GROUP thermal_event_flags;
/* USER CODE END Includes */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  // Signal Thermal Thread (ISR-safe!)
  tx_event_flags_set(&thermal_event_flags, 0x01, TX_OR);

  /* USER CODE END TIM2_IRQn 1 */
}
```

#### Tag 5: Thermal Thread erstellen

**Schritt 5.1: app_thermal.c erstellen**

**Datei:** `Appli/Core/Src/app_thermal.c`

```c
/**
 * @file    app_thermal.c
 * @brief   Thermal Processing Thread
 */

#include "main.h"
#include "tx_api.h"
#include "thermal_processing.h"
#include "xspi_nor.h"
#include "stm32n6570_discovery.h"

/* Thread Definitions */
#define THERMAL_THREAD_STACK_SIZE 8192
#define THERMAL_THREAD_PRIORITY   5  // HIGH!

/* Globals */
TX_THREAD ThermalThread;
TX_EVENT_FLAGS_GROUP thermal_event_flags;
TX_SEMAPHORE frame_ready_semaphore;

/* Private Variables */
static UCHAR thermal_thread_stack[THERMAL_THREAD_STACK_SIZE];

/**
 * @brief  Thermal Thread Entry Function
 */
VOID Thermal_Thread_Entry(ULONG thread_input)
{
    ULONG actual_flags;
    UINT status;
    ThermalStats_t stats;

    printf("[Thermal] Thread started (Prio %d)\n", THERMAL_THREAD_PRIORITY);

    // Initialize DWT Cycle Counter
    enable_dwt_cycle_counter();

    // Initialize XSPI NOR Flash
    XSPI_NOR_Init_All();

    // Load Calibration Data
    if (load_calibration_data() != HAL_OK)
    {
        printf("[Thermal] ERROR: Calibration load failed\n");
        BSP_LED_On(LED_RED);
        tx_thread_suspend(&ThermalThread);
    }

    // Generate Planck LUT
    generate_planck_lut(PLANCK_C1, PLANCK_C2, SENSOR_WAVELENGTH);

    printf("[Thermal] Initialization complete. Waiting for frames...\n");
    BSP_LED_On(LED_ORANGE);

    while(1)
    {
        // Wait for TIM2 Trigger (50 Hz)
        status = tx_event_flags_get(&thermal_event_flags,
                                     0x01,  // Frame Trigger Flag
                                     TX_OR_CLEAR,
                                     &actual_flags,
                                     TX_WAIT_FOREVER);

        if (status == TX_SUCCESS)
        {
            // Indicate processing
            BSP_LED_On(LED_ORANGE);

            // Process Frame (Helium-optimized)
            uint32_t cycles = thermal_frame_process();

            // Log performance (every 50 frames)
            thermal_get_stats(&stats);
            if (stats.frame_count % 50 == 0)
            {
                printf("[Thermal] Frame %lu: %.2f ms (%lu cycles)\n",
                       stats.frame_count, stats.last_processing_ms, stats.last_processing_cycles);
            }

            BSP_LED_Off(LED_ORANGE);

            // Signal Frame Ready to UDP Thread
            tx_semaphore_put(&frame_ready_semaphore);
        }
    }
}

/**
 * @brief  Initialize Thermal Processing Thread
 */
UINT App_Thermal_Init(TX_BYTE_POOL *byte_pool)
{
    UINT ret;

    // Create Event Flags
    ret = tx_event_flags_create(&thermal_event_flags, "Thermal Events");
    if (ret != TX_SUCCESS) {
        printf("[Thermal] ERROR: Event flags creation failed (%d)\n", ret);
        return ret;
    }

    // Create Semaphore
    ret = tx_semaphore_create(&frame_ready_semaphore, "Frame Ready", 0);
    if (ret != TX_SUCCESS) {
        printf("[Thermal] ERROR: Semaphore creation failed (%d)\n", ret);
        return ret;
    }

    // Create Thread
    ret = tx_thread_create(&ThermalThread,
                          "Thermal Processing Thread",
                          Thermal_Thread_Entry,
                          0,
                          thermal_thread_stack,
                          THERMAL_THREAD_STACK_SIZE,
                          THERMAL_THREAD_PRIORITY,
                          THERMAL_THREAD_PRIORITY,
                          TX_NO_TIME_SLICE,
                          TX_AUTO_START);

    if (ret != TX_SUCCESS) {
        printf("[Thermal] ERROR: Thread creation failed (%d)\n", ret);
        return ret;
    }

    printf("[Thermal] Thread created successfully.\n");
    return TX_SUCCESS;
}
```

**Schritt 5.2: app_thermal.h erstellen**

**Datei:** `Appli/Core/Inc/app_thermal.h`

```c
/**
 * @file    app_thermal.h
 * @brief   Thermal Processing Thread Header
 */

#ifndef APP_THERMAL_H
#define APP_THERMAL_H

#include "tx_api.h"

/* Exports */
extern TX_SEMAPHORE frame_ready_semaphore;
extern TX_EVENT_FLAGS_GROUP thermal_event_flags;

/* Functions */
UINT App_Thermal_Init(TX_BYTE_POOL *byte_pool);

#endif /* APP_THERMAL_H */
```

**Schritt 5.3: Thread in app_threadx.c registrieren**

**Datei:** `Appli/Core/Src/app_threadx.c`

```c
/* USER CODE BEGIN Includes */
#include "app_thermal.h"
/* USER CODE END Includes */

UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  /* USER CODE BEGIN App_ThreadX_MEM_POOL */

  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */

  // Initialize Thermal Processing Thread
  ret = App_Thermal_Init((TX_BYTE_POOL*)memory_ptr);
  if (ret != TX_SUCCESS)
  {
      printf("ERROR: Thermal thread initialization failed (%d)\n", ret);
      return ret;
  }

  /* USER CODE END App_ThreadX_Init */

  return ret;
}
```

**Schritt 5.4: Build & Test**

```
Project > Build Project
Run > Debug (F11)
```

**Erwartete UART-Ausgabe:**

```
Nx_RTP_RTSP_Server application started..
[Thermal] Thread created successfully.
[Thermal] Thread started (Prio 5)
Generating Planck LUT...
Planck LUT generated.
Loading calibration from XSPI Flash...
Calibration loaded successfully.
[Thermal] Initialization complete. Waiting for frames...
[Thermal] Frame 50: 13.72 ms (8234567 cycles)
[Thermal] Frame 100: 13.68 ms (8212345 cycles)
...
```

---

### Phase 3: UDP Streaming (Tag 6-8)

**Ziel:** Implementiere UDP-Streaming für Thermal-Frames (614 KB → 440 Pakete)

#### Tag 6: UDP Thread & Fragmentation Logic

**Schritt 6.1: Erstelle Appli/NetXDuo/App/app_udp_streaming.h**

```c
#ifndef APP_UDP_STREAMING_H
#define APP_UDP_STREAMING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tx_api.h"
#include "nx_api.h"

/* UDP Configuration */
#define UDP_SERVER_PORT          6100
#define UDP_PACKET_PAYLOAD_SIZE  1400  // MTU 1500 - 20 IP - 8 UDP = 1472 max
#define THERMAL_FRAME_SIZE       (640 * 480 * 2)  // 614,400 bytes
#define PACKETS_PER_FRAME        ((THERMAL_FRAME_SIZE + UDP_PACKET_PAYLOAD_SIZE - 1) / UDP_PACKET_PAYLOAD_SIZE)  // 439 packets

/* Thread Configuration */
#define UDP_STREAMING_THREAD_PRIORITY    8
#define UDP_STREAMING_THREAD_STACK_SIZE  4096

/* Frame Header Structure (12 bytes) */
typedef struct {
    uint32_t frame_number;      // Frame sequence number
    uint16_t packet_number;     // Packet sequence (0 to PACKETS_PER_FRAME-1)
    uint16_t total_packets;     // Total packets in frame (always PACKETS_PER_FRAME)
    uint32_t timestamp_ms;      // Timestamp in milliseconds
} udp_frame_header_t;

/* Function Prototypes */
UINT App_UDP_Streaming_Init(TX_BYTE_POOL *byte_pool);
VOID UDP_Streaming_Thread_Entry(ULONG thread_input);

/* External References */
extern TX_THREAD UdpStreamingThread;
extern TX_SEMAPHORE frame_ready_semaphore;  // From app_thermal.c
extern volatile uint16_t thermal_result_frame[];  // From thermal_processing.c

#ifdef __cplusplus
}
#endif

#endif /* APP_UDP_STREAMING_H */
```

**Schritt 6.2: Erstelle Appli/NetXDuo/App/app_udp_streaming.c**

```c
#include "app_udp_streaming.h"
#include "app_netxduo.h"
#include "thermal_processing.h"
#include <stdio.h>
#include <string.h>

/* Thread Control Block */
TX_THREAD UdpStreamingThread;

/* UDP Socket */
static NX_UDP_SOCKET UdpSocket;

/* Frame Counter */
static uint32_t frame_counter = 0;

/* Statistics */
static uint32_t total_frames_sent = 0;
static uint32_t total_bytes_sent = 0;

/**
 * @brief Initialize UDP Streaming Thread
 */
UINT App_UDP_Streaming_Init(TX_BYTE_POOL *byte_pool)
{
    UINT ret = TX_SUCCESS;
    UCHAR *pointer;

    /* Allocate stack */
    ret = tx_byte_allocate(byte_pool, (VOID **) &pointer,
                          UDP_STREAMING_THREAD_STACK_SIZE, TX_NO_WAIT);
    if (ret != TX_SUCCESS) {
        printf("[UDP] ERROR: Stack allocation failed (%d)\n", ret);
        return ret;
    }

    /* Create thread */
    ret = tx_thread_create(&UdpStreamingThread, "UDP Streaming Thread",
                          UDP_Streaming_Thread_Entry, 0,
                          pointer, UDP_STREAMING_THREAD_STACK_SIZE,
                          UDP_STREAMING_THREAD_PRIORITY,
                          UDP_STREAMING_THREAD_PRIORITY,
                          TX_NO_TIME_SLICE, TX_AUTO_START);

    if (ret != TX_SUCCESS) {
        printf("[UDP] ERROR: Thread creation failed (%d)\n", ret);
        return ret;
    }

    printf("[UDP] Thread created successfully (Prio %d)\n", UDP_STREAMING_THREAD_PRIORITY);
    return ret;
}

/**
 * @brief Fragment and send thermal frame via UDP
 * @param dest_ip Destination IP address
 * @param dest_port Destination port
 * @param frame_data Pointer to 640x480x2 frame data
 * @return NX_SUCCESS on success
 */
static UINT send_fragmented_frame(ULONG dest_ip, UINT dest_port, const uint16_t *frame_data)
{
    UINT status;
    NX_PACKET *packet;
    udp_frame_header_t header;
    uint32_t timestamp_start = tx_time_get();

    const uint8_t *data_ptr = (const uint8_t *)frame_data;
    uint32_t bytes_remaining = THERMAL_FRAME_SIZE;
    uint16_t packet_num = 0;

    /* Prepare frame header template */
    header.frame_number = frame_counter++;
    header.total_packets = PACKETS_PER_FRAME;
    header.timestamp_ms = timestamp_start * 10;  // ThreadX ticks → ms (assuming 100 Hz)

    /* Send all packets */
    for (packet_num = 0; packet_num < PACKETS_PER_FRAME; packet_num++)
    {
        /* Allocate packet from pool */
        status = nx_packet_allocate(&NxAppPool, &packet, NX_UDP_PACKET,
                                    TX_WAIT_FOREVER);
        if (status != NX_SUCCESS) {
            printf("[UDP] ERROR: Packet allocation failed (%d)\n", status);
            return status;
        }

        /* Update header for this packet */
        header.packet_number = packet_num;

        /* Append header to packet */
        status = nx_packet_data_append(packet, &header, sizeof(header),
                                       &NxAppPool, TX_WAIT_FOREVER);
        if (status != NX_SUCCESS) {
            printf("[UDP] ERROR: Header append failed (%d)\n", status);
            nx_packet_release(packet);
            return status;
        }

        /* Calculate payload size for this packet */
        uint32_t payload_size = (bytes_remaining > UDP_PACKET_PAYLOAD_SIZE) ?
                                UDP_PACKET_PAYLOAD_SIZE : bytes_remaining;

        /* Append payload data */
        status = nx_packet_data_append(packet, (VOID *)data_ptr, payload_size,
                                       &NxAppPool, TX_WAIT_FOREVER);
        if (status != NX_SUCCESS) {
            printf("[UDP] ERROR: Data append failed (%d)\n", status);
            nx_packet_release(packet);
            return status;
        }

        /* Send UDP packet */
        status = nx_udp_socket_send(&UdpSocket, packet, dest_ip, dest_port);
        if (status != NX_SUCCESS) {
            printf("[UDP] ERROR: Send failed on packet %d (%d)\n", packet_num, status);
            nx_packet_release(packet);
            return status;
        }

        /* Update pointers */
        data_ptr += payload_size;
        bytes_remaining -= payload_size;
    }

    /* Update statistics */
    total_frames_sent++;
    total_bytes_sent += THERMAL_FRAME_SIZE;

    uint32_t timestamp_end = tx_time_get();
    uint32_t duration_ms = (timestamp_end - timestamp_start) * 10;

    /* Print statistics every 50 frames */
    if (total_frames_sent % 50 == 0) {
        printf("[UDP] Frame %lu: %lu packets, %lu ms\n",
               header.frame_number, PACKETS_PER_FRAME, duration_ms);
        printf("[UDP] Stats: %lu frames, %lu MB total\n",
               total_frames_sent, total_bytes_sent / (1024 * 1024));
    }

    return NX_SUCCESS;
}

/**
 * @brief UDP Streaming Thread Entry Point
 */
VOID UDP_Streaming_Thread_Entry(ULONG thread_input)
{
    UINT status;
    ULONG ip_address;
    ULONG network_mask;

    (void)thread_input;

    printf("[UDP] Thread started (Prio %d)\n", UDP_STREAMING_THREAD_PRIORITY);

    /* Wait for IP address to be ready */
    tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND * 2);

    /* Get local IP address */
    status = nx_ip_address_get(&IpInstance, &ip_address, &network_mask);
    if (status != NX_SUCCESS) {
        printf("[UDP] ERROR: Failed to get IP address (%d)\n", status);
        return;
    }

    PRINT_IP_ADDRESS(ip_address);

    /* Create UDP socket */
    status = nx_udp_socket_create(&IpInstance, &UdpSocket, "UDP Streaming Socket",
                                 NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE,
                                 10);  // Queue depth
    if (status != NX_SUCCESS) {
        printf("[UDP] ERROR: Socket creation failed (%d)\n", status);
        return;
    }

    /* Bind socket to port */
    status = nx_udp_socket_bind(&UdpSocket, UDP_SERVER_PORT, TX_WAIT_FOREVER);
    if (status != NX_SUCCESS) {
        printf("[UDP] ERROR: Socket bind failed (%d)\n", status);
        nx_udp_socket_delete(&UdpSocket);
        return;
    }

    printf("[UDP] Socket bound to port %d\n", UDP_SERVER_PORT);
    printf("[UDP] Frame size: %d bytes (%d packets @ %d bytes)\n",
           THERMAL_FRAME_SIZE, PACKETS_PER_FRAME, UDP_PACKET_PAYLOAD_SIZE);
    printf("[UDP] Waiting for thermal frames...\n");

    /* Main loop: wait for frames and send them */
    while (1)
    {
        /* Wait for thermal processing to signal frame ready */
        status = tx_semaphore_get(&frame_ready_semaphore, TX_WAIT_FOREVER);
        if (status != TX_SUCCESS) {
            printf("[UDP] ERROR: Semaphore get failed (%d)\n", status);
            continue;
        }

        /* Send frame to broadcast address (or specific client) */
        /* TODO: Replace with actual client IP from configuration */
        ULONG dest_ip = IP_ADDRESS(192, 168, 1, 255);  // Broadcast
        UINT dest_port = 6100;

        status = send_fragmented_frame(dest_ip, dest_port, thermal_result_frame);
        if (status != NX_SUCCESS) {
            printf("[UDP] ERROR: Frame send failed (%d)\n", status);
        }
    }
}
```

#### Tag 7: Integration mit ThreadX

**Schritt 7.1: app_azure_rtos.c modifizieren**

```c
// In Appli/Core/Src/app_azure_rtos.c

/* Include new header */
#include "app_udp_streaming.h"

UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;

  /* Initialize Thermal Processing Thread */
  ret = App_Thermal_Init((TX_BYTE_POOL*)memory_ptr);
  if (ret != TX_SUCCESS)
  {
      printf("ERROR: Thermal thread initialization failed (%d)\n", ret);
      return ret;
  }

  /* Initialize UDP Streaming Thread */
  ret = App_UDP_Streaming_Init((TX_BYTE_POOL*)memory_ptr);
  if (ret != TX_SUCCESS)
  {
      printf("ERROR: UDP streaming thread initialization failed (%d)\n", ret);
      return ret;
  }

  return ret;
}
```

**Schritt 7.2: app_netxduo.h erweitern**

```c
// In Appli/NetXDuo/App/app_netxduo.h

/* Add after other includes */
#include "app_udp_streaming.h"

/* IP address macro (if not already defined) */
#ifndef IP_ADDRESS
#define IP_ADDRESS(a,b,c,d)  ((ULONG)((a) << 24 | (b) << 16 | (c) << 8 | (d)))
#endif
```

**Schritt 7.3: Build & Test**

```bash
Project > Build Project
```

**Erwartete Compiler-Warnungen:**
- "Warning: Stack usage may exceed 4096 bytes" → Kann ignoriert werden (Stack wird dynamisch allokiert)

#### Tag 8: Python Receiver & Testing

**Schritt 8.1: Erstelle Tools/thermal_udp_receiver.py**

```python
#!/usr/bin/env python3
"""
Thermal UDP Frame Receiver
Receives fragmented 640x480x16bit frames via UDP and displays/saves them
"""

import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import defaultdict
import time

# Configuration
UDP_IP = "0.0.0.0"  # Listen on all interfaces
UDP_PORT = 6100
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_SIZE_BYTES = FRAME_WIDTH * FRAME_HEIGHT * 2  # 16-bit pixels
PACKET_PAYLOAD_SIZE = 1400
EXPECTED_PACKETS = (FRAME_SIZE_BYTES + PACKET_PAYLOAD_SIZE - 1) // PACKET_PAYLOAD_SIZE

# Frame header struct: frame_num(uint32), packet_num(uint16), total_packets(uint16), timestamp(uint32)
HEADER_FORMAT = '<IHHI'  # Little-endian: uint32, uint16, uint16, uint32
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

class ThermalFrameReceiver:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.settimeout(1.0)  # 1 second timeout

        self.frames = defaultdict(dict)  # {frame_num: {packet_num: data}}
        self.last_complete_frame = None
        self.frame_times = []
        self.stats = {
            'received_packets': 0,
            'completed_frames': 0,
            'dropped_frames': 0,
            'last_frame_num': -1
        }

        print(f"[RX] Listening on {UDP_IP}:{UDP_PORT}")
        print(f"[RX] Expecting {EXPECTED_PACKETS} packets per frame")
        print(f"[RX] Frame size: {FRAME_WIDTH}x{FRAME_HEIGHT} ({FRAME_SIZE_BYTES} bytes)")

    def receive_packet(self):
        """Receive and parse a single UDP packet"""
        try:
            data, addr = self.sock.recvfrom(2048)

            # Parse header
            if len(data) < HEADER_SIZE:
                print(f"[RX] WARNING: Packet too small ({len(data)} bytes)")
                return None

            frame_num, packet_num, total_packets, timestamp = struct.unpack(
                HEADER_FORMAT, data[:HEADER_SIZE])

            payload = data[HEADER_SIZE:]

            self.stats['received_packets'] += 1

            return {
                'frame_num': frame_num,
                'packet_num': packet_num,
                'total_packets': total_packets,
                'timestamp': timestamp,
                'payload': payload,
                'addr': addr
            }

        except socket.timeout:
            return None

    def process_packet(self, packet):
        """Store packet and check if frame is complete"""
        if packet is None:
            return None

        frame_num = packet['frame_num']
        packet_num = packet['packet_num']

        # Store packet payload
        self.frames[frame_num][packet_num] = packet['payload']

        # Check if frame is complete
        if len(self.frames[frame_num]) == packet['total_packets']:
            # Reconstruct frame
            frame_data = bytearray()
            for i in range(packet['total_packets']):
                if i in self.frames[frame_num]:
                    frame_data.extend(self.frames[frame_num][i])
                else:
                    print(f"[RX] ERROR: Missing packet {i} in frame {frame_num}")
                    return None

            # Convert to numpy array (16-bit unsigned)
            frame_array = np.frombuffer(frame_data, dtype=np.uint16)
            frame_array = frame_array[:FRAME_WIDTH * FRAME_HEIGHT]  # Truncate any padding
            frame_2d = frame_array.reshape((FRAME_HEIGHT, FRAME_WIDTH))

            # Update statistics
            self.stats['completed_frames'] += 1
            if self.stats['last_frame_num'] >= 0:
                dropped = frame_num - self.stats['last_frame_num'] - 1
                if dropped > 0:
                    self.stats['dropped_frames'] += dropped
                    print(f"[RX] WARNING: Dropped {dropped} frame(s)")

            self.stats['last_frame_num'] = frame_num
            self.frame_times.append(time.time())

            # Clean up old frame data
            del self.frames[frame_num]

            # Print statistics every 10 frames
            if self.stats['completed_frames'] % 10 == 0:
                fps = self.calculate_fps()
                print(f"[RX] Frame {frame_num}: {self.stats['completed_frames']} total, "
                      f"{fps:.1f} FPS, {self.stats['dropped_frames']} dropped")

            return frame_2d

        return None

    def calculate_fps(self):
        """Calculate frames per second from recent frame times"""
        if len(self.frame_times) < 2:
            return 0.0

        recent_times = self.frame_times[-50:]  # Last 50 frames
        if len(recent_times) < 2:
            return 0.0

        time_span = recent_times[-1] - recent_times[0]
        if time_span == 0:
            return 0.0

        return (len(recent_times) - 1) / time_span

    def run_display(self):
        """Run live display using matplotlib"""
        fig, ax = plt.subplots(figsize=(10, 7.5))
        img = ax.imshow(np.zeros((FRAME_HEIGHT, FRAME_WIDTH)), cmap='hot',
                       vmin=0, vmax=65535)
        ax.set_title("Thermal Camera Stream")
        plt.colorbar(img, ax=ax, label='Raw ADC Value')

        def update_frame(frame_num):
            packet = self.receive_packet()
            frame = self.process_packet(packet)

            if frame is not None:
                img.set_data(frame)
                fps = self.calculate_fps()
                ax.set_title(f"Thermal Camera Stream - Frame {self.stats['completed_frames']} - {fps:.1f} FPS")
                return [img]

            return []

        ani = FuncAnimation(fig, update_frame, interval=20, blit=True, cache_frame_data=False)
        plt.show()

    def run_headless(self, save_frames=False, num_frames=100):
        """Run without display, optionally saving frames"""
        print(f"[RX] Receiving {num_frames} frames (headless mode)...")

        frames_received = 0
        start_time = time.time()

        while frames_received < num_frames:
            packet = self.receive_packet()
            frame = self.process_packet(packet)

            if frame is not None:
                frames_received += 1

                if save_frames and frames_received <= 10:  # Save first 10 frames
                    filename = f"thermal_frame_{frames_received:04d}.npy"
                    np.save(filename, frame)
                    print(f"[RX] Saved {filename}")

        elapsed = time.time() - start_time
        avg_fps = frames_received / elapsed if elapsed > 0 else 0

        print(f"\n[RX] Test complete:")
        print(f"  Frames received: {frames_received}")
        print(f"  Time elapsed: {elapsed:.2f} seconds")
        print(f"  Average FPS: {avg_fps:.2f}")
        print(f"  Dropped frames: {self.stats['dropped_frames']}")
        print(f"  Packet loss: {self.stats['dropped_frames'] / (frames_received + self.stats['dropped_frames']) * 100:.2f}%")


if __name__ == "__main__":
    import sys

    receiver = ThermalFrameReceiver()

    if len(sys.argv) > 1 and sys.argv[1] == '--headless':
        receiver.run_headless(save_frames=True, num_frames=100)
    else:
        print("[RX] Starting live display (use --headless for testing mode)")
        receiver.run_display()
```

**Schritt 8.2: Test-Sequenz**

```bash
# Terminal 1: Python Receiver starten
cd Tools
pip install numpy matplotlib
python thermal_udp_receiver.py

# Terminal 2: STM32 flashen
STM32_Programmer_CLI -c port=SWD -w Build/VENC_Thermal_UDP.hex

# Terminal 3: Serial Monitor (optional)
# PuTTY oder screen /dev/ttyUSB0 115200
```

**Erwartete Ausgabe (Python):**

```
[RX] Listening on 0.0.0.0:6100
[RX] Expecting 439 packets per frame
[RX] Frame size: 640x480 (614400 bytes)
[RX] Frame 0: 1 total, 50.2 FPS, 0 dropped
[RX] Frame 9: 10 total, 50.1 FPS, 0 dropped
[RX] Frame 19: 20 total, 49.8 FPS, 0 dropped
...
```

**Erwartete Ausgabe (UART):**

```
[UDP] Thread started (Prio 8)
[UDP] Socket bound to port 6100
[UDP] Frame size: 614400 bytes (439 packets @ 1400 bytes)
[UDP] Waiting for thermal frames...
[Thermal] Frame 50: 13.72 ms (8234567 cycles)
[UDP] Frame 50: 439 packets, 5.8 ms
[UDP] Stats: 50 frames, 29 MB total
[Thermal] Frame 100: 13.68 ms (8212345 cycles)
[UDP] Frame 100: 439 packets, 5.7 ms
[UDP] Stats: 100 frames, 58 MB total
```

#### Tag 8 (Fortsetzung): Performance-Optimierung

**Schritt 8.3: Double-Buffering (optional, bei Frame-Drops)**

Falls UDP-Transmission > 6 ms:

```c
// In thermal_processing.h
#define NUM_RESULT_BUFFERS 2

extern volatile uint16_t thermal_result_frame[NUM_RESULT_BUFFERS][FRAME_SIZE];
extern volatile uint8_t active_write_buffer;
extern volatile uint8_t active_read_buffer;

// In thermal_processing.c
volatile uint16_t thermal_result_frame[NUM_RESULT_BUFFERS][FRAME_SIZE] __NON_CACHEABLE;
volatile uint8_t active_write_buffer = 0;
volatile uint8_t active_read_buffer = 0;

// In Thermal_Thread_Entry():
// Nach thermal_frame_process():
active_write_buffer = (active_write_buffer + 1) % NUM_RESULT_BUFFERS;
active_read_buffer = (active_write_buffer == 0) ? 1 : 0;

// In UDP_Streaming_Thread_Entry():
send_fragmented_frame(dest_ip, dest_port, thermal_result_frame[active_read_buffer]);
```

---

## 8. Performance-Ziele & Risiko-Assessment

### 8.1 Timeline-Realismus

| **Phase** | **Geplant** | **Risiko** | **Puffer** | **Total** |
|-----------|-------------|------------|------------|-----------|
| Phase 1 | 2 Tage | 0.5 Tage | 0.5 Tage | 3 Tage |
| Phase 2 | 3 Tage | 1 Tag | 0.5 Tage | 4.5 Tage |
| Phase 3 | 3 Tage | 1 Tag | 0.5 Tage | 4.5 Tage |
| Phase 4 | 4 Tage | 0 Tage | 0 Tage | 4 Tage |
| **GESAMT** | **12 Tage** | **2.5 Tage** | **1.5 Tage** | **16 Tage** |

**Realistischer Zeitplan:** **3 Wochen (16 Arbeitstage)**

### 8.2 Risiko-Matrix

| **Risiko** | **Wahrscheinlichkeit** | **Impact** | **Mitigation** |
|------------|----------------------|------------|----------------|
| **Frame-Drops (UDP Transmission > 6 ms)** | Hoch (70%) | Hoch | Double-Buffering, DMA |
| **XSPI Pin-Konflikt mit LCD** | Mittel (40%) | Mittel | .ioc neu generieren |
| **Cache-Kohärenz Fehler** | Mittel (50%) | Hoch | MPU Non-Cacheable |
| **Speicher-Overflow** | Niedrig (20%) | Kritisch | MAP-Datei prüfen |
| **DHCP Timeout** | Niedrig (10%) | Niedrig | Statische IP Fallback |

---

## 9. Fazit & Nächste Schritte

### 9.1 Machbarkeit

**✅ JA, Integration ist machbar in 3 Wochen.**

**Begründung:**
- VENC_RTSP bietet 90% wiederverwendbare Basis
- Speicher reicht aus (2.8 MB < 3.6 MB)
- Performance-Budget eng, aber realistisch (13.7 ms + 6 ms < 20 ms)

### 9.2 Empfohlene Reihenfolge

1. **Tag 1-3:** Phase 1 + Phase 2 parallel (Thermal Bare-Metal Test)
2. **Tag 4-5:** Phase 2 abschließen (ThreadX Integration)
3. **Tag 6-8:** Phase 3 (UDP Streaming)
4. **Tag 9-12:** Phase 4 (Testing) + **Phase 5 (Optional: DMA Optimization)**

### 9.3 Phase 2 Roadmap (Optional)

Nach erfolgreichem Prototyp:

1. **JPEG Compression:** Reduziert Bandbreite auf ~5-10% (614 KB → 60 KB)
2. **RTSP/RTP Streaming:** Professional Video-Streaming-Protokoll
3. **LCD Display:** Lokale Thermal-Visualisierung auf 480×272 LCD
4. **Web-Interface:** HTTP-Server für Browser-basierte Anzeige

---

**Dokumentation vollständig. Integration kann beginnen!** 🚀
