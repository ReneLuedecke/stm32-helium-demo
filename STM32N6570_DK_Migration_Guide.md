# STM32N6570-DK Discovery Kit Migration & Integration Guide

**Target Board:** STM32N6570-DK Discovery Kit
**Source Projects:**
1. Thermal Camera Demo (NUCLEO-N657X0-Q)
2. UDP Echo Server (NUCLEO-N657X0-Q)

**Ziel:** Thermische Bilddaten über UDP/Ethernet versenden auf dem Discovery Kit

**Erstellt:** 2025-10-28
**Status:** Migration Guide v1.0

---

## Inhaltsverzeichnis

1. [Hardware-Überblick: STM32N6570-DK](#hardware-überblick-stm32n6570-dk)
2. [NUCLEO vs Discovery Kit Unterschiede](#nucleo-vs-discovery-kit-unterschiede)
3. [Pin-Mapping Migration](#pin-mapping-migration)
4. [Projekt-Setup für Discovery Kit](#projekt-setup-für-discovery-kit)
5. [Schritt-für-Schritt Migration](#schritt-für-schritt-migration)
6. [Integration: Thermal + UDP auf Discovery Kit](#integration-thermal--udp-auf-discovery-kit)
7. [Testing & Debugging](#testing--debugging)
8. [Troubleshooting](#troubleshooting)

---

## Hardware-Überblick: STM32N6570-DK

### Board-Spezifikationen

| **Feature** | **STM32N6570-DK Details** |
|-------------|---------------------------|
| **MCU** | STM32N657X0H3QU (TFBGA361) |
| **CPU** | ARM Cortex-M55 @ 600 MHz |
| **RAM** | 2 MB AXISRAM + 128 KB DTCM + 64 KB ITCM |
| **Flash** | Extern: 512 Mbit Octal SPI NOR (MX25UM51245G) |
| **Ethernet PHY** | RTL8211F-CG (Gigabit, RGMII) oder LAN8742A (100 Mbit, RMII) |
| **Display** | 4.3" LCD (480×272) mit Touch |
| **Audio** | Audio Codec (SAI) |
| **USB** | USB-C (OTG FS/HS) |
| **Debug** | ST-LINK V3E (onboard) |
| **Expansion** | Arduino Uno V3, PMOD, MB1166 connectors |

### Wichtige Board-Ressourcen

| **Komponente** | **Interface** | **Pins/Details** |
|----------------|---------------|------------------|
| **User LEDs** | GPIO | LED1 (Orange), LED2 (Green), LED3 (Red) |
| **User Buttons** | GPIO | USER (Blue), RESET |
| **UART Console** | USART1 | PE5 (TX), PE6 (RX) - via ST-LINK VCP |
| **Ethernet** | ETH1 (RMII/RGMII) | Siehe Pin-Mapping Tabelle |
| **XSPI NOR Flash** | XSPI2 (Octal DTR) | PG6-13, PO0-7 |
| **SD Card** | SDMMC1 | PC8-12, PD2 |
| **LCD Interface** | LTDC | Diverse Pins |

### Power Supply

- **USB-C:** 5V via ST-LINK oder USB OTG
- **External:** 5V via DC Jack (optional)
- **VDDIO:** Mehrere I/O-Domänen (VDDIO2-4) aktiviert

---

## NUCLEO vs Discovery Kit Unterschiede

### 1. Formfaktor

| **Aspekt** | **NUCLEO-N657X0-Q** | **STM32N6570-DK** |
|------------|---------------------|-------------------|
| **Größe** | Nucleo-144 (130×100mm) | Discovery Kit (~150×120mm) |
| **Steckverbinder** | Arduino + Morpho Headers | Arduino + PMOD + MB1166 |
| **Display** | Keine | 4.3" LCD (480×272) |
| **Audio** | Keine | Audio Codec |

### 2. Pin-Unterschiede (Kritisch!)

#### UART1 (Console)

| **Signal** | **NUCLEO** | **Discovery Kit** | **Konflikt?** |
|------------|-----------|-------------------|--------------|
| **USART1_TX** | ? (unbekannt) | PE5 | ⚠️ Muss geprüft werden |
| **USART1_RX** | ? (unbekannt) | PE6 | ⚠️ Muss geprüft werden |

**Hinweis:** Das UDP Echo Server Projekt verwendet bereits PE5/PE6 (siehe stm32n6xx_hal_msp.c:225-241). Dies ist kompatibel mit dem DK.

#### Ethernet

| **Signal** | **Discovery Kit** | **Alternative Funktion** |
|------------|-------------------|-------------------------|
| **ETH_REF_CLK** | PF7 | - |
| **ETH_MDIO** | PF4 | - |
| **ETH_MDC** | PG11 | - |
| **ETH_CRS_DV** | PF10 | - |
| **ETH_RXD0** | PF14 | - |
| **ETH_RXD1** | PF15 | - |
| **ETH_TX_EN** | PF11 | - |
| **ETH_TXD0** | PF12 | - |
| **ETH_TXD1** | PF13 | - |
| **PHY_RST** | PF0 (AF12) | - |

**Status:** ✅ Das UDP Echo Server Projekt verwendet bereits diese Pins (siehe stm32n6xx_hal_msp.c:107-145).

#### User LEDs

| **LED** | **NUCLEO** | **Discovery Kit** | **Änderung nötig?** |
|---------|-----------|-------------------|---------------------|
| **LED1** | GPIOG Pin 8 | Orange LED (Pin TBD) | ⚠️ Ja |
| **LED2** | ? | Green LED (Pin TBD) | ⚠️ Ja |
| **LED3** | ? | Red LED (Pin TBD) | ⚠️ Ja |

**Action Required:** BSP-Treiber für Discovery Kit LEDs verwenden.

#### XSPI2 (NOR Flash)

| **Signal** | **Discovery Kit** | **Notizen** |
|------------|-------------------|-------------|
| **XSPI2_CLK** | PG6 | Shared mit LCD |
| **XSPI2_NCS** | PG12 | - |
| **XSPI2_DQS** | PG15 | - |
| **XSPI2_IO[0-7]** | PO0-PO7 (Port O) | Octal Interface |

**Potentielles Problem:** LCD nutzt möglicherweise einige XSPI-Pins. Konflikterkennung in STM32CubeMX erforderlich.

### 3. Ethernet PHY Unterschied

| **Board** | **PHY Chip** | **Interface** | **Max Speed** |
|-----------|-------------|---------------|---------------|
| **NUCLEO** | LAN8742A (wahrscheinlich) | RMII | 100 Mbit/s |
| **Discovery Kit** | RTL8211F-CG | RMII/RGMII | 1 Gbit/s |

**Implikation:** PHY-Treiber im UDP Echo Server Code prüfen:
- `Drivers/BSP/Components/lan8742/` (Nucleo)
- `Drivers/BSP/Components/rtl8211/` (Discovery Kit möglich)

**Action Required:** PHY-Auswahl in Code anpassen oder BSP-Auto-Detection nutzen.

---

## Pin-Mapping Migration

### Strategie

Da das UDP Echo Server Projekt bereits **PE5/PE6 für UART** und **PF/PG-Pins für Ethernet** verwendet, ist die Ethernet/UART-Konfiguration wahrscheinlich kompatibel. **Hauptproblem:** LED-GPIOs und potentielle LCD-Pin-Konflikte.

### Pin-Mapping Tabelle (Discovery Kit)

#### Bekannte Pins (aus UDP Echo Server Code)

| **Peripheral** | **Pin** | **Function** | **Alt Function** | **Verified** |
|----------------|---------|--------------|-----------------|--------------|
| USART1_TX | PE5 | UART Console | AF7_USART1 | ✅ |
| USART1_RX | PE6 | UART Console | AF7_USART1 | ✅ |
| ETH_REF_CLK | PF7 | Ethernet | AF11_ETH1 | ✅ |
| ETH_MDIO | PF4 | Ethernet | AF11_ETH1 | ✅ |
| ETH_MDC | PG11 | Ethernet | AF11_ETH1 | ✅ |
| ETH_CRS_DV | PF10 | Ethernet | AF11_ETH1 | ✅ |
| ETH_RXD0 | PF14 | Ethernet | AF11_ETH1 | ✅ |
| ETH_RXD1 | PF15 | Ethernet | AF11_ETH1 | ✅ |
| ETH_TX_EN | PF11 | Ethernet | AF11_ETH1 | ✅ |
| ETH_TXD0 | PF12 | Ethernet | AF11_ETH1 | ✅ |
| ETH_TXD1 | PF13 | Ethernet | AF11_ETH1 | ✅ |
| ETH_PHY_RST | PF0 | PHY Reset | AF12_ETH1 | ✅ |

#### Unbekannte/Zu klärende Pins

| **Peripheral** | **Expected Function** | **Discovery Kit Pin** | **Action** |
|----------------|----------------------|----------------------|------------|
| LED1 (Orange) | Status-Anzeige | ?? | BSP Discovery Kit verwenden |
| LED2 (Green) | Link Status | ?? | BSP Discovery Kit verwenden |
| LED3 (Red) | Error Indicator | ?? | BSP Discovery Kit verwenden |
| TIM2_CH1 | Frame Trigger (intern) | N/A (intern) | Kein Pin-Konflikt |

### BSP-Treiber Nutzung

**Empfehlung:** Anstatt direkte GPIO-Manipulation zu verwenden, nutzen Sie die STM32N6570-DK BSP-Treiber:

```c
// Anstatt:
HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET);

// Verwenden:
#include "stm32n6570_discovery.h"
BSP_LED_On(LED_ORANGE);  // LED1
BSP_LED_On(LED_GREEN);   // LED2
BSP_LED_On(LED_RED);     // LED3
```

**BSP-Dateien (zu kopieren/verlinken):**
```
Drivers/BSP/STM32N6570-DK/
├── stm32n6570_discovery.c
├── stm32n6570_discovery.h
├── stm32n6570_discovery_bus.c
├── stm32n6570_discovery_bus.h
├── stm32n6570_discovery_conf_template.h
└── ... (weitere Komponenten)
```

---

## Projekt-Setup für Discovery Kit

### Option 1: Neues Projekt mit STM32CubeMX erstellen (EMPFOHLEN)

#### Vorteile
- Garantiert korrekte Pin-Konfiguration für DK
- Automatische BSP-Integration
- Sauberer Start ohne Legacy-Code

#### Schritte

1. **STM32CubeMX öffnen:**
   ```
   Datei > Neues Projekt > Board Selector > STM32N6570-DK
   ```

2. **Peripherals konfigurieren:**

   **A. Grundkonfiguration:**
   - ✅ RCC (HSE, PLL)
   - ✅ SYS (Debug: Serial Wire, Timebase: TIM6)
   - ✅ CORTEX_M55 (NVIC, Cache)

   **B. UART Console:**
   - USART1: Asynchronous Mode
   - Baud Rate: 115200
   - Pins: PE5 (TX), PE6 (RX)

   **C. Ethernet:**
   - ETH1: RMII Mode
   - PHY Address: 0x00 (RTL8211) oder 0x01 (LAN8742)
   - Interrupt: Enable

   **D. XSPI Flash:**
   - XSPI2: Octal DTR Mode
   - Flash Size: 512 Mbit (64 MB)
   - Pins: Auto-konfiguriert

   **E. Timer für Frame-Trigger:**
   - TIM2: Internal Clock
   - Prescaler: 9999 (600 MHz / 10000 = 60 kHz)
   - Counter Period: 1199 (60 kHz / 1200 = 50 Hz)
   - Interrupt: Enable

3. **Middleware aktivieren:**
   - ✅ Azure RTOS > ThreadX
   - ✅ Azure RTOS > NetX Duo
   - ✅ NetX Duo > Addons > DHCP Client

4. **Clock Configuration:**
   - HSE: 24 MHz (Discovery Kit hat 24 MHz Quarz)
   - SYSCLK: 600 MHz (PLL)
   - AHB: 600 MHz
   - APB1/2/3: Optimal für Peripherals (z.B. APB1 = 150 MHz)

5. **Project Settings:**
   - Project Name: `Thermal_UDP_Server_DK`
   - Toolchain: STM32CubeIDE
   - ✅ Generate peripheral initialization as a pair of '.c/.h' files
   - ✅ Backup previous files when re-generating

6. **Code generieren:**
   - Project > Generate Code
   - Open Project

### Option 2: Bestehendes Projekt migrieren

#### UDP Echo Server als Basis verwenden

**Vorteil:** Netzwerk-Stack bereits konfiguriert.

**Schritte:**

1. **Projekt kopieren:**
   ```bash
   cd D:\Code\STM32\RLU_Expriemente\stm32-helium-demo
   cp -r ide_ws_thomas/Nx_UDP_Echo_Server Thermal_UDP_DK
   ```

2. **Pin-Konfiguration prüfen:**
   - Öffnen: `Thermal_UDP_DK/Nx_UDP_Echo_Server.ioc` (falls vorhanden)
   - Alternativ: Manuell in `stm32n6xx_hal_msp.c` anpassen

3. **BSP Discovery Kit hinzufügen:**
   - Kopiere `Drivers/BSP/STM32N6570-DK/` aus STM32CubeN6 Package
   - Aktualisiere Include-Pfade in IDE

4. **LED-Code anpassen:**
   - Suche: `HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, ...)`
   - Ersetze durch: `BSP_LED_Toggle(LED_ORANGE)`

---

## Schritt-für-Schritt Migration

### Phase 1: Basis-Setup (Tag 1)

#### Schritt 1.1: Hardware verifizieren

**Checkliste:**

- [ ] STM32N6570-DK Board vorhanden
- [ ] USB-C Kabel (ST-LINK VCP)
- [ ] Ethernet-Kabel (RJ45)
- [ ] DHCP-Server im Netzwerk (Router)
- [ ] UART-Terminal (PuTTY, TeraTerm, etc.) @ 115200 baud
- [ ] Wireshark (optional, für Netzwerk-Debugging)

**Boot-Modus einstellen:**
- BOOT0: 2-3 (Development Mode, RAM)
- BOOT1: 2-3 (Development Mode)

#### Schritt 1.2: UDP Echo Server auf DK flashen & testen

**Ziel:** Verifizieren, dass das Ethernet funktioniert.

1. **Projekt öffnen:**
   ```
   STM32CubeIDE > Datei > Importieren > Existing Projects
   > Wähle: ide_ws_thomas/Nx_UDP_Echo_Server
   ```

2. **Build:**
   ```
   Project > Build Project (Ctrl+B)
   ```

3. **Flash:**
   ```
   Run > Debug (F11)
   ```

4. **Terminal öffnen:**
   - PuTTY: COM-Port auswählen (z.B. COM4), 115200-8-N-1
   - Erwartete Ausgabe:
     ```
     Nx_UDP_Echo_Server application started..
     DHCP: IP address acquired: 192.168.1.xxx
     UDP Socket bound to port 6000
     ```

5. **Ethernet Test mit echotool:**
   ```cmd
   C:\> echotool.exe 192.168.1.xxx /p udp /r 6000 /n 5 /d "Hello from PC"
   ```

   **Erwartete Ausgabe:**
   ```
   Reply from 192.168.1.xxx:6000, time 12 ms OK
   Reply from 192.168.1.xxx:6000, time 11 ms OK
   ...
   ```

**Wenn Fehler:**
- LED-Blinken prüfen (Green = OK, Red = Error)
- UART-Logs lesen
- PHY-Adresse prüfen (siehe Troubleshooting)

#### Schritt 1.3: Thermal Camera Code portieren (Bare-Metal Test)

**Ziel:** Thermal-Pipeline ohne Netzwerk auf DK ausführen.

1. **Neues Projekt erstellen:**
   - STM32CubeMX: Board Selector > STM32N6570-DK
   - Peripherals: USART1, TIM2, XSPI2, GPIO (LED)
   - **KEIN Azure RTOS** (Bare-Metal Test)

2. **Code portieren:**
   - Kopiere aus `CORTEX_HELIUM/FSBL/Src/main.c`:
     - Pipeline-Funktionen (`thermal_frame_process`, `process_thermal_line_fastest`)
     - Datenstrukturen (ROI_Stats, Pipeline_Verification)
     - Globale Arrays (`raw[]`, `dark[]`, `gain[]`, `planck[]`)

   - Kopiere `xspi_nor.c` / `xspi_nor.h`

3. **LED-Code anpassen:**
   ```c
   // Original (NUCLEO):
   HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET);

   // Discovery Kit:
   #include "stm32n6570_discovery.h"
   BSP_LED_Toggle(LED_ORANGE);
   ```

4. **TIM2 Interrupt einrichten:**
   ```c
   void TIM2_IRQHandler(void)
   {
       HAL_TIM_IRQHandler(&htim2);

       // Frame-Trigger @ 50 Hz
       vsync_count++;
       frame_ready = 1;
   }
   ```

5. **Linker-Script anpassen:**
   ```ld
   MEMORY
   {
       ROM      (xrw) : ORIGIN = 0x341d0400, LENGTH = 255K
       RAM      (xrw) : ORIGIN = 0x34000000, LENGTH = 1024K
       AXISRAM1 (xrw) : ORIGIN = 0x24060000, LENGTH = 1024K  /* Planck LUT */
       AXISRAM2 (xrw) : ORIGIN = 0x24160000, LENGTH = 1024K  /* Thermal Frames */
       DTCM     (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
   }

   SECTIONS
   {
       .planck_lut (NOLOAD) : { *(.plnk) } > AXISRAM1
       .thermal_frames (NOLOAD) : { *(.thermal) } > AXISRAM2
   }
   ```

6. **Arrays deklarieren:**
   ```c
   // main.c
   #define HPIX 640
   #define VPIX 480
   #define FRAME_SIZE (HPIX * VPIX)

   __attribute__((section(".thermal"))) uint16_t raw[FRAME_SIZE];
   __attribute__((section(".thermal"))) uint16_t dark[FRAME_SIZE];
   __attribute__((section(".thermal"))) uint16_t gain[FRAME_SIZE];
   __attribute__((section(".plnk"))) uint16_t planck[65536];
   __attribute__((section(".thermal"))) volatile uint16_t res[FRAME_SIZE];
   ```

7. **Build, Flash, Test:**
   - Build-Erwartung: ~200 KB Code, ~2.5 MB Data
   - UART-Ausgabe:
     ```
     Thermal Pipeline initialized
     Frame processing @ 50 Hz
     Cycle count: 8234567 (~13.7 ms @ 600 MHz)
     ```

**Erfolg:** Wenn UART zeigt "Frame processing @ 50 Hz" → Weiter zu Phase 2.

---

### Phase 2: ThreadX Integration (Tag 2-3)

#### Schritt 2.1: Neues ThreadX-Projekt erstellen

**Basis:** UDP Echo Server Projekt + Thermal Code

1. **Projekt duplizieren:**
   ```bash
   cp -r ide_ws_thomas/Nx_UDP_Echo_Server Thermal_UDP_ThreadX
   ```

2. **STM32CubeMX öffnen (falls .ioc vorhanden):**
   - Middleware > Azure RTOS > ThreadX: Konfigurieren
   - Timer für Thermal-Trigger hinzufügen (TIM2)

#### Schritt 2.2: Thermal Thread implementieren

**Datei:** `Core/Src/app_thermal.c` (neu erstellen)

```c
/* Includes ------------------------------------------------------------------*/
#include "app_thermal.h"
#include "main.h"
#include "tx_api.h"
#include "arm_mve.h"

/* Defines -------------------------------------------------------------------*/
#define THERMAL_THREAD_STACK_SIZE 4096
#define THERMAL_THREAD_PRIORITY   5  // Hoch!

/* Globals -------------------------------------------------------------------*/
TX_THREAD ThermalThread;
TX_EVENT_FLAGS_GROUP thermal_event_flags;
TX_SEMAPHORE frame_ready_semaphore;

extern uint16_t raw[];
extern uint16_t dark[];
extern uint16_t gain[];
extern uint16_t planck[];
extern volatile uint16_t res[];

/* Private variables ---------------------------------------------------------*/
static UCHAR thermal_thread_stack[THERMAL_THREAD_STACK_SIZE];

/* Thread Entry --------------------------------------------------------------*/
VOID Thermal_Thread_Entry(ULONG thread_input)
{
    ULONG actual_flags;
    UINT status;

    printf("[Thermal] Thread started (Priority %d)\n", THERMAL_THREAD_PRIORITY);

    // Initialisierung
    xspi_nor_init();  // Flash
    load_calibration_data();  // Dark/Gain aus Flash laden
    generate_planck_lut();    // Planck-LUT generieren

    printf("[Thermal] Initialization complete, waiting for frames...\n");

    while(1)
    {
        // Warten auf Timer-Trigger (50 Hz)
        status = tx_event_flags_get(&thermal_event_flags,
                                     0x01,  // Frame-Trigger-Flag
                                     TX_OR_CLEAR,
                                     &actual_flags,
                                     TX_WAIT_FOREVER);

        if (status == TX_SUCCESS)
        {
            // Frame verarbeiten (Helium-optimiert)
            thermal_frame_process();  // Original-Code aus CORTEX_HELIUM

            // Frame bereit signalisieren
            tx_semaphore_put(&frame_ready_semaphore);
        }
    }
}

/* Thread Creation -----------------------------------------------------------*/
UINT App_Thermal_Init(TX_BYTE_POOL *byte_pool)
{
    UINT ret;

    // Event Flags erstellen
    ret = tx_event_flags_create(&thermal_event_flags, "Thermal Events");
    if (ret != TX_SUCCESS) return ret;

    // Semaphore erstellen
    ret = tx_semaphore_create(&frame_ready_semaphore, "Frame Ready", 0);
    if (ret != TX_SUCCESS) return ret;

    // Thread erstellen
    ret = tx_thread_create(&ThermalThread,
                          "Thermal Thread",
                          Thermal_Thread_Entry,
                          0,
                          thermal_thread_stack,
                          THERMAL_THREAD_STACK_SIZE,
                          THERMAL_THREAD_PRIORITY,
                          THERMAL_THREAD_PRIORITY,
                          TX_NO_TIME_SLICE,
                          TX_AUTO_START);

    return ret;
}
```

**Header:** `Core/Inc/app_thermal.h`

```c
#ifndef __APP_THERMAL_H
#define __APP_THERMAL_H

#include "tx_api.h"

/* Exports -------------------------------------------------------------------*/
extern TX_SEMAPHORE frame_ready_semaphore;
extern TX_EVENT_FLAGS_GROUP thermal_event_flags;

/* Functions -----------------------------------------------------------------*/
UINT App_Thermal_Init(TX_BYTE_POOL *byte_pool);

#endif /* __APP_THERMAL_H */
```

#### Schritt 2.3: Timer-Interrupt anpassen

**Datei:** `Core/Src/stm32n6xx_it.c`

```c
/* USER CODE BEGIN Includes */
#include "tx_api.h"
#include "app_thermal.h"
extern TX_EVENT_FLAGS_GROUP thermal_event_flags;
/* USER CODE END Includes */

void TIM2_IRQHandler(void)
{
    /* USER CODE BEGIN TIM2_IRQn 0 */

    /* USER CODE END TIM2_IRQn 0 */
    HAL_TIM_IRQHandler(&htim2);
    /* USER CODE BEGIN TIM2_IRQn 1 */

    // Signal an Thermal Thread senden (ISR-safe!)
    tx_event_flags_set(&thermal_event_flags, 0x01, TX_OR);

    /* USER CODE END TIM2_IRQn 1 */
}
```

#### Schritt 2.4: App_ThreadX_Init erweitern

**Datei:** `Core/Src/app_threadx.c`

```c
UINT App_ThreadX_Init(VOID *memory_ptr)
{
    UINT ret = TX_SUCCESS;
    TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

    /* USER CODE BEGIN App_ThreadX_Init */

    // Thermal Thread initialisieren
    ret = App_Thermal_Init(byte_pool);
    if (ret != TX_SUCCESS)
    {
        printf("ERROR: Thermal Thread creation failed (code %d)\n", ret);
        return ret;
    }

    /* USER CODE END App_ThreadX_Init */

    return ret;
}
```

#### Schritt 2.5: Build & Test

1. **Build:**
   - Erwartete Warnungen: Keine kritischen
   - Code-Size: ~250 KB (mit ThreadX)
   - RAM-Usage: ~2.6 MB

2. **Flash & Debug:**
   - Breakpoint in `Thermal_Thread_Entry`
   - Verifiziere Thread-Erstellung
   - Prüfe Event-Flags bei TIM2-Interrupt

3. **UART-Ausgabe:**
   ```
   Nx_UDP_Echo_Server application started..
   [Thermal] Thread started (Priority 5)
   [Thermal] Initialization complete, waiting for frames...
   DHCP: IP address acquired: 192.168.1.xxx
   [Thermal] Frame processed (cycle count: 8234567)
   [Thermal] Frame processed (cycle count: 8234512)
   ...
   ```

**Erfolg:** Thermal-Thread läuft parallel zum Netzwerk → Weiter zu Phase 3.

---

### Phase 3: UDP Transmission (Tag 3-4)

#### Schritt 3.1: UDP Thread erweitern

**Datei:** `NetXDuo/App/app_netxduo.c` (modifizieren)

```c
/* USER CODE BEGIN Includes */
#include "app_thermal.h"
extern volatile uint16_t res[];  // Thermal result frame
#define HPIX 640
#define VPIX 480
#define FRAME_SIZE (HPIX * VPIX * 2)  // 614400 Bytes
/* USER CODE END Includes */

static VOID App_UDP_Thread_Entry(ULONG thread_input)
{
    UINT ret;
    NX_PACKET *packet;
    ULONG destination_ip = 0xC0A80164;  // 192.168.1.100 (PC)
    UINT destination_port = 7000;        // Thermal Data Port

    const UINT CHUNK_SIZE = 1400;  // MTU - Header
    UINT chunk_number = 0;
    UINT total_chunks = (FRAME_SIZE + CHUNK_SIZE - 1) / CHUNK_SIZE;  // ~439 chunks

    printf("[UDP] Thread started\n");

    // Socket erstellen
    ret = nx_udp_socket_create(&NetXDuoEthIpInstance, &UDPSocket,
                               "UDP Thermal Socket", NX_IP_NORMAL,
                               NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, 50);
    if (ret != NX_SUCCESS) {
        printf("[UDP] ERROR: Socket creation failed (%d)\n", ret);
        return;
    }

    // Socket binden
    ret = nx_udp_socket_bind(&UDPSocket, destination_port, TX_WAIT_FOREVER);
    if (ret != NX_SUCCESS) {
        printf("[UDP] ERROR: Socket bind failed (%d)\n", ret);
        return;
    }

    printf("[UDP] Socket bound to port %d\n", destination_port);
    printf("[UDP] Waiting for thermal frames...\n");

    while(1)
    {
        // Warten auf fertiges Frame
        ret = tx_semaphore_get(&frame_ready_semaphore, TX_WAIT_FOREVER);
        if (ret != TX_SUCCESS) continue;

        printf("[UDP] Sending frame (%d chunks)...\n", total_chunks);

        // Frame fragmentieren und senden
        for (chunk_number = 0; chunk_number < total_chunks; chunk_number++)
        {
            UINT offset = chunk_number * CHUNK_SIZE;
            UINT len = (offset + CHUNK_SIZE > FRAME_SIZE) ?
                       (FRAME_SIZE - offset) : CHUNK_SIZE;

            // Packet allokieren
            ret = nx_packet_allocate(&NxAppPool, &packet,
                                     NX_UDP_PACKET, TX_WAIT_FOREVER);
            if (ret != NX_SUCCESS) {
                printf("[UDP] ERROR: Packet allocation failed\n");
                break;
            }

            // Header hinzufügen (Frame-ID + Chunk-Nummer)
            struct {
                uint32_t frame_id;
                uint32_t chunk_num;
                uint32_t total_chunks;
            } header = {
                .frame_id = vsync_count,
                .chunk_num = chunk_number,
                .total_chunks = total_chunks
            };

            nx_packet_data_append(packet, &header, sizeof(header),
                                 &NxAppPool, TX_WAIT_FOREVER);

            // Thermal-Daten hinzufügen
            nx_packet_data_append(packet,
                                 (VOID*)((uint8_t*)res + offset),
                                 len,
                                 &NxAppPool, TX_WAIT_FOREVER);

            // Senden
            ret = nx_udp_socket_send(&UDPSocket, packet,
                                     destination_ip, destination_port);
            if (ret != NX_SUCCESS) {
                printf("[UDP] ERROR: Send failed (chunk %d)\n", chunk_number);
                nx_packet_release(packet);
            }

            // Rate-Limiting (optional, um Netzwerk nicht zu überlasten)
            tx_thread_sleep(1);  // 10 ms @ 100 ticks/sec
        }

        printf("[UDP] Frame sent successfully\n");
    }
}
```

#### Schritt 3.2: Packet Pool vergrößern

**Datei:** `AZURE_RTOS/App/app_azure_rtos_config.h`

```c
// Original:
// #define NX_APP_MEM_POOL_SIZE  (10 * 1024)  // 10 KB → zu klein!

// Neu (für Thermal Frames):
#define NX_APP_MEM_POOL_SIZE  (128 * 1024)  // 128 KB
```

**Oder in:** `NetXDuo/App/app_netxduo.h`

```c
#define NX_APP_PACKET_POOL_SIZE (100 * 1024)  // 100 KB für Packets
#define DEFAULT_PAYLOAD_SIZE     1500          // MTU
```

#### Schritt 3.3: Build & Test

1. **Build:**
   - Speicher prüfen: MAP-Datei analysieren
   - Erwartete RAM: ~2.7 MB (inkl. Packet Pools)

2. **Flash & Debug:**
   - UART:
     ```
     [Thermal] Frame processed
     [UDP] Sending frame (439 chunks)...
     [UDP] Frame sent successfully
     ```

3. **PC-seitiges Empfangen (Python-Script):**

   **`receive_thermal.py`**:
   ```python
   import socket
   import struct
   import numpy as np

   UDP_IP = "0.0.0.0"
   UDP_PORT = 7000

   sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
   sock.bind((UDP_IP, UDP_PORT))

   print(f"Listening on {UDP_IP}:{UDP_PORT}...")

   chunks = {}
   while True:
       data, addr = sock.recvfrom(2048)

       # Header parsen
       frame_id, chunk_num, total_chunks = struct.unpack('III', data[:12])
       thermal_data = data[12:]

       if frame_id not in chunks:
           chunks[frame_id] = {}

       chunks[frame_id][chunk_num] = thermal_data

       if len(chunks[frame_id]) == total_chunks:
           print(f"Frame {frame_id} complete! Reassembling...")

           # Frame reassemblieren
           sorted_chunks = [chunks[frame_id][i] for i in range(total_chunks)]
           frame_bytes = b''.join(sorted_chunks)

           # Zu uint16 Array konvertieren
           frame_array = np.frombuffer(frame_bytes, dtype=np.uint16)
           frame_array = frame_array.reshape((480, 640))

           print(f"Frame shape: {frame_array.shape}, min: {frame_array.min()}, max: {frame_array.max()}")

           # Hier: Anzeigen/Speichern/Weiterverarbeiten
           # cv2.imshow("Thermal", frame_array.astype(np.uint8))

           del chunks[frame_id]
   ```

4. **Python ausführen:**
   ```bash
   python receive_thermal.py
   ```

   **Erwartete Ausgabe:**
   ```
   Listening on 0.0.0.0:7000...
   Frame 1234 complete! Reassembling...
   Frame shape: (480, 640), min: 12045, max: 15678
   Frame 1235 complete! Reassembling...
   ...
   ```

**Erfolg:** PC empfängt vollständige Thermal-Frames via UDP → Integration abgeschlossen!

---

### Phase 4: Optimierung & Finalisierung (Tag 4-5)

#### 4.1 Performance-Tuning

**Problem:** 439 Chunks × 10 ms Delay = 4.39 Sekunden pro Frame (0.23 FPS) ❌

**Lösungen:**

1. **Zero-Copy mit `nx_packet_data_extract_offset`:**
   ```c
   // Anstatt jeden Chunk zu kopieren, direkt aus res[] referenzieren
   // (Fortgeschritten, erfordert Non-Cacheable Memory)
   ```

2. **Burst-Sending (kein Delay):**
   ```c
   // Entferne tx_thread_sleep(1) in der Send-Loop
   // Vorsicht: Netzwerk-Überlastung möglich
   ```

3. **Kompression (optional):**
   ```c
   // JPEG-Kompression vor Versand (Middleware nötig)
   ```

#### 4.2 Error Handling

**Retry-Mechanismus:**

```c
#define MAX_RETRIES 3

for (retry = 0; retry < MAX_RETRIES; retry++)
{
    ret = nx_udp_socket_send(&UDPSocket, packet, destination_ip, destination_port);
    if (ret == NX_SUCCESS) break;

    printf("[UDP] Send failed, retry %d/%d\n", retry+1, MAX_RETRIES);
    tx_thread_sleep(10);  // 100 ms
}

if (ret != NX_SUCCESS) {
    printf("[UDP] ERROR: Max retries exceeded, dropping frame\n");
    nx_packet_release(packet);
}
```

#### 4.3 LED-Statusanzeigen

**Datei:** `Core/Src/app_thermal.c`

```c
VOID Thermal_Thread_Entry(ULONG thread_input)
{
    // ...

    while(1)
    {
        BSP_LED_On(LED_ORANGE);  // Processing

        tx_event_flags_get(&thermal_event_flags, 0x01, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);

        thermal_frame_process();

        BSP_LED_Off(LED_ORANGE);
        BSP_LED_Toggle(LED_GREEN);  // Frame Ready

        tx_semaphore_put(&frame_ready_semaphore);
    }
}
```

**UDP Thread:**

```c
static VOID App_UDP_Thread_Entry(ULONG thread_input)
{
    // ...

    while(1)
    {
        tx_semaphore_get(&frame_ready_semaphore, TX_WAIT_FOREVER);

        BSP_LED_On(LED_GREEN);  // Sending

        // ... Frame senden ...

        BSP_LED_Off(LED_GREEN);
    }
}
```

**Error Handler:**

```c
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        BSP_LED_Toggle(LED_RED);
        HAL_Delay(200);
    }
}
```

---

## Testing & Debugging

### 6.1 Unit Tests

#### Test 1: Thermal Pipeline (Bare-Metal)

**Ziel:** Verifizieren, dass Frame-Verarbeitung korrekt funktioniert.

```c
void test_thermal_pipeline(void)
{
    printf("=== Thermal Pipeline Test ===\n");

    // 1. Mock-Frame erstellen
    for (int i = 0; i < FRAME_SIZE; i++) {
        raw[i] = 12000 + (i % 100);  // Simulated sensor data
    }

    // 2. Verarbeiten
    thermal_frame_process();

    // 3. Ergebnis prüfen
    uint16_t min_val = 65535, max_val = 0;
    for (int i = 0; i < FRAME_SIZE; i++) {
        if (res[i] < min_val) min_val = res[i];
        if (res[i] > max_val) max_val = res[i];
    }

    printf("Result: min=%d, max=%d\n", min_val, max_val);
    printf(min_val > 0 && max_val < 65535 ? "✅ PASS\n" : "❌ FAIL\n");
}
```

#### Test 2: UDP Fragmentation

**Ziel:** Sicherstellen, dass alle Chunks ankommen.

```c
void test_udp_fragmentation(void)
{
    printf("=== UDP Fragmentation Test ===\n");

    // Sende Test-Frame (alle Bytes = 0xAA)
    memset((void*)res, 0xAA, FRAME_SIZE);
    tx_semaphore_put(&frame_ready_semaphore);

    // Warte auf Sende-Completion
    tx_thread_sleep(500);  // 5 Sekunden

    printf("Check PC-seitig, ob alle 439 Chunks mit 0xAA empfangen wurden.\n");
}
```

### 6.2 Performance-Messung

**DWT Cycle Counter:**

```c
// main.c
void enable_dwt_cycle_counter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// In Thermal Thread:
uint32_t start = DWT->CYCCNT;
thermal_frame_process();
uint32_t cycles = DWT->CYCCNT - start;
printf("Frame processing: %lu cycles (%.2f ms @ 600 MHz)\n",
       cycles, (float)cycles / 600000.0f);
```

**ThreadX Performance Metrics:**

```c
// tx_user.h (aktivieren)
#define TX_THREAD_ENABLE_PERFORMANCE_INFO

// In Code:
TX_THREAD_PERFORMANCE_INFO perf_info;
tx_thread_performance_info_get(&ThermalThread, &perf_info);
printf("Thermal Thread: %lu resumptions, %lu suspensions\n",
       perf_info.tx_thread_performance_resumption_count,
       perf_info.tx_thread_performance_suspension_count);
```

### 6.3 Debugging-Tipps

#### ST-LINK GDB-Server

```bash
# Terminal 1: GDB-Server starten
st-util

# Terminal 2: GDB verbinden
arm-none-eabi-gdb Thermal_UDP_DK.elf
(gdb) target extended-remote :4242
(gdb) load
(gdb) break Thermal_Thread_Entry
(gdb) continue
```

#### Memory Regions prüfen

```gdb
(gdb) info mem
(gdb) x/10x 0x24060000  # Planck LUT (erste 10 Werte)
(gdb) x/10x 0x34000000  # AXISRAM2 (res[])
```

#### ThreadX Thread-Status

```gdb
(gdb) print ThermalThread
(gdb) print ThermalThread.tx_thread_state
# 0x02 = TX_READY, 0x06 = TX_EVENT_FLAG
```

---

## Troubleshooting

### Problem 1: "DHCP timeout, no IP address"

**Symptome:**
- UART: `DHCP: Timeout after 30 seconds`
- Green LED blinkt nicht

**Ursachen & Lösungen:**

1. **Kein DHCP-Server im Netzwerk:**
   - Lösung: Router prüfen oder statische IP verwenden
   ```c
   // app_netxduo.c (anstatt DHCP):
   nx_ip_address_set(&NetXDuoEthIpInstance,
                      IP_ADDRESS(192, 168, 1, 100),
                      0xFFFFFF00UL);
   ```

2. **Ethernet-Kabel nicht verbunden:**
   - Lösung: Kabel prüfen, Link-LED am Board prüfen

3. **PHY-Adresse falsch:**
   ```c
   // In stm32n6xx_hal_conf.h oder main.c:
   #define ETH_PHY_ADDRESS  0x00  // RTL8211
   // oder
   #define ETH_PHY_ADDRESS  0x01  // LAN8742
   ```

### Problem 2: "Hard Fault bei Frame-Verarbeitung"

**Symptome:**
- Board hängt bei `thermal_frame_process()`
- HardFault_Handler wird aufgerufen

**Ursachen & Lösungen:**

1. **Speicher-Alignment:**
   ```c
   // Arrays müssen 8-Byte aligned sein für Helium
   __attribute__((aligned(8))) uint16_t raw[FRAME_SIZE];
   ```

2. **Stack-Overflow:**
   - Lösung: Thread-Stack vergrößern
   ```c
   #define THERMAL_THREAD_STACK_SIZE 8192  // War 4096
   ```

3. **Cache-Kohärenz:**
   ```c
   // res[] muss non-cacheable sein für DMA
   __attribute__((section(".noncacheable"))) volatile uint16_t res[FRAME_SIZE];
   ```

### Problem 3: "UDP Packets kommen nicht an"

**Symptome:**
- Python-Script empfängt keine Daten
- Wireshark zeigt keine Pakete

**Ursachen & Lösungen:**

1. **Firewall blockiert Port 7000:**
   ```bash
   # Windows Firewall (Admin-CMD):
   netsh advfirewall firewall add rule name="Thermal UDP" dir=in action=allow protocol=UDP localport=7000
   ```

2. **Falsche Ziel-IP:**
   ```c
   // PC IP-Adresse prüfen (ipconfig)
   ULONG destination_ip = 0xC0A80164;  // 192.168.1.100
   ```

3. **Packet Pool zu klein:**
   - Symptom: `nx_packet_allocate` gibt `NX_NO_PACKET` zurück
   - Lösung: `NX_APP_PACKET_POOL_SIZE` erhöhen

### Problem 4: "Frame-Rate zu langsam (< 1 FPS)"

**Symptome:**
- Python zeigt nur 0.2 FPS
- Board sendet 4+ Sekunden pro Frame

**Ursachen & Lösungen:**

1. **`tx_thread_sleep(1)` in Send-Loop:**
   - Lösung: Entfernen oder auf 0 setzen

2. **Packet Allocation langsam:**
   - Lösung: Packet Pool vergrößern, Pre-Allocation nutzen

3. **Netzwerk-Bandbreite:**
   - 614400 Bytes @ 100 Mbit/s = ~50 ms Minimum
   - Lösung: Gigabit Ethernet nutzen (RGMII-Modus für RTL8211)

### Problem 5: "Board bootet nicht nach Flash-Programmierung"

**Symptome:**
- LED leuchtet nicht
- Kein UART-Output

**Ursachen & Lösungen:**

1. **Boot-Pins falsch:**
   - BOOT0: 2-3 (Development)
   - BOOT1: 2-3 (Development)

2. **Linker-Script falsch:**
   - Prüfe: ROM-Region @ 0x341d0400
   - Prüfe: _estack innerhalb RAM

3. **Clock-Konfiguration falsch:**
   - STM32CubeMX: Clock Configuration prüfen
   - HSE: 24 MHz für STM32N6570-DK

---

## Anhang A: Discovery Kit Pin-Mapping (Vollständig)

### Ethernet (RMII Mode)

| **Signal** | **Pin** | **Alt Function** | **Notizen** |
|------------|---------|-----------------|-------------|
| REF_CLK | PF7 | AF11_ETH1 | 50 MHz Clock |
| MDIO | PF4 | AF11_ETH1 | Management Data I/O |
| MDC | PG11 | AF11_ETH1 | Management Clock |
| CRS_DV | PF10 | AF11_ETH1 | Carrier Sense / Data Valid |
| RXD0 | PF14 | AF11_ETH1 | Receive Data 0 |
| RXD1 | PF15 | AF11_ETH1 | Receive Data 1 |
| TX_EN | PF11 | AF11_ETH1 | Transmit Enable |
| TXD0 | PF12 | AF11_ETH1 | Transmit Data 0 |
| TXD1 | PF13 | AF11_ETH1 | Transmit Data 1 |
| PHY_RST | PF0 | AF12_ETH1 | PHY Reset (GPIO) |
| PHY_INT | PG3 | GPIO | PHY Interrupt (optional) |

### UART (Console via ST-LINK VCP)

| **Signal** | **Pin** | **Alt Function** |
|------------|---------|-----------------|
| USART1_TX | PE5 | AF7_USART1 |
| USART1_RX | PE6 | AF7_USART1 |

### XSPI2 (NOR Flash)

| **Signal** | **Pin** | **Alt Function** |
|------------|---------|-----------------|
| XSPI2_CLK | PG6 | AF9_XSPI2 |
| XSPI2_NCS | PG12 | AF9_XSPI2 |
| XSPI2_DQS | PG15 | AF9_XSPI2 |
| XSPI2_IO0 | PO0 | AF9_XSPI2 |
| XSPI2_IO1 | PO1 | AF9_XSPI2 |
| XSPI2_IO2 | PO2 | AF9_XSPI2 |
| XSPI2_IO3 | PO3 | AF9_XSPI2 |
| XSPI2_IO4 | PO4 | AF9_XSPI2 |
| XSPI2_IO5 | PO5 | AF9_XSPI2 |
| XSPI2_IO6 | PO6 | AF9_XSPI2 |
| XSPI2_IO7 | PO7 | AF9_XSPI2 |

### User LEDs (BSP-definiert)

| **LED** | **Color** | **Pin** (geschätzt) | **Funktion** |
|---------|-----------|---------------------|-------------|
| LED1 | Orange | ?  | Status / Processing |
| LED2 | Green | ?  | Link OK / Frame Ready |
| LED3 | Red | ?  | Error Indicator |

**Empfehlung:** BSP-Treiber verwenden (`BSP_LED_On(LED_ORANGE)` etc.)

---

## Anhang B: Benötigte Software-Tools

| **Tool** | **Version** | **Zweck** | **Download** |
|----------|-------------|-----------|--------------|
| STM32CubeIDE | ≥ 1.14 | IDE & Debugger | [st.com](https://www.st.com/stm32cubeide) |
| STM32CubeMX | ≥ 6.10 | Code Generator | [st.com](https://www.st.com/stm32cubemx) |
| STM32CubeN6 | Latest | HAL/BSP | [st.com](https://www.st.com/stm32cuben6) |
| STM32CubeProgrammer | ≥ 2.15 | Flash Tool | [st.com](https://www.st.com/stm32cubeprog) |
| Python | ≥ 3.8 | UDP Receiver | [python.org](https://www.python.org) |
| NumPy | Latest | Frame Processing | `pip install numpy` |
| Wireshark | Latest | Network Debug | [wireshark.org](https://www.wireshark.org) |
| echotool | 1.5.0.0 | UDP Testing | [GitHub](https://github.com/PavelBansky/EchoTool) |

---

## Anhang C: Referenzen

1. **STM32N6570-DK User Manual (UM3253):**
   [https://www.st.com/resource/en/user_manual/um3253-stmicroelectronics.pdf](https://www.st.com/resource/en/user_manual/um3253-stmicroelectronics.pdf)

2. **STM32N657 Reference Manual (RM0493):**
   [https://www.st.com/resource/en/reference_manual/rm0493-stmicroelectronics.pdf](https://www.st.com/resource/en/reference_manual/rm0493-stmicroelectronics.pdf)

3. **Azure RTOS ThreadX User Guide:**
   [https://docs.microsoft.com/azure/rtos/threadx/](https://docs.microsoft.com/azure/rtos/threadx/)

4. **Azure RTOS NetX Duo User Guide:**
   [https://docs.microsoft.com/azure/rtos/netx-duo/](https://docs.microsoft.com/azure/rtos/netx-duo/)

5. **ARM Helium (MVE) Programming Guide:**
   [https://developer.arm.com/documentation/102107/](https://developer.arm.com/documentation/102107/)

---

## Anhang D: Changelog

| **Datum** | **Version** | **Änderungen** |
|-----------|-------------|----------------|
| 2025-10-28 | 1.0 | Initiales Discovery Kit Migrationsdokument erstellt |

---

**Nächste Schritte:**

1. ✅ Hardware bereitstellen (STM32N6570-DK)
2. ⬜ UDP Echo Server auf DK testen (Phase 1.2)
3. ⬜ Thermal Code portieren (Phase 1.3)
4. ⬜ ThreadX Integration (Phase 2)
5. ⬜ UDP Transmission implementieren (Phase 3)
6. ⬜ Performance-Optimierung (Phase 4)

**Viel Erfolg bei der Migration!** 🚀
