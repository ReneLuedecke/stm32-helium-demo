#ifndef XSPI_NOR_H
#define XSPI_NOR_H

#include "stm32n657xx.h"
#include "stm32n6xx_hal.h"

/* ===== XSPI2 Handle (definiert in main.c) ===== */
extern XSPI_HandleTypeDef hxspi2;

/* ===== Funktionsdeklarationen ===== */

/**
 * @brief Initialisiert XSPI2 NOR Flash in Octal DTR Mode
 */
void XSPI_NOR_Init_All(void);

/**
 * @brief Liest Daten aus XSPI2 NOR Flash
 * @param pData Zeiger auf Buffer zum Speichern der Daten
 * @param Address Speicheradresse
 * @param Size Anzahl der zu lesenden Bytes
 */
HAL_StatusTypeDef XSPI_NOR_Read(uint8_t *pData, uint32_t Address, uint32_t Size);

/**
 * @brief Schreibt Daten in XSPI2 NOR Flash
 * @param pData Zeiger auf Daten zum Schreiben
 * @param Address Speicheradresse
 * @param Size Anzahl der zu schreibenden Bytes
 */
HAL_StatusTypeDef XSPI_NOR_Write(uint8_t *pData, uint32_t Address, uint32_t Size);

/**
 * @brief Löscht einen Sektor des XSPI2 NOR Flash
 * @param Address Speicheradresse des zu löschenden Sektors
 */
HAL_StatusTypeDef XSPI_NOR_EraseSector(uint32_t Address);

/**
 * @brief Aktiviert Memory-Mapped Mode
 */
void XSPI_NOR_EnableMemoryMappedMode(void);

/**
 * @brief Wartet auf Ready-Status (Auto-Polling)
 */
void XSPI_NOR_AutoPollingReady(void);

#endif
