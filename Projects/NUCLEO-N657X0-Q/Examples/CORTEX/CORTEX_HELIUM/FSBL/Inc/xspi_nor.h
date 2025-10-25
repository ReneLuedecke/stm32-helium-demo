#ifndef XSPI_NOR_H
#define XSPI_NOR_H

#include "stm32n6xx_hal.h"
#include "stm32n6xx_nucleo_xspi.h"
#include <stdint.h>

/* ===== Function Prototypes ===== */

/**
 * @brief Initialize XSPI2 NOR Flash in Octal DTR Mode
 */
void XSPI_NOR_Init_All(void);

/**
 * @brief Read data from XSPI NOR Flash
 * @param pData Pointer to data buffer
 * @param Address Flash address to read from
 * @param Size Number of bytes to read
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef XSPI_NOR_Read(uint8_t *pData, uint32_t Address, uint32_t Size);

/**
 * @brief Write data to XSPI NOR Flash
 * @param pData Pointer to data to write
 * @param Address Flash address to write to
 * @param Size Number of bytes to write
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef XSPI_NOR_Write(uint8_t *pData, uint32_t Address, uint32_t Size);

/**
 * @brief Erase a sector (64KB) of XSPI NOR Flash
 * @param Address Sector start address (must be 64KB aligned)
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef XSPI_NOR_EraseSector(uint32_t Address);

/**
 * @brief Erase entire chip
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef XSPI_NOR_EraseChip(void);

/**
 * @brief Get flash status
 * @return BSP_ERROR_NONE if ready, BSP_ERROR_BUSY if busy
 */
int32_t XSPI_NOR_GetStatus(void);

/**
 * @brief Get flash information
 * @param pInfo Pointer to info structure
 * @return BSP_ERROR_NONE on success
 */
int32_t XSPI_NOR_GetInfo(BSP_XSPI_NOR_Info_t *pInfo);

/**
 * @brief Enable memory-mapped mode
 * @return BSP_ERROR_NONE on success
 */
int32_t XSPI_NOR_EnableMemoryMappedMode(void);

/**
 * @brief Disable memory-mapped mode
 * @return BSP_ERROR_NONE on success
 */
int32_t XSPI_NOR_DisableMemoryMappedMode(void);

/**
 * @brief Read flash ID
 * @param Id Pointer to 3-byte ID buffer
 * @return BSP_ERROR_NONE on success
 */
int32_t XSPI_NOR_ReadID(uint8_t *Id);

/**
 * @brief Configure flash interface mode
 * @param Mode SPI or OPI mode
 * @param Rate STR or DTR transfer rate
 * @return BSP_ERROR_NONE on success
 */
int32_t XSPI_NOR_ConfigFlash(BSP_XSPI_NOR_Interface_t Mode, BSP_XSPI_NOR_Transfer_t Rate);

// ═══════════════════════════════════════════════════════════════════
// CALIBRATION DATA WRITE/READ (HIGH-LEVEL)
// ═══════════════════════════════════════════════════════════════════

/**
 * @brief Write gain calibration frame to XSPI Flash
 *
 * This function:
 * 1. Exits memory-mapped mode
 * 2. Erases required 64KB blocks
 * 3. Programs data page-by-page (256 bytes)
 * 4. Re-enters memory-mapped mode
 *
 * @param gain_frame Pointer to 640×480 uint16_t gain frame (614 KB)
 * @param flash_offset Flash offset (should be 64KB aligned)
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef XSPI_NOR_WriteGainFrame(const uint16_t *gain_frame,
                                          uint32_t flash_offset);

/**
 * @brief Read gain calibration frame from XSPI Flash
 *
 * Reads gain frame from flash to RAM buffer.
 * Note: In memory-mapped mode, you can also just read directly
 * from the memory-mapped address!
 *
 * @param gain_frame Pointer to destination buffer (640×480 uint16_t)
 * @param flash_offset Flash offset
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef XSPI_NOR_ReadGainFrame(uint16_t *gain_frame,
                                         uint32_t flash_offset);

/**
 * @brief Write generic calibration data to XSPI Flash
 *
 * Generic function for writing any calibration data (dark, gain, offset)
 *
 * @param data Pointer to source data
 * @param flash_offset Flash offset (should be 64KB aligned for best performance)
 * @param size_bytes Size in bytes
 * @param name Debug name for printf
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef XSPI_NOR_WriteCalibration(const uint16_t *data,
                                             uint32_t flash_offset,
                                             uint32_t size_bytes,
                                             const char *name);

/**
 * @brief Verify written calibration data
 *
 * Reads back data and compares with expected values
 *
 * @param expected Pointer to expected data
 * @param flash_offset Flash offset
 * @param size_bytes Size in bytes
 * @param name Debug name for printf
 * @return HAL_OK if verification passed, HAL_ERROR on mismatch
 */
HAL_StatusTypeDef XSPI_NOR_VerifyCalibration(const uint16_t *expected,
                                              uint32_t flash_offset,
                                              uint32_t size_bytes,
                                              const char *name);

#endif /* XSPI_NOR_H */
