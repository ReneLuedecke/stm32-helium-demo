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

#endif /* XSPI_NOR_H */