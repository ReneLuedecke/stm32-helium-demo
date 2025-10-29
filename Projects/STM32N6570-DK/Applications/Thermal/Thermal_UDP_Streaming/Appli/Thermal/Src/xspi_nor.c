#include "xspi_nor.h"
#include <stdio.h>

void XSPI_NOR_Init_All(void)
{
    printf("[XSPI] Stub\n");
}

HAL_StatusTypeDef XSPI_NOR_Read(uint8_t *pData, uint32_t Address, uint32_t Size)
{
    (void)pData; (void)Address; (void)Size;
    return HAL_OK;
}

HAL_StatusTypeDef XSPI_NOR_WriteGainFrame(const uint16_t *gain_frame, uint32_t flash_offset)
{
    (void)gain_frame; (void)flash_offset;
    return HAL_OK;
}

HAL_StatusTypeDef XSPI_NOR_ReadGainFrame(uint16_t *gain_frame, uint32_t flash_offset)
{
    (void)gain_frame; (void)flash_offset;
    return HAL_OK;
}
