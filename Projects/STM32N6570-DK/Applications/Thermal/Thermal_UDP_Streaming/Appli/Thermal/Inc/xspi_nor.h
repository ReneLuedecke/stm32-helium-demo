#ifndef XSPI_NOR_H
#define XSPI_NOR_H

#include "stm32n6xx_hal.h"

void XSPI_NOR_Init_All(void);
HAL_StatusTypeDef XSPI_NOR_Read(uint8_t *pData, uint32_t Address, uint32_t Size);  // ← NEU
HAL_StatusTypeDef XSPI_NOR_WriteGainFrame(const uint16_t *gain_frame, uint32_t flash_offset);
HAL_StatusTypeDef XSPI_NOR_ReadGainFrame(uint16_t *gain_frame, uint32_t flash_offset);

#endif
