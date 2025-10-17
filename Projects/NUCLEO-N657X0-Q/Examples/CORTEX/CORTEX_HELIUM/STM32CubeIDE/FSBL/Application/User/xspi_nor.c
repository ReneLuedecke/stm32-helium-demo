/*
 * xspi_nor.c
 * XSPI NOR Flash wrapper using ST BSP
 * Nutzt stm32n6xx_nucleo_xspi.c BSP-Funktionen
 */

#include "xspi_nor.h"
#include <stdio.h>

/* ===== Initialization ===== */
//void XSPI_NOR_Init_All(void)
//{
//	BSP_XSPI_NOR_Init_t init;
//
//	printf("Initializing XSPI2 NOR Flash (Octal DTR Mode)...\n");
//
//	/* Configure for Octal DTR mode */
//	init.InterfaceMode = BSP_XSPI_NOR_OPI_MODE;
//	init.TransferRate = BSP_XSPI_NOR_DTR_TRANSFER;
//
//	/* Initialize XSPI with BSP function */
//	if (BSP_XSPI_NOR_Init(0, &init) != BSP_ERROR_NONE) {
//		printf("ERROR: XSPI NOR initialization failed\n");
//		Error_Handler();
//	}
//
//	printf("XSPI2 NOR Flash initialized successfully!\n");
//}

void XSPI_NOR_Init_All(void)
{
    BSP_XSPI_NOR_Init_t init;
    
    printf("Initializing XSPI2 NOR Flash (SPI Mode for testing)...\n");
    
    /* Test mit SPI-Mode statt Octal DTR */
    init.InterfaceMode = BSP_XSPI_NOR_SPI_MODE;    // SPI statt OPI
    init.TransferRate = BSP_XSPI_NOR_STR_TRANSFER; // STR statt DTR
    
    if (BSP_XSPI_NOR_Init(0, &init) != BSP_ERROR_NONE) {
        printf("ERROR: XSPI NOR initialization failed\n");
        Error_Handler();
    }
    
    printf("XSPI2 NOR Flash initialized successfully!\n");
}

/* ===== Read Data ===== */
HAL_StatusTypeDef XSPI_NOR_Read(uint8_t *pData, uint32_t Address, uint32_t Size)
{
    int32_t ret;
    
    ret = BSP_XSPI_NOR_Read(0, pData, Address, Size);
    
    if (ret != BSP_ERROR_NONE) {
        printf("ERROR: Read failed (code: %ld)\n", ret);
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

/* ===== Write Data ===== */
HAL_StatusTypeDef XSPI_NOR_Write(uint8_t *pData, uint32_t Address, uint32_t Size)
{
    int32_t ret;
    
    ret = BSP_XSPI_NOR_Write(0, pData, Address, Size);
    
    if (ret != BSP_ERROR_NONE) {
        printf("ERROR: Write failed (code: %ld)\n", ret);
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

/* ===== Erase Sector (64KB) ===== */
HAL_StatusTypeDef XSPI_NOR_EraseSector(uint32_t Address)
{
    int32_t ret;
    
    ret = BSP_XSPI_NOR_Erase_Block(0, Address, BSP_XSPI_NOR_ERASE_64K);
    
    if (ret != BSP_ERROR_NONE) {
        printf("ERROR: Erase failed (code: %ld)\n", ret);
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

/* ===== Erase Chip ===== */
HAL_StatusTypeDef XSPI_NOR_EraseChip(void)
{
    int32_t ret;
    
    printf("Erasing entire chip...\n");
    ret = BSP_XSPI_NOR_Erase_Chip(0);
    
    if (ret != BSP_ERROR_NONE) {
        printf("ERROR: Chip erase failed (code: %ld)\n", ret);
        return HAL_ERROR;
    }
    
    printf("Chip erase complete!\n");
    return HAL_OK;
}

/* ===== Get Status ===== */
int32_t XSPI_NOR_GetStatus(void)
{
    return BSP_XSPI_NOR_GetStatus(0);
}

/* ===== Get Flash Info ===== */
int32_t XSPI_NOR_GetInfo(BSP_XSPI_NOR_Info_t *pInfo)
{
    return BSP_XSPI_NOR_GetInfo(0, pInfo);
}

/* ===== Enable Memory-Mapped Mode ===== */
int32_t XSPI_NOR_EnableMemoryMappedMode(void)
{
    printf("Enabling Memory-Mapped Mode...\n");
    return BSP_XSPI_NOR_EnableMemoryMappedMode(0);
}

/* ===== Disable Memory-Mapped Mode ===== */
int32_t XSPI_NOR_DisableMemoryMappedMode(void)
{
    printf("Disabling Memory-Mapped Mode...\n");
    return BSP_XSPI_NOR_DisableMemoryMappedMode(0);
}

/* ===== Read Flash ID ===== */
int32_t XSPI_NOR_ReadID(uint8_t *Id)
{
    return BSP_XSPI_NOR_ReadID(0, Id);
}

/* ===== Configure Flash Mode ===== */
int32_t XSPI_NOR_ConfigFlash(BSP_XSPI_NOR_Interface_t Mode, BSP_XSPI_NOR_Transfer_t Rate)
{
    printf("Configuring Flash to Mode %d, Rate %d...\n", Mode, Rate);
    return BSP_XSPI_NOR_ConfigFlash(0, Mode, Rate);
}
