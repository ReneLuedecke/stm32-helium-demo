/*
 * xspi_nor.c
 * XSPI NOR Flash wrapper using ST BSP
 * Nutzt stm32n6xx_nucleo_xspi.c BSP-Funktionen
 */

#include "xspi_nor.h"
#include <stdio.h>
#include <string.h>

/* ===== Private Defines ===== */
#define BLOCK_SIZE_64K  (64 * 1024)
#define PAGE_SIZE       256

/* ===== Initialization ===== */
void XSPI_NOR_Init_All(void)
{
    BSP_XSPI_NOR_Init_t init;

    printf("Initializing XSPI2 NOR Flash (Octal DTR Mode)...\n");

    /* Configure for Octal DTR mode */
    init.InterfaceMode = BSP_XSPI_NOR_OPI_MODE;
    init.TransferRate = BSP_XSPI_NOR_DTR_TRANSFER;

    /* Initialize XSPI with BSP function */
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

// ═══════════════════════════════════════════════════════════════════
// CALIBRATION DATA WRITE/READ (HIGH-LEVEL)
// ═══════════════════════════════════════════════════════════════════

/**
 * @brief Write gain calibration frame to XSPI Flash
 */
HAL_StatusTypeDef XSPI_NOR_WriteGainFrame(const uint16_t *gain_frame,
                                          uint32_t flash_offset)
{
    const uint32_t GAIN_FRAME_SIZE = 640 * 480 * 2;  // 614400 bytes

    return XSPI_NOR_WriteCalibration(gain_frame, flash_offset,
                                     GAIN_FRAME_SIZE, "Gain Frame");
}

/**
 * @brief Read gain calibration frame from XSPI Flash
 */
HAL_StatusTypeDef XSPI_NOR_ReadGainFrame(uint16_t *gain_frame,
                                         uint32_t flash_offset)
{
    const uint32_t GAIN_FRAME_SIZE = 640 * 480 * 2;

    return XSPI_NOR_Read((uint8_t*)gain_frame, flash_offset, GAIN_FRAME_SIZE);
}

/**
 * @brief Write generic calibration data to XSPI Flash
 */
HAL_StatusTypeDef XSPI_NOR_WriteCalibration(const uint16_t *data,
                                             uint32_t flash_offset,
                                             uint32_t size_bytes,
                                             const char *name)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════╗\n");
    printf("║  Writing %s to XSPI Flash                        \n", name);
    printf("╚═══════════════════════════════════════════════════════════╝\n");
    printf("\n");

    printf("Target address: 0x%08lX\n", flash_offset);
    printf("Size:           %lu bytes (%lu KB)\n", size_bytes, size_bytes / 1024);

    // ═══════════════════════════════════════════════════════════════
    // 1. Disable Memory-Mapped Mode
    // ═══════════════════════════════════════════════════════════════
    printf("\n[1/4] Exiting memory-mapped mode...\n");

    if (XSPI_NOR_DisableMemoryMappedMode() != BSP_ERROR_NONE) {
        printf("  ✗ Failed to exit memory-mapped mode!\n");
        return HAL_ERROR;
    }

    printf("  ✓ Memory-mapped mode disabled\n");

    // ═══════════════════════════════════════════════════════════════
    // 2. Erase Required Blocks (64KB each)
    // ═══════════════════════════════════════════════════════════════
    uint32_t num_blocks = (size_bytes + BLOCK_SIZE_64K - 1) / BLOCK_SIZE_64K;

    printf("\n[2/4] Erasing %lu blocks (64KB each)...\n", num_blocks);

    for (uint32_t i = 0; i < num_blocks; i++) {
        uint32_t block_addr = flash_offset + (i * BLOCK_SIZE_64K);

        if (i % 2 == 0 || i == num_blocks - 1) {
            printf("  Erasing block %lu/%lu @ 0x%08lX...\r",
                   i + 1, num_blocks, block_addr);
            fflush(stdout);
        }

        if (XSPI_NOR_EraseSector(block_addr) != HAL_OK) {
            printf("\n  ✗ Erase failed at block %lu!\n", i);
            XSPI_NOR_EnableMemoryMappedMode();  // Try to recover
            return HAL_ERROR;
        }
    }

    printf("\n  ✓ Erase complete\n");

    // ═══════════════════════════════════════════════════════════════
    // 3. Program Data (256 bytes per page)
    // ═══════════════════════════════════════════════════════════════
    const uint8_t *src = (const uint8_t *)data;
    uint32_t num_pages = (size_bytes + PAGE_SIZE - 1) / PAGE_SIZE;

    printf("\n[3/4] Programming %lu pages (256 bytes each)...\n", num_pages);

    for (uint32_t i = 0; i < num_pages; i++) {
        uint32_t page_addr = flash_offset + (i * PAGE_SIZE);
        uint32_t page_size = PAGE_SIZE;

        // Last page might be smaller
        if (i == num_pages - 1) {
            uint32_t remainder = size_bytes % PAGE_SIZE;
            if (remainder != 0) {
                page_size = remainder;
            }
        }

        // Progress indicator
        if (i % 100 == 0 || i == num_pages - 1) {
            uint32_t percent = (i * 100) / num_pages;
            printf("  Programming: %lu%% (%lu/%lu pages)...\r",
                   percent, i + 1, num_pages);
            fflush(stdout);
        }

        // Write page using BSP function
        if (XSPI_NOR_Write((uint8_t*)&src[i * PAGE_SIZE],
                          page_addr, page_size) != HAL_OK) {
            printf("\n  ✗ Program failed at page %lu (addr: 0x%08lX)!\n",
                   i, page_addr);
            XSPI_NOR_EnableMemoryMappedMode();  // Try to recover
            return HAL_ERROR;
        }
    }

    printf("\n  ✓ Programming complete\n");

    // ═══════════════════════════════════════════════════════════════
    // 4. Re-enter Memory-Mapped Mode
    // ═══════════════════════════════════════════════════════════════
    printf("\n[4/4] Re-entering memory-mapped mode...\n");

    if (XSPI_NOR_EnableMemoryMappedMode() != BSP_ERROR_NONE) {
        printf("  ✗ Failed to re-enter memory-mapped mode!\n");
        return HAL_ERROR;
    }

    printf("  ✓ Memory-mapped mode enabled\n");

    // ═══════════════════════════════════════════════════════════════
    // Success!
    // ═══════════════════════════════════════════════════════════════
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════╗\n");
    printf("║  ✓ %s written successfully!                       \n", name);
    printf("╚═══════════════════════════════════════════════════════════╝\n");
    printf("\n");

    return HAL_OK;
}

/**
 * @brief Verify written calibration data
 */
HAL_StatusTypeDef XSPI_NOR_VerifyCalibration(const uint16_t *expected,
                                              uint32_t flash_offset,
                                              uint32_t size_bytes,
                                              const char *name)
{
    printf("Verifying %s...\n", name);

    // Read back from memory-mapped address
    const uint32_t XSPI_BASE = 0x70000000;
    volatile uint16_t *flash_ptr = (volatile uint16_t *)(XSPI_BASE + flash_offset);

    uint32_t num_words = size_bytes / 2;
    uint32_t errors = 0;
    uint32_t samples_to_check = (num_words < 100) ? num_words : 100;

    // Check first, middle, and last samples
    uint32_t check_indices[] = {
        0,                    // First
        num_words / 4,        // 25%
        num_words / 2,        // 50%
        num_words * 3 / 4,    // 75%
        num_words - 1         // Last
    };

    for (uint32_t i = 0; i < 5; i++) {
        uint32_t idx = check_indices[i];
        if (idx >= num_words) continue;

        uint16_t read_val = flash_ptr[idx];
        uint16_t expected_val = expected[idx];

        if (read_val != expected_val) {
            printf("  ✗ Mismatch at [%lu]: expected 0x%04X, got 0x%04X\n",
                   idx, expected_val, read_val);
            errors++;
        }
    }

    if (errors == 0) {
        printf("  ✓ Verification passed (checked 5 key samples)\n");
        return HAL_OK;
    } else {
        printf("  ⚠ Verification had %lu errors!\n", errors);
        return HAL_ERROR;
    }
}
