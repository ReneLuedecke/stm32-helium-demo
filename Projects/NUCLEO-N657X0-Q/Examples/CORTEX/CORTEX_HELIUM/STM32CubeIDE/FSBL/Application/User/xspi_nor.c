/*
 * xspi_nor.c
 * XSPI NOR Flash driver für STM32N6 - Standard SPI Mode
 */

#include "xspi_nor.h"
#include <string.h>
#include <stdio.h>

/* ===== NOR Flash Commands ===== */
#define XSPI_CMD_READ_STATUS       0x05
#define XSPI_CMD_WRITE_ENABLE      0x06
#define XSPI_CMD_READ              0x03
#define XSPI_CMD_WRITE             0x02
#define XSPI_CMD_SECTOR_ERASE      0x20

/* ===== Auto-Polling: Warten auf Ready Status ===== */
void XSPI_NOR_AutoPollingReady(void)
{
    XSPI_AutoPollingTypeDef autoPollingConfig = {0};

    autoPollingConfig.MatchValue = 0x00;
    autoPollingConfig.MatchMask = 0x01;
    autoPollingConfig.MatchMode = HAL_XSPI_MATCH_MODE_AND;
    autoPollingConfig.IntervalTime = 0x10;
    autoPollingConfig.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;

    HAL_XSPI_AutoPolling(&hxspi2, &autoPollingConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
}

/* ===== Initialisierung ===== */
void XSPI_NOR_Init_All(void)
{
    // XSPI2 ist bereits in MX_XSPI2_Init() initialisiert
    printf("XSPI2 NOR Flash initialized\n");
}

/* ===== Speicher lesen ===== */
HAL_StatusTypeDef XSPI_NOR_Read(uint8_t *pData, uint32_t Address, uint32_t Size)
{
    XSPI_RegularCmdTypeDef sCommand = {0};
    HAL_StatusTypeDef status;

    sCommand.OperationType = HAL_XSPI_OPTYPE_READ_CFG;
    sCommand.Instruction = XSPI_CMD_READ;
    sCommand.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    sCommand.Address = Address;
    sCommand.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
    sCommand.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
    sCommand.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
    sCommand.DataMode = HAL_XSPI_DATA_1_LINE;
    sCommand.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    sCommand.DummyCycles = 8;
    sCommand.DQSMode = HAL_XSPI_DQS_DISABLE;

    status = HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
    if (status != HAL_OK) {
        printf("DEBUG: Read command config failed: %d\n", status);
        return status;
    }

    status = HAL_XSPI_Receive(&hxspi2, pData, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
    if (status != HAL_OK) {
        printf("DEBUG: Receive failed: %d\n", status);
        return status;
    }

    return HAL_OK;
}

/* ===== Speicher schreiben ===== */
HAL_StatusTypeDef XSPI_NOR_Write(uint8_t *pData, uint32_t Address, uint32_t Size)
{
    XSPI_RegularCmdTypeDef sCommand = {0};
    uint32_t bytesWritten = 0;
    uint32_t pageSize = 256;
    HAL_StatusTypeDef status;

    while (bytesWritten < Size) {
        uint32_t chunkSize = (Size - bytesWritten > pageSize) ?
                            pageSize : (Size - bytesWritten);

        // Write Enable
        sCommand.OperationType = HAL_XSPI_OPTYPE_WRITE_CFG;
        sCommand.Instruction = XSPI_CMD_WRITE_ENABLE;
        sCommand.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
        sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
        sCommand.AddressMode = HAL_XSPI_ADDRESS_NONE;
        sCommand.DataMode = HAL_XSPI_DATA_NONE;
        sCommand.DummyCycles = 0;

        status = HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
        if (status != HAL_OK) {
            printf("DEBUG: Write Enable failed: %d\n", status);
            return status;
        }

        // Page Program
        sCommand.Instruction = XSPI_CMD_WRITE;
        sCommand.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
        sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
        sCommand.Address = Address + bytesWritten;
        sCommand.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
        sCommand.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
        sCommand.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
        sCommand.DataMode = HAL_XSPI_DATA_1_LINE;
        sCommand.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
        sCommand.DataLength = chunkSize;  // WICHTIG: Größe setzen!
        sCommand.DummyCycles = 0;

        status = HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
        if (status != HAL_OK) {
            printf("DEBUG: Program command failed: %d\n", status);
            return status;
        }

        status = HAL_XSPI_Transmit(&hxspi2, &pData[bytesWritten], HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
        if (status != HAL_OK) {
            printf("DEBUG: Transmit failed: %d\n", status);
            return status;
        }

        // Wait for ready
        XSPI_NOR_AutoPollingReady();

        bytesWritten += chunkSize;
    }

    return HAL_OK;
}

/* ===== Speicher-Sektor löschen ===== */
HAL_StatusTypeDef XSPI_NOR_EraseSector(uint32_t Address)
{
    XSPI_RegularCmdTypeDef sCommand = {0};
    HAL_StatusTypeDef status;

    // Write Enable
    sCommand.OperationType = HAL_XSPI_OPTYPE_WRITE_CFG;
    sCommand.Instruction = XSPI_CMD_WRITE_ENABLE;
    sCommand.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    sCommand.AddressMode = HAL_XSPI_ADDRESS_NONE;
    sCommand.DataMode = HAL_XSPI_DATA_NONE;
    sCommand.DummyCycles = 0;

    status = HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
    if (status != HAL_OK) {
        printf("DEBUG: Erase Write Enable failed: %d\n", status);
        return status;
    }

    // Erase Sector
    sCommand.Instruction = XSPI_CMD_SECTOR_ERASE;
    sCommand.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    sCommand.Address = Address;
    sCommand.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
    sCommand.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
    sCommand.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
    sCommand.DataMode = HAL_XSPI_DATA_NONE;
    sCommand.DummyCycles = 0;

    status = HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
    if (status != HAL_OK) {
        printf("DEBUG: Erase command failed: %d\n", status);
        return status;
    }

    // Wait for ready
    XSPI_NOR_AutoPollingReady();

    return HAL_OK;
}

/* ===== Memory-Mapped Mode aktivieren ===== */
void XSPI_NOR_EnableMemoryMappedMode(void)
{
    XSPI_RegularCmdTypeDef sCommand = {0};
    XSPI_MemoryMappedTypeDef memMappedCfg = {0};

    sCommand.OperationType = HAL_XSPI_OPTYPE_READ_CFG;
    sCommand.Instruction = XSPI_CMD_READ;
    sCommand.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    sCommand.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
    sCommand.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
    sCommand.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
    sCommand.DataMode = HAL_XSPI_DATA_1_LINE;
    sCommand.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    sCommand.DummyCycles = 8;
    sCommand.DQSMode = HAL_XSPI_DQS_DISABLE;

    HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);

    memMappedCfg.TimeOutActivation = HAL_XSPI_TIMEOUT_COUNTER_ENABLE;

    HAL_XSPI_MemoryMapped(&hxspi2, &memMappedCfg);
}
