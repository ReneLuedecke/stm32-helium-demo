/*
 * xspi_nor.c
 * XSPI NOR Flash driver für STM32N6 - Octal DTR Mode (HIGH SPEED)
 */

#include "xspi_nor.h"
#include <string.h>
#include <stdio.h>

/* ===== Command Definitions - Macronix Octal DTR Mode ===== */
#define OCTAL_IO_READ_CMD                0xEC13
#define OCTAL_IO_DTR_READ_CMD            0xEE11
#define OCTAL_PAGE_PROG_CMD              0x12ED
#define OCTAL_READ_STATUS_REG_CMD        0x05FA
#define OCTAL_SECTOR_ERASE_CMD           0x21DE
#define OCTAL_WRITE_ENABLE_CMD           0x06F9
#define WRITE_ENABLE_CMD                 0x06
#define READ_STATUS_REG_CMD              0x05
#define WRITE_CFG_REG_2_CMD              0x72

/* ===== Timing & Polling Constants ===== */
#define DUMMY_CLOCK_CYCLES_READ         6
#define DUMMY_CLOCK_CYCLES_READ_REG     4
#define DUMMY_CLOCK_CYCLES_READ_OCTAL   6
#define WRITE_ENABLE_MATCH_VALUE        0x02
#define WRITE_ENABLE_MASK_VALUE         0x02
#define MEMORY_READY_MATCH_VALUE        0x00
#define MEMORY_READY_MASK_VALUE         0x01
#define AUTO_POLLING_INTERVAL           0x10
#define MEMORY_REG_WRITE_DELAY          40
#define MEMORY_PAGE_PROG_DELAY          2

/* Memory Configuration */
#define CONFIG_REG2_ADDR1               0x0000000
#define CR2_DTR_OPI_ENABLE              0x02
#define CONFIG_REG2_ADDR3               0x00000300
#define CR2_DUMMY_CYCLES_66MHZ          0x07

/* ===== Forward Declarations ===== */
static void XSPI_WriteEnable(XSPI_HandleTypeDef *hxspi);
static void XSPI_AutoPollingMemReady(XSPI_HandleTypeDef *hxspi);
static void XSPI_NOR_OctalDTRModeCfg(XSPI_HandleTypeDef *hxspi);

/* ===== Write Enable - mit Status Polling ===== */
static void XSPI_WriteEnable(XSPI_HandleTypeDef *hxspi)
{
    XSPI_RegularCmdTypeDef sCommand = {0};
    XSPI_AutoPollingTypeDef sConfig = {0};

    /* Enable write operations ------------------------------------------ */
    sCommand.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    sCommand.Instruction = OCTAL_WRITE_ENABLE_CMD;
    sCommand.InstructionMode = HAL_XSPI_INSTRUCTION_8_LINES;
    sCommand.InstructionWidth = HAL_XSPI_INSTRUCTION_16_BITS;
    sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
    sCommand.AddressMode = HAL_XSPI_ADDRESS_NONE;
    sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    sCommand.DataMode = HAL_XSPI_DATA_NONE;
    sCommand.DummyCycles = 0;
    sCommand.DQSMode = HAL_XSPI_DQS_DISABLE;

    if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        Error_Handler();
    }

    /* Configure automatic polling mode to wait for write enabling ---- */
    sCommand.Instruction = OCTAL_READ_STATUS_REG_CMD;
    sCommand.Address = 0x0;
    sCommand.AddressMode = HAL_XSPI_ADDRESS_8_LINES;
    sCommand.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
    sCommand.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_ENABLE;
    sCommand.DataMode = HAL_XSPI_DATA_8_LINES;
    sCommand.DataDTRMode = HAL_XSPI_DATA_DTR_ENABLE;
    sCommand.DataLength = 2;
    sCommand.DummyCycles = DUMMY_CLOCK_CYCLES_READ_OCTAL;
    sCommand.DQSMode = HAL_XSPI_DQS_ENABLE;

    if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        Error_Handler();
    }

    sConfig.MatchMode = HAL_XSPI_MATCH_MODE_AND;
    sConfig.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
    sConfig.IntervalTime = AUTO_POLLING_INTERVAL;
    sConfig.MatchMask = WRITE_ENABLE_MASK_VALUE;
    sConfig.MatchValue = WRITE_ENABLE_MATCH_VALUE;

    if (HAL_XSPI_AutoPolling(hxspi, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        Error_Handler();
    }
}

/* ===== Wait for Memory Ready ===== */
static void XSPI_AutoPollingMemReady(XSPI_HandleTypeDef *hxspi)
{
    XSPI_RegularCmdTypeDef sCommand = {0};
    XSPI_AutoPollingTypeDef sConfig = {0};

    /* Configure Status Register Read Command (Octal DTR) */
    sCommand.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    sCommand.Instruction = OCTAL_READ_STATUS_REG_CMD;
    sCommand.InstructionMode = HAL_XSPI_INSTRUCTION_8_LINES;
    sCommand.InstructionWidth = HAL_XSPI_INSTRUCTION_16_BITS;
    sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
    sCommand.Address = 0x0;
    sCommand.AddressMode = HAL_XSPI_ADDRESS_8_LINES;
    sCommand.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
    sCommand.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_ENABLE;
    sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    sCommand.DataMode = HAL_XSPI_DATA_8_LINES;
    sCommand.DataDTRMode = HAL_XSPI_DATA_DTR_ENABLE;
    sCommand.DataLength = 2;
    sCommand.DummyCycles = DUMMY_CLOCK_CYCLES_READ_OCTAL;
    sCommand.DQSMode = HAL_XSPI_DQS_ENABLE;

    if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        printf("ERROR: Memory ready command failed\n");
    }

    /* Poll until WIP (Write In Progress) bit = 0 */
    sConfig.MatchMode = HAL_XSPI_MATCH_MODE_AND;
    sConfig.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
    sConfig.IntervalTime = AUTO_POLLING_INTERVAL;
    sConfig.MatchMask = MEMORY_READY_MASK_VALUE;
    sConfig.MatchValue = MEMORY_READY_MATCH_VALUE;

    if (HAL_XSPI_AutoPolling(hxspi, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        printf("ERROR: Memory ready polling timeout\n");
    }
}

/* ===== Configure Flash to Octal DTR Mode ===== */
static void XSPI_NOR_OctalDTRModeCfg(XSPI_HandleTypeDef *hxspi)
{
    uint8_t reg = 0;
    XSPI_RegularCmdTypeDef sCommand = {0};
    XSPI_AutoPollingTypeDef sConfig = {0};

    printf("Configuring Flash to Octal DTR Mode...\n");

    /* PHASE 1: SPI Mode (1-Line) Commands */
    sCommand.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    sCommand.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    sCommand.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    sCommand.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
    sCommand.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
    sCommand.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
    sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    sCommand.DataMode = HAL_XSPI_DATA_1_LINE;
    sCommand.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    sCommand.DummyCycles = 0;
    sCommand.DQSMode = HAL_XSPI_DQS_DISABLE;

    sConfig.MatchMode = HAL_XSPI_MATCH_MODE_AND;
    sConfig.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
    sConfig.IntervalTime = 0x10;

    /* Step 1: Write Enable (SPI Mode) */
    sCommand.Instruction = WRITE_ENABLE_CMD;
    sCommand.DataMode = HAL_XSPI_DATA_NONE;
    sCommand.AddressMode = HAL_XSPI_ADDRESS_NONE;

    if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        printf("ERROR: Write enable failed\n");
        return;
    }

    HAL_Delay(10);  /* Small delay */

    /* Step 2: Write Configuration Register 2 (Enable Octal I/O) - SPI Mode */
    sCommand.Instruction = WRITE_CFG_REG_2_CMD;
    sCommand.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
    sCommand.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
    sCommand.Address = 0x0;
    sCommand.DataMode = HAL_XSPI_DATA_1_LINE;
    sCommand.DataLength = 1;
    reg = 0x2;  /* Enable Octal I/O */

    if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        printf("ERROR: Write config register failed\n");
        return;
    }

    if (HAL_XSPI_Transmit(hxspi, &reg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        printf("ERROR: Transmit config value failed\n");
        return;
    }

    HAL_Delay(10);  /* Wait for flash to process */

    printf("Flash configured for Octal DTR Mode\n");
}

/* ===== Initialization ===== */
void XSPI_NOR_Init_All(void)
{
    printf("Initializing XSPI2 with Octal DTR Mode...\n");
    XSPI_NOR_OctalDTRModeCfg(&hxspi2);
    printf("XSPI2 NOR Flash initialized\n");
}

/* ===== Read Data ===== */
HAL_StatusTypeDef XSPI_NOR_Read(uint8_t *pData, uint32_t Address, uint32_t Size)
{
    XSPI_RegularCmdTypeDef sCommand = {0};
    HAL_StatusTypeDef status;

    sCommand.OperationType = HAL_XSPI_OPTYPE_READ_CFG;
    sCommand.Instruction = OCTAL_IO_DTR_READ_CMD;
    sCommand.InstructionMode = HAL_XSPI_INSTRUCTION_8_LINES;
    sCommand.InstructionWidth = HAL_XSPI_INSTRUCTION_16_BITS;
    sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
    sCommand.Address = Address;
    sCommand.AddressMode = HAL_XSPI_ADDRESS_8_LINES;
    sCommand.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_ENABLE;
    sCommand.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
    sCommand.DataMode = HAL_XSPI_DATA_8_LINES;
    sCommand.DataDTRMode = HAL_XSPI_DATA_DTR_ENABLE;
    sCommand.DataLength = Size;
    sCommand.DummyCycles = DUMMY_CLOCK_CYCLES_READ_OCTAL;
    sCommand.DQSMode = HAL_XSPI_DQS_ENABLE;

    status = HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
    if (status != HAL_OK) {
        printf("ERROR: Read command config failed\n");
        return status;
    }

    status = HAL_XSPI_Receive(&hxspi2, pData, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
    if (status != HAL_OK) {
        printf("ERROR: Receive failed\n");
        return status;
    }

    return HAL_OK;
}

/* ===== Write Data ===== */
HAL_StatusTypeDef XSPI_NOR_Write(uint8_t *pData, uint32_t Address, uint32_t Size)
{
    XSPI_RegularCmdTypeDef sCommand = {0};
    uint32_t bytesWritten = 0;
    uint32_t pageSize = 256;
    HAL_StatusTypeDef status;

    while (bytesWritten < Size) {
        uint32_t chunkSize = (Size - bytesWritten > pageSize) ?
                            pageSize : (Size - bytesWritten);

        /* Enable Write */
        XSPI_WriteEnable(&hxspi2);

        /* Configure Page Program Command (Octal DTR) */
        sCommand.OperationType = HAL_XSPI_OPTYPE_WRITE_CFG;
        sCommand.Instruction = OCTAL_PAGE_PROG_CMD;
        sCommand.InstructionMode = HAL_XSPI_INSTRUCTION_8_LINES;
        sCommand.InstructionWidth = HAL_XSPI_INSTRUCTION_16_BITS;
        sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
        sCommand.Address = Address + bytesWritten;
        sCommand.AddressMode = HAL_XSPI_ADDRESS_8_LINES;
        sCommand.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_ENABLE;
        sCommand.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
        sCommand.DataMode = HAL_XSPI_DATA_8_LINES;
        sCommand.DataDTRMode = HAL_XSPI_DATA_DTR_ENABLE;
        sCommand.DataLength = chunkSize;
        sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
        sCommand.DummyCycles = 0;
        sCommand.DQSMode = HAL_XSPI_DQS_ENABLE;

        status = HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
        if (status != HAL_OK) {
            printf("ERROR: Page program command failed\n");
            return status;
        }

        status = HAL_XSPI_Transmit(&hxspi2, &pData[bytesWritten], HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
        if (status != HAL_OK) {
            printf("ERROR: Transmit failed\n");
            return status;
        }

        /* Wait for Write Complete */
        XSPI_AutoPollingMemReady(&hxspi2);

        bytesWritten += chunkSize;
    }

    return HAL_OK;
}

/* ===== Erase Sector ===== */
HAL_StatusTypeDef XSPI_NOR_EraseSector(uint32_t Address)
{
    XSPI_RegularCmdTypeDef sCommand = {0};
    HAL_StatusTypeDef status;

    /* Enable Write */
    XSPI_WriteEnable(&hxspi2);

    /* Configure Sector Erase Command (Octal DTR) */
    sCommand.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    sCommand.Instruction = OCTAL_SECTOR_ERASE_CMD;
    sCommand.InstructionMode = HAL_XSPI_INSTRUCTION_8_LINES;
    sCommand.InstructionWidth = HAL_XSPI_INSTRUCTION_16_BITS;
    sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
    sCommand.Address = Address;
    sCommand.AddressMode = HAL_XSPI_ADDRESS_8_LINES;
    sCommand.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_ENABLE;
    sCommand.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
    sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    sCommand.DataDTRMode = HAL_XSPI_DATA_DTR_ENABLE;
    sCommand.DataMode = HAL_XSPI_DATA_NONE;
    sCommand.DummyCycles = 0;
    sCommand.DQSMode = HAL_XSPI_DQS_ENABLE;

    status = HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
    if (status != HAL_OK) {
        printf("ERROR: Erase command failed\n");
        return status;
    }

    /* Wait for Erase Complete */
    XSPI_AutoPollingMemReady(&hxspi2);

    return HAL_OK;
}

/* ===== Memory-Mapped Mode ===== */
void XSPI_NOR_EnableMemoryMappedMode(void)
{
    XSPI_RegularCmdTypeDef sCommand = {0};
    XSPI_MemoryMappedTypeDef memMappedCfg = {0};

    sCommand.OperationType = HAL_XSPI_OPTYPE_READ_CFG;
    sCommand.Instruction = OCTAL_IO_DTR_READ_CMD;
    sCommand.InstructionMode = HAL_XSPI_INSTRUCTION_8_LINES;
    sCommand.InstructionWidth = HAL_XSPI_INSTRUCTION_16_BITS;
    sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
    sCommand.AddressMode = HAL_XSPI_ADDRESS_8_LINES;
    sCommand.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_ENABLE;
    sCommand.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
    sCommand.DataMode = HAL_XSPI_DATA_8_LINES;
    sCommand.DataDTRMode = HAL_XSPI_DATA_DTR_ENABLE;
    sCommand.DummyCycles = DUMMY_CLOCK_CYCLES_READ_OCTAL;
    sCommand.DQSMode = HAL_XSPI_DQS_ENABLE;

    HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);

    memMappedCfg.TimeOutActivation = HAL_XSPI_TIMEOUT_COUNTER_ENABLE;
    HAL_XSPI_MemoryMapped(&hxspi2, &memMappedCfg);

    printf("Memory-Mapped Mode enabled\n");
}
