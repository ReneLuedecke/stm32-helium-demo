/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_mve.h"
#include "stdio.h"
#include <stdint.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HPIX 640
#define VPIX 480

// XSPI Memory Map (wie Nucleo)
#define XSPI_BASE_ADDRESS      0x70000000
#define XSPI_DARK_OFFSET       0x00000000  // 600 KB
#define XSPI_GAIN_OFFSET       0x00096000  // 600 KB
#define XSPI_EMISSIVITY_OFFSET 0x0012C000  // 600 KB
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart1;

XSPI_HandleTypeDef hxspi2;

/* USER CODE BEGIN PV */
/* Buffer used for transmission */
const uint8_t aTxBuffer[] = " ****Memory-mapped XSPI communication****  ****Memory-mapped XSPI communication****  ****Memory-mapped XSPI communication****  ****Memory-mapped XSPI communication****  ****Memory-mapped XSPI communication****  ****Memory-mapped XSPI communication**** ";
uint8_t aRxBuffer[BUFFERSIZE] = {0};
__IO uint8_t CmdCplt,TxCplt;

#define TEST_LINES 10

__attribute__((section(".axisram1"), aligned(32)))
static uint16_t test_frame_a[TEST_LINES][HPIX];

__attribute__((section(".axisram1"), aligned(32)))
static uint16_t test_frame_b[TEST_LINES][HPIX];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_XSPI2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void XSPI_WriteEnable(XSPI_HandleTypeDef *hxspi);
static void XSPI_AutoPollingMemReady(XSPI_HandleTypeDef *hxspi);
static void XSPI_NOR_OctalDTRModeCfg(XSPI_HandleTypeDef *hxspi);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if !defined(TERMINAL_IO)
#if defined(__ICCARM__)
int iar_fputc(int ch);
#define PUTCHAR_PROTOTYPE int iar_fputc(int ch)
#elif defined ( __CC_ARM ) || defined(__ARMCC_VERSION)
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#elif defined(__GNUC__)
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#endif
#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  XSPI_RegularCmdTypeDef sCommand = {0};
  XSPI_MemoryMappedTypeDef sMemMappedCfg = {0};

  uint16_t index = 0;
  uint16_t res = 0;
  uint32_t address = 0;
  __IO uint8_t step = 0;
  __IO uint8_t *mem_addr;

  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //BSP_LED_Init(LED_GREEN);
  //BSP_LED_Init(LED_RED);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_XSPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  setvbuf(stdout, NULL, _IONBF, 0);

  printf("\n");
  printf("STM32N6 Thermal Imaging Pipeline Demoboard\n");
  printf("\n");

  printf("System Configuration:\n");
  printf("  SYSCLK:     %lu MHz\n", HAL_RCC_GetSysClockFreq() / 1000000);
  printf("  HCLK:       %lu MHz\n", HAL_RCC_GetHCLKFreq() / 1000000);
  printf("  UART:       USART1 @ 115200 baud\n");
  printf("  LED_GREEN:  Blinking\n");
  printf("  LED_RED:    Ready\n\n");

  printf("Testing printf with float: %.2f\n", 3.14159f);
  printf("Testing printf with hex: 0x%08X\n", 0xDEADBEEF);
  printf("Testing printf with long: %lu\n\n", 1234567890UL);



  /* Configure the memory in octal DTR mode ----------------------------------- */
  XSPI_NOR_OctalDTRModeCfg(&hxspi2);
  /* USER CODE END 2 */


  printf("\n");
  printf("+--------------------------------------------------+\n");
  printf("| MEMORY TEST                                      |\n");
  printf("+--------------------------------------------------+\n");

  // Test 1: Fill pattern
  printf("Test 1: Writing pattern to test buffers...\n");
  for (uint32_t y = 0; y < TEST_LINES; y++) {
      for (uint32_t x = 0; x < HPIX; x++) {
          test_frame_a[y][x] = (uint16_t)((y * HPIX + x) & 0xFFFF);
      }
  }
  printf("  ✓ Written %lu bytes\n", (uint32_t)(TEST_LINES * HPIX * 2));

  // Test 2: Verify pattern
  printf("Test 2: Verifying pattern...\n");
  uint32_t errors = 0;
  for (uint32_t y = 0; y < TEST_LINES; y++) {
      for (uint32_t x = 0; x < HPIX; x++) {
          uint16_t expected = (uint16_t)((y * HPIX + x) & 0xFFFF);
          if (test_frame_a[y][x] != expected) {
              errors++;
              if (errors < 10) {  // Show first 10 errors only
                  printf("  Error @ [%lu][%lu]: 0x%04X != 0x%04X\n",
                         y, x, test_frame_a[y][x], expected);
              }
          }
      }
  }

  if (errors == 0) {
      printf("All %lu pixels correct!\n", (uint32_t)(TEST_LINES * HPIX));
  } else {
      printf("%lu errors found!\n", errors);
  }
  printf("Test 3: Memory copy test...\n");
  uint32_t start_tick = HAL_GetTick();

  for (uint32_t y = 0; y < TEST_LINES; y++) {
      memcpy(test_frame_b[y], test_frame_a[y], HPIX * sizeof(uint16_t));
  }

  uint32_t end_tick = HAL_GetTick();
  uint32_t duration = end_tick - start_tick;

  printf("  ✓ Copied %lu KB in %lu ms\n",
         (TEST_LINES * HPIX * 2) / 1024, duration);

  // Test 4: Verify copy
  printf("Test 4: Verify copy...\n");
  errors = 0;
  for (uint32_t y = 0; y < TEST_LINES; y++) {
      for (uint32_t x = 0; x < HPIX; x++) {
          if (test_frame_a[y][x] != test_frame_b[y][x]) {
              errors++;
          }
      }
  }

  if (errors == 0) {
      printf("  ✓ Copy verified!\n");
  } else {
      printf("  ✗ %lu copy errors!\n", errors);
  }

  printf("+--------------------------------------------------+\n\n");

  // Visual success indicator
  if (errors == 0) {
      printf("✓ Memory test PASSED!\n\n");
  } else {
      printf("✗ Memory test FAILED!\n\n");
  }

  printf("+--------------------------------------------------+\n");
  printf("| XSPI FLASH TEST                                  |\n");
  printf("+--------------------------------------------------+\n");


  XSPI_AutoPollingMemReady(&hxspi2);

  /* Cached data is not up to date due to indirect write.
     Force new read by invalidating the corresponding cache lines */
  SCB_InvalidateDCache_by_Addr((void *)(XSPI2_BASE + address), BUFFERSIZE);

  /* Memory-mapped mode configuration ------------------------------- */
  sCommand.OperationType = HAL_XSPI_OPTYPE_WRITE_CFG;
  sCommand.Instruction   = OCTAL_PAGE_PROG_CMD;
  sCommand.DataMode      = HAL_XSPI_DATA_8_LINES;
  sCommand.DataLength    = 1;
  sCommand.DQSMode       = HAL_XSPI_DQS_ENABLE;

  if (HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  sCommand.OperationType = HAL_XSPI_OPTYPE_READ_CFG;
  sCommand.Instruction   = OCTAL_IO_DTR_READ_CMD;
  sCommand.DummyCycles   = DUMMY_CLOCK_CYCLES_READ;
  sCommand.DQSMode       = HAL_XSPI_DQS_ENABLE;

  if (HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  sMemMappedCfg.NoPrefetchAXI       = HAL_XSPI_AXI_PREFETCH_ENABLE;
  sMemMappedCfg.NoPrefetchData      = HAL_XSPI_AUTOMATIC_PREFETCH_ENABLE;
  sMemMappedCfg.TimeOutActivation   = HAL_XSPI_TIMEOUT_COUNTER_DISABLE;
  sMemMappedCfg.TimeoutPeriodClock  = 0x40;

  if (HAL_XSPI_MemoryMapped(&hxspi2, &sMemMappedCfg) != HAL_OK)
  {
    Error_Handler();
  }

  // Test 1: Memory-mapped read test
  printf("Test 1: Reading XSPI @ 0x%08X...\n", XSPI_BASE_ADDRESS);

  volatile uint32_t *xspi_ptr = (volatile uint32_t *)XSPI_BASE_ADDRESS;
  uint32_t test_value = xspi_ptr[0];

  printf("  First word: 0x%08X\n", test_value);
  printf("  ✓ XSPI memory-mapped read works!\n");

  // Test 2: Read pattern
  printf("Test 2: Reading 16 words...\n");
  for (int i = 0; i < 16; i++) {
      printf("  [%02d]: 0x%08X\n", i, xspi_ptr[i]);
  }

  address += XSPI_PAGE_SIZE;
  if(address >= XSPI_END_ADDR)
  {
    address = 0;
  }

  /* Abort xSPI driver to stop the memory-mapped mode ------------ */
  if (HAL_XSPI_Abort(&hxspi2) != HAL_OK)
  {
    Error_Handler();
  }
  printf("+--------------------------------------------------+\n\n");


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    switch(step)
    {
      /* Step 0: Enable write operation (WE bit) and Erase Sector related
      to address: address */
    case 0:
      CmdCplt = 0;

      /* Enable write operations ------------------------------------------ */
      XSPI_WriteEnable(&hxspi2);

      /* Erasing Sequence ------------------------------------------------- */
      sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
      sCommand.Instruction        = OCTAL_SECTOR_ERASE_CMD;
      sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
      sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
      sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
      sCommand.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
      sCommand.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
      sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
      sCommand.Address            = address;
      sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
      sCommand.DataDTRMode        = HAL_XSPI_DATA_DTR_ENABLE;
      sCommand.DataMode           = HAL_XSPI_DATA_NONE;
      sCommand.DummyCycles        = 0;
      sCommand.DQSMode            = HAL_XSPI_DQS_ENABLE;

      if (HAL_XSPI_Command_IT(&hxspi2, &sCommand) != HAL_OK)
      {
        Error_Handler();
      }

      step++;
      break;

      /* Step 1: Wait end of Write In Progress (WIP bit), enable write operation
      (WE bit) and Write data present at address: aTxBuffer */
    case 1:
      if(CmdCplt != 0)
      {
        CmdCplt = 0;

        /* Configure automatic polling mode to wait for end of erase -------- */
        XSPI_AutoPollingMemReady(&hxspi2);

        /* Enable write operations ------------------------------------------ */
        XSPI_WriteEnable(&hxspi2);

        /* Writing Sequence ------------------------------------------------- */
        sCommand.Instruction = OCTAL_PAGE_PROG_CMD;
        sCommand.DataMode    = HAL_XSPI_DATA_8_LINES;
        sCommand.DataLength  = BUFFERSIZE;
        sCommand.Address     = address;

        if (HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        {
          Error_Handler();
        }

        if (HAL_XSPI_Transmit_IT(&hxspi2, aTxBuffer) != HAL_OK)
        {
          Error_Handler();
        }

        step++;
      }
      break;

      /* Step 2:  Wait end of Write In Progress (WIP bit), Switch memory in Mapped Mode,
      read data into memory and check validity */
    case 2:
      if(TxCplt != 0)
      {
        TxCplt = 0;

        /* Configure automatic polling mode to wait for end of erase -------- */
        XSPI_AutoPollingMemReady(&hxspi2);

        /* Cached data is not up to date due to indirect write.
           Force new read by invalidating the corresponding cache lines */
        SCB_InvalidateDCache_by_Addr((void *)(XSPI2_BASE + address), BUFFERSIZE);

        /* Memory-mapped mode configuration ------------------------------- */
        sCommand.OperationType = HAL_XSPI_OPTYPE_WRITE_CFG;
        sCommand.Instruction   = OCTAL_PAGE_PROG_CMD;
        sCommand.DataMode      = HAL_XSPI_DATA_8_LINES;
        sCommand.DataLength    = 1;
        sCommand.DQSMode       = HAL_XSPI_DQS_ENABLE;

        if (HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        {
          Error_Handler();
        }

        sCommand.OperationType = HAL_XSPI_OPTYPE_READ_CFG;
        sCommand.Instruction   = OCTAL_IO_DTR_READ_CMD;
        sCommand.DummyCycles   = DUMMY_CLOCK_CYCLES_READ;
        sCommand.DQSMode       = HAL_XSPI_DQS_ENABLE;

        if (HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        {
          Error_Handler();
        }

        sMemMappedCfg.NoPrefetchAXI       = HAL_XSPI_AXI_PREFETCH_ENABLE;
        sMemMappedCfg.NoPrefetchData      = HAL_XSPI_AUTOMATIC_PREFETCH_ENABLE;
        sMemMappedCfg.TimeOutActivation   = HAL_XSPI_TIMEOUT_COUNTER_DISABLE;
        sMemMappedCfg.TimeoutPeriodClock  = 0x40;

        if (HAL_XSPI_MemoryMapped(&hxspi2, &sMemMappedCfg) != HAL_OK)
        {
          Error_Handler();
        }

        /* Reading Sequence ------------------------------------------------- */
        mem_addr = (uint8_t *)(XSPI2_BASE + address);
        for (index = 0; index < BUFFERSIZE ; index++)
        {
          aRxBuffer[index] = *mem_addr;
          if (aRxBuffer[index]  != aTxBuffer[index])
          {
            res++;
          }
          mem_addr++;
        }

        if (res != 0)
        {
          BSP_LED_On(LED_RED);
          res = 0 ;
        }
        else
        {
          BSP_LED_Toggle(LED_GREEN);
          HAL_Delay(50);
        }

        address += XSPI_PAGE_SIZE;
        if(address >= XSPI_END_ADDR)
        {
          address = 0;
        }

        /* Abort xSPI driver to stop the memory-mapped mode ------------ */
        if (HAL_XSPI_Abort(&hxspi2) != HAL_OK)
        {
          Error_Handler();
        }

        step = 0;
      }
      break;

    default :
      Error_Handler();
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
/* USER CODE BEGIN CLK 1 */
/* USER CODE END CLK 1 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
   {
       RCC_OscInitTypeDef RCC_OscInitStruct = {0};
       RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

       /** Configure the System Power Supply */
       if (HAL_PWREx_ConfigSupply(PWR_EXTERNAL_SOURCE_SUPPLY) != HAL_OK)
       {
           Error_Handler();
       }

       /* Enable HSI */
       RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
       RCC_OscInitStruct.HSIState = RCC_HSI_ON;
       RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
       RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
       RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_NONE;
       RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
       RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
       RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;
       if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
       {
           Error_Handler();
       }

       /** Get current CPU/System buses clocks configuration */
       HAL_RCC_GetClockConfig(&RCC_ClkInitStruct);
       if ((RCC_ClkInitStruct.CPUCLKSource == RCC_CPUCLKSOURCE_IC1) ||
           (RCC_ClkInitStruct.SYSCLKSource == RCC_SYSCLKSOURCE_IC2_IC6_IC11))
       {
           RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK);
           RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_HSI;
           RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
           if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
           {
               Error_Handler();
           }
       }

       /** Initializes the RCC Oscillators */
       RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
       RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_ON;
       RCC_OscInitStruct.PLL1.PLLSource = RCC_PLLSOURCE_HSI;
       RCC_OscInitStruct.PLL1.PLLM = 4;
       RCC_OscInitStruct.PLL1.PLLN = 75;
       RCC_OscInitStruct.PLL1.PLLFractional = 0;
       RCC_OscInitStruct.PLL1.PLLP1 = 1;
       RCC_OscInitStruct.PLL1.PLLP2 = 1;
       RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
       RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
       RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;
       if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
       {
           Error_Handler();
       }

       /** Initializes the CPU, AHB and APB buses clocks */
       RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_CPUCLK|RCC_CLOCKTYPE_HCLK
                                    |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                                    |RCC_CLOCKTYPE_PCLK2|RCC_CLOCKTYPE_PCLK5
                                    |RCC_CLOCKTYPE_PCLK4;
       RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_IC1;
       RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_IC2_IC6_IC11;
       RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
       RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
       RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
       RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
       RCC_ClkInitStruct.APB5CLKDivider = RCC_APB5_DIV1;
       RCC_ClkInitStruct.IC1Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
       RCC_ClkInitStruct.IC1Selection.ClockDivider = 2;
       RCC_ClkInitStruct.IC2Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
       RCC_ClkInitStruct.IC2Selection.ClockDivider = 3;
       RCC_ClkInitStruct.IC6Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
       RCC_ClkInitStruct.IC6Selection.ClockDivider = 4;
       RCC_ClkInitStruct.IC11Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
       RCC_ClkInitStruct.IC11Selection.ClockDivider = 3;

       if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
       {
           Error_Handler();
       }
   }

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief XSPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_XSPI2_Init(void)
{

  /* USER CODE BEGIN XSPI2_Init 0 */

  /* USER CODE END XSPI2_Init 0 */

  XSPIM_CfgTypeDef sXspiManagerCfg = {0};

  /* USER CODE BEGIN XSPI2_Init 1 */

  /* USER CODE END XSPI2_Init 1 */
  /* XSPI2 parameter configuration*/
  hxspi2.Instance = XSPI2;
  hxspi2.Init.FifoThresholdByte = 4;
  hxspi2.Init.MemoryMode = HAL_XSPI_SINGLE_MEM;
  hxspi2.Init.MemoryType = HAL_XSPI_MEMTYPE_MACRONIX;
  hxspi2.Init.MemorySize = HAL_XSPI_SIZE_1GB;
  hxspi2.Init.ChipSelectHighTimeCycle = 1;
  hxspi2.Init.FreeRunningClock = HAL_XSPI_FREERUNCLK_DISABLE;
  hxspi2.Init.ClockMode = HAL_XSPI_CLOCK_MODE_0;
  hxspi2.Init.WrapSize = HAL_XSPI_WRAP_NOT_SUPPORTED;
  hxspi2.Init.ClockPrescaler = 1;
  hxspi2.Init.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_NONE;
  hxspi2.Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_ENABLE;
  hxspi2.Init.ChipSelectBoundary = HAL_XSPI_BONDARYOF_NONE;
  hxspi2.Init.MaxTran = 0;
  hxspi2.Init.Refresh = 0;
  hxspi2.Init.MemorySelect = HAL_XSPI_CSSEL_NCS1;
  if (HAL_XSPI_Init(&hxspi2) != HAL_OK)
  {
    Error_Handler();
  }
  sXspiManagerCfg.nCSOverride = HAL_XSPI_CSSEL_OVR_NCS1;
  sXspiManagerCfg.IOPort = HAL_XSPIM_IOPORT_2;
  sXspiManagerCfg.Req2AckTime = 1;
  if (HAL_XSPIM_Config(&hxspi2, &sXspiManagerCfg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN XSPI2_Init 2 */

  /* USER CODE END XSPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPION_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) {
    (void)file;
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

// (optional, aber praktisch) sorge dafür, dass fputc -> __io_putchar geht
int fputc(int ch, FILE *f) {
    (void)f;
    return __io_putchar(ch);
}
/**
  * @brief  Command completed callback.
  * @param  hxspi: XSPI handle
  * @retval None
  */
void HAL_XSPI_CmdCpltCallback(XSPI_HandleTypeDef *hxspi)
{
  CmdCplt++;
}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  hxspi XSPI handle
  * @retval None
  */
void HAL_XSPI_TxCpltCallback(XSPI_HandleTypeDef *hxspi)
{
  TxCplt++;
}

/**
* @brief  Transfer Error callback.
* @param  hxspi: XSPI handle
* @retval None
*/
void HAL_XSPI_ErrorCallback(XSPI_HandleTypeDef *hxspi)
{
  Error_Handler();
}

/**
* @brief  This function send a Write Enable and wait it is effective.
* @param  hxspi: XSPI handle
* @retval None
*/
static void XSPI_WriteEnable(XSPI_HandleTypeDef *hxspi)
{
  XSPI_RegularCmdTypeDef  sCommand = {0};
  XSPI_AutoPollingTypeDef sConfig  = {0};

  /* Enable write operations ------------------------------------------ */
  sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommand.Instruction        = OCTAL_WRITE_ENABLE_CMD;
  sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
  sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
  sCommand.AddressMode        = HAL_XSPI_ADDRESS_NONE;
  sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommand.DataMode           = HAL_XSPI_DATA_NONE;
  sCommand.DummyCycles        = 0;
  sCommand.DQSMode            = HAL_XSPI_DQS_DISABLE;

  if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure automatic polling mode to wait for write enabling ---- */
  sCommand.Instruction        = OCTAL_READ_STATUS_REG_CMD;
  sCommand.Address            = 0x0;
  sCommand.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
  sCommand.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
  sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
  sCommand.DataMode           = HAL_XSPI_DATA_8_LINES;
  sCommand.DataDTRMode        = HAL_XSPI_DATA_DTR_ENABLE;
  sCommand.DataLength         = 2;
  sCommand.DummyCycles        = DUMMY_CLOCK_CYCLES_READ_OCTAL;
  sCommand.DQSMode            = HAL_XSPI_DQS_ENABLE;

  if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.MatchMode           = HAL_XSPI_MATCH_MODE_AND;
  sConfig.AutomaticStop       = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
  sConfig.IntervalTime        = AUTO_POLLING_INTERVAL;
  sConfig.MatchMask           = WRITE_ENABLE_MASK_VALUE;
  sConfig.MatchValue          = WRITE_ENABLE_MATCH_VALUE;

  if (HAL_XSPI_AutoPolling(hxspi, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
* @brief  This function read the SR of the memory and wait the EOP.
* @param  hxspi: XSPI handle
* @retval None
*/
static void XSPI_AutoPollingMemReady(XSPI_HandleTypeDef *hxspi)
{
  XSPI_RegularCmdTypeDef  sCommand = {0};
  XSPI_AutoPollingTypeDef sConfig  = {0};

  /* Configure automatic polling mode to wait for memory ready ------ */
  sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommand.Instruction        = OCTAL_READ_STATUS_REG_CMD;
  sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
  sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
  sCommand.Address            = 0x0;
  sCommand.AddressMode        = HAL_XSPI_ADDRESS_8_LINES;
  sCommand.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
  sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_ENABLE;
  sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommand.DataMode           = HAL_XSPI_DATA_8_LINES;
  sCommand.DataDTRMode        = HAL_XSPI_DATA_DTR_ENABLE;
  sCommand.DataLength         = 2;
  sCommand.DummyCycles        = DUMMY_CLOCK_CYCLES_READ_OCTAL;
  sCommand.DQSMode            = HAL_XSPI_DQS_ENABLE;

  if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.MatchMode           = HAL_XSPI_MATCH_MODE_AND;
  sConfig.AutomaticStop       = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
  sConfig.IntervalTime        = AUTO_POLLING_INTERVAL;
  sConfig.MatchMask           = MEMORY_READY_MASK_VALUE;
  sConfig.MatchValue          = MEMORY_READY_MATCH_VALUE;

  if (HAL_XSPI_AutoPolling(hxspi, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
* @brief  This function configure the memory in Octal DTR mode.
* @param  hxspi: XSPI handle
* @retval None
*/
static void XSPI_NOR_OctalDTRModeCfg(XSPI_HandleTypeDef *hxspi)
{
  uint8_t reg = 0;
  XSPI_RegularCmdTypeDef  sCommand = {0};
  XSPI_AutoPollingTypeDef sConfig  = {0};

  sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
  sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_1_LINE;
  sCommand.InstructionWidth    = HAL_XSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_DISABLE;
  sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  sCommand.DataDTRMode        = HAL_XSPI_DATA_DTR_DISABLE;
  sCommand.DummyCycles        = 0;
  sCommand.DQSMode            = HAL_XSPI_DQS_DISABLE;
  sConfig.MatchMode           = HAL_XSPI_MATCH_MODE_AND;
  sConfig.AutomaticStop       = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
  sConfig.IntervalTime        = 0x10;


  /* Enable write operations */
  sCommand.Instruction = WRITE_ENABLE_CMD;
  sCommand.DataMode    = HAL_XSPI_DATA_NONE;
  sCommand.AddressMode = HAL_XSPI_ADDRESS_NONE;

  if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Reconfigure XSPI to automatic polling mode to wait for write enabling */
  sConfig.MatchMask           = 0x02;
  sConfig.MatchValue          = 0x02;

  sCommand.Instruction        = READ_STATUS_REG_CMD;
  sCommand.DataMode           = HAL_XSPI_DATA_1_LINE;
  sCommand.DataLength         = 1;

  if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_XSPI_AutoPolling(hxspi, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Write Configuration register 2 (with Octal I/O SPI protocol) */
  sCommand.Instruction  = WRITE_CFG_REG_2_CMD;
  sCommand.AddressMode  = HAL_XSPI_ADDRESS_1_LINE;
  sCommand.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;

  sCommand.Address = 0;
  reg = 0x2;


  if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_XSPI_Transmit(hxspi, &reg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  sCommand.Instruction    = READ_STATUS_REG_CMD;
  sCommand.DataMode       = HAL_XSPI_DATA_1_LINE;
  sCommand.DataLength     = 1;

  if (HAL_XSPI_Command(hxspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_XSPI_AutoPolling(hxspi, &sConfig, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    HAL_Delay(250);
    BSP_LED_Toggle(LED_RED);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
