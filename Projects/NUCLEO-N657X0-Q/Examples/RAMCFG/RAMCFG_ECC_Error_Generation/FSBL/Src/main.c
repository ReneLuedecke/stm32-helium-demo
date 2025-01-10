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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Single and Double ECC error flags declaration */
__IO uint32_t ECC_SingleError = 0U, ECC_DoubleError = 0U;

//__IO uint32_t IndexTmp = 0;

/* Single and double error data buffers */
uint32_t SingleErrorData_Buffer[BKPSRAM_SINGLE_ERROR_ADDRESS_SIZE];
uint32_t DoubleErrorData_Buffer[BKPSRAM_DOUBLE_ERROR_ADDRESS_SIZE];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

RAMCFG_HandleTypeDef hramcfg_BKPSRAM;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_RAMCFG_Init(void);
/* USER CODE BEGIN PFP */
static void Fill_wMemory(uint32_t *pMemory, uint16_t MemorySize, uint32_t Data);
static void Read_wMemory(RAMCFG_HandleTypeDef *hramcfg, uint32_t *pMemory, uint32_t *pBuffer, uint16_t BufferSize, uint32_t Interrupt);
static BufferContentStatus Check_wBuffer(uint32_t *pBuffer, uint16_t BufferSize, uint32_t CompareData);

static void SingleErrorDetectionCallback(RAMCFG_HandleTypeDef *hramcfg);
static void DoubleErrorDetectionCallback(RAMCFG_HandleTypeDef *hramcfg);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* STM32N6xx HAL library initialization:
       - Systick timer is configured by default as source of time base, but user
         can eventually implement his proper time base source (a general purpose
         timer for example or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* Initialize LED1 and LED2 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  HAL_PWR_EnableBkUpAccess();

      /* Register MSP Init callback for RAMCFG peripheral */
  if (HAL_RAMCFG_RegisterCallback(&hramcfg_BKPSRAM, HAL_RAMCFG_MSPINIT_CB_ID, HAL_RAMCFG_MspInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Register MSP Init callback for RAMCFG peripheral */
  if (HAL_RAMCFG_RegisterCallback(&hramcfg_BKPSRAM, HAL_RAMCFG_MSPDEINIT_CB_ID, HAL_RAMCFG_MspDeInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_RAMCFG_Init();
  /* USER CODE BEGIN 2 */


  /* Fill memory with known data */
  Fill_wMemory((uint32_t *)BKPSRAM_SINGLE_ERROR_ADDRESS_BASE, (BKPSRAM_SINGLE_ERROR_ADDRESS_SIZE * 2U), DATA_VALUE);

  /* Stop ECC for BKPSRAM */
  if (HAL_RAMCFG_StopECC(&hramcfg_BKPSRAM) != HAL_OK)
  {
    Error_Handler();
  }

  /* Fill memory with single error data */
  Fill_wMemory((uint32_t *)BKPSRAM_SINGLE_ERROR_ADDRESS_BASE, BKPSRAM_SINGLE_ERROR_ADDRESS_SIZE, SINGLE_ERROR_DATA_VALUE);

  /* Fill memory with single error data */
  Fill_wMemory((uint32_t *)BKPSRAM_DOUBLE_ERROR_ADDRESS_BASE, BKPSRAM_DOUBLE_ERROR_ADDRESS_SIZE, DOUBLE_ERROR_DATA_VALUE);

  /*##-3- Read memory content and check that single and double ECC errors are
          generated ##########################################################*/
  /* Start ECC for BKPSRAM */
  if (HAL_RAMCFG_StartECC(&hramcfg_BKPSRAM) != HAL_OK)
  {
    Error_Handler();
  }

  /* Register callback for RAMCFG peripheral single ECC error interrupt */
  if (HAL_RAMCFG_RegisterCallback(&hramcfg_BKPSRAM, HAL_RAMCFG_SE_DETECT_CB_ID, SingleErrorDetectionCallback) != HAL_OK)
  {
    Error_Handler();
  }

  /* Register callback for RAMCFG peripheral double ECC error interrupt */
  if (HAL_RAMCFG_RegisterCallback(&hramcfg_BKPSRAM, HAL_RAMCFG_DE_DETECT_CB_ID, DoubleErrorDetectionCallback) != HAL_OK)
  {
    Error_Handler();
  }

  /* Enable RAMECC single and double ECC error interrupts */
  if (HAL_RAMCFG_EnableNotification(&hramcfg_BKPSRAM, (RAMCFG_IT_SINGLEERR | RAMCFG_IT_DOUBLEERR)) != HAL_OK)
  {
    Error_Handler();
  }

  /* Read single error memory area */
  Read_wMemory(&hramcfg_BKPSRAM, (uint32_t *)BKPSRAM_SINGLE_ERROR_ADDRESS_BASE, SingleErrorData_Buffer, BKPSRAM_SINGLE_ERROR_ADDRESS_SIZE, RAMCFG_IT_SINGLEERR);

  /* Read double error memory area */
  Read_wMemory(&hramcfg_BKPSRAM, (uint32_t *)BKPSRAM_DOUBLE_ERROR_ADDRESS_BASE, DoubleErrorData_Buffer, BKPSRAM_DOUBLE_ERROR_ADDRESS_SIZE, RAMCFG_IT_DOUBLEERR);

  /* Check that all single error data are corrected via RAMCFG peripheral */
  if (Check_wBuffer(SingleErrorData_Buffer, BKPSRAM_SINGLE_ERROR_ADDRESS_SIZE, DATA_VALUE) != MATCH)
  {
    Error_Handler();
  }

  /* Check that all double error data are not corrected via RAMCFG peripheral */
  if (Check_wBuffer(DoubleErrorData_Buffer, BKPSRAM_DOUBLE_ERROR_ADDRESS_SIZE, DOUBLE_ERROR_DATA_VALUE) != MATCH)
  {
    Error_Handler();
  }

  /* Check the number of ECC single errors detected */
  if (ECC_SingleError != EXPECTED_SINGLE_ERROR_NUM)
  {
    Error_Handler();
  }

  /* Check the number of ECC double errors detected */
  if (ECC_DoubleError != EXPECTED_DOUBLE_ERROR_NUM)
  {
    Error_Handler();
  }

  /* DeInitialize the RAMCFG peripheral */
  if (HAL_RAMCFG_DeInit(&hramcfg_BKPSRAM) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(250);
    BSP_LED_Toggle(LED1);
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

  /** Configure the System Power Supply
  */
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

  /** Get current CPU/System buses clocks configuration and if necessary switch
 to intermediate HSI clock to ensure target clock can be set
  */
  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct);
  if ((RCC_ClkInitStruct.CPUCLKSource == RCC_CPUCLKSOURCE_IC1) ||
     (RCC_ClkInitStruct.SYSCLKSource == RCC_SYSCLKSOURCE_IC2_IC6_IC11))
  {
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK);
    RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
    {
      /* Initialization Error */
      Error_Handler();
    }
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  * @brief RAMCFG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RAMCFG_Init(void)
{

  /* USER CODE BEGIN RAMCFG_Init 0 */

  /* USER CODE END RAMCFG_Init 0 */

  /* USER CODE BEGIN RAMCFG_Init 1 */

  /* USER CODE END RAMCFG_Init 1 */

  /** Initialize RAMCFG BKPSRAM
  */
  hramcfg_BKPSRAM.Instance = RAMCFG_BKPSRAM;
  if (HAL_RAMCFG_Init(&hramcfg_BKPSRAM) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_RAMCFG_Erase(&hramcfg_BKPSRAM) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_RAMCFG_StartECC(&hramcfg_BKPSRAM) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RAMCFG_Init 2 */

  /* USER CODE END RAMCFG_Init 2 */

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Fill a global 32-bit memory
  * @param  pMemory    : pointer on the Memory to fill
  * @param  MemorySize : size of the memory to fill
  * @param  Data       : data value
  * @retval None
  */
static void Fill_wMemory(uint32_t *pMemory, uint16_t MemorySize, uint32_t Data)
{
  __IO uint32_t IndexTmp;

  /* Repeat for all buffer size */
  for (IndexTmp = 0; IndexTmp < MemorySize; IndexTmp++)
  {
    pMemory[IndexTmp] = Data;
  }
}

/**
  * @brief  Read a global 32-bit memory
  * @param  pMemory    : pointer on the memory to read
  * @param  pBuffer    : pointer on the Buffer to fill
  * @param  BufferSize : size of the data to read
  * @retval None
  */
static void Read_wMemory(RAMCFG_HandleTypeDef *hramcfg, uint32_t *pMemory, uint32_t *pBuffer, uint16_t BufferSize, uint32_t Interrupt)
{
  __IO uint32_t IndexTmp = 0;
  uint32_t VarTmp;

  /* Repeat for all buffer size */
  for (IndexTmp = 0; IndexTmp < BufferSize; IndexTmp++)
  {
    HAL_RAMCFG_EnableNotification(hramcfg, Interrupt);
    VarTmp = pMemory[IndexTmp];
    pBuffer[IndexTmp] = VarTmp;
  }
}

/**
  * @brief  Check a global 32-bit buffer content
  * @param  pBuffer      : pointer on the Buffer to check
  * @param  BufferSize   : size of the data to check
  * @param  ComparedData : data to be compared with buffer content
  * @retval None
  */
static BufferContentStatus Check_wBuffer(uint32_t *pBuffer, uint16_t BufferSize, uint32_t CompareData)
{
  __IO uint32_t IndexTmp;

  /* Repeat for all buffer size */
  for (IndexTmp = 0; IndexTmp < BufferSize; IndexTmp++)
  {
    if (pBuffer[IndexTmp] != CompareData)
    {
      return NOTMATCH;
    }
  }

  return MATCH;
}

/**
  * @brief  RAMCFG single error detection callback
  * @param  hramcfg : Pointer to a RAMCFG_HandleTypeDef structure that contains
  *                   the configuration information for the specified RAMCFG
  * @retval None
  */
static void SingleErrorDetectionCallback(RAMCFG_HandleTypeDef *hramcfg)
{
  ECC_SingleError++;
  HAL_RAMCFG_DisableNotification(hramcfg, RAMCFG_IT_SINGLEERR);
}

/**
  * @brief  RAMCFG single error detection callback
  * @param  hramcfg : Pointer to a RAMCFG_HandleTypeDef structure that contains
  *                   the configuration information for the specified RAMCFG
  * @retval None
  */
static void DoubleErrorDetectionCallback(RAMCFG_HandleTypeDef *hramcfg)
{
  ECC_DoubleError++;
  HAL_RAMCFG_DisableNotification(hramcfg, RAMCFG_IT_DOUBLEERR);
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
  /* Turn LED2 on */
  BSP_LED_On(LED2);
  while (1)
  {
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
