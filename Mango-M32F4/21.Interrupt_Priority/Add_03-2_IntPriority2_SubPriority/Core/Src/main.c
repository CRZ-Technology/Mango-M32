/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART1 and Loop until the end of transmission */

    /* Write a character to the USART */  
    if(ch == '\n')
    {
        ch = '\r';
        HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
        ch = '\n';
        HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    }else {
        HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    }

    return ch;
}

void System_Information(void)
{
    printf("SYSCLK_Frequency = %d\n", (unsigned int)HAL_RCC_GetSysClockFreq());
    printf("HCLK_Frequency = %d\n", (unsigned int)HAL_RCC_GetHCLKFreq());
    printf("PCLK1_Frequency = %d\n", (unsigned int)HAL_RCC_GetPCLK1Freq());
    printf("PCLK2_Frequency = %d\n", (unsigned int)HAL_RCC_GetPCLK2Freq());
}

void KEY_Test(void)
{
    HAL_GPIO_WritePin(GPIOF, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_SET);

    while(1)
    {
        if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
        {
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        }

        if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
        {
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
        }
    }
}

void Bit_Banding_Test(void)
{
#define RAM_BASE       0x20000000
#define RAM_BB_BASE    0x22000000
 
#define Var_ResetBit_BB(VarAddr, BitNumber)   \
          (*(__IO uint32_t *) (RAM_BB_BASE | \
          ((VarAddr - RAM_BASE) << 5) | ((BitNumber) << 2)) = 0)
   
#define Var_SetBit_BB(VarAddr, BitNumber)     \
          (*(__IO uint32_t *) (RAM_BB_BASE | \
          ((VarAddr - RAM_BASE) << 5) | ((BitNumber) << 2)) = 1)

#define Var_GetBit_BB(VarAddr, BitNumber)     \
          (*(__IO uint32_t *) (RAM_BB_BASE | \
          ((VarAddr - RAM_BASE) << 5) | ((BitNumber) << 2)))

__IO unsigned int Var, VarAddr = 0, VarBitValue = 0;

    printf("Bit Banding Test Start ...\n");

    Var = 0x00005AA5;
    printf("(1) Var: %0X\n", Var);

    /* Get the variable address */ 
    VarAddr = (uint32_t)&Var; 
    printf("(2) VarAddr: %0X\n", VarAddr);

    /* Modify variable bit using bit-band access */
    /* Modify Var variable bit 0 */
    Var_ResetBit_BB(VarAddr, 0);  /* Var = 0x00005AA4 */
    printf("(3) Var: %0X\n", Var);
    Var_SetBit_BB(VarAddr, 0);    /* Var = 0x00005AA5 */
    printf("(4) Var: %0X\n", Var);

    /* Modify Var variable bit 11 */
    Var_ResetBit_BB(VarAddr, 11);             /* Var = 0x000052A5 */
    printf("(5) Var: %0X\n", Var);
    /* Get Var variable bit 11 value */
    VarBitValue = Var_GetBit_BB(VarAddr, 11); /* VarBitValue = 0x0 */
    printf("(6) VarBitValue: %0X\n", VarBitValue);

    Var_SetBit_BB(VarAddr, 11);               /* Var = 0x00005AA5 */
    printf("(7) Var: %0X\n", Var);
    /* Get Var variable bit 11 value */
    VarBitValue = Var_GetBit_BB(VarAddr, 11); /* VarBitValue = 0x1 */
    printf("(8) VarBitValue: %0X\n", VarBitValue);
}

/* Private define ------------------------------------------------------------*/
#define SP_PROCESS_SIZE             0x200  /* Process stack size */
#define SP_PROCESS                  0x02   /* Process stack */
#define SP_MAIN                     0x00   /* Main stack */
#define THREAD_MODE_PRIVILEGED      0x00   /* Thread mode has privileged access */
#define THREAD_MODE_UNPRIVILEGED    0x01   /* Thread mode has unprivileged access */

/* Private macro -------------------------------------------------------------*/ 
#if defined ( __CC_ARM   )
__ASM void __SVC(void) 
{ 
  SVC 0x01 
  BX R14
}
#elif defined ( __ICCARM__ )
static __INLINE  void __SVC()                     { __ASM ("svc 0x01");}
#elif defined   (  __GNUC__  )
static __INLINE void __SVC()                      { __ASM volatile ("svc 0x01");}
#endif

/* Private variables ---------------------------------------------------------*/
__IO uint8_t PSPMemAlloc[SP_PROCESS_SIZE];

unsigned int  Get_Current_ThreadMode(void)
{
    /* Check Thread mode privilege status */
    if((__get_CONTROL() & 0x01) == THREAD_MODE_PRIVILEGED)
    {
        /* Thread mode has privileged access  */
        return THREAD_MODE_PRIVILEGED;
    }
    else
    {
        /* Thread mode has unprivileged access*/
        return THREAD_MODE_UNPRIVILEGED;
    }
}

unsigned int  Get_Current_Stack(void)
{
    if((__get_CONTROL() & 0x02) == SP_MAIN)
    {
        /* Main stack is used as the current stack */
        return SP_MAIN;
    }
    else
    {
        /* Process stack is used as the current stack */
        return SP_PROCESS;
    }
}

void Mode_Privilege_Test(void)
{
    __IO uint32_t Index;

    printf("Mode_Privilege_Test Start ...\n");

    /* Switch Thread mode Stack from Main to Process */
    /* Initialize memory reserved for Process Stack */
    for(Index = 0; Index < SP_PROCESS_SIZE; Index++)
    {
        PSPMemAlloc[Index] = 0x00;
    }
    printf("PSPMemAlloc address = 0x%0X\n", (unsigned int)PSPMemAlloc);

    printf("(1) CurrentStack = %d, ThreadMode = %d\n",
           Get_Current_Stack(), Get_Current_ThreadMode());

    /* Set Process stack value */ 
    __set_PSP((uint32_t)PSPMemAlloc + SP_PROCESS_SIZE);
    printf("(2) CurrentStack = %d, ThreadMode = %d\n",
           Get_Current_Stack(), Get_Current_ThreadMode());

    /* Select Process Stack as Thread mode Stack */
    __set_CONTROL(SP_PROCESS);
    printf("(3) CurrentStack = %d, ThreadMode = %d\n",
           Get_Current_Stack(), Get_Current_ThreadMode());

    /* Get the Thread mode stack used */
    if(Get_Current_Stack() == SP_MAIN)
    {
        printf("CurrentStack is Main Stack\n");
    }
    else
    {
        printf("CurrentStack is Process Stack\n");

        /* Get process stack pointer value */
        printf("PSPValue = 0x%0X\n", (unsigned int)__get_PSP());
    }
  
    /* Switch Thread mode from privileged to unprivileged */
    /* Thread mode has unprivileged access */
    __set_CONTROL(THREAD_MODE_UNPRIVILEGED | SP_PROCESS);
    /* Unprivileged access mainly affect ability to:
    - Use or not use certain instructions such as MSR fields
    - Access System Control Space (SCS) registers such as NVIC and SysTick */
    printf("(4) CurrentStack = %d, ThreadMode = %d\n",
           Get_Current_Stack(), Get_Current_ThreadMode());

    /* Switch back Thread mode from unprivileged to privileged */
    /* Try to switch back Thread mode to privileged (Not possible, this can be
    done only in Handler mode) */
    __set_CONTROL(THREAD_MODE_PRIVILEGED | SP_PROCESS);
    printf("(5) CurrentStack = %d, ThreadMode = %d\n",
           Get_Current_Stack(), Get_Current_ThreadMode());

    /* Generate a system call exception, and in the ISR switch back Thread mode
    to privileged */
    __SVC();
    printf("(6) CurrentStack = %d, ThreadMode = %d\n",
           Get_Current_Stack(), Get_Current_ThreadMode());

    /* Select Main Stack */
    __set_CONTROL(SP_MAIN);
    printf("(7) CurrentStack = %d, ThreadMode = %d\n",
           Get_Current_Stack(), Get_Current_ThreadMode());
}

uint8_t g_RxBuffer;

HAL_StatusTypeDef Uart_Start_IT_Receive_forComm(void)
{
    return HAL_UART_Receive_IT(&huart1, &g_RxBuffer, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        int k;
        for(k = 0; k <10; k ++)
        {
            for(volatile int i = 3000000; i > 0; i --);
            printf("HAL_UART_RxCpltCallback called k: %d!\n", k);
        }

        Uart_Start_IT_Receive_forComm();
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  Uart_Start_IT_Receive_forComm();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    printf("Hello World! - Mango ^)^\n");
    System_Information();

//    KEY_Test();
//    Bit_Banding_Test();
//    Mode_Privilege_Test();

    HAL_Delay(5000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
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
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY1_Pin KEY2_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin|KEY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
