/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "fatfs.h"
#include "lwip.h"
#include "usb_device.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/apps/httpd.h"
#include "test_sensor_LPS25H.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LPS25H_SLAVE_ADDRESS (0x5D << 1)

#define COUNT_AVERAGE_VAL 1000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static unsigned int count = 0;
static uint8_t LPS25H_Buffer_Tx1[100];

#define hUART           huart3
int _write( int32_t file , uint8_t *ptr , int32_t len )
{
    /* Implement your write code here, this is used by puts and printf for example */
    for ( int16_t i = 0 ; i < len ; ++i )
    {
        HAL_UART_Transmit( &hUART, ptr++, 1, 100);
    }
    return len;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == KEY1_Pin)
	{
		if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
		{
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin );
		}
	}
	else if(GPIO_Pin == KEY2_Pin)
	{
		if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
		{
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin );
		}
	}
}
 uint8_t SENSOR_LPS25H_Read_Bytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
	HAL_StatusTypeDef  i2cRW_status;

    i2cRW_status = HAL_I2C_Master_Transmit(&hi2c1, devAddr, &regAddr, 1, 1000);
    if(HAL_OK != i2cRW_status)
    {
    	printf("Write error:%d\r\n", i2cRW_status);
        return 0;
    }

    i2cRW_status = HAL_I2C_Master_Receive(&hi2c1, devAddr, data, length, 1000);
    if(HAL_OK != i2cRW_status)
    {
    	printf("Read error:%d\r\n", i2cRW_status);
        return 0;
    }

    return length;
}

int8_t SENSOR_LPS25H_Write_Bytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
	HAL_StatusTypeDef  i2cRW_status;

    LPS25H_Buffer_Tx1[0] = regAddr;
    memcpy(&LPS25H_Buffer_Tx1[1], data, length);

    i2cRW_status = HAL_I2C_Master_Transmit(&hi2c1, devAddr, LPS25H_Buffer_Tx1, length + 1, 1000);
    if(HAL_OK != i2cRW_status)
    {
    	printf("Write error:%d\r\n", i2cRW_status);
        return -1;
    }

    return 0;
}

/**
 * @brief  Set the slave address according to SA0 bit.
 * @param  SA0_Bit_Status: LPS25H_SA0_LOW or LPS25H_SA0_HIGH
 * @retval None
 */
void LPS25H_Init(void)
{
    uint8_t dataVal = 0x00;
    uint8_t dataValRead = 0x00;

    //PowerOn
    dataVal|= LPS25H_MODE_ACTIVE;
    /* Output Data Rate selection */
    dataVal |= LPS25H_ODR_25Hz;        //Output Data Rate: P - 25Hz, T - 25H
    /* Interrupt circuit selection */
    dataVal |= LPS25H_DIFF_ENABLE;     //interrupt circuit enabled
    /* Block Data Update selection */
    dataVal |= LPS25H_BDU_READ;        //output registers not updated until MSB and LSB reading
    /* Serial Interface Mode selection */
    dataVal |= LPS25H_SPI_SIM_3W;      //3-wire interface
    SENSOR_LPS25H_Write_Bytes(LPS25H_SLAVE_ADDRESS, LPS25H_CTRL_REG1_ADDR, 1, &dataVal);
    SENSOR_LPS25H_Read_Bytes(LPS25H_SLAVE_ADDRESS, LPS25H_CTRL_REG1_ADDR, 1, &dataValRead);
    printf("LPS25H_CTRL_REG1_ADDR init read : 0x%02X\r\n", dataValRead);
    dataVal = 0x00;
    dataVal |= LPS25H_P_RES_AVG_512;    //LPS25H Pressure resolution selection AVG_51
    dataVal |= LPS25H_T_RES_AVG_64;     //LPS25H Temperature resolution selection AVG_64
    SENSOR_LPS25H_Write_Bytes(LPS25H_SLAVE_ADDRESS, LPS25H_RES_CONF_ADDR, 1, &dataVal);
    SENSOR_LPS25H_Read_Bytes(LPS25H_SLAVE_ADDRESS, LPS25H_RES_CONF_ADDR, 1, &dataValRead);
    printf("LPS25H_RES_CONF_ADDR init read : 0x%02X\r\n", dataValRead);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    uint8_t   dataVal[3];
    uint8_t   buffer[3]={0}, i;
    uint32_t  data32Val;
    int16_t   data16Val;
    uint16_t  unsigned_data16Val;

    uint32_t  k;

    float calcData;
    uint32_t  data32Val_sum;

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
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  MX_FATFS_Init();
  MX_USB_HOST_Init();
  MX_LWIP_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  printf("I2C-LPS25 test\r\n");
  httpd_init();

  HAL_Delay(5000);
  LPS25H_Init();
  HAL_Delay(300);
  SENSOR_LPS25H_Read_Bytes(LPS25H_SLAVE_ADDRESS, LPS25H_WHO_AM_I_ADDR, 1, dataVal);
  printf("Device id : 0x%02X\r\n", dataVal[0]);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	MX_LWIP_Process();

	if(count++ % 10000 == 0)
	{		data32Val = 0;
		data32Val_sum = 0;

		for(k = 0; k < COUNT_AVERAGE_VAL; k ++)
		{
			/*
			The device is a slave with 1 byte sub-address which MSB should be ‘1’ to enable the
			multiple data read/write at increasing addresses (bit 7 = 1 enables I²C sub-address
			multi-byte auto-increment)
			*/
			SENSOR_LPS25H_Read_Bytes(LPS25H_SLAVE_ADDRESS, LPS25H_PRESS_POUT_XL_ADDR + 0x80, 3, buffer);
			// printf("data[0]: 0x%02X, data[1]: 0x%02X, data[2]: 0x%02X\n", buffer[0], buffer[1], buffer[2]);

			/* Build the raw data */
			for(i = 0; i < 3; i ++)
			{
				data32Val |= (((uint32_t) buffer[i]) << (8 * i));
			}

			/* convert the 2's complement 24 bit to 2's complement 32 bit */
			if (data32Val & 0x00800000)
			{
				printf("data32Val & 0x00800000) is true \n");
				data32Val |= 0xFF000000;
			}

			data32Val_sum += data32Val;

			HAL_Delay(1);
		}

		calcData = (float)data32Val_sum / COUNT_AVERAGE_VAL / 4096.0;

		printf("Pressure: %5.2f hPa\r\n", calcData);

        data16Val=0;

        /*
         * @brief Read LPS25H output register, and calculate the raw temperature.
         * @param int16_t *raw_data: temperature raw value.
         * @retval LPS25H_ERROR or LPS25H_OK.
         */
        SENSOR_LPS25H_Read_Bytes(LPS25H_SLAVE_ADDRESS, LPS25H_TEMP_OUT_L_ADDR + 0x80, 2, buffer);

        printf("buffer[0]: 0x%02X, buffer[1]: 0x%02X\r\n", buffer[0], buffer[1]);

        data16Val = (((int16_t)buffer[1]) << 8) + (int16_t)buffer[0];

        calcData = (((float)data16Val/480.0) + 42.5);

        unsigned_data16Val=data16Val;
        printf("data16Val: 0x%4X, Temperature: %f°C\r\n", unsigned_data16Val, calcData);
    }
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : KEY1_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY2_Pin */
  GPIO_InitStruct.Pin = KEY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
