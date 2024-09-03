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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void LCD(uint8_t val_1, uint8_t cmd)
{
	uint8_t data1;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, cmd); // set RS = cmd; (cmd=0)=>Command; (cmd=1) => data

	data1 = val_1 & 0x01;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, data1);

	data1 = (val_1 >> 1) & 0x01;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, data1);

	data1 = (val_1 >> 2) & 0x01;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, data1);

	data1 = (val_1 >> 3) & 0x01;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, data1);

	data1 = (val_1 >> 4) & 0x01;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, data1);

	data1 = (val_1 >> 5) & 0x01;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, data1);

	data1 = (val_1 >> 6) & 0x01;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, data1);

	data1 = (val_1 >> 7) & 0x01;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, data1);

	//Enable
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
}

// Helper function to send command to the LCD
void LCD_command(uint8_t command)
{
    LCD(command, 0); // Send command
}

// Helper function to send data to the LCD
void LCD_data(const char* str)
{
    // Iterate through each character in the string
    while (*str)
    {
        // Cast the character to uint8_t and send it to the LCD as data
        LCD((uint8_t)(*str), 1);
        str++;  // Move to the next character
    }
}

void LCD_init()
{
	LCD_command(0x38); //2 lines, 5*7 matrix
	LCD_command(0x0C); //Display on, cursor off
	LCD_command(0x06); //Increment cursor (shift to right)
	LCD_command(0x01); //Clear display screen
	LCD_command(0x80); //Force cursors to beginning (1st line)
}

void Print(float number)
{
	int ceo, ostCeo=0;
	float ost;
	int i=0;
	char buffer[10];

	ceo = number;
	ost = number-ceo;
	ostCeo = ost*100;
    ostCeo=abs(ostCeo);

    sprintf(buffer,"%d",ceo);
    while(buffer[i])
	{
	    LCD(buffer[i],1);
		i++;
	}

	LCD('.',1);
	sprintf(buffer,"%d",ostCeo);
	i=0;

	while(buffer[i])
	{
		LCD(buffer[i],1);
		i++;
	}
}

void ADC_Select_CH0 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_0;
	  sConfig.Rank = 1;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH1 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_1;
	  sConfig.Rank = 1;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH2 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_2;
	  sConfig.Rank = 1;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  uint16_t X, Y, Z;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	  	  LCD_init();
	  	  LCD_data("MERIM UGAO...");
	  	  HAL_Delay(500);

	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	  	  LCD_init();
	  	  HAL_Delay(200);

	      ADC_Select_CH0();
	      HAL_ADC_Start(&hadc1);
	  	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  	  X = HAL_ADC_GetValue(&hadc1);
	      HAL_ADC_Stop(&hadc1);

	      ADC_Select_CH1();
	      HAL_ADC_Start(&hadc1);
	      HAL_ADC_PollForConversion(&hadc1, 1000);
	      Y = HAL_ADC_GetValue(&hadc1);
	      HAL_ADC_Stop(&hadc1);

	      ADC_Select_CH2();
	      HAL_ADC_Start(&hadc1);
	      HAL_ADC_PollForConversion(&hadc1, 1000);
	      Z = HAL_ADC_GetValue(&hadc1);
	      HAL_ADC_Stop(&hadc1);

	      float degrees = (500.0*X)/2702.0 - 250;
	      LCD_data("X:");
	      Print(degrees);
	      LCD_data(" ");
	      LCD(223, 1);
	      LCD_data("/s");
	      HAL_Delay(500);
	      LCD_init();

	      degrees = (500.0*Y)/2702.0 - 250;
	      LCD_data("Y:");
	      Print(degrees);
	      LCD_data(" ");
	      LCD(223, 1);
	      LCD_data("/s");
	      HAL_Delay(500);
	      LCD_init();

          degrees = (500.0*Z)/2702.0 - 250;
          LCD_data("Z:");
          LCD_data(" ");
          Print(degrees);
          LCD(223, 1);
          LCD_data("/s");
          HAL_Delay(1000);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
/*  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
*/
  /** Configure Regular Channel
  */
/*  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
*/
  /** Configure Regular Channel
  */
/*  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }*/
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB6 PB7
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
