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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void Start_Frame_Pulses(void);
void Get_Coded_Data(void);
void Decode_Data(uint32_t data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t data;
char result;
/*Function Definitions And Implementation*/
void Delay_Microseconds_DWT(uint32_t delay){
	uint32_t cycles = (HAL_RCC_GetHCLKFreq()/1000000)*delay;
	uint32_t start = DWT->CYCCNT;
	while(DWT->CYCCNT - start < cycles);
}
void Delay_Milliseconds_DWT(uint32_t delay){
	uint32_t cycles = (HAL_RCC_GetHCLKFreq()/1000)*delay;
	uint32_t start = DWT->CYCCNT;
	while(DWT->CYCCNT - start < cycles);
}
void Delay_Seconds_DWT(uint32_t delay){
	uint32_t cycles = (HAL_RCC_GetHCLKFreq()/1)*delay;
	uint32_t start = DWT->CYCCNT;
	while(DWT->CYCCNT - start < cycles);
}

/*Data Watch Point Enabling Function*/
void Enable_DWT(void){
    DWT->CTRL |= 1;
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
  Enable_DWT();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Get_Coded_Data();
	  Decode_Data(data);
	  HAL_Delay(1000);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Start_Frame_Pulses(void){
  while (!(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_10)));
  while (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_10));
}

void Get_Coded_Data(void){
	while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_10)));
	while (!HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_10));
	while (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_10));
	for (int i=0; i<32; i++)
	{
	  int count=0;
	  while (!(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_10))); // wait for pin to go high.. this is 562.5us LOW

	  while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_10)))  // count the space length while the pin is high
	 {
	  count++;
	  Delay_Microseconds_DWT(100);
	 }

	 if (count > 12) // if the space is more than 1.2 ms
	 {
	  data |= (1UL << (31-i));   // write 1
	 }

	 else data &= ~(1UL << (31-i));  // write 0
	}
}

void Decode_Data(uint32_t data){
	switch (data)
	{
		case (0xFFA25D):
			result = '1';
			break;

		case (0xFF629D):
			result = '2';
			break;

		case (0xFFE21D):
		result = '3';
			break;

		case (0xFF22DD):
		    result = '4';
			break;

		case (0xFF02FD):
			result = '5';
			break;

		case (0xFFC23D):
			result = '6';
			break;

		case (0xFFE01F):
			result = '7';
			break;

		case (0xFFA857):
		result = '8';
			break;

		case (0xFF906F):
		result = '9';
			break;

		case (0xFFB04F):
		result = '#';
			break;

		case (0XFF6897):
		result = '*';
			break;

		case (0xFF9867):
		result = '0';
			break;

		case (0xFF38C7):
		result = 'K';
			break;

		case (0xFF18E7):
		    result = '^';
			break;

		case (0xFF10EF):
		    result = '<';
			break;

		case (0xFF5AA5):
		    result = '>';
			break;

		case (0xFF4AB5):
		    result = 'u';
			break;

		default :
			break;
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
