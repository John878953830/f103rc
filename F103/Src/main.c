/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
IO io_array[8];
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
/* USER CODE BEGIN PFP */
void init(void)
{
	uint8_t i=0,j=0;
	//状态全部拉高
	for(i=0;i<8;i++)
	{
		for(j=0;j<8;j++)
		{
			io_array[i].status[j]=1;
		}
	}
	//初始化各个数组元素的值
	//1,2,3LED +
	io_array[0].type=1;            
	io_array[0].csg=GPIOB;
	io_array[0].num=GPIO_PIN_10;
	
	//1,2,3 LED -
	io_array[1].type=1;
	io_array[1].csg=GPIOB;
	io_array[1].num=GPIO_PIN_11;
	
	//4, 5, 6 LED +
	io_array[2].type=1;
	io_array[2].csg=GPIOB;
	io_array[2].num=GPIO_PIN_12;
	
	//4 5 6 LED -
	io_array[3].type=1;
	io_array[3].csg=GPIOB;
	io_array[3].num=GPIO_PIN_13;
	
	// 7 8 LED +
	io_array[4].type=1;
	io_array[4].csg=GPIOB;
	io_array[4].num=GPIO_PIN_14;
	
	//7 8 LED -
	io_array[5].type=1;
	io_array[5].csg=GPIOB;
	io_array[5].num=GPIO_PIN_15;
	
	//钮子开关输入
	io_array[6].type=0;
	io_array[6].csg=GPIOB;
	io_array[6].num=GPIO_PIN_8;
	
	//金属开关
	io_array[7].type=0;
	io_array[7].csg=GPIOB;
	io_array[7].num=GPIO_PIN_9;
	
	//初始化IO量，所有灯置于熄灭状态
	//GPIOC置于输出模式
	GPIOC_OUTPUT();
	__NOP();
	__NOP();
	__NOP();
	GPIOC->ODR|=0x00FF;
	__NOP();
	__NOP();
	__NOP();
	//573 锁存
	for(i=0;i<6;i++)
	{
		HAL_GPIO_WritePin(io_array[i].csg,io_array[i].num,GPIO_PIN_SET);
		__NOP();
		__NOP();
		HAL_GPIO_WritePin(io_array[i].csg,io_array[i].num,GPIO_PIN_RESET);
	}
	
	//获取开关的初始状态，记录进数组
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);   //钮子开关245传输禁止
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);   //金属开关245传输禁止
	GPIOC_INPUT();
	//取得钮子开关状态
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
	__NOP();
	for(i=0;i<8;i++)
	{
		io_array[6].status[i]=(uint8_t)HAL_GPIO_ReadPin(io_array[6].csg,1<<i);
	}
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET); 
	
	//取得金属开关的初始状态
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
	__NOP();
	for(i=0;i<8;i++)
	{
		io_array[7].status[i]=(uint8_t)HAL_GPIO_ReadPin(io_array[7].csg,1<<i);
	}
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET); 	
	;
	return;
}
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		;
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
