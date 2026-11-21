/**************************************************************************
**************************************************************************/
#include "main.h"
#include "ShareWare.h"

/*
电机驱动1 Pin:GPIO_PIN_4 GPIOC
电机驱动2 Pin:GPIO_PIN_13 GPIOC
电机驱动3 Pin:GPIO_PIN_14 GPIOC
电机驱动4 Pin:GPIO_PIN_15 GPIOC
*/
void SystemClock_Config(void);       

__IO uint16_t Loop_5msTime=0;//5ms轮询时间
__IO uint16_t Loop_10msTime=0;//10ms轮询时间
__IO uint16_t Loop_500msTime=0;//500ms轮询时间
__IO uint16_t HeartbeatTime=0;//心跳包时间

int main(void)
{
	HAL_Init();//初始化HAL库
	SystemClock_Config();//系统时钟配置
	MX_TIM8_Init();//TIM8-PWM初始化

	MX_ADC1_Init();//ADC1初始化
	
	MX_USART2_Init();//串口2初始化函数
	MX_UART4_Init();//串口4初始化函数
	
	MX_TIM2_Init();//TIM2-编码器模式初始化
	MX_TIM3_Init();//TIM3-编码器模式初始化
	MX_TIM4_Init();//TIM4-编码器模式初始化
	MX_TIM5_Init();//TIM5-编码器模式初始化
	
	Buzzer_GPIO_Init();//蜂鸣器IO口初始化
	LED_Green_GPIO_Init();//绿色LED IO口初始化
	LED_Red_GPIO_Init();//红色LED IO口初始化
	
	
	HAL_Delay(600);
	MX_I2C1_Init();//I2C1初始化  
	
	BuzzerControl(2, 50);//蜂鸣器控制函数
	LED_Green_Control(2,50);//绿色LED灯控制函数
	LED_Red_Control(2,50);//红色LED灯控制函数
	
	while (1)
  {
	}
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}



/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
// _Error_Handler(__FILE__, __LINE__);
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
