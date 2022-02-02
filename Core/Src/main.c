/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "myiic.h"
#include "stdio.h"
#include "string.h"
#include "control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//extern   uint8_t  USART1_RX_Buff[20];
//extern   uint8_t  USART2_RX_Buff[20];
//extern   uint8_t  USART3_RX_Buff[20];
//extern   uint8_t  USART4_RX_Buff[20];
//extern   uint8_t  USART5_RX_Buff[20];

extern   uint8_t  Distance_M1_Buff[20];                //定义Rx接收数组
extern   uint8_t  Distance_M2_Buff[20];   
extern   uint8_t  Distance_Ultrasound_Buff[20];
extern   uint8_t  Order_Buff[20];
extern   uint8_t  Order2_Buff[20];
         uint8_t  SensorData[20]="0";
extern   UART_HandleTypeDef huart1;
uint8_t  Remote_Flag=0;



//extern   uint32_t  SetPWM,SetDAC;


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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile  uint16_t Period = 999;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
 // 发送测试字符串
  SensorData[0]='D';
	SensorData[1]='A';
	SensorData[2]='T';
	SensorData[3]='A';
	SensorData[18]=0x0D;
	SensorData[19]=0x0A;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
	uint32_t num=0;
	uint32_t num_old=0;
//	float PWM_out;
	extern uint8_t flag;
  /* USER CODE END SysInit */
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  IIC_Init();
	//IIC初始化
  CLOSE_RELAY1;CLOSE_RELAY2; CLOSE_RELAY3; CLOSE_RELAY4; CLOSE_RELAY5; 
  CLOSE_RELAY6;CLOSE_RELAY7;CLOSE_RELAY8;
	 HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
  /* USER CODE END 2 */

	HAL_Delay(500);
//	Set_OutVol_Fast(0,0.001);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_Delay(10);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,750);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,750);
	HAL_Delay(500);
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	Set_OutVol_Fast(0,5);
  while (1)
  {
			if(flag==0)
			{
					num_old= (uint32_t)(__HAL_TIM_GET_COUNTER(&htim2));//获取定时器的值
			}				
		  if(flag==1)
			{
					num= (uint32_t)(__HAL_TIM_GET_COUNTER(&htim2));//获取定时器的值
				  num=num-num_old;
				  SensorData[14]=num & 0xff000000;
				  SensorData[15]=num & 0x00ff0000;				  
				  SensorData[16]=num & 0x0000ff00;
				  SensorData[17]=num & 0x000000ff;
			}		
			 SensorData[4]=Distance_Ultrasound_Buff[1];
			 SensorData[5]=Distance_Ultrasound_Buff[2];
		
			 SensorData[6]=Distance_M1_Buff[9];
			 SensorData[7]=Distance_M1_Buff[10];
			 SensorData[8]=Distance_M1_Buff[11];
			 SensorData[9]=Distance_M1_Buff[12];
		
			 SensorData[10]=Distance_M2_Buff[9];
			 SensorData[11]=Distance_M2_Buff[10];
			 SensorData[12]=Distance_M2_Buff[11];
			 SensorData[13]=Distance_M2_Buff[12];
		 			 
			 HAL_UART_Transmit_DMA(&huart1,SensorData,20);
			
			 if(Remote_Flag==1)
				 { 
						Order_Juge(Order2_Buff);}
			  else
					{ 
					  Order_Juge(Order_Buff); }
			
			 HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);

			 HAL_Delay(200);

		
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
