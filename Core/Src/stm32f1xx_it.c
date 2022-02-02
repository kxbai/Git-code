/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "string.h"
#include "usart.h"
#include "tim.h"
#include "dma.h"
#include "control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
extern   uint8_t  USART1_RX_Buff[20];
extern   uint8_t  USART2_RX_Buff[20];
extern   uint8_t  USART3_RX_Buff[21];
extern   uint8_t  USART4_RX_Buff[20];
extern   uint8_t  USART5_RX_Buff[20];

extern   uint8_t  Distance_M1_Buff[20];                //定义Rx接收数组
extern   uint8_t  Distance_M2_Buff[20];   
extern   uint8_t  Distance_Ultrasound_Buff[20];
extern   uint8_t  Order_Buff[20];
extern   uint8_t  Order2_Buff[20];

extern   uint8_t   Remote_Flag;

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool flag=0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */

  /* USER CODE END DMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
  flag=!flag;
  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	if(RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))   //判断是否是空闲中断
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);                       //清除空闲中断标志
		HAL_UART_DMAStop(&huart1);                                //停止本次DMA传输   
		uint8_t data_length1  = 20 - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);   //计算接收到的数据长度  20-返回当前 DMA 通道传输中剩余数据单元的数量
		if(USART1_RX_Buff[0]=='S'&&USART1_RX_Buff[1]=='H'&&USART1_RX_Buff[2]=='L'&&USART1_RX_Buff[3]=='G'&&
			USART1_RX_Buff[18]=='L'&&USART1_RX_Buff[19]=='G')
		{
		  for(uint8_t i=0;i<20;i++) Order_Buff[i]=USART1_RX_Buff[i];
		}
//		HAL_UART_Transmit_DMA(&huart1,USART1_RX_Buff,data_length);           //测试函数：将接收到的数据打印出去    
//		HAL_UART_Transmit_DMA(&huart1,Order_Buff,20);
		memset(USART1_RX_Buff,0,data_length1);                                  //清空RX缓冲区                                          
		data_length1 = 0;
		HAL_UART_Receive_DMA(&huart1, (uint8_t*)USART1_RX_Buff,20);          //重启开始DMA传输，中断结束
		
	}
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	if(RESET != __HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))   //判断是否是空闲中断
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);                       //清除空闲中断标志
		HAL_UART_DMAStop(&huart3);                                //停止本次DMA传输   
		uint8_t data_length3  = 20 - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);   //计算接收到的数据长度
//		HAL_UART_Transmit(&huart1,USART2_RX_Buff,data_length,0x200);           //测试函数：将接收到的数据打印出去 
		for(uint8_t i=0;i< data_length3;i++) Distance_Ultrasound_Buff[i]=USART3_RX_Buff[i];
		//这边写控制数组填充
		memset(USART3_RX_Buff,0,data_length3);                                  //清空RX缓冲区                                          
		data_length3 = 0;
		HAL_UART_Receive_DMA(&huart3, (uint8_t*)USART3_RX_Buff, 20);          //重启开始DMA传输，中断结束
	}
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	if(RESET != __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))   //判断是否是空闲中断
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);                       //清除空闲中断标志
		HAL_UART_DMAStop(&huart2);                                //停止本次DMA传输   
		uint8_t data_length2  = 20 - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);   //计算接收到的数据长度
    if(USART2_RX_Buff[0]=='E'&&USART2_RX_Buff[2]=='S'&&USART2_RX_Buff[3]=='T')
		{
			Order2_Buff[12]=USART2_RX_Buff[10];//RELAY
			Order2_Buff[13]=USART2_RX_Buff[11];
			Order2_Buff[14]=USART2_RX_Buff[6];
			Order2_Buff[17]=USART2_RX_Buff[5];
			
			Order2_Buff[4] =USART2_RX_Buff[13];//PWM
			Order2_Buff[5] =USART2_RX_Buff[14];
			Order2_Buff[6] ='0';
			Order2_Buff[7] ='0';
			
			Order2_Buff[8] =USART2_RX_Buff[16];//DAC
			Order2_Buff[9] =USART2_RX_Buff[17];
			Order2_Buff[10]=USART2_RX_Buff[18];
			Order2_Buff[11]='0';
		}
     if(Order2_Buff[17]=='1') { Remote_Flag=1; }
			else { Remote_Flag=0; }
//		for(uint8_t i=0;i<data_length3;i++) Order_Buff[i]=USART3_RX_Buff[i];
//  		HAL_UART_Transmit_DMA(&huart1,USART2_RX_Buff,20);
//		  HAL_UART_Transmit_DMA(&huart2,Order_Buff,20);
//		HAL_UART_Transmit(&huart1,USART3_RX_Buff,data_length,0x200);           //测试函数：将接收到的数据打印出去    
		//这边写控制数组填充
		memset(USART2_RX_Buff,0,data_length2);                                  //清空RX缓冲区                                          
		data_length2 = 0;
		HAL_UART_Receive_DMA(&huart2, (uint8_t*)USART2_RX_Buff, 20);          //重启开始DMA传输，中断结束
	}
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
	if(RESET != __HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE))   //判断是否是空闲中断
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart4);                       //清除空闲中断标志
		HAL_UART_DMAStop(&huart4);                                //停止本次DMA传输   
		uint8_t data_length4  = 20 - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);   //计算接收到的数据长度
//		HAL_UART_Transmit(&huart1,USART4_RX_Buff,data_length,0x200);           //测试函数：将接收到的数据打印出去    
//		//这边写控制数组填充
//这里将 9 10 11 12填充到数组中进行存储
		for(uint8_t i=0;i< data_length4;i++) Distance_M1_Buff[i]=USART4_RX_Buff[i];
		memset(USART4_RX_Buff,0,data_length4);                                  //清空RX缓冲区                                          
		data_length4 = 0;
		HAL_UART_Receive_DMA(&huart4, (uint8_t*)USART4_RX_Buff, 20);          //重启开始DMA传输，中断结束
	}
  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */
	if(RESET != __HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE))   //判断是否是空闲中断
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart5);
		HAL_UART_Receive_IT(&huart5, (uint8_t*)USART5_RX_Buff, 20);
		for(uint8_t i=0;i< 20;i++) Distance_M2_Buff[i]=USART5_RX_Buff[i];
//    HAL_UART_Transmit_DMA(&huart1,USART5_RX_Buff,20);
//	  memset(USART5_RX_Buff,0,20);                                  //清空RX缓冲区    
		
	}
  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */

  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 channel3 global interrupt.
  */
void DMA2_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel3_IRQn 0 */

  /* USER CODE END DMA2_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA2_Channel3_IRQn 1 */

  /* USER CODE END DMA2_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA2 channel4 and channel5 global interrupts.
  */
void DMA2_Channel4_5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel4_5_IRQn 0 */

  /* USER CODE END DMA2_Channel4_5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
  /* USER CODE BEGIN DMA2_Channel4_5_IRQn 1 */

  /* USER CODE END DMA2_Channel4_5_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
