/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

#define	 CLOSE_RELAY1     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET)
#define	 CLOSE_RELAY2     HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET)
#define  CLOSE_RELAY3 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_SET)
#define  CLOSE_RELAY4  	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_SET)
#define  CLOSE_RELAY5  	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_SET)
#define  CLOSE_RELAY6    	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET)
#define  CLOSE_RELAY7  	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_SET)
#define  CLOSE_RELAY8   	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_SET)

#define	 OPEN_RELAY1      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET)
#define	 OPEN_RELAY2      HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_RESET)
#define  OPEN_RELAY3 	    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_RESET)
#define  OPEN_RELAY4    	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_RESET)
#define  OPEN_RELAY5    	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET)
#define  OPEN_RELAY6    	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET)
#define  OPEN_RELAY7    	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET)
#define  OPEN_RELAY8    	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_RESET)

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
