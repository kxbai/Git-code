#ifndef __MYIIC_H
#define __MYIIC_H
#include "stm32f1xx_hal.h"
 
#define  SDA_IN()  {GPIOA->CRH&=0XFFF0FFFF;GPIOA->CRH|=(uint32_t)8<<16;}
#define  SDA_OUT() {GPIOA->CRH&=0XFFF0FFFF;GPIOA->CRH|=(uint32_t)3<<16;}


#define   Vref  5.02
#define   Base  1

//IO操作函数	 


#define IIC_SCL0  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET)	
#define IIC_SDA0  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET)	
#define IIC_SCL1	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET)	
#define IIC_SDA1	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET)	
#define READ_SDA  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12)==1  //输入SDA 



//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号
void MCP4725_WriteThreeByte(uint8_t Data1,uint8_t Data2,uint8_t Data3);
void Set_OutVol_Fast(uint8_t add,float Vol);


void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	  
#endif
















