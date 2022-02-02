#ifndef __MYIIC_H
#define __MYIIC_H
#include "stm32f1xx_hal.h"
 
#define  SDA_IN()  {GPIOA->CRH&=0XFFF0FFFF;GPIOA->CRH|=(uint32_t)8<<16;}
#define  SDA_OUT() {GPIOA->CRH&=0XFFF0FFFF;GPIOA->CRH|=(uint32_t)3<<16;}


#define   Vref  5.02
#define   Base  1

//IO��������	 


#define IIC_SCL0  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET)	
#define IIC_SDA0  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET)	
#define IIC_SCL1	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET)	
#define IIC_SDA1	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET)	
#define READ_SDA  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12)==1  //����SDA 



//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(uint8_t txd);			//IIC����һ���ֽ�
uint8_t IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
uint8_t IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�
void MCP4725_WriteThreeByte(uint8_t Data1,uint8_t Data2,uint8_t Data3);
void Set_OutVol_Fast(uint8_t add,float Vol);


void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	  
#endif
















