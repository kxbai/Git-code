#include "myiic.h"

//初始化IIC
void IIC_Init(void)
{		
  GPIO_InitTypeDef GPIO_InitStruct = {0};	
	__HAL_RCC_GPIOA_CLK_ENABLE();	//使能GPIOA时钟  
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12; //PB14--SCL  PB15--SDA
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);	//SDA
}
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA1;	  	  
	IIC_SCL1;
	HAL_Delay(4);
 	IIC_SDA0;//START:when CLK is high,DATA change form high to low 
	HAL_Delay(4);
	IIC_SCL0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL0;
	IIC_SDA0;//STOP:when CLK is high DATA change form low to high
 	HAL_Delay(4);
	IIC_SCL1; 
	IIC_SDA1;//发送I2C总线结束信号
	HAL_Delay(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA1;
	HAL_Delay(1);	   
	IIC_SCL1;
	HAL_Delay(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL0;
	SDA_OUT();
	IIC_SDA0;
	HAL_Delay(2);
	IIC_SCL1;
	HAL_Delay(2);
	IIC_SCL0;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL0;
	SDA_OUT();
	IIC_SDA1;
	HAL_Delay(2);
	IIC_SCL1;
	HAL_Delay(2);
	IIC_SCL0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	  SDA_OUT(); 	    
    IIC_SCL0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        //IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
		 IIC_SDA1;
		else
		 IIC_SDA0;
		txd<<=1; 	  
		HAL_Delay(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL1;
		HAL_Delay(2); 
		IIC_SCL0;	
		HAL_Delay(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL0; 
        HAL_Delay(2);
		    IIC_SCL1;
        receive<<=1;
        if(READ_SDA)receive++;   
		HAL_Delay(1); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}

void MCP4725_WriteThreeByte(uint8_t Data1,uint8_t Data2,uint8_t Data3)
{				   	  	    																 
    IIC_Start();  

		IIC_Send_Byte(Data1);	       
		IIC_Wait_Ack();
	
	  IIC_Send_Byte(Data2);	  
	  IIC_Wait_Ack();
	
		IIC_Send_Byte(Data3);    
    IIC_Wait_Ack(); 	
	
    IIC_Stop();                    
	  HAL_Delay(4);  
}

void Set_OutVol_Fast(uint8_t add,float Vol)
{
		uint8_t out_h,out_l;
	  uint16_t out;
	
//		Vol = Vol / Base;
		out = (Vol * 4096 / Vref);//out 为设置DA数值
	
	  out_h = 0x00 | ( (out & 0X0F00) >>8 );
	  out_l = out & 0X00FF;
	
	  if(add==0)
		{
				MCP4725_WriteThreeByte(0xc0,out_h,out_l);
		}
	  if(add==1)
		{
				MCP4725_WriteThreeByte(0xc2,out_h,out_l);
		}
}



























