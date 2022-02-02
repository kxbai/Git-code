#include "myiic.h"

//��ʼ��IIC
void IIC_Init(void)
{		
  GPIO_InitTypeDef GPIO_InitStruct = {0};	
	__HAL_RCC_GPIOA_CLK_ENABLE();	//ʹ��GPIOAʱ��  
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12; //PB14--SCL  PB15--SDA
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);	//SDA
}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA1;	  	  
	IIC_SCL1;
	HAL_Delay(4);
 	IIC_SDA0;//START:when CLK is high,DATA change form high to low 
	HAL_Delay(4);
	IIC_SCL0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL0;
	IIC_SDA0;//STOP:when CLK is high DATA change form low to high
 	HAL_Delay(4);
	IIC_SCL1; 
	IIC_SDA1;//����I2C���߽����ź�
	HAL_Delay(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
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
	IIC_SCL0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
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
//������ACKӦ��		    
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
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	  SDA_OUT(); 	    
    IIC_SCL0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        //IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
		 IIC_SDA1;
		else
		 IIC_SDA0;
		txd<<=1; 	  
		HAL_Delay(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL1;
		HAL_Delay(2); 
		IIC_SCL0;	
		HAL_Delay(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
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
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
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
		out = (Vol * 4096 / Vref);//out Ϊ����DA��ֵ
	
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



























