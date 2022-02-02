#include "control.h"
#include "gpio.h"
#include "usart.h"
#include "stdlib.h"
#include "myiic.h"
#include "tim.h"


extern   uint8_t  Distance_M1_Buff[20];                //定义Rx接收数组
extern   uint8_t  Distance_M2_Buff[20];   
extern   uint8_t  Distance_Ultrasound_Buff[20];
extern   uint8_t  Order_Buff[20];
extern   uint8_t  Order2_Buff[20];
extern   UART_HandleTypeDef huart1;

void Order_Juge(uint8_t Juge_Buff[20])
{
		uint32_t  SetPWM=0,SetDAC=0;
		uint32_t  SetPWM_OLD=0;
		uint32_t  DAC_out_OLD=0;
	
		float DAC_out=0;

		SetPWM=(Juge_Buff[4]-'0')*1000+(Juge_Buff[5]-'0')*100+(Juge_Buff[6]-'0')*10+(Juge_Buff[7]-'0');
	  SetDAC=(Juge_Buff[8]-'0')*1000+(Juge_Buff[9]-'0')*100+(Juge_Buff[10]-'0')*10+(Juge_Buff[11]-'0');
	
		SetDAC=SetDAC*2;
    DAC_out=SetDAC*0.001;
	
	if(DAC_out_OLD !=DAC_out && Juge_Buff[17]=='1')
	{ 
  	  
			if(DAC_out==0)
			{
				CLOSE_RELAY4;
				CLOSE_RELAY5;
				HAL_Delay(100);
				Set_OutVol_Fast(0,0.001);
			}
			if( 0<DAC_out &&  DAC_out<=5)
			{
				OPEN_RELAY4;
				CLOSE_RELAY5;
				HAL_Delay(100);
				Set_OutVol_Fast(0,DAC_out);
			}
			if( 5<DAC_out && DAC_out<=10)
			{
				DAC_out=DAC_out-5;
				OPEN_RELAY5;
				CLOSE_RELAY4;
				HAL_Delay(100);
				Set_OutVol_Fast(0,DAC_out);
			}//设置DAC电压，Set_OutVol_Fast(地址,电压值)，地址为0---> OUT2  1--->OUT3
			DAC_out_OLD = DAC_out;
	}
	
		if(SetPWM_OLD != SetPWM && Juge_Buff[17]=='1')
		{ 
			 SetPWM_OLD = SetPWM;
      if(SetPWM>=3900)  SetPWM=3900;
      if(SetPWM<=1250)  SetPWM=1250;  
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,(1000-(SetPWM/5)));
		}
		if( Juge_Buff[17]=='1')
	{
		if (Juge_Buff[12]=='1') OPEN_RELAY1;
	   else  CLOSE_RELAY1;
	  if (Juge_Buff[13]=='1') OPEN_RELAY2;
	   else  CLOSE_RELAY2;
  	if (Juge_Buff[14]=='1') OPEN_RELAY3;
	   else  CLOSE_RELAY3;
		if (Juge_Buff[15]=='1') OPEN_RELAY6;
			else  CLOSE_RELAY6;
		if (Juge_Buff[16]=='1') OPEN_RELAY8;
			else  CLOSE_RELAY8;
		if (Juge_Buff[17]=='1') OPEN_RELAY7;
			else  CLOSE_RELAY7;
	}
	if ( Juge_Buff[17]=='0') 
	{ 
		 for(uint8_t i=12;i<20;i++){ Juge_Buff[i]='0';}
		 CLOSE_RELAY1;CLOSE_RELAY2; CLOSE_RELAY3; CLOSE_RELAY4; 
		 CLOSE_RELAY5; CLOSE_RELAY6;CLOSE_RELAY7;CLOSE_RELAY8;
	}

}
	


