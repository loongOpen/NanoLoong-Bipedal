#include "head.h"
#include "mpu9250.h"
#include "imu.h"
#include "adc.h"
#include "mymath.h"
#include "rc_mine.h"
#include "time.h"
//��ʱ�����ò����� ʹ�ö�ʱ��3
void Time4ON(void)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		//������ʱ������ʱ��
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		//���ö�ʱ������
		TIM_DeInit(TIM4); 
		TIM_TimeBaseStructure.TIM_Period = 2000; 								 	//1ms��ʱ			 
		TIM_TimeBaseStructure.TIM_Prescaler = (72000000/1000000 - 1);              
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);	
		//�ж�����
		NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�2 �����ȼ����ж� 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  	 //��Ӧ���ȼ�0 �߼������Ӧ�ж�
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
		NVIC_Init(&NVIC_InitStructure);	  
		//���ж�
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);					  
		TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); 
		//������ʱ��			 
		TIM_Cmd(TIM4, ENABLE); 
}

//��ʱ���жϴ��� ��stm32f10x_it.c���

u8 test_itoa[3];
u8 flag_ms[100]={0};
u8 test=20;
void Time4_IntHandle(void)
{ static u16 cnt_2ms=0,cnt_5ms=0,cnt_10ms=0,cnt_1s,cnt_50ms,cnt_20ms,cnt_25ms,cnt_5s;
	u8 i;
	static u16 pic_cnt=0;u8 time_now_char[15]={0};
		//���жϱ�ʶ
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);
		//---------------- �жϴ���  ---------------------
		{
			  flag_ms[1]=1;
			
			if(cnt_2ms++>2/2-1){
				cnt_2ms=0;
				flag_ms[2]=1;
			}
			if(cnt_5ms++>5/2-1)
			{ cnt_5ms=0;
				flag_ms[5]=1;
			}//--end of 2ms task
			
			if(cnt_10ms++>10/2-1)
			{cnt_10ms=0;
       flag_ms[10]=1;
		
			}//--end of 10 task
			
			if(cnt_20ms++>20/2-1)
			{cnt_20ms=0;
       flag_ms[20]=1;
		
			}//--end of 10 task
			
			if(cnt_25ms++>25/2-1)
			{cnt_25ms=0;
       flag_ms[25]=1;
		
			}//--end of 10 task
			
			if(cnt_50ms++>50/2-1)
			{cnt_50ms=0;
       flag_ms[50]=1;
		
			}//--end of 10 task
			
			if(cnt_1s++>1000/2-1)
			{cnt_1s=0;
 
			}
			
				
			if(cnt_5s++>5000/2-1)
			{cnt_5s=0;
			 //OLED_Init();
			}
			
			
		}
}



