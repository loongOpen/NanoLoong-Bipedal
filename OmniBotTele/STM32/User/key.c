#include "stm32f10x.h"
#include "key.h"
#include "delay.h"
/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly_Remotor
 * ������������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/
		
//����IO��ʼ������
void keyInit(void) 
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD ; //�������� 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD ; //�������� 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

char key_sel[4];
char key_rc[2];
void KEY_Scan(float dt)
{	 
 	 key_sel[0]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8);
	 key_sel[1]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9);
	 key_sel[2]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);
	 key_sel[3]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7);
	
	 key_rc[0]=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5);
	 key_rc[1]=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
}


void LED_Init(void)
{
	GPIO_InitTypeDef gpio_Struct_tmp = {0};
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
	
	gpio_Struct_tmp.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	gpio_Struct_tmp.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_Struct_tmp.GPIO_Mode = GPIO_Mode_Out_PP;
	
	GPIO_Init(GPIOB, &gpio_Struct_tmp); 	
	for(int i=0;i<4;i++){
		LED_On(i);
		delay_ms(200);
		LED_Off(i);
		delay_ms(200);
	}	
}


void LED_Off(char sel)
{
    switch(sel)
    {
        case 0:
            GPIO_ResetBits(GPIOB, GPIO_Pin_12);
            break;
        case 1:
            GPIO_ResetBits(GPIOB, GPIO_Pin_13);
            break;
        case 2:
            GPIO_ResetBits(GPIOB, GPIO_Pin_14);
            break;
        case 3:
            GPIO_ResetBits(GPIOB, GPIO_Pin_15);
            break;
    }
}

void LED_On(char sel)
{
    switch(sel)
    {
        case 0:
            GPIO_SetBits(GPIOB, GPIO_Pin_12);
            break;
        case 1:
            GPIO_SetBits(GPIOB, GPIO_Pin_13);
            break;
        case 2:
            GPIO_SetBits(GPIOB, GPIO_Pin_14);
            break;
        case 3:
            GPIO_SetBits(GPIOB, GPIO_Pin_15);
            break;
    }
}






