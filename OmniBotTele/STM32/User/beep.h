#ifndef __BEEP_H
#define __BEEP_H	 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//��������������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
//�������˿ڶ���
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
void Beep_Init(u32 arr,u32 psc);
void Tone(u8 level, u8 tone);
void Play_Music_Direct(u8 sel);
u8 Play_Music_In_Task(u8 *music, u16 st,u16 ed,u8 en,float dt);
void Play_Music_Task(u8 sel,float dt);
#define MEMS_RIGHT_BEEP 0
#define MEMS_ERROR_BEEP 1
#define START_BEEP 2
#define BAT_ERO_BEEP 3
#define RC_ERO_BEEP 4
#define RC_RESET_BEEP 5
#define UPLOAD_BEEP 6
#endif

