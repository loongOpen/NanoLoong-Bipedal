#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//Mini STM32������
//SPI ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2010/6/13 
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  

 
 				  	    													  
void SPI1_Init(void);			 //��ʼ��SPI��
void SPI1_SetSpeed(u8 SpeedSet); //����SPI�ٶ�   
u8 Spi_RW(u8 TxData);//SPI���߶�дһ���ֽ�
void SPI_CS(u8 sel,u8 set);		 
		 
#define CS_MPU9250 1
#define CS_SD 2
#define CS_NRF  3
#define CS_MAG 4

#define SPI_CE_H()   GPIO_SetBits(GPIOA, GPIO_Pin_3) 
#define SPI_CE_L()   GPIO_ResetBits(GPIOA, GPIO_Pin_3)

#define SPI_CSN_H()  SPI_CS(CS_NRF,1)
#define SPI_CSN_L()  SPI_CS(CS_NRF,0)



uint8_t SPI_Write_Reg(u8 sel,uint8_t reg,uint8_t value);
uint8_t SPI_Read_Reg(u8 sel,uint8_t reg);
uint8_t SPI_Write_Buf(u8 sel,uint8_t reg, uint8_t *pBuf, uint8_t uchars);
uint8_t SPI_Read_Buf(u8 sel,uint8_t reg, uint8_t *pBuf, uint8_t uchars);
void SPI_BufferRead(u8 sel,u8*buf, u8 add, u8 len);
void SPI_Transmit(uint8_t *pData, uint16_t Size);
void SPI_Receive(uint8_t *pData, uint16_t Size);



#endif

