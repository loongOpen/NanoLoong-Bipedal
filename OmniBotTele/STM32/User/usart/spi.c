#include "spi.h"
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
 

//������SPIģ��ĳ�ʼ�����룬���ó�����ģʽ������SD Card/W25X16/24L01/JF24C							  
//SPI�ڳ�ʼ��
//�������Ƕ�SPI1�ĳ�ʼ��

SPI_InitTypeDef  SPI_InitStructure;

void SPI1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_SPI1, ENABLE );	
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

 	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_7; //ce
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//��ͨ���ģʽ
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_3; //ce
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//��ͨ���ģʽ
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	SPI_CS(CS_SD,1);
	SPI_CS(CS_NRF,1);
	SPI_CS(CS_MPU9250,1);
	
	//����ֻ���SPI�ڳ�ʼ��
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//��λSPI1
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//ֹͣ��λSPI1 
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //????? 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; //??? 
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; //????8? 
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //????,????? 
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //?1?????,???????? 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; //NSS??????? 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //8??,9MHz 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; //???? 
	SPI_InitStructure.SPI_CRCPolynomial = 7; 
	SPI_Init(SPI1, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI1, ENABLE); //ʹ��SPI����
	 
}   
//SPI �ٶ����ú���
//SpeedSet:
//SPI_BaudRatePrescaler_2   2��Ƶ   (SPI 36M@sys 72M)
//SPI_BaudRatePrescaler_8   8��Ƶ   (SPI 9M@sys 72M)
//SPI_BaudRatePrescaler_16  16��Ƶ  (SPI 4.5M@sys 72M)
//SPI_BaudRatePrescaler_256 256��Ƶ (SPI 281.25K@sys 72M)
  
void SPI1_SetSpeed(u8 SpeedSet)
{
	SPI_InitStructure.SPI_BaudRatePrescaler = SpeedSet ;
  	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1,ENABLE);
} 

//SPIx ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 Spi_RW1(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
		{
		retry++;
		if(retry>200)return 0;
		}			  
	SPI_I2S_SendData(SPI1, TxData); //ͨ������SPIx����һ������
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)//���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI1); //����ͨ��SPIx������յ�����					    
}


u8 Spi_RW(u8 dat) 
{ 
	/* ? SPI?????????? */ 
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); 
	/* ?? SPI2??????? */ 
	SPI_I2S_SendData(SPI1, dat); 
	/* ?SPI?????????? */ 
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 
	/* Return the byte read from the SPI bus */ 
	return SPI_I2S_ReceiveData(SPI1); 
}


void SPI_CS(u8 sel,u8 set)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_10|GPIO_Pin_7);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	delay_us(10);
switch(sel)
{
	case 1:
  if(set)	
	GPIO_SetBits(GPIOB, GPIO_Pin_10);
	else
	GPIO_ResetBits(GPIOB, GPIO_Pin_10);
	delay_us(10);
	break;
	case 2:
	if(set)	
	GPIO_SetBits(GPIOB, GPIO_Pin_7);
	else
	GPIO_ResetBits(GPIOB, GPIO_Pin_7);
	delay_us(10);
	break;
	case 3:
	if(set)	
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	else
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	delay_us(10);
	break;
}
}	


//====value:д���ֵ
uint8_t SPI_Write_Reg(u8 sel,uint8_t reg,uint8_t value)
{
	uint8_t status;
	SPI_CS(sel,0);
	if (sel==CS_MAG)
		reg=0x80|reg;
	
	status=Spi_RW(reg); //����д����+�Ĵ�����
	Spi_RW(value);//д��Ĵ���ֵ
	SPI_CS(sel,1);
	return(status);//����״ֵ̬
}

//====SPI��ȡ�Ĵ���
//====reg:ָ���ļĴ�����ַ
uint8_t SPI_Read_Reg(u8 sel,uint8_t reg)
{
	uint8_t reg_val;
	SPI_CS(sel,0);
	Spi_RW(reg|0x80); //====���Ͷ�����+�Ĵ�����
	reg_val=Spi_RW(0xff);//====��ȡ�Ĵ���ֵ
	SPI_CS(sel,1);
	return(reg_val);
}


uint8_t SPI_Write_Buf(u8 sel,uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	SPI_CS(sel,0);
	status = Spi_RW(reg);	/* ?????? */
	for(i=0; i<uchars; i++)
	{
		Spi_RW(pBuf[i]);		/* ??? */
	}
	SPI_CS(sel,1);
  return 	status;	
}


uint8_t SPI_Read_Buf(u8 sel,uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	SPI_CS(sel,0);
	status = Spi_RW(reg);	/* ?????? */
	for(i=0; i<uchars; i++)
	{
		pBuf[i] = Spi_RW(0); /* ?????? */ 	
	}
	SPI_CS(sel,1);
  return 	status;
}

void SPI_BufferRead(u8 sel,u8*buf, u8 add, u8 len)
{
	u8 i=0;
 SPI_CS(sel,0);
	if(sel!=CS_MAG)
	Spi_RW(add|0x80);	
	else
	Spi_RW(add|0xC0);//���������ӵ�ַ
	for(i=0;i<len;i++)
	{
	if(sel!=CS_MAG)
	 *buf++ = Spi_RW(0xff); 
	else
	 *buf++ = Spi_RW(0xff); 
	} 
 SPI_CS(sel,1);
}

void SPI_Transmit(uint8_t *pData, uint16_t Size)
{uint16_t i;
    for( i=0; i<Size; i++)
    {
        Spi_RW(pData[i]);
    }
}

void SPI_Receive(uint8_t *pData, uint16_t Size)
{uint16_t i;
    for( i=0; i<Size; i++)
    {
        pData[i] = Spi_RW(0);
    }
}