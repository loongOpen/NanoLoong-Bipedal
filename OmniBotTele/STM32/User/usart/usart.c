#include "usart.h"
#include "stm32f10x_it.h"
#include "stm32f10x_dma.h"
#include "des.h"	
#include "head.h"	
#include "rtc.h"
#include "math.h"
#include "adc.h"
#include "key.h"
/******************************************************************************/
uint8_t SendBuff[SENDBUFF_SIZE]={' ','D','M','A','1',' '};//���顾0���޷�ʹ��
u8 uart_test[4]={0,0,0,0}; 
u8 dma_can_tx=1;
u8 Addr_485=0;
void USART_ON(u8 uart_num,u32 baud,u8 m_irq,u8 s_irq)
{
//GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
     //USART1_TX   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);  

   //Usart1 NVIC ����

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = baud;//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

    USART_Init(USART1, &USART_InitStructure); //��ʼ������
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�����ж�
    USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ��� 

}

void USART1_DMA_Config(void)
{
		DMA_InitTypeDef DMA_InitStructure;
	
		/*����DMAʱ��*/
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
		//NVIC_Config();	   			//����DMA�ж�

		/*����DMAԴ���������ݼĴ�����ַ*/
		DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;	   

		/*�ڴ��ַ(Ҫ����ı�����ָ��)-----------------------��������*/
		DMA_InitStructure.DMA_MemoryBaseAddr = (u32)SendBuff;

		/*���򣺴��ڴ浽����*/		
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	

		/*�����СDMA_BufferSize=SENDBUFF_SIZE*/	
		DMA_InitStructure.DMA_BufferSize = SENDBUFF_SIZE;

		/*�����ַ����*/	    
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 

		/*�ڴ��ַ����*/
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	

		/*�������ݵ�λ*/	
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

		/*�ڴ����ݵ�λ 8bit*/
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 

		/*DMAģʽ������ѭ��*/
		//DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	 

		/*���ȼ�����*/	
		DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  

		/*��ֹ�ڴ浽�ڴ�Ĵ���	*/
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

		/*����DMA1��4ͨ��*/		   
		DMA_Init(DMA1_Channel4, &DMA_InitStructure); 	   
		
		/*ʹ��DMA*/
		DMA_Cmd (DMA1_Channel4,ENABLE);					
		//DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);  //����DMA������ɺ�����ж�
}


void USART2_DMA_Config(void)
{	NVIC_InitTypeDef NVIC_InitStructure; 
		DMA_InitTypeDef DMA_InitStructure;
	
		/*����DMAʱ��*/
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
		//NVIC_Config();	   			//����DMA�ж�

		/*����DMAԴ���������ݼĴ�����ַ*/
		DMA_InitStructure.DMA_PeripheralBaseAddr = USART2_DR_Base;	   

		/*�ڴ��ַ(Ҫ����ı�����ָ��)-----------------------��������*/
		DMA_InitStructure.DMA_MemoryBaseAddr = (u32)SendBuff;

		/*���򣺴��ڴ浽����*/		
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	

		/*�����СDMA_BufferSize=SENDBUFF_SIZE*/	
		DMA_InitStructure.DMA_BufferSize = SENDBUFF_SIZE;

		/*�����ַ����*/	    
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 

		/*�ڴ��ַ����*/
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	

		/*�������ݵ�λ*/	
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

		/*�ڴ����ݵ�λ 8bit*/
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 

		/*DMAģʽ������ѭ��*/
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;
		//DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	 

		/*���ȼ�����*/	
		DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  

		/*��ֹ�ڴ浽�ڴ�Ĵ���	*/
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

		/*����DMA1��4ͨ��*/		   
		DMA_Init(DMA1_Channel7, &DMA_InitStructure); 	   
		
		/*ʹ��DMA*/
		DMA_Cmd (DMA1_Channel7,ENABLE);					
		DMA_ITConfig(DMA1_Channel7,DMA_IT_TC,ENABLE);  //����DMA������ɺ�����ж�
		
			//DMA�����ж�����  
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;  
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
		NVIC_Init(&NVIC_InitStructure);  
}
void USART_init(void)
{
	USART_ON(1,256000,2,1);//485
}

ID CARD[MAX_SAVE];
u8 open_lock1=0;
void UsartSend(uint8_t ch)
{
	USART_SendData(USART1, (uint8_t) ch);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{}
}


void my_itoa(int num,u8* str)  
{  
    int sign = num;  
    int i = 0;  
    int j = 0;  
    char temp[100];  
    //����Ǹ�����ȥ������,��-1234ת��1234  
    if(sign < 0)  
    {  
        num = -num;  
    }  
    //ת���ַ�����1234ת��"4321"  
    do  
    {  
        temp[i] = num % 10 + '0';  
        num /= 10;  
        i++;  
    }while(num > 0);  
    //����Ǹ����Ļ����Ӹ�������ĩβ���磺"4321-"  
    if(sign < 0)  
    {  
        temp[i++] = '-';  
    }  
    temp[i] = '\0';  
    i--;  
    //��temp�������������뵽str������  
    //��"4321-" ====> "-1234"  
    while(i >= 0)  
    {  
        str[j] = temp[i];  
        j++;  
        i--;  
    }  
    //�ַ���������ʶ  
    str[j] = '\0';   
}  


#define MAX_NUM 125

char RX_BUF[MAX_NUM],RX_BUF_OUT[MAX_NUM];
	
void USART3_IRQ_TASK(void)//FLOW
{
	
}

void UART_TX_CHAR(u8 sel ,char data)
{
	if(sel==1)
	{
	USART_SendData(USART1, (u8) data);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		;
	}
	else 	if(sel==2)
	{
	USART_SendData(USART2, (u8) data);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
		;
	}
	else 	if(sel==3)
	{
	USART_SendData(USART3, (u8) data);
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
		;
	}
}


/// �ض���c�⺯��printf��USART
int fputc(int ch, FILE *f)
{
		/* ����һ���ֽ����ݵ�USART1 */
		USART_SendData(TEST_USART, (uint8_t) ch);
		
		/* �ȴ�������� */
		while (USART_GetFlagStatus(TEST_USART, USART_FLAG_TC) == RESET);		
	
		return (ch);
}

/// �ض���c�⺯��scanf��USART
int fgetc(FILE *f)
{
		/* �ȴ�����1�������� */
		while (USART_GetFlagStatus(TEST_USART, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(TEST_USART);
}


static void Send_Data_GOL_LINK(char *dataToSend , char length)
{
	int i;
  for(i=0;i<length;i++)
     UsartSend(dataToSend[i]);
}


void Send_To_Wio(void)
{ char i;	char sum = 0;
	char data_to_send[50];
	char _cnt=0;
	vs16 _temp;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//������
	data_to_send[_cnt++]=0;//������
	_temp = (vs16)(adc_rc[0].bat_percent);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)(adc_rc[0].x);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(adc_rc[0].y);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
  _temp = (vs16)(adc_rc[1].x);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(adc_rc[1].y);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=key_rc[0];
	data_to_send[_cnt++]=key_rc[1];
	
	data_to_send[_cnt++]=key_sel[0];
	data_to_send[_cnt++]=key_sel[1];
	data_to_send[_cnt++]=key_sel[2];
	data_to_send[_cnt++]=key_sel[3];
	 
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}

/*********************************************END OF FILE**********************/
