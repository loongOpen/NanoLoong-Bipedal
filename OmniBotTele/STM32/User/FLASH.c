#include "mpu9250.h"
#include "stm32f10x_flash.h"//flash�����ӿ��ļ����ڿ��ļ��У�������Ҫ����
#include "FLASH.h"
#include "imu.h"
#include "rc_mine.h"
union {
float Bit32;
unsigned char Bit8[4];
}flash;

/****************************************************************
*Function:	STM32F103ϵ���ڲ�Flash��д����
*Author:    ValerianFan
*Date:		2014/04/09
*E-Mail:	fanwenjingnihao@163.com
*Other:		�ó�����ֱ�ӱ������У�ֻ������Flash��д����
****************************************************************/
#define  STARTADDR  0x0801F000                   	 //STM32F103RB �����ͺŻ������ã�δ����
//#define  STARTADDR  0x08040000//0x080350A8
volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;      //Flash����״̬����
/****************************************************************
*Name:		ReadFlashNBtye
*Function:	���ڲ�Flash��ȡN�ֽ�����
*Input:		ReadAddress�����ݵ�ַ��ƫ�Ƶ�ַ��ReadBuf������ָ��	ReadNum����ȡ�ֽ���
*Output:	��ȡ���ֽ���
*Author:    ValerianFan
*Date:		2014/04/09
*E-Mail:	fanwenjingnihao@163.com
*Other:
****************************************************************/
int ReadFlashNBtye(uint32_t ReadAddress, uint8_t *ReadBuf, int32_t ReadNum)
{
        int DataNum = 0;
		ReadAddress = (uint32_t)STARTADDR + ReadAddress;
        while(DataNum < ReadNum)
		{
           *(ReadBuf + DataNum) = *(__IO uint8_t*) ReadAddress++;
           DataNum++;
        }
        return DataNum;
}

/****************************************************************
*Name:		WriteFlashOneWord
*Function:	���ڲ�Flashд��32λ����
*Input:		WriteAddress�����ݵ�ַ��ƫ�Ƶ�ַ��WriteData��д������
*Output:	NULL
*Author:    ValerianFan
*Date:		2014/04/09
*E-Mail:	fanwenjingnihao@163.com
*Other:
****************************************************************/
void WriteFlashOneWord(uint32_t WriteAddress,uint32_t WriteData)
{
	FLASH_UnlockBank1();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
        FLASHStatus = FLASH_ErasePage(STARTADDR);
	if(FLASHStatus == FLASH_COMPLETE)
	{
		FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress, WriteData);    //flash.c ��API����
		//FLASHStatus = FLASH_ProgramWord(StartAddress+4, 0x56780000);//��Ҫд���������ʱ����
		//FLASHStatus = FLASH_ProgramWord(StartAddress+8, 0x87650000);//��Ҫд���������ʱ����
	}
	FLASH_LockBank1();
}


void WriteFlashHarfWord(uint32_t WriteAddress,uint16_t WriteData)
{
	FLASH_UnlockBank1();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
        FLASHStatus = FLASH_ErasePage(STARTADDR);
	if(FLASHStatus == FLASH_COMPLETE)
	{
            FLASHStatus = FLASH_ProgramHalfWord(STARTADDR + WriteAddress, WriteData);    //flash.c ��API����
	}
	FLASH_LockBank1();
}

void WriteFlashNineFloat(uint32_t WriteAddress,
                         float WriteData1,
                         float WriteData2,
                         float WriteData3,
                         float WriteData4,
                         float WriteData5,
                         float WriteData6,
                         float WriteData7,
                         float WriteData8,
                         float WriteData9)
{
        uint32_t Buf[9]={0};
        Buf[0]=*(uint32_t *)(&WriteData1);//���ڴ��������ĸ��ֽ�д�뵽Flash
        Buf[1]=*(uint32_t *)(&WriteData2);
        Buf[2]=*(uint32_t *)(&WriteData3);
        Buf[3]=*(uint32_t *)(&WriteData4);
        Buf[4]=*(uint32_t *)(&WriteData5);
        Buf[5]=*(uint32_t *)(&WriteData6);
        Buf[6]=*(uint32_t *)(&WriteData7);
        Buf[7]=*(uint32_t *)(&WriteData8);
        Buf[8]=*(uint32_t *)(&WriteData9);

        FLASH_UnlockBank1();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
        FLASHStatus = FLASH_ErasePage(STARTADDR);
	if(FLASHStatus == FLASH_COMPLETE)
	{
		FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress,Buf[0]);
		FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+4,Buf[1]);
		FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+8,Buf[2]);
                FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+12,Buf[3]);
		FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+16,Buf[4]);
		FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+20,Buf[5]);
                FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+24,Buf[6]);
		FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+28,Buf[7]);
		FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+32,Buf[8]);

	}
	FLASH_LockBank1();
}

uint8_t ReadFlashThreeFloat(uint32_t ReadAddress,
                         float *WriteData1,
                         float *WriteData2,
                         float *WriteData3)
{
    uint8_t buf[12];
    uint16_t i=0;
    uint8_t flag=0x00;
    ReadAddress = (uint32_t)STARTADDR + ReadAddress;
    *WriteData1=*(float *)(ReadAddress);
    *WriteData2=*(float *)(ReadAddress+4);
    *WriteData3=*(float *)(ReadAddress+8);
    FLASH_LockBank1();

    for(i=0;i<12;i++)//���ֽ�����
    {
        *(buf+i)=*(__IO uint8_t*) ReadAddress++;
    }
    if((buf[0]==0xff&&buf[1]==0xff&&buf[2]==0xff&&buf[3]==0xff))
       flag=flag|0x01;
    if((buf[4]==0xff&&buf[5]==0xff&&buf[6]==0xff&&buf[7]==0xff))
       flag=flag|0x02;
    if((buf[8]==0xff&&buf[9]==0xff&&buf[10]==0xff&&buf[11]==0xff))
       flag=flag|0x04;
    return flag;
}


u8 Parameter_Init(void)
{
	  float temp;
    ReadFlashThreeFloat(Accel_Offset_Address,
                         &mpu6050_fc.Acc_Offset.x,
                         &mpu6050_fc.Acc_Offset.y,
                         &mpu6050_fc.Acc_Offset.z);

    ReadFlashThreeFloat(Gyro_Offset_Address,
                         &mpu6050_fc.Gyro_Offset.x,
                         &mpu6050_fc.Gyro_Offset.y,
                         &mpu6050_fc.Gyro_Offset.z);
	
	  ReadFlashThreeFloat(Mag_Offset_Address,
                         &off_att[0],
                         &off_att[1],
                         &temp);
	  CHANNAL=temp;
	
	
	  //plane.PID_RX[19][2]=temp;
}

void WRITE_PARM(void){
//WriteFlashNineFloat(Accel_Offset_Address,
//                        mpu6050_fc.Acc_Offset.x,
//                        mpu6050_fc.Acc_Offset.y,
//                        mpu6050_fc.Acc_Offset.z,
//                        mpu6050_fc.Gyro_Offset.x,
//                        mpu6050_fc.Gyro_Offset.y,
//                        mpu6050_fc.Gyro_Offset.z,
//                        off_att[0],
//                        off_att[1],
//                        plane.PID_RX[19][2]);//д����ٶ����ƫִ�����������ƫִ
}