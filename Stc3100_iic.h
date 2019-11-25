#ifndef __STC3100_IIC_H__
#define __STC3100_IIC_H__
#include "stm32f0xx.h"

/******************************** IIC driver ********************************************/
#define STC_IIC_RCC            RCC_AHBPeriph_GPIOB
#define STC_IIC_PORT           GPIOB
#define STC_IIC_SCL_PIN        GPIO_Pin_2
#define STC_IIC_SDA_PIN        GPIO_Pin_10

//IO��������
#define STC_SDA_IN()  {STC_IIC_PORT->MODER&=~(3<<(10*2));STC_IIC_PORT->MODER|=0<<10*2;}	  //PB10����ģʽ
#define STC_SDA_OUT() {STC_IIC_PORT->MODER&=~(3<<(10*2));STC_IIC_PORT->MODER|=1<<10*2;}   //PB10���ģʽ

//IO��������	 
#define STC_IIC_SCL_0    GPIO_ResetBits(STC_IIC_PORT,STC_IIC_SCL_PIN)
#define STC_IIC_SCL_1    GPIO_SetBits(STC_IIC_PORT,STC_IIC_SCL_PIN)
#define STC_IIC_SDA_0    GPIO_ResetBits(STC_IIC_PORT,STC_IIC_SDA_PIN)
#define STC_IIC_SDA_1    GPIO_SetBits(STC_IIC_PORT,STC_IIC_SDA_PIN)
#define STC_READ_SDA     GPIO_ReadInputDataBit(STC_IIC_PORT,STC_IIC_SDA_PIN)

//IIC���в�������
void STC_IIC_Init(void);                //��ʼ��IIC��IO��				 
void STC_IIC_Start(void);				//����IIC��ʼ�ź�
void STC_IIC_Stop(void);	  			//����IICֹͣ�ź�

void STC_IIC_Send_Byte(uint8_t txd);		 //IIC����һ���ֽ�
uint8_t STC_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�

uint8_t STC_IIC_Wait_Ack(void); 		//IIC�ȴ�ACK�ź�
void STC_IIC_Ack(void);					//IIC����ACK�ź�
void STC_IIC_NAck(void);				//IIC����NACK�ź�

uint8_t STC_IIC_MemWrite(uint8_t SlaveAddr, uint8_t RegisterAddr, uint8_t *Txbuf, uint8_t ByteCnt);
uint8_t STC_IIC_MemRead(uint8_t SlaveAddr, uint8_t RegisterAddr, uint8_t *Rxbuf, uint8_t ByteCnt);
/******************************** STC3100 driver ********************************************/

#define STC31xx_I2C_OK         0
#define STC31xx_I2C_ERROR      1
#define STC31xx_I2C_ERR_BUFFER 2

int STC3100_Write(unsigned char ByteCount, unsigned char RegisterAddr , unsigned char * TxBuffer);
int STC3100_Read(unsigned char ByteCount, unsigned char RegisterAddr , unsigned char * RxBuffer);
int Stc3100_Init(void);

#endif
