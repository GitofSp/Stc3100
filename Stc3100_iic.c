#include "Stc3100_iic.h"
#include "delay.h"
#include "stdio.h"

#define STC3100_SLAVE_ADDRESS_8BIT       0xE0   /* STC3100 8-bit address byte */
#define STC3100_SLAVE_ADDRESS_7BIT       0x70   /* STC3100 7-bit address byte */
#define NBRETRY 1

//��ʼ��IIC
void STC_IIC_Init(void){			
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(STC_IIC_RCC, ENABLE);//ʹ��GPIO������ʱ��

	//GPIOB8,B9��ʼ������
	GPIO_InitStructure.GPIO_Pin = STC_IIC_SCL_PIN | STC_IIC_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  //��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(STC_IIC_PORT, &GPIO_InitStructure);//��ʼ��
	
	STC_IIC_SCL_1;
	STC_IIC_SDA_1;
}

//����IIC��ʼ�ź�
void STC_IIC_Start(void)
{
	STC_SDA_OUT();     //sda�����
	STC_IIC_SDA_1;	  	
	STC_IIC_SCL_1;
	delay_us(4);
 	STC_IIC_SDA_0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	STC_IIC_SCL_0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void STC_IIC_Stop(void)
{
	STC_SDA_OUT();//sda�����
	STC_IIC_SCL_0;
	STC_IIC_SDA_0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	STC_IIC_SCL_1; 
	STC_IIC_SDA_1;//����I2C���߽����ź�
	delay_us(4);							   	
}
uint8_t STC_IIC_Wait_Ack(void)
{
	uint16_t ucErrTime=0,res = 0;

	STC_SDA_IN(); delay_us(1);   //SDA����Ϊ����  
	
	while(STC_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250){
			res = 1;
			break;
		}
	}
	STC_IIC_SCL_1;delay_us(1);	
	STC_IIC_SCL_0;delay_us(1);//ʱ�����0 	   
	return res;  
} 
//����ACKӦ��
void STC_IIC_Ack(void)
{
	STC_IIC_SCL_0;
	STC_SDA_OUT();
	STC_IIC_SDA_0;
	delay_us(2);
	STC_IIC_SCL_1;
	delay_us(2);
	STC_IIC_SCL_0;
	STC_SDA_IN();
}
//����NACKӦ��		    
void STC_IIC_NAck(void)
{
	STC_IIC_SCL_0;
	STC_SDA_OUT();
	STC_IIC_SDA_1;
	delay_us(2);
	STC_IIC_SCL_1;
	delay_us(2);
	STC_IIC_SCL_0;
	STC_SDA_IN();
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void STC_IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	STC_SDA_OUT(); 	    
    STC_IIC_SCL_0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
		if(((txd&0x80)>>7)){
			STC_IIC_SDA_1;
		}else{
			STC_IIC_SDA_0;
		}
        txd<<=1; 	  
		delay_us(2);   
		STC_IIC_SCL_1;
		delay_us(2); 
		STC_IIC_SCL_0;	
		delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t STC_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	
	STC_SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        STC_IIC_SCL_0; 
        delay_us(2);
		STC_IIC_SCL_1;
        receive<<=1;
        if(STC_READ_SDA)receive++;   
		delay_us(1); 
    }
	if (!ack)
        STC_IIC_NAck();//����nACK
    else
        STC_IIC_Ack(); //����ACK
    return receive;
}

uint8_t STC_IIC_MemWrite(uint8_t SlaveAddr, uint8_t RegisterAddr, uint8_t *Txbuf, uint8_t ByteCnt)
{
	uint8_t i,ret = 0;
	
	STC_IIC_Start(); 
	STC_IIC_Send_Byte((SlaveAddr<<1)|0x00);
	if(STC_IIC_Wait_Ack()){
		printf("STC_IIC_MemWrite 2 ack err\r\n");
		ret = 1;
	}
	
	/* �Ĵ�����ַ д*/
	STC_IIC_Send_Byte(RegisterAddr);
	if(STC_IIC_Wait_Ack()){
		printf("STC_IIC_MemWrite 2 ack err\r\n");
		ret = 2;
	}
	
	for(i=0;i<ByteCnt;i++){
		STC_IIC_Send_Byte(Txbuf[i]);
		STC_IIC_Wait_Ack();
	}
	STC_IIC_Stop(); 
	
	delay_ms(20);
	return ret;
}
uint8_t STC_IIC_MemRead(uint8_t SlaveAddr, uint8_t RegisterAddr, uint8_t *Rxbuf, uint8_t ByteCnt)
{
	uint8_t i=0,ret = 0;
	
	STC_IIC_Start(); 

	/* �豸��ַ д */
	STC_IIC_Send_Byte((SlaveAddr<<1)|0x00);
	if(STC_IIC_Wait_Ack()){
		ret = 1;
	}
	
	/* �Ĵ�����ַ д*/
	STC_IIC_Send_Byte(RegisterAddr);
	if(STC_IIC_Wait_Ack()){
		ret = 2;
	}
	
	STC_IIC_Start();
	/* �豸��ַ �� */
	STC_IIC_Send_Byte((SlaveAddr<<1)|0x01);
	if(STC_IIC_Wait_Ack()){
		ret = 3;
	}
	
	/* ���� */
	for(i=0;i<ByteCnt-1;i++){
		Rxbuf[i] = STC_IIC_Read_Byte(1);
	}
	Rxbuf[ByteCnt-1] = STC_IIC_Read_Byte(0);
	STC_IIC_Stop(); 

	delay_ms(20);
	return ret;
}


/*********************************************************************************/
int Stc3100_Init(void){
	STC_IIC_Init();
	return 0;
}
/*******************************************************************************
* Function Name  : STC3100_Write
* Description    : utility function to write several bytes to STC3100 registers
* Return         : error status
*******************************************************************************/
int STC3100_Write(unsigned char ByteCount, unsigned char RegisterAddr , unsigned char * TxBuffer) {
	int Status = STC31xx_I2C_ERROR;
	
	Status = STC_IIC_MemWrite(STC3100_SLAVE_ADDRESS_7BIT, RegisterAddr, TxBuffer, ByteCount);
	
	if(Status != STC31xx_I2C_OK){
		printf("STC_IIC_MemWrite error\n");
	}
	return Status;
}

/*******************************************************************************
* Function Name  : STC3100_Read
* Description    : utility function to read several bytes from STC3100 registers
* Return         : error status
*******************************************************************************/

int STC3100_Read(unsigned char ByteCount, unsigned char RegisterAddr , unsigned char * RxBuffer)
{
	int Status = STC31xx_I2C_ERROR;
	
	Status = STC_IIC_MemRead(STC3100_SLAVE_ADDRESS_7BIT, RegisterAddr, RxBuffer, ByteCount);
		
	if(Status != STC31xx_I2C_OK) {
		printf("STC_IIC_MemRead error\r\n");
	}

	return Status;
}
