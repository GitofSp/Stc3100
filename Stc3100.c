#include "Stc3100.h"
#include <stdio.h>
#include "server.h"

/* Private  Variables ---------------------------------------*/
uint16_t u16_BattVoltage;     // battery voltage in mV
uint16_t u16_BattCurrent;     // battery current in mA
uint16_t u16_BattTemperature; // battery Temperature in 0.1癈
uint16_t u16_BattChargeCount; // battery charge in mA.h
uint16_t u16_BattCounter;     // conv counter

/* --- constants ---------------------------------------------------------- */

#define CurrentFactor      (48210/SENSERESISTOR) 
// LSB=11.77uV/R= ~48210/R/4096 - convert to mA

#define ChargeCountFactor  (27443/SENSERESISTOR)
// LSB=6.7uVh/R ~27443/R/4096 - converter to mAh

#define VoltageFactor  9994  
// LSB=2.44mV ~9994/4096 - convert to mV

#define TemperatureFactor  5120  
// LSB=0.125癈 ~5120/4096 - convert to 0.1癈

/*******************************************************************************
* Function Name  : conv
* Description    : conversion utility 
*  convert a raw 16-bit value from STC3100 registers into user units (mA, mAh, mV, 癈)
*  (optimized routine for efficient operation on 8-bit processors such as ST7/STM8)
* Input          :s16_value, u16_factor
* Return         : result = value * factor / 4096
*******************************************************************************/
static uint16_t conv(uint16_t s16_value, uint16_t u16_factor){
	return( ( (uint32_t) s16_value * u16_factor ) >> 12 );
}
/*******************************************************************************
* Function Name  : STC3100_ReadByte
* Description    : utility function to read the value stored in one register
* Input          : u8_register: STC3100 register,
* Return         : 8-bit value, or 0 if error
*******************************************************************************/
int STC3100_ReadByte(uint8_t Addr){
	
	uint8_t u8_value;
	uint8_t pu8_data[2];
	int res;

	res=STC3100_Read(1, Addr, pu8_data);

	if (res == STC3100_OK){
		u8_value = pu8_data[0];
	} else {
		return (int)(-1);
	}

	return (u8_value);
}


/*******************************************************************************
* Function Name  : STC3100_ReadWord
* Description    : utility function to read the value stored in a register pair
* Input          : u8_register: STC3100 register,
* Return         : 16-bit value, or 0 if error
*******************************************************************************/
int STC3100_ReadWord(uint8_t Addr)
{
	uint16_t u16_value;
	uint8_t pu8_data[2];
	int res;

	res=STC3100_Read(2, Addr, pu8_data);

	if (res == STC3100_OK)
	{
		// no error
		u16_value = pu8_data[0];
		u16_value |= ((uint16_t)pu8_data[1]) << 8;
	}
	else
	{
		return (int)(-1); //error
	}

	return (u16_value);
}



/*******************************************************************************
* Function Name  : STC3100_WriteByte
* Description    : utility function to write a 8-bit value into a register
* Input          : u8_register: STC3100 register, u8_value: 8-bit value to write
* Return         : error status (STC3100_OK, !STC3100_OK)
*******************************************************************************/
int STC3100_WriteByte(uint8_t u8_Register, uint8_t u8_value)
{
	int res = STC31xx_I2C_ERROR;
	uint8_t pu8_data[2];

	pu8_data[0]=u8_value; 
	res = STC3100_Write(1, u8_Register, pu8_data);
	
	return res;
}


/* -----------------------------------------------------------------
----------------------------------------------------------------- */
/*******************************************************************************
* Function Name  : STC3100_WriteWord
* Description    : utility function to write a 16-bit value into one register pair
* Input          : u8_register: STC3100 register, s16_value: 16-bit value to write
* Return         : error status (STC3100_OK, !STC3100_OK)
*******************************************************************************/
int STC3100_WriteWord(uint8_t u8_Register, uint16_t u16_value)
{
	int res;
	uint8_t pu8_data[2];

	pu8_data[0]=u16_value & 0xFF; 
	pu8_data[1]=u16_value>>8; 
	res = STC3100_Write(1, u8_Register, pu8_data);

	if (res == STC3100_OK) {
		res = 0;
	}
	else
	{
		res = (int)(-1); //error
	}

	return(res);
}

/*******************************************************************************
* Function Name  : STC3100_Startup
* Description    :  initialize and start the STC3100 at application startup
* Input          : None
* Return         : error status (STC3100_OK, !STC3100_OK)
*******************************************************************************/
int STC3100_Startup(void)
{
	int s32_res;

	// first, check the presence of the STC3100 by reading first byte of dev. ID
	s32_res = STC3100_ReadByte(STC3100_REG_ID0);
	if (s32_res!= 0x10) return (-1);
	
	// read the REG_CTRL to reset the GG_EOC and VTM_EOC bits
	STC3100_ReadByte(STC3100_REG_CTRL);

	// write 0x02 into the REG_CTRL to reset the accumulator and counter and clear the PORDET bit,
	s32_res = STC3100_WriteByte(STC3100_REG_CTRL, 0x02);
	if (s32_res!= STC3100_OK) return (s32_res);

	// then 0x10 into the REG_MODE register to start the STC3100 in 14-bit resolution mode.
	s32_res = STC3100_WriteByte(STC3100_REG_MODE, 0x10);
	if (s32_res!= STC3100_OK) return (s32_res);
	
	return (STC3100_OK);
}


/*******************************************************************************
* Function Name  : STC3100_Powerdown
* Description    :  stop the STC3100 at application power down
* Input          : None
* Return         : error status (STC3100_OK, !STC3100_OK)
*******************************************************************************/
int STC3100_Powerdown(void)
{
	int s32_res;

	// write 0 into the REG_MODE register to put the STC3100 in standby mode
	s32_res = STC3100_WriteByte(STC3100_REG_MODE, 0);
	if (s32_res!= STC3100_OK) return (s32_res);

	return (STC3100_OK);
}


/*******************************************************************************
* Function Name  : ReadBatteryData
* Description    :  utility function to read the battery data from STC3100
*                  to be called every 2s or so
* Input          : None
* Return         : error status (STC3100_OK, !STC3100_OK)
* Affect         : global battery variables
*******************************************************************************/
#define N 10
int ReadBatteryData(void)
{
	uint8_t Vc[N];
	int u32_res;
	uint16_t u16_value;
#if 0
	// read STC3100 registers 0 to 11
	u32_res=STC3100_Read(1, STC3100_REG_CURRENT_LOW, VcL);
	if (u32_res!=STC3100_OK) return(u32_res);  // read failed
	u32_res=STC3100_Read(1, STC3100_REG_CURRENT_HIGH, VcH);
	if (u32_res!=STC3100_OK) return(u32_res);  // read failed
	// current
	u16_value=VcH[0]; u16_value = (u16_value<<8) + VcL[0];
	u16_value &= 0x3fff; // mask unused bits
	if (u16_value>=0x2000) u16_value -= 0x4000;  // convert to signed value
	u16_BattCurrent = conv(u16_value,CurrentFactor);  // result in mA	
	printf("current ok \r\n");
	
	VcH[0] = 0; VcL[0] = 0;
	u32_res=STC3100_Read(1, STC3100_REG_VOLTAGE_LOW, VcL);
	u32_res=STC3100_Read(1, STC3100_REG_VOLTAGE_HIGH, VcH);
	// voltage
	u16_value=VcH[0]; u16_value = (u16_value<<8) + VcL[0];
	u16_value &= 0x0fff; // mask unused bits
	if (u16_value>=0x0800) u16_value -= 0x1000;  // convert to signed value
	u16_BattVoltage = conv(u16_value,VoltageFactor);  // result in mV
	printf("voltage ok \r\n");
#else 
	u32_res=STC3100_Read(N, STC3100_REG_CHARGE_LOW, Vc);
	if (u32_res!=STC3100_OK) return(u32_res);	
	// charge count
	u16_value=Vc[1]; u16_value = (u16_value<<8) + Vc[0];
	u16_BattChargeCount = conv(u16_value,ChargeCountFactor);//conv(u16_value,ChargeCountFactor);  // result in mAh
	
	// conversion counter
	u16_value=Vc[3]; u16_value = (u16_value<<8) + Vc[2];
	u16_BattCounter = u16_value;

	// current
	u16_value=Vc[5]; u16_value = (u16_value<<8) + Vc[4];
	u16_value &= 0x1fff; // mask unused bits
	if (u16_value>=0x2000) u16_value -= 0x4000;  // convert to signed value
	u16_BattCurrent =conv(u16_value,CurrentFactor);// conv(u16_value,CurrentFactor);  // result in mA

	// voltage
	u16_value=Vc[7]; u16_value = (u16_value<<8) + Vc[6];
	u16_value &= 0x0fff; // mask unused bits
	if (u16_value>=0x0800) u16_value -= 0x1000;  // convert to signed value
	u16_BattVoltage = conv(u16_value,VoltageFactor);  // result in mV

	// temperature
	u16_value=Vc[9]; u16_value = (u16_value<<8) + Vc[8];
	u16_value &= 0x0fff; // mask unused bits
	if (u16_value>=0x0800) u16_value -= 0x1000;  // convert to signed value
	u16_BattTemperature = conv(u16_value,TemperatureFactor);  // result in 0.1癈


//	u32_res=STC3100_Read(4, STC3100_REG_CURRENT_LOW, Vc);
//	if (u32_res!=STC3100_OK) return (u32_res);	
//	
//	// current
//	u16_value=Vc[1]; u16_value = (u16_value<<8) + Vc[0];
//	u16_value &= 0x3fff; // mask unused bits
//	if (u16_value>=0x2000) u16_value -= 0x4000;  // convert to signed value
//	u16_BattCurrent = conv(u16_value,CurrentFactor);  // result in mA	
//	
//	// voltage
//	u16_value=Vc[3]; u16_value = (u16_value<<8) + Vc[2];
//	u16_value &= 0x0fff; // mask unused bits
//	if (u16_value>=0x0800) u16_value -= 0x1000;  // convert to signed value
//	u16_BattVoltage = conv(u16_value,VoltageFactor);  // result in mV
#endif

	return (STC3100_OK);
}

void BattaryDataShow(uint8_t *Stc_Elec)
{
	int status = 0;
	uint32_t val = 0;
	
	status = ReadBatteryData();
	if (status == STC3100_OK) {
		/* results available */
//		printf("Coulomb counter value (mAh): %d\r\n", u16_BattChargeCount);
//		printf("convertion counter         : %d\r\n", u16_BattCounter);
//		printf("Battery current (mA)       : %d\r\n", u16_BattCurrent);
		printf("当前电池电压   : %0.3f V\r\n", ((float)u16_BattVoltage)/1000);	
		printf("当前电池温度   : %0.1f °C\r\n", ((float)u16_BattTemperature/10));
	} else {
		printf("Error with ReadBatteryData\n");
	}
}
#if 0
int ReadBatteryData(void)
{
	uint8_t pu8_data[12];
	int u32_res;
	uint16_t u16_value;

	// read STC3100 registers 0 to 11
	u32_res=STC3100_Read(12, 0, pu8_data);

	if (u32_res!=STC3100_OK) return(u32_res);  // read failed

	// fill the battery status data

	// charge count
	u16_value=pu8_data[3]; u16_value = (u16_value<<8) + pu8_data[2];
	u16_BattChargeCount = conv(u16_value,ChargeCountFactor);  // result in mAh

	// conversion counter
	u16_value=pu8_data[5]; u16_value = (u16_value<<8) + pu8_data[4];
	u16_BattCounter = u16_value;

	// current
	u16_value=pu8_data[7]; u16_value = (u16_value<<8) + pu8_data[6];
	u16_value &= 0x3fff; // mask unused bits
	if (u16_value>=0x2000) u16_value -= 0x4000;  // convert to signed value
	u16_BattCurrent = conv(u16_value,CurrentFactor);  // result in mA

	// voltage
	u16_value=pu8_data[9]; u16_value = (u16_value<<8) + pu8_data[8];
	u16_value &= 0x0fff; // mask unused bits
	if (u16_value>=0x0800) u16_value -= 0x1000;  // convert to signed value
	u16_BattVoltage = conv(u16_value,VoltageFactor);  // result in mV

	// temperature
	u16_value=pu8_data[11]; u16_value = (u16_value<<8) + pu8_data[10];
	u16_value &= 0x0fff; // mask unused bits
	if (u16_value>=0x0800) u16_value -= 0x1000;  // convert to signed value
	u16_BattTemperature = conv(u16_value,TemperatureFactor);  // result in 0.1癈

	return (STC3100_OK);
}
#endif
