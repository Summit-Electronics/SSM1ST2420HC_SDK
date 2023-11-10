/*
 * SSM1ST2420HC.c
 *
 *  Created on: 3 nov. 2022
 *      Author: Wesley Goedegebuure
 *      Email: wesley@top-electronics.com
 */
#include "SSM1ST2420HC.h"
//#include <math.h>

uint16_t AMSoffset = 0;
uint32_t SG_RESULTS[1000];
uint32_t T_STEP[1000];
int x = 0;

/* AMS VARIABLES */ //TODO: wegwerken in SSM1ST24HC library
uint16_t Angles[1000];	//remove "//" for logging angle data
int Ax = 0;				// counter for buffer
uint8_t AMS_Ready;		//check for interrupt
uint16_t pAngle[100];

void TMC5160_Basic_Init(CurrentConfig *Current)
{
	/* CURRENT SETTINGS
	 I_RUN, Max run current = 20 = ~2.0A
	 I_HOLD, Max Hold current = 20 = ~2.0A
	*/

	uint32_t IHOLD_IRUN = 0x00070000; // standard IHOLD DELAY value
	//uint32_t GSTAT_VALUE = 0x00000000; //default value for GSTAT

	if(Current->IHOLD > 20) // set upper current limit ~2.0A
	{
		Current->IHOLD = 20;
	}

	if(Current->IRUN > 20) // set upper current limit ~2.0A
	{
		Current->IRUN = 20;
	}

	IHOLD_IRUN += Current->IHOLD + (Current->IRUN <<8);

	TMC5160_SPIWrite(0x00, 0x00000008, 1); // writing value 0x00000008 = 8 = 0.0 to address 0 = 0x00(GCONF)
	TMC5160_SPIWrite(0x00, 0x00000008, 0); // writing value 0x00000008 = 8 = 0.0 to address 0 = 0x00(GCONF)

	/*
	GSTAT_VALUE = TMC5160_SPIWrite(0x01, 0x00000000, 0); // read GSTAT (should be all 0)
	if(GSTAT_VALUE == 1)
	{
		TMC5160_SPIWrite(0x01, 0x00000001, 1); // write 1 bit to GSTAT to clear all error flags
		// TODO: is clearing the flags enough? or poweqr cycle needed?
	}*/

	TMC5160_SPIWrite(0x03, 0x00000000, 1); // writing value 0x00000000 = 0 = 0.0 to address 1 = 0x03(SLAVECONF)
	TMC5160_SPIWrite(0x05, 0x00000000, 1); // writing value 0x00000000 = 0 = 0.0 to address 2 = 0x05(X_COMPARE)
	TMC5160_SPIWrite(0x06, 0x00000000, 1); // writing value 0x00000000 = 0 = 0.0 to address 3 = 0x06(OTP_PROG)
	TMC5160_SPIWrite(0x08, 0x0000000F, 1); // writing value 0x0000000F = 15 = 0.0 to address 4 = 0x08(FACTORY_CONF)
	TMC5160_SPIWrite(0x09, 0x00010606, 1); // writing value 0x00010606 = 67078 = 0.0 to address 5 = 0x09(SHORT_CONF)
	TMC5160_SPIWrite(0x0A, 0x00080400, 1); // writing value 0x00080400 = 525312 = 0.0 to address 6 = 0x0A(DRV_CONF)
	TMC5160_SPIWrite(0x0B, 0x00000000, 1); // writing value 0x00000000 = 0 = 0.0 to address 7 = 0x0B(GLOBAL_SCALER)

	TMC5160_SPIWrite(0x10, IHOLD_IRUN, 1); // writing value 0x00070A03 = 461315 = 0.0 to address 8 = 0x10(IHOLD_IRUN)

	TMC5160_SPIWrite(0x11, 0x0000000A, 1); // writing value 0x0000000A = 10 = 0.0 to address 9 = 0x11(TPOWERDOWN)
	TMC5160_SPIWrite(0x13, 0x00000000, 1); // writing value 0x00000000 = 0 = 0.0 to address 10 = 0x13(TPWMTHRS)
	TMC5160_SPIWrite(0x14, 0x00000000, 1); // writing value 0x00000010 = 16 = 0.0 to address 11 = 0x14(TCOOLTHRS)
	TMC5160_SPIWrite(0x15, 0x00000000, 1); // writing value 0x00000000 = 0 = 0.0 to address 12 = 0x15(THIGH)
	TMC5160_SPIWrite(0x2C, 0x00000000, 1); // writing value 0x00000000 = 0 = 0.0 to address 23 = 0x2C(TZEROWAIT)
	TMC5160_SPIWrite(0x33, 0x00000000, 1); // writing value 0x00000000 = 0 = 0.0 to address 25 = 0x33(VDCMIN)
	TMC5160_SPIWrite(0x34, 0x00000000, 1); // writing value 0x00000000 = 0 = 0.0 to address 26 = 0x34(SW_MODE)
	TMC5160_SPIWrite(0x38, 0x00000000, 1); // writing value 0x00000000 = 0 = 0.0 to address 27 = 0x38(ENCMODE)
	TMC5160_SPIWrite(0x39, 0x00000000, 1); // writing value 0x00000000 = 0 = 0.0 to address 28 = 0x39(X_ENC)
	TMC5160_SPIWrite(0x3A, 0x00010000, 1); // writing value 0x00010000 = 65536 = 0.0 to address 29 = 0x3A(ENC_CONST)
	TMC5160_SPIWrite(0x3D, 0x00000000, 1); // writing value 0x00000000 = 0 = 0.0 to address 30 = 0x3D(ENC_DEVIATION)
	TMC5160_SPIWrite(0x60, 0xAAAAB554, 1); // writing value 0xAAAAB554 = 0 = 0.0 to address 31 = 0x60(MSLUT[0])
	TMC5160_SPIWrite(0x61, 0x4A9554AA, 1); // writing value 0x4A9554AA = 1251300522 = 0.0 to address 32 = 0x61(MSLUT[1])
	TMC5160_SPIWrite(0x62, 0x24492929, 1); // writing value 0x24492929 = 608774441 = 0.0 to address 33 = 0x62(MSLUT[2])
	TMC5160_SPIWrite(0x63, 0x10104222, 1); // writing value 0x10104222 = 269500962 = 0.0 to address 34 = 0x63(MSLUT[3])
	TMC5160_SPIWrite(0x64, 0xFBFFFFFF, 1); // writing value 0xFBFFFFFF = 0 = 0.0 to address 35 = 0x64(MSLUT[4])
	TMC5160_SPIWrite(0x65, 0xB5BB777D, 1); // writing value 0xB5BB777D = 0 = 0.0 to address 36 = 0x65(MSLUT[5])
	TMC5160_SPIWrite(0x66, 0x49295556, 1); // writing value 0x49295556 = 1227445590 = 0.0 to address 37 = 0x66(MSLUT[6])
	TMC5160_SPIWrite(0x67, 0x00404222, 1); // writing value 0x00404222 = 4211234 = 0.0 to address 38 = 0x67(MSLUT[7])
	TMC5160_SPIWrite(0x68, 0xFFFF8056, 1); // writing value 0xFFFF8056 = 0 = 0.0 to address 39 = 0x68(MSLUTSEL)
	TMC5160_SPIWrite(0x69, 0x00F70000, 1); // writing value 0x00F70000 = 16187392 = 0.0 to address 40 = 0x69(MSLUTSTART)
	TMC5160_SPIWrite(0x6C, 0x00410153, 1); // writing value 0x00410153 = 4260099 = 0.0 to address 41 = 0x6C(CHOPCONF)
	TMC5160_SPIWrite(0x6D, 0x00000000, 1); // writing value 0x00000000 = 0 = 0.0 to address 42 = 0x6D(COOLCONF)
	TMC5160_SPIWrite(0x6E, 0x00000000, 1); // writing value 0x00000000 = 0 = 0.0 to address 43 = 0x6E(DCCTRL)
	TMC5160_SPIWrite(0x70, 0xC40C001E, 1); // writing value 0xC40C001E = 0 = 0.0 to address 44 = 0x70(PWMCONF)

}

void TMC5160_Init_Stealthchop(void)
{
	TMC5160_SPIWrite(0x00, 0x0000000C, 1); //enable PWM
	TMC5160_SPIWrite(0x70, 0xC40C011E, 1); //enable PWM autoscale
	TMC5160_SPIWrite(0x35, 0x00000040, 1); //0x35(RAMP_STAT)

	//TODO: finilize
}

void TMC5160_Init_Stallguard(int reset)
{
	if(reset == 0)//basic stall setup
	{
		uint32_t SGT = 1;
		uint32_t COOLCONF = 0;

		COOLCONF += (SGT <<16);

		//TODO: test COOLCONF register write to match register value

		//TMC5160_SPIWrite(0x6D, COOLCONF, 1); //0x6D(COOLCONF) //Stallguard Threshold
		TMC5160_SPIWrite(0x6D, 0x00010000, 1); //0x6D(COOLCONF) //Stallguard Threshold
	}

	if(reset == 1)//perform reset of stall, to allow movement again
	{
		TMC5160_SPIWrite(0x35, 0x00000040, 1); //0x35(RAMP_STAT)
	}

	//TMC5160_SPIWrite(0x14, 0x00000000, 1); //0x14(TCOOLTHRS)
	// write TCOOLTHRS to a higher value than TSTEP as measured at velocity X

	//TMC5160_SPIWrite(0x15, 0x00000000, 1); //0x15(THIGH)


	// Stall is dependent on the used motor, current setting and speed settings.
	// Tuning must be done inside the application
	// first rotate the stepper at a speed appropriate for your application
	// Next read SG_RESULT
	// Apply load and monitor SG_RESULT
	// If SG_RESULT reaches 0 before "stall event" , increase SGT value (+63 highest) , if 0 is not reached lower SGT value (-64 lowest)

	// Sudden motor stops can cause for a stall event.
	// In this case , increase TCOOLTHRS to increase the duration before stallguard is activated
}

int TMC5160_Monitor_Stallguard(void)
{
	uint32_t DRV_STATUS;
	int Stall_Flag = 0;

	DRV_STATUS = TMC5160_SPIWrite(0x6F, 0x00000000, 0); //Read (DRV_STATUS)
	//T_STEP[x] = TMC5160_SPIWrite(0x12, 0x00000000, 0); // 0x12(TSTEP)

	Stall_Flag = (DRV_STATUS & (1 << 24)); //bit 24 of DRV_STATUS is Stallguard flag
	Stall_Flag = (Stall_Flag >> 24); // bitshift stall flag is 0 or 1

	if(Stall_Flag != 0) //stall detected -> stop motor
	{
		TMC5160_Stop();
	}

	for(int y = 32; y > 10; y--) // clear other data
	{
		DRV_STATUS &= ~(1 << y);
	}

	SG_RESULTS[x] = DRV_STATUS; // see SG_RESULTS in explorer for tuning stallguard (SG_RESULT 0 = stall detected)
	x++;

	if(x >= 1000) // stop stallguard demo or it will overflow
	{
		x = 0;

	    HAL_GPIO_WritePin(GPIOA,DRV_ENN_Pin,1); // LOW = ON
	    TMC5160_Stop();
	}

	return Stall_Flag;
}

void TMC5160_Basic_Rotate(uint8_t Mode, RampConfig *Ramp) // 0 = Velocity Mode + , 1 = Velocity Mode -
{
	//Velocity mode , using VMAX and AMAX

	TMC5160_SPIWrite(0x11, 	0x0000000A, 1); 	// writing value 0x0000000A = 10 = 0.0 to address 9 = 0x11(TPOWERDOWN)
	TMC5160_SPIWrite(0x13, 	0x000001F4, 1); 	// writing value 0x00000000 = 0 = 0.0 to address 10 = 0x13(TPWMTHRS)

	TMC5160_SPIWrite(0x23, Ramp->VSTART, 1); 	// = 1000 = 0.0 to address 15 = 0x23(VSTART)
	TMC5160_SPIWrite(0x26, Ramp->AMAX, 1); 		// = 12800 = 0.0 to address 18 = 0x26(AMAX)
	TMC5160_SPIWrite(0x27, Ramp->VMAX, 1); 		// = 51200 = 0.0 to address 19 = 0x27(VMAX)
	TMC5160_SPIWrite(0x2B, Ramp->VSTOP, 1); 	// = 10 = 0.0 to address 22 = 0x2B(VSTOP)

	if(Mode == 0)
	{
		TMC5160_SPIWrite(0x20, 	0x00000001, 1); 		// writing value 0x00000001 = 0 = 0.0 to address 13 = 0x20(RAMPMODE)VM +
	}

	else if(Mode == 1)
	{
		TMC5160_SPIWrite(0x20, 	0x00000002, 1); 		// writing value 0x00000002 = 0 = 0.0 to address 13 = 0x20(RAMPMODE) VM -
	}
}


void TMC5160_Rotate_To(uint32_t Position, RampConfig *Ramp)
{
	uint32_t Target_Angle = 0;
	uint32_t AMS_Angle = 0;
	uint32_t ENC_Angle = 0;
	uint32_t TMC_Angle = 0;

	TMC5160_SPIWrite(0x11, 	0x0000000A, 1); 	// writing value 0x0000000A = 10 = 0.0 to address 9 = 0x11(TPOWERDOWN)
	//TMC5160_SPIWrite(0x00, 	0x00000008, 1); 	// writing value 0x00000008 = 8 = 0.0 to address 0 = 0x00(GCONF)
	TMC5160_SPIWrite(0x13, 	0x000001F4, 1); 	// writing value 0x00000000 = 0 = 0.0 to address 10 = 0x13(TPWMTHRS)

	// setup for profile
	TMC5160_SPIWrite(0x23, Ramp->VSTART, 1); 	// = 1000 = 0.0 to address 15 = 0x23(VSTART)
	TMC5160_SPIWrite(0x24, Ramp->A1, 1); 		// = 5600 = 0.0 to address 16 = 0x24(A1)
	TMC5160_SPIWrite(0x25, Ramp->V1, 1); 		// = 12800 = 0.0 to address 17 = 0x25(V1)
	TMC5160_SPIWrite(0x26, Ramp->AMAX, 1); 		// = 12800 = 0.0 to address 18 = 0x26(AMAX)
	TMC5160_SPIWrite(0x27, Ramp->VMAX, 1); 		// = 51200 = 0.0 to address 19 = 0x27(VMAX)
	TMC5160_SPIWrite(0x28, Ramp->DMAX, 1); 		// = 700 = 0.0 to address 20 = 0x28(DMAX)
	TMC5160_SPIWrite(0x2A, Ramp->D1, 1); 		// = 1400 = 0.0 to address 21 = 0x2A(D1)
	TMC5160_SPIWrite(0x2B, Ramp->VSTOP, 1); 	// = 10 = 0.0 to address 22 = 0x2B(VSTOP)

	TMC5160_SPIWrite(0x20, 	0x00000000, 1); 	// writing value 0x00000000 = 0 = 0.0 to address 13 = 0x20(RAMPMODE) MTP
	TMC5160_SPIWrite(0x2D, Position, 1); 		// writing value to address 24 = 0x2D(XTARGET)  1 lap
	TMC5160_SPIWrite(0x21,	0x00000000, 0);		// READ position register

	/*Convert to Angle so we can compare to Hall Sensor*/

	if((int)Position < 0)
	{
		Position = Position * (-1); 			// make positive for easier math
	}

	if(Position == 0) 							// to fix the first reading when target position = 0
	{
		AMS_Angle = 1;
		ENC_Angle = 1;
		TMC_Angle = 1;
	}

    Target_Angle = ((Position / 256) * 1.8);	//convert target position to angle

	// enter loop to check if position is reached
	while(TMC_Angle != Target_Angle) //&& AMS_Angle != Target_Angle && ENC_Angle != Target_Angle
	{
		TMC_Angle = TMC_Get_Position();

		if(AMS_ENB == 1)// Hall sensor is enabled
		{
			HAL_Delay(5);//to reduce sensor readout freq
			AMS_Angle = AMS5055_Get_Position();
		}

		if(ENC_ENB == 1)// Encoder is enabled
		{
			ENC_Angle = ENC_Get_Position();
		}
	}
 }

uint16_t TMC_Get_Position()
{
	uint32_t AngleT = 0;

	AngleT = TMC5160_SPIWrite(0x21, 0x00000000, 0); //read step counter from TMC5160


	if((int)AngleT < 0)
	{
		AngleT = AngleT * (-1); // make positive for easier math
	}

	AngleT = ((AngleT / 256)* 1.8);//convert target position to angle

	return AngleT;
}

void TMC5160_Stop(void)
{
	TMC5160_SPIWrite(0x27,0x00000000, 1); //set VMAX to 0
}

void Drive_Enable(int state)
{
	if(state == 1) // Enable driver
	{
		HAL_GPIO_WritePin(GPIOA, DRV_ENN_Pin, 0); // LOW = ON
		HAL_Delay(10);
	}

	if(state == 0) // disable drive
	{
		HAL_GPIO_WritePin(GPIOA, DRV_ENN_Pin, 1); // HIGH = OFF
		HAL_Delay(10);
	}
}


uint32_t TMC5160_SPIWrite(uint8_t Address, uint32_t Value, int Action)
{

	uint8_t SPI2TxData[5];  //TX data array SPI2
	uint8_t SPI2RxData[5];  //RX data array SPI2
	uint32_t SPI2Rx = 0;

	  HAL_GPIO_WritePin(GPIOB,TMC_CS_Pin,0); // set TMC CS low

	  if (Action == 1) //Write
	  {
		SPI2TxData[0] = Address + 0x80;
	  }

	  else //Read
	  {
		SPI2TxData[0] = Address;
	  }

	  SPI2TxData[1] = Value >> 24;
	  SPI2TxData[2] = Value >> 16;
	  SPI2TxData[3] = Value >> 8;
	  SPI2TxData[4] = Value;

	  HAL_SPI_TransmitReceive(&hspi2, SPI2TxData, SPI2RxData, 0x05, 100);


	  SPI2Rx += (SPI2RxData[1] << 24);
	  SPI2Rx += (SPI2RxData[2] << 16);
	  SPI2Rx += (SPI2RxData[3] << 8);
	  SPI2Rx += (SPI2RxData[4] << 0);

	  HAL_GPIO_WritePin(GPIOB,TMC_CS_Pin,1); // set TMC CS high

	  return SPI2Rx;
}

void AMS5055_Basic_Init(void)
{
	//start new angle measuremnt
	AMSoffset = AMS5055_Get_Position();  // Angle read when standstill is offset
}

uint16_t AMS5055_Get_Position(void)
{
	uint16_t Angle = 0;

	AMS5055_SPIWriteInt(ANGULAR_DATA,1);

	while(AMS_Ready != 1) //wacht op INT
	{
	}

	Angle = AMS5055_SPIWriteInt(NOP,1);

	if(Angle > 32768)
	{
		Angle = Angle - 32768;
	}

	if(Angle > 16384)
	{
		Angle = Angle - 16384;
	}

	if(Angle > 8192)
	{
		Angle = Angle - 8192;
	}

	Angle >>= 1;

	Angle = Angle - AMSoffset;  // AMS is not calibrated, so angle needs to be fixed
	Angle = ((float)Angle / 4095.0) * 360.0; //12 bit resolution
	Angles[Ax] = Angle;  //uncomment to enable logging of Angle position

	if (Ax >= 1000) // to prevent overflow
	{
		Ax = 0;
	}

	else
	{
		Ax++;
		AMS_Ready = 0;
	}

	return Angle;
}

uint8_t AMSParity(uint16_t value)
{
	uint8_t cnt = 0;
	uint8_t i;

	for (i = 0; i < 16; i++)
	{
		if (value & 0x1)
		{
			cnt++;
		}

		value >>= 1;
	}
	return cnt & 0x1;
}

uint16_t AMS5055_SPIWriteInt(uint16_t Address, int Action)
{
	/* WRITE = 0
	| 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01  00 |
 	| 0	|			 Data 14:1		   		      | PAR|


 	(Data (14 bit) +  + parity)
	| 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01  00 |
 	|		 		 Data 15:2		   		   |EF| PAR|

 	EF-> 0 = no command frame error occurred, 1 = error occurred
 	PAR -> Parity bit
	*/

	/* READ = 1
	| 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01  00 |
 	| 1	|			 Data 14:1		   		      | PAR|



	| 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01  00 |
 	|				 Data 15:2		   		   |DC| PAR|

 	DC -> Don't care
 	PAR -> Parity bit
	*/

	uint8_t SPI1TxData[2];	//TX data array SPI1
	uint8_t SPI1RxData[2];	//RX data array SPI1

	uint16_t SPI1Rx = 0;
	uint16_t SPI1Tx = 0;

	HAL_GPIO_WritePin(GPIOA,AMS_CS_Pin,0); // set TMC CS low


	  SPI1Tx = (Address << 1);

	  if (Action == 1) //READ
	  {
		  SPI1Tx = SPI1Tx | 0x8000;
	  }

	  SPI1Tx = SPI1Tx | AMSParity(SPI1Tx);

	 // HAL_GPIO_WritePin(GPIOA,AMS_CS_Pin,0); // set TMC CS low

	  SPI1TxData[0] = SPI1Tx >> 8;
	  SPI1TxData[1] = SPI1Tx;

	  HAL_SPI_TransmitReceive(&hspi1, SPI1TxData, SPI1RxData, 0x02, 100);
	//HAL_SPI_TransmitReceive(&hspi1, SPI1Tx, SPI1Rx, 0x02, 100);

	  //HAL_GPIO_WritePin(GPIOA,AMS_CS_Pin,1); // set TMC CS high

	  SPI1Rx += (SPI1RxData[0] << 8);	//<< 8
	  SPI1Rx += (SPI1RxData[1] << 0);	//<< 0

	  HAL_GPIO_WritePin(GPIOA,AMS_CS_Pin,1); // set TMC CS high

	  return SPI1Rx;
}

uint16_t ENC_Get_Position(void)
{
	uint32_t Enc_Position = TMC5160_SPIWrite(0x39, 0x00000000, 0); // read encoder position

	if((int)Enc_Position <= 0)
	{
		Enc_Position = Enc_Position * (-1);
	}

	Enc_Position = ((Enc_Position / 256)* 1.8);

	return Enc_Position;
}


void Toggle_OUT(int port ,uint16_t time)
{
	if(port == 1)//Enable OUT1 for 1 sec (24V)
	{
	 HAL_GPIO_WritePin(GPIOB,EXT_OUT_1_Pin,1);
	 HAL_Delay(time);
	 HAL_GPIO_WritePin(GPIOB,EXT_OUT_1_Pin,0);
	}

	if(port == 2) // Enable OUT2 for 1 sec (24V)
	{
	 HAL_GPIO_WritePin(GPIOB,EXT_OUT_2_Pin,1);
	 HAL_Delay(time);
	 HAL_GPIO_WritePin(GPIOB,EXT_OUT_2_Pin,0);
	}
}

int Read_IN(int port)
{
	int val = 0;

	if(port == 1)//Read IN1 (5 to 24V = 1)
	{
		val = HAL_GPIO_ReadPin(GPIOB, REFL_UC_Pin);
	}

	if(port == 2)//Read IN1 (5 to 24V = 1)
	{
		val = HAL_GPIO_ReadPin(GPIOB, REFR_UC_Pin);

	}

	return val;
}

uint16_t Read_AIN(void)
{
	uint16_t ADCReading = 0;

	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 10);
	ADCReading = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);

	return ADCReading;
}


