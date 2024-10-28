/*
 * SSM1ST2420HC.c
 *
 *  Created on: 3 nov. 2022
 *      Author: Wesley Goedegebuure
 *      Email: wesley@top-electronics.com
 */
#include "SSM1ST2420HC.h"

/* AMS VARIABLES */
uint8_t AMS_Ready;		//check for interrupt
uint32_t AMSAngles[1000]; //for logging
uint32_t roughAngles[1000];
uint16_t AmsPos = 0; //for logging
uint32_t AMSoffset = 0;
uint32_t Previous_AMS_Angle = 2000;
uint32_t Wrapper = 0;


/* SPI VARIABLES */
#define MAX_WRITE_ACTIONS 80 //TMC5160 has less registers
uint8_t writeAddresses[MAX_WRITE_ACTIONS];
uint32_t writeValues[MAX_WRITE_ACTIONS];
int numWriteActions = 0; // Counter for the number of write actions stored
uint8_t ResetCount = 0;
uint8_t ResetWriteAction[MAX_WRITE_ACTIONS];
uint8_t SPICheckValue = 0;

/* TMC VARIABLES */
uint8_t TMC_Boot = 0;
uint32_t SG_RESULTS[1000]; //for logging
int x = 0; // for logging
#define POSITION_TOLERANCE 256 // Adjust this value as needed, currently it's set to 0.5%

double ENC_Factor = 0;
double AMS_Factor = 0;

void TMC5160_Basic_Init(CurrentConfig *Current)
{
	/* CURRENT SETTINGS
	 I_RUN, Max run current = 20 = ~2.0A
	 I_HOLD, Max Hold current = 20 = ~2.0A
	*/

	TMC5160_Startup(); //check if SPI communication is ok
	TMC5160_Stop(); //after SPI ok, stop motor

	uint32_t IHOLD_IRUN = 0x00070000; // standard IHOLD DELAY value

	if(Current->IHOLD > 20) // set upper current limit ~2.0A
	{
		Current->IHOLD = 20;
	}

	if(Current->IRUN > 20) // set upper current limit ~2.0A
	{
		Current->IRUN = 20;
	}

	IHOLD_IRUN += Current->IHOLD + (Current->IRUN <<8);

	TMC5160_SPIWrite(0x03, 0x00000000, 1, &SPICheckValue); // writing value 0x00000000 = 0 = 0.0 to address 1 = 0x03(SLAVECONF)
	TMC5160_SPIWrite(0x05, 0x00000000, 1, &SPICheckValue); // writing value 0x00000000 = 0 = 0.0 to address 2 = 0x05(X_COMPARE)
	TMC5160_SPIWrite(0x06, 0x00000000, 1, &SPICheckValue); // writing value 0x00000000 = 0 = 0.0 to address 3 = 0x06(OTP_PROG)
	TMC5160_SPIWrite(0x08, 0x0000000F, 1, &SPICheckValue); // writing value 0x0000000F = 15 = 0.0 to address 4 = 0x08(FACTORY_CONF)
	TMC5160_SPIWrite(0x09, 0x00010606, 1, &SPICheckValue); // writing value 0x00010606 = 67078 = 0.0 to address 5 = 0x09(SHORT_CONF)
	TMC5160_SPIWrite(0x0A, 0x00080400, 1, &SPICheckValue); // writing value 0x00080400 = 525312 = 0.0 to address 6 = 0x0A(DRV_CONF)
	TMC5160_SPIWrite(0x0B, 0x00000000, 1, &SPICheckValue); // writing value 0x00000000 = 0 = 0.0 to address 7 = 0x0B(GLOBAL_SCALER)

	TMC5160_SPIWrite(0x10, IHOLD_IRUN, 1, &SPICheckValue); // writing value 0x00070A03 = 461315 = 0.0 to address 8 = 0x10(IHOLD_IRUN)

	TMC5160_SPIWrite(0x11, 0x0000000A, 1, &SPICheckValue); // writing value 0x0000000A = 10 = 0.0 to address 9 = 0x11(TPOWERDOWN)
	TMC5160_SPIWrite(0x13, 0x00000000, 1, &SPICheckValue); // writing value 0x00000000 = 0 = 0.0 to address 10 = 0x13(TPWMTHRS)
	TMC5160_SPIWrite(0x14, 0x00000000, 1, &SPICheckValue); // writing value 0x00000010 = 16 = 0.0 to address 11 = 0x14(TCOOLTHRS)
	TMC5160_SPIWrite(0x15, 0x00000000, 1, &SPICheckValue); // writing value 0x00000000 = 0 = 0.0 to address 12 = 0x15(THIGH)
	TMC5160_SPIWrite(0x2C, 0x00000000, 1, &SPICheckValue); // writing value 0x00000000 = 0 = 0.0 to address 23 = 0x2C(TZEROWAIT)
	TMC5160_SPIWrite(0x33, 0x00000000, 1, &SPICheckValue); // writing value 0x00000000 = 0 = 0.0 to address 25 = 0x33(VDCMIN)
	TMC5160_SPIWrite(0x34, 0x00000000, 1, &SPICheckValue); // writing value 0x00000000 = 0 = 0.0 to address 26 = 0x34(SW_MODE)
	TMC5160_SPIWrite(0x38, 0x00000000, 1, &SPICheckValue); // writing value 0x00000000 = 0 = 0.0 to address 27 = 0x38(ENCMODE)
	TMC5160_SPIWrite(0x3A, 0x00010000, 1, &SPICheckValue); // writing value 0x00010000 = 65536 = 0.0 to address 29 = 0x3A(ENC_CONST)
	TMC5160_SPIWrite(0x3D, 0x00000000, 1, &SPICheckValue); // writing value 0x00000000 = 0 = 0.0 to address 30 = 0x3D(ENC_DEVIATION)
	TMC5160_SPIWrite(0x60, 0xAAAAB554, 1, &SPICheckValue); // writing value 0xAAAAB554 = 0 = 0.0 to address 31 = 0x60(MSLUT[0])
	TMC5160_SPIWrite(0x61, 0x4A9554AA, 1, &SPICheckValue); // writing value 0x4A9554AA = 1251300522 = 0.0 to address 32 = 0x61(MSLUT[1])
	TMC5160_SPIWrite(0x62, 0x24492929, 1, &SPICheckValue); // writing value 0x24492929 = 608774441 = 0.0 to address 33 = 0x62(MSLUT[2])
	TMC5160_SPIWrite(0x63, 0x10104222, 1, &SPICheckValue); // writing value 0x10104222 = 269500962 = 0.0 to address 34 = 0x63(MSLUT[3])
	TMC5160_SPIWrite(0x64, 0xFBFFFFFF, 1, &SPICheckValue); // writing value 0xFBFFFFFF = 0 = 0.0 to address 35 = 0x64(MSLUT[4])
	TMC5160_SPIWrite(0x65, 0xB5BB777D, 1, &SPICheckValue); // writing value 0xB5BB777D = 0 = 0.0 to address 36 = 0x65(MSLUT[5])
	TMC5160_SPIWrite(0x66, 0x49295556, 1, &SPICheckValue); // writing value 0x49295556 = 1227445590 = 0.0 to address 37 = 0x66(MSLUT[6])
	TMC5160_SPIWrite(0x67, 0x00404222, 1, &SPICheckValue); // writing value 0x00404222 = 4211234 = 0.0 to address 38 = 0x67(MSLUT[7])
	TMC5160_SPIWrite(0x68, 0xFFFF8056, 1, &SPICheckValue); // writing value 0xFFFF8056 = 0 = 0.0 to address 39 = 0x68(MSLUTSEL)
	TMC5160_SPIWrite(0x69, 0x00F70000, 1, &SPICheckValue); // writing value 0x00F70000 = 16187392 = 0.0 to address 40 = 0x69(MSLUTSTART)
	TMC5160_SPIWrite(0x6C, 0x00410153, 1, &SPICheckValue); // writing value 0x00410153 = 4260099 = 0.0 to address 41 = 0x6C(CHOPCONF)
	TMC5160_SPIWrite(0x6D, 0x00000000, 1, &SPICheckValue); // writing value 0x00000000 = 0 = 0.0 to address 42 = 0x6D(COOLCONF)
	TMC5160_SPIWrite(0x6E, 0x00000000, 1, &SPICheckValue); // writing value 0x00000000 = 0 = 0.0 to address 43 = 0x6E(DCCTRL)
	TMC5160_SPIWrite(0x70, 0xC40C001E, 1, &SPICheckValue); // writing value 0xC40C001E = 0 = 0.0 to address 44 = 0x70(PWMCONF)

	ENC_Factor = (51200.0 / ENC_Resolution);
	AMS_Factor = (51200.0 / AMS_Resolution);

	if(ENC_HOME == 1)
	  {
		TMC5160_SPIWrite(0x38, 0x00000134, 1, &SPICheckValue); // writing value 0x00000000 = 0 = 0.0 to address 27 = 0x38(ENCMODE)
							 //0x00000124 for clear ONCE at N-event

		//latch and clear encoder counter X_ENC ONCE at N-event
		//N-event occurs when polarities B and A ,match both set at 0
		//pol-N = 1 active high
		//encoder X_ENC will be set to 0 when N event occurs which occurs when crossing Home position.

		ENC_Start_position(); // read encoder current position
	  }

	  if(AMS_ENB == 1)
	  {
		  AMS5055_Basic_Init(); // set Hall sensor offset to current position
	  }

}

void TMC5160_Init_Stealthchop(void)//TODO: finalize
{
	TMC5160_SPIWrite(0x00, 0x0000000C, 1, &SPICheckValue); //enable PWM
	TMC5160_SPIWrite(0x70, 0xC40C011E, 1, &SPICheckValue); //enable PWM autoscale
	//TMC5160_SPIWrite(0x35, 0x00000040, 1, &SPICheckValue); //0x35(RAMP_STAT)

	//check 0x6f DRV_STATUS for stealth flag
}

void TMC5160_Init_Stallguard(int reset, uint32_t SGT)
{
	if(reset == 0)//basic stall setup
	{
		if((int32_t)SGT > 63)
		{
			SGT = 63; //SGT max value
		}

		else if((int32_t)SGT < -64)
		{
			SGT = -64; //SGT min value
		}

		uint32_t COOLCONF = 0;

		COOLCONF += (SGT <<16);

		TMC5160_SPIWrite(0x6D, COOLCONF, 1, &SPICheckValue); //0x6D(COOLCONF) //Stallguard Threshold
	}

	if(reset == 1)//Reset stall flag, to allow movement again
	{
		TMC5160_SPIWrite(0x35, 0x00000040, 1, &SPICheckValue); //0x35(RAMP_STAT)
	}

	// Stall is dependent on the used motor, current setting and speed settings.
	// Tuning must be done inside the application
	// first rotate the stepper at a speed appropriate for your application
	// Next read SG_RESULT
	// Apply load and monitor SG_RESULT
	// If SG_RESULT reaches 0 before "stall event" , increase SGT value (+63 highest) , if 0 is not reached lower SGT value (-64 lowest)
	// Sudden motor stops can cause a stall event.
	// In this case , increase TCOOLTHRS to increase the duration before stallguard is activated (not included in this function).
}

int TMC5160_Monitor_Stallguard(void)
{
	uint32_t DRV_STATUS;
	int Stall_Flag = 0;

	DRV_STATUS = TMC5160_SPIWrite(0x6F, 0x00000000, 0, &SPICheckValue); //Read (DRV_STATUS)

	Stall_Flag = (DRV_STATUS & (1 << 24)); //bit 24 of DRV_STATUS is Stallguard flag
	Stall_Flag = (Stall_Flag >> 24); // bitshift stall flag is 0 or 1

	for(int y = 32; y > 10; y--) // clear other data but leave SG_RESULT
	{
		DRV_STATUS &= ~(1 << y);
	}

	SG_RESULTS[x] = DRV_STATUS; // see SG_RESULTS in explorer for tuning stallguard (SG_RESULT 0 = stall detected)

	if(Stall_Flag != 0 && SG_RESULTS[x] == 0) //stall detected -> stop motor
	{
		Stall_Flag = 1;
		TMC5160_Init_Stallguard(1,0); // clear stall flag
		TMC5160_Stop();
	}

	else
	{
		Stall_Flag = 0;
	}


	x++;

	if(x > 999) // prevent logging overflow
	{
		x = 0;
	}

	return Stall_Flag;
}

void TMC5160_Set_Home(void)
{
	TMC5160_Drive_Enable(0);//disable motor
	TMC5160_SPIWrite(0x21, 0, 1, &SPICheckValue); //write current pos as home "0"
	TMC5160_Stop();
	TMC5160_Drive_Enable(1);//enable motor
}

void TMC5160_Basic_Rotate(uint8_t Mode, RampConfig *Ramp) // 0 = Velocity Mode + , 1 = Velocity Mode -
{
	//Velocity mode , using VMAX and AMAX

	TMC5160_SPIWrite(0x11, 	0x0000000A, 1, &SPICheckValue); 	// writing value 0x0000000A = 10 = 0.0 to address 9 = 0x11(TPOWERDOWN)
	TMC5160_SPIWrite(0x13, 	0x000001F4, 1, &SPICheckValue); 	// writing value 0x00000000 = 0 = 0.0 to address 10 = 0x13(TPWMTHRS)

	TMC5160_SPIWrite(0x23, Ramp->VSTART, 1, &SPICheckValue); 	// = 1000 = 0.0 to address 15 = 0x23(VSTART)
	TMC5160_SPIWrite(0x26, Ramp->AMAX, 1, &SPICheckValue); 		// = 12800 = 0.0 to address 18 = 0x26(AMAX)
	TMC5160_SPIWrite(0x27, Ramp->VMAX, 1, &SPICheckValue); 		// = 51200 = 0.0 to address 19 = 0x27(VMAX)
	TMC5160_SPIWrite(0x2B, Ramp->VSTOP, 1, &SPICheckValue); 	// = 10 = 0.0 to address 22 = 0x2B(VSTOP)

	if(Mode == 0)
	{
		TMC5160_SPIWrite(0x20, 	0x00000001, 1, &SPICheckValue); 		// writing value 0x00000001 = 0 = 0.0 to address 13 = 0x20(RAMPMODE)VM +
	}

	else if(Mode == 1)
	{
		TMC5160_SPIWrite(0x20, 	0x00000002, 1, &SPICheckValue); 		// writing value 0x00000002 = 0 = 0.0 to address 13 = 0x20(RAMPMODE) VM -
	}
}


void TMC5160_Rotate_To(uint32_t Position, RampConfig *Ramp)
{
	uint32_t Target_Pos = 0;
	uint32_t AMS_Pos = 0;
	uint32_t ENC_Pos = 0;
	uint32_t TMC_Pos = 0;
	int Pos_Reached = 0;

	TMC5160_SPIWrite(0x11, 	0x0000000A, 1, &SPICheckValue); 	// writing value 0x0000000A = 10 = 0.0 to address 9 = 0x11(TPOWERDOWN)
	TMC5160_SPIWrite(0x13, 	0x000001F4, 1, &SPICheckValue); 	// writing value 0x00000000 = 0 = 0.0 to address 10 = 0x13(TPWMTHRS)

	// setup for profile
	TMC5160_SPIWrite(0x23, Ramp->VSTART, 1, &SPICheckValue); 	// = 1000 = 0.0 to address 15 = 0x23(VSTART)
	TMC5160_SPIWrite(0x24, Ramp->A1, 1, &SPICheckValue); 		// = 5600 = 0.0 to address 16 = 0x24(A1)
	TMC5160_SPIWrite(0x25, Ramp->V1, 1, &SPICheckValue); 		// = 12800 = 0.0 to address 17 = 0x25(V1)
	TMC5160_SPIWrite(0x26, Ramp->AMAX, 1, &SPICheckValue); 		// = 12800 = 0.0 to address 18 = 0x26(AMAX)
	TMC5160_SPIWrite(0x27, Ramp->VMAX, 1, &SPICheckValue); 		// = 51200 = 0.0 to address 19 = 0x27(VMAX)
	TMC5160_SPIWrite(0x28, Ramp->DMAX, 1, &SPICheckValue); 		// = 700 = 0.0 to address 20 = 0x28(DMAX)
	TMC5160_SPIWrite(0x2A, Ramp->D1, 1, &SPICheckValue); 		// = 1400 = 0.0 to address 21 = 0x2A(D1)
	TMC5160_SPIWrite(0x2B, Ramp->VSTOP, 1, &SPICheckValue); 	// = 10 = 0.0 to address 22 = 0x2B(VSTOP)

	TMC5160_SPIWrite(0x20, 	0x00000000, 1, &SPICheckValue); 	// writing value 0x00000000 = 0 = 0.0 to address 13 = 0x20(RAMPMODE) MTP
	TMC5160_SPIWrite(0x2D, Position, 1, &SPICheckValue); 		// writing value to address 24 = 0x2D(XTARGET)  1 lap
	TMC5160_SPIWrite(0x21,	0x00000000, 0, &SPICheckValue);		// READ position register

	Target_Pos = Position;

	if(Position == 0) //To fix the first reading when target position = 0
	{
		AMS_Pos = 1;
		ENC_Pos = 1;
		TMC_Pos = 1;
	}


	//Enter loop to check if position is reached
	while(Pos_Reached != 1) // loop until Position is reached
	//while(TMC_Pos != Target_Pos)
	{
		TMC_Pos = TMC5160_Get_Position();

		if(AMS_ENB == 1)// if enabled, check Hall sensor position
		{
			AMS_Pos = ((int32_t)AMS5055_Get_Position(0)) * AMS_Factor * (-1);
		}

		if(ENC_ENB == 1)// if enabled, check Encoder position
		{
			ENC_Pos = ENC_Get_Position();
		}


		if (SPICheckValue & 0x20) //bit 5 = position reached
		{
			/*
			 *	Compare to AMS & ENC sensor data
			 */

			Pos_Reached = 1; //to break loop
		}
	}
 }

uint32_t TMC5160_Get_Position()
{
	uint32_t AngleT = 0;

	AngleT = TMC5160_SPIWrite(0x21, 0x00000000, 0, &SPICheckValue); //read step counter from TMC5160

	return AngleT;
}

void TMC5160_Stop(void)
{
	TMC5160_SPIWrite(0x27,0x00000000, 1, &SPICheckValue); //set VMAX to 0
}

void TMC5160_Drive_Enable(int state)
{
	int Started = 0;
	uint32_t DRV_STATUS = 0;
	uint32_t IOIN = 0;

	if(state == 1) // Enable driver
	{
		while(Started == 0)
		{
			HAL_GPIO_WritePin(GPIOA, DRV_ENN_Pin, 0); // LOW = ON
			HAL_Delay(10);
			DRV_STATUS = TMC5160_SPIWrite(0x6F, 0x00000000, 0, &SPICheckValue);

			if (DRV_STATUS & 0x18003000) // bit 28 / 27 / 13 / 12 short in H-bridge -> toggle DRV_enn
			{
				HAL_GPIO_WritePin(GPIOA, DRV_ENN_Pin, 1); // High = OFF
				HAL_Delay(1000);

			continue;
			}

	    	if (SPICheckValue & 0x02) //bit 1 = driver_error
			{
				HAL_GPIO_WritePin(GPIOA, DRV_ENN_Pin, 1); // HIGH = OFF
				HAL_Delay(1000);

				TMC5160_SPIWrite(0x01, 0x00000002, 1, &SPICheckValue); //clear drive error bit

				continue;
			}

	    	IOIN = TMC5160_SPIWrite(0x04, 0x00000000, 0, &SPICheckValue);


	    	if((IOIN & (1 <<4)) == 0) // check if DRV_ENN is set in software
	    	{
	    		Started = 1; //no issues during motor power up.
	    	}
	    	else
	    	{
	    		HAL_GPIO_WritePin(GPIOA, DRV_ENN_Pin, 1);  // HIGH = OFF if not successfully enabled
	    		HAL_Delay(1000);  // Retry after delay
	    	}
		}
	}

	if(state == 0) // disable drive
	{
		HAL_GPIO_WritePin(GPIOA, DRV_ENN_Pin, 1); // HIGH = OFF
		HAL_Delay(10);
	}
}

void TMC5160_Startup(void)
{
	uint32_t GCONF = 0;
    uint8_t SPI2TxData[5];  // TX data array SPI2
    uint8_t SPI2RxData[5];  // RX data array SPI2

	HAL_GPIO_WritePin(GPIOB, TMC_CS_Pin, 0); // set TMC CS low
	SPI2TxData[0] = 0x81;
	SPI2TxData[1] = 0x00;
	SPI2TxData[2] = 0x00;
	SPI2TxData[3] = 0x00;
	SPI2TxData[4] = 0x01; // clear reset flag
	HAL_SPI_TransmitReceive(&hspi2, SPI2TxData, SPI2RxData, 0x05, 100); //transmit, clear reset flag
	HAL_GPIO_WritePin(GPIOB, TMC_CS_Pin, 1); // set TMC CS high

	HAL_Delay(5);

	HAL_GPIO_WritePin(GPIOB, TMC_CS_Pin, 0); // set TMC CS low
	SPI2TxData[0] = 0x01;
	SPI2TxData[1] = 0x00;
	SPI2TxData[2] = 0x00;
	SPI2TxData[3] = 0x00;
	SPI2TxData[4] = 0x00;
	HAL_SPI_TransmitReceive(&hspi2, SPI2TxData, SPI2RxData, 0x05, 100); //read, reset flag
	HAL_GPIO_WritePin(GPIOB, TMC_CS_Pin, 1); // set TMC CS high

	TMC5160_SPIWrite(0x00, 0x00000008, 1, &SPICheckValue); // writing value 0x00000008 = 8 = 0.0 to address 0 = 0x00(GCONF)
	GCONF = TMC5160_SPIWrite(0x00, 0x00000008, 0, &SPICheckValue); // writing value 0x00000008 = 8 = 0.0 to address 0 = 0x00(GCONF)

	if (!(GCONF & 0x00000008)) //0x00000008 is present, SPI is ok
	{
		//TMC not correctly written, try again.
		TMC5160_SPIWrite(0x00, 0x00000008, 1, &SPICheckValue); // writing value 0x00000008 = 8 = 0.0 to address 0 = 0x00(GCONF)
		GCONF = TMC5160_SPIWrite(0x00, 0x00000008, 0, &SPICheckValue); // writing value 0x00000008 = 8 = 0.0 to address 0 = 0x00(GCONF)
		while (!(GCONF & 0x00000008))
		{
			//TMC not correctly written, try again.
			TMC5160_SPIWrite(0x00, 0x00000008, 1, &SPICheckValue); // writing value 0x00000008 = 8 = 0.0 to address 0 = 0x00(GCONF)
			GCONF = TMC5160_SPIWrite(0x00, 0x00000008, 0, &SPICheckValue); // writing value 0x00000008 = 8 = 0.0 to address 0 = 0x00(GCONF)
		}
	}

	//if reset bit is set, clear it and read if it's cleared
	if (SPI2RxData[0] & 0x01) // reset bit set , rewrite all registers from backup
	{
		HAL_GPIO_WritePin(GPIOB, TMC_CS_Pin, 0); // set TMC CS low
		SPI2TxData[0] = 0x81;
		SPI2TxData[1] = 0x00;
		SPI2TxData[2] = 0x00;
		SPI2TxData[3] = 0x00;
		SPI2TxData[4] = 0x01; // clear reset flag
		HAL_SPI_TransmitReceive(&hspi2, SPI2TxData, SPI2RxData, 0x05, 100); //transmit, clear reset flag
		HAL_GPIO_WritePin(GPIOB, TMC_CS_Pin, 1); // set TMC CS high

		HAL_Delay(5);

		HAL_GPIO_WritePin(GPIOB, TMC_CS_Pin, 0); // set TMC CS low
		SPI2TxData[0] = 0x01;
		SPI2TxData[1] = 0x00;
		SPI2TxData[2] = 0x00;
		SPI2TxData[3] = 0x00;
		SPI2TxData[4] = 0x00;
		HAL_SPI_TransmitReceive(&hspi2, SPI2TxData, SPI2RxData, 0x05, 100); //read, reset flag
	}

	TMC_Boot = 1;
}

uint32_t TMC5160_SPIWrite(uint8_t Address, uint32_t Value, int Action, uint8_t *SPICheck)
{
    uint8_t SPI2TxData[5];  // TX data array SPI2
    uint8_t SPI2RxData[5];  // RX data array SPI2
    uint32_t SPI2Rx = 0;
    int index; //register index

    HAL_GPIO_WritePin(GPIOB, TMC_CS_Pin, 0); // set TMC CS low

    if (Action == 1) // Write
    {
        SPI2TxData[0] = Address + 0x80;
    }
    else // Read
    {
        SPI2TxData[0] = Address;
    }

    SPI2TxData[1] = Value >> 24;
    SPI2TxData[2] = Value >> 16;
    SPI2TxData[3] = Value >> 8;
    SPI2TxData[4] = Value;

    HAL_SPI_TransmitReceive(&hspi2, SPI2TxData, SPI2RxData, 0x05, 100);
    *SPICheck = SPI2RxData[0]; // Update SPICheck

	if (Action != 1)
	{
		//ignore first response so toggle CS pin
		HAL_GPIO_WritePin(GPIOB, TMC_CS_Pin, 1); // set TMC CS high
		HAL_Delay(2); //5
		HAL_GPIO_WritePin(GPIOB, TMC_CS_Pin, 0); // set TMC CS low
		HAL_SPI_TransmitReceive(&hspi2, SPI2TxData, SPI2RxData, 0x05, 100); // overwrite the data

	}

    *SPICheck = SPI2RxData[0]; // Update SPICheck

    SPI2Rx += (SPI2RxData[1] << 24);
    SPI2Rx += (SPI2RxData[2] << 16);
    SPI2Rx += (SPI2RxData[3] << 8);
    SPI2Rx += (SPI2RxData[4] << 0);

    HAL_GPIO_WritePin(GPIOB, TMC_CS_Pin, 1); // set TMC CS high

    if (Action == 1) // Write action, so store adress and value for backup
	{
		// Check if the address already exists in the array
		for (index = 0; index < numWriteActions; index++)
		{
			if (writeAddresses[index] == Address)
			{
				// Overwrite the existing value
				writeValues[index] = Value;
				break;
			}
		}

		// If the address doesn't exist, add it to the arrays
		if (index == numWriteActions)
		{
			if (numWriteActions < MAX_WRITE_ACTIONS)
			{
				writeAddresses[numWriteActions] = Address;
				writeValues[numWriteActions] = Value;
				numWriteActions++;
			}
			else
			{
				// Handle the case where the maximum number of write actions is exceeded
				// MAX_WRITE_ACTIONS is larger then number of registers on the TMC5160
			}
		}
	}

	if (SPI2RxData[0] & 0x01 && TMC_Boot == 1) // reset bit set , rewrite all registers from backup
	{
		//CLEAR RESET FLAG
		HAL_GPIO_WritePin(GPIOB, TMC_CS_Pin, 0); // set TMC CS low
		SPI2TxData[0] = 0x81;
		SPI2TxData[1] = 0x00;
		SPI2TxData[2] = 0x00;
		SPI2TxData[3] = 0x00;
		SPI2TxData[4] = 0x01; // clear reset
		HAL_SPI_TransmitReceive(&hspi2, SPI2TxData, SPI2RxData, 0x05, 100); // overwrite the data
		HAL_GPIO_WritePin(GPIOB, TMC_CS_Pin, 1); // set TMC CS high

		HAL_Delay(5);

		HAL_GPIO_WritePin(GPIOB, TMC_CS_Pin, 0); // set TMC CS low
		SPI2TxData[0] = 0x01;
		SPI2TxData[1] = 0x00;
		SPI2TxData[2] = 0x00;
		SPI2TxData[3] = 0x00;
		SPI2TxData[4] = 0x00;
		HAL_SPI_TransmitReceive(&hspi2, SPI2TxData, SPI2RxData, 0x05, 100); // overwrite the data
		HAL_GPIO_WritePin(GPIOB, TMC_CS_Pin, 1); // set TMC CS high

		//WRITE BACKUP REGISTERS AS THEY HAVE BEEN RESET
		ResetWriteAction[ResetCount] = numWriteActions;
		ResetCount++; //times reset occured

		for (int i = 0; i < numWriteActions; i++)
		{
			SPI2TxData[0] = writeAddresses[i] + 0x80;
			SPI2TxData[1] = writeValues[i] >> 24;
			SPI2TxData[2] = writeValues[i] >> 16;
			SPI2TxData[3] = writeValues[i] >> 8;
			SPI2TxData[4] = writeValues[i];

			HAL_GPIO_WritePin(GPIOB, TMC_CS_Pin, 0); // set TMC CS low
			HAL_SPI_TransmitReceive(&hspi2, SPI2TxData, SPI2RxData, 0x05, 100);
			HAL_GPIO_WritePin(GPIOB, TMC_CS_Pin, 1); // set TMC CS high
		}
	}

	return SPI2Rx;
}

void AMS5055_Basic_Init(void)
{
	uint32_t Offset[5];

	HAL_GPIO_WritePin(GPIOA,AMS_CS_Pin,0); // set TMC CS low
	HAL_Delay(5);
	HAL_GPIO_WritePin(GPIOA,AMS_CS_Pin,1); // set TMC CS high
	AMS5055_Get_Position(0); //discard the first measurement

	HAL_GPIO_WritePin(GPIOA,AMS_CS_Pin,0); // set TMC CS low
	HAL_Delay(5);
	HAL_GPIO_WritePin(GPIOA,AMS_CS_Pin,1); // set TMC CS high
	Offset[0] = AMS5055_Get_Position(1);

	HAL_GPIO_WritePin(GPIOA,AMS_CS_Pin,0); // set TMC CS low
	HAL_Delay(5);
	HAL_GPIO_WritePin(GPIOA,AMS_CS_Pin,1); // set TMC CS high
	Offset[1] = AMS5055_Get_Position(1);

	HAL_GPIO_WritePin(GPIOA,AMS_CS_Pin,0); // set TMC CS low
	HAL_Delay(5);
	HAL_GPIO_WritePin(GPIOA,AMS_CS_Pin,1); // set TMC CS high
	Offset[2] = AMS5055_Get_Position(1);

	HAL_GPIO_WritePin(GPIOA,AMS_CS_Pin,0); // set TMC CS low
	HAL_Delay(5);
	HAL_GPIO_WritePin(GPIOA,AMS_CS_Pin,1); // set TMC CS high
	Offset[3] = AMS5055_Get_Position(1);

	HAL_GPIO_WritePin(GPIOA,AMS_CS_Pin,0); // set TMC CS low
	HAL_Delay(5);
	HAL_GPIO_WritePin(GPIOA,AMS_CS_Pin,1); // set TMC CS high
	Offset[4] = AMS5055_Get_Position(1);

	AMSoffset = ((Offset[0] + Offset[1] + Offset[2] + Offset[3] + Offset[4]) / 5); //Take average of 5 measurements for Offset
}

uint32_t AMS5055_Get_Position(int Calibration)
{
	uint32_t Angle = 0;
	int AlarmLo = 0;
	int AlarmHi = 0;

	AMS5055_SPIWriteInt(ANGULAR_DATA,1);

	while(AMS_Ready != 1) //wacht op INT
	{
	}

	Angle = AMS5055_SPIWriteInt(NOP,1);

	AlarmHi = (Angle >> 12) & 0x01; // AlamHi = b14
	AlarmLo = (Angle >> 13) & 0x01; // AlarmLo = b15


	if(AlarmLo == 1 && AlarmHi == 1)
	{
		//do X
	}

	if(AlarmLo == 1 && AlarmHi == 0)
	{
		//Magnetic field too weak, increase AGC ?
	}

	if(AlarmLo == 0 && AlarmHi == 1)
	{
		//magnetic field too strong , lower AGC ?
	}

	// remove first 2 and last 2 bits
	Angle &= ~(1 << 14);
	Angle &= ~(1 << 15);
	Angle = (Angle >> 2);

	/* Currently 12 bit Angle data = 0 t/m 4096
	 *
	 * To achieve:
	 * 32 bit Angle data = -2.147.483.648 t/m 2.147.483.647
	 */

	if(Calibration == 1) //return Angle as is
	{
		AMS_Ready = 0;
		return Angle;
	}

	else //perform calculation and log the value
	{

		if (Angle >= AMSoffset)
		{
			Angle -= AMSoffset; // AMS is not calibrated, so angle needs to be fixed
		}
		else if(Angle < AMSoffset)
		{
			Angle = (Angle + 4095) - AMSoffset;
		}

		roughAngles[AmsPos] = Angle;

		if (Angle > 4050 && Previous_AMS_Angle < 50)
		{
		 		Wrapper -= 1;
		}

		else if (Angle < 50  && Previous_AMS_Angle > 4050)
		{
		 		Wrapper += 1;
		}

		Previous_AMS_Angle = Angle;


		if(Wrapper > 0)
		{
			Angle = Angle + (Wrapper * 4095);
		}

		else if(Wrapper < 0)
		{
		 	Angle = Angle + (Wrapper * 4095);

		}

		//double Normalized_Angle = (double)Angle / 51200.0;
		//Angle = (int32_t)(Normalized_Angle * (INT32_MAX - INT32_MIN) + INT32_MIN);

		AMSAngles[AmsPos] = Angle; //logging of AMS angle

		if(AmsPos >= 999)// Prevent overflow
		{
			AmsPos = 0;
		}
		else
		{
			AmsPos ++;
		}

		AMS_Ready = 0;
		return Angle;
	}
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

	HAL_GPIO_WritePin(GPIOA,AMS_CS_Pin,0); // set AMS CS low


	  SPI1Tx = (Address << 1);

	  if (Action == 1) //READ
	  {
		  SPI1Tx = SPI1Tx | 0x8000;
	  }

	  SPI1Tx = SPI1Tx | AMSParity(SPI1Tx);

	  SPI1TxData[0] = SPI1Tx >> 8;
	  SPI1TxData[1] = SPI1Tx;

	  HAL_SPI_TransmitReceive(&hspi1, SPI1TxData, SPI1RxData, 0x02, 100);

	  SPI1Rx += (SPI1RxData[0] << 8);	//<< 8
	  SPI1Rx += (SPI1RxData[1] << 0);	//<< 0

	  HAL_GPIO_WritePin(GPIOA,AMS_CS_Pin,1); // set AMS CS high

	  return SPI1Rx;
}

int32_t ENC_Get_Position(void)
{
	int32_t Enc_Position = TMC5160_SPIWrite(0x39, 0x00000000, 0, &SPICheckValue); // read encoder position
	float Fenc_Pos = Enc_Position;

	Fenc_Pos = Fenc_Pos * ENC_Factor;
	Enc_Position = (int32_t)Fenc_Pos;

	Enc_Position = ((Enc_Position) * (-1)); // to fix orientation

	return Enc_Position;
}

void ENC_Start_position(void) //set current position to start position
{
	TMC5160_SPIWrite(0x39, 0x00000000, 1, &SPICheckValue); //write current position to 0
}


void Toggle_OUT(int port ,uint16_t time)
{
	if(port == 1)//Enable OUT1 for X sec (24V)
	{
	 HAL_GPIO_WritePin(GPIOB,EXT_OUT_1_Pin,1);
	 HAL_Delay(time);
	 HAL_GPIO_WritePin(GPIOB,EXT_OUT_1_Pin,0);
	}

	if(port == 2) // Enable OUT2 for X sec (24V)
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


