/*
 * TMC5160.c
 *
 *  Created on: 03.07.2017
 *      Author: LK
 */

#include "TMC5160.h"

//void TMC5160_SPIWrite(uint8_t Adress, uint32_t Value, int Action);
//void TMC5160_Stop(void);

// => SPI wrapper
// Send [length] bytes stored in the [data] array over SPI and overwrite [data]
// with the reply. The first byte sent/received is data[0].
extern void tmc5160_readWriteArray(uint8_t channel, uint8_t *data, size_t length);
// <= SPI wrapper

// Writes (x1 << 24) | (x2 << 16) | (x3 << 8) | x4 to the given address
void tmc5160_writeDatagram(TMC5160TypeDef *tmc5160, uint8_t address, uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4)
{
	uint8_t data[5] = { address | TMC5160_WRITE_BIT, x1, x2, x3, x4 };
	tmc5160_readWriteArray(tmc5160->config->channel, &data[0], 5);

	int32_t value = ((uint32_t)x1 << 24) | ((uint32_t)x2 << 16) | (x3 << 8) | x4;

	// Write to the shadow register and mark the register dirty
	address = TMC_ADDRESS(address);
	tmc5160->config->shadowRegister[address] = value;
	tmc5160->registerAccess[address] |= TMC_ACCESS_DIRTY;
}

// Write an integer to the given address
void tmc5160_writeInt(TMC5160TypeDef *tmc5160, uint8_t address, int32_t value)
{
	tmc5160_writeDatagram(tmc5160, address, BYTE(value, 3), BYTE(value, 2), BYTE(value, 1), BYTE(value, 0));
}

// Read an integer from the given address
int32_t tmc5160_readInt(TMC5160TypeDef *tmc5160, uint8_t address)
{
	address = TMC_ADDRESS(address);

	// register not readable -> shadow register copy
	if(!TMC_IS_READABLE(tmc5160->registerAccess[address]))
		return tmc5160->config->shadowRegister[address];

	uint8_t data[5] = { 0, 0, 0, 0, 0 };

	data[0] = address;
	tmc5160_readWriteArray(tmc5160->config->channel, &data[0], 5);

	data[0] = address;
	tmc5160_readWriteArray(tmc5160->config->channel, &data[0], 5);

	return ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) | (data[3] << 8) | data[4];
}

// Initialize a TMC5160 IC.
// This function requires:
//     - tmc5160: The pointer to a TMC5160TypeDef struct, which represents one IC
//     - channel: The channel index, which will be sent back in the SPI callback
//     - config: A ConfigurationTypeDef struct, which will be used by the IC
//     - registerResetState: An int32_t array with 128 elements. This holds the values to be used for a reset.
void tmc5160_init(TMC5160TypeDef *tmc5160, uint8_t channel, ConfigurationTypeDef *config, const int32_t *registerResetState)
{
	/*
	tmc5160->velocity  = 0;
	tmc5160->oldTick   = 0;
	tmc5160->oldX      = 0;

	tmc5160->config               = config;
	tmc5160->config->callback     = NULL;
	tmc5160->config->channel      = channel;
	tmc5160->config->configIndex  = 0;
	tmc5160->config->state        = CONFIG_READY;

	size_t i;
	for(i = 0; i < TMC5160_REGISTER_COUNT; i++)
	{
		tmc5160->registerAccess[i]      = tmc5160_defaultRegisterAccess[i];
		tmc5160->registerResetState[i]  = registerResetState[i];
	}*/
}


void TMC5160_Basic_Init(void)
{
		TMC5160_Stop();
		TMC5160_SPIWrite(0x00, 	0x00000008, 1); 		// writing value 0x00000008 = 8 = 0.0 to address 0 = 0x00(GCONF)
		TMC5160_SPIWrite(0x00, 	0x00000008, 0); 		// writing value 0x00000008 = 8 = 0.0 to address 0 = 0x00(GCONF)
		TMC5160_SPIWrite(0x03, 	0x00000000, 1); 		// writing value 0x00000000 = 0 = 0.0 to address 1 = 0x03(SLAVECONF)
		TMC5160_SPIWrite(0x05, 	0x00000000, 1); 		// writing value 0x00000000 = 0 = 0.0 to address 2 = 0x05(X_COMPARE)
		TMC5160_SPIWrite(0x06, 	0x00000000, 1); 		// writing value 0x00000000 = 0 = 0.0 to address 3 = 0x06(OTP_PROG)
		TMC5160_SPIWrite(0x08, 	0x0000000F, 1); 		// writing value 0x0000000F = 15 = 0.0 to address 4 = 0x08(FACTORY_CONF)
		TMC5160_SPIWrite(0x09, 	0x00010606, 1); 		// writing value 0x00010606 = 67078 = 0.0 to address 5 = 0x09(SHORT_CONF)
		TMC5160_SPIWrite(0x0A, 	0x00080400, 1); 		// writing value 0x00080400 = 525312 = 0.0 to address 6 = 0x0A(DRV_CONF)
		TMC5160_SPIWrite(0x0B, 	0x00000000, 1); 		// writing value 0x00000000 = 0 = 0.0 to address 7 = 0x0B(GLOBAL_SCALER)
		TMC5160_SPIWrite(0x10, 	0x00070A03, 1); 		// writing value 0x00070A03 = 461315 = 0.0 to address 8 = 0x10(IHOLD_IRUN)
		TMC5160_SPIWrite(0x11, 	0x0000000A, 1); 		// writing value 0x0000000A = 10 = 0.0 to address 9 = 0x11(TPOWERDOWN)
		TMC5160_SPIWrite(0x13, 	0x00000000, 1); 		// writing value 0x00000000 = 0 = 0.0 to address 10 = 0x13(TPWMTHRS)
		TMC5160_SPIWrite(0x14, 	0x00000000, 1); 		// writing value 0x00000010 = 16 = 0.0 to address 11 = 0x14(TCOOLTHRS)
		TMC5160_SPIWrite(0x15, 	0x00000000, 1); 		// writing value 0x00000000 = 0 = 0.0 to address 12 = 0x15(THIGH)
		TMC5160_SPIWrite(0x20, 	0x00000001, 1); 		// writing value 0x00000001 = 1 = 0.0 to address 13 = 0x20(RAMPMODE)
		TMC5160_SPIWrite(0x21, 	0x00000000, 1); 		// writing value 0x0016DFDD = 1499101 = 0.0 to address 14 = 0x21(XACTUAL)
		TMC5160_SPIWrite(0x23, 	0x000003E8, 1); 		// writing value 0x000003E8 = 1000 = 0.0 to address 15 = 0x23(VSTART)
		TMC5160_SPIWrite(0x24, 	0x000003E8, 1); 		// writing value 0x000003E8 = 1000 = 0.0 to address 16 = 0x24(A1)
		TMC5160_SPIWrite(0x25, 	0x00000000, 1); 		// writing value 0x00000000 = 0 = 0.0 to address 17 = 0x25(V1)
		TMC5160_SPIWrite(0x26, 	0x000009C4, 1); 		// writing value 0x000009C4 = 2500 = 0.0 to address 18 = 0x26(AMAX)
		TMC5160_SPIWrite(0x27, 	0x00000000, 1); 		// writing value 0x00000000 = 0 = 0.0 to address 19 = 0x27(VMAX)
		TMC5160_SPIWrite(0x28, 	0x000002BC, 1); 		// writing value 0x000002BC = 700 = 0.0 to address 20 = 0x28(DMAX)
		TMC5160_SPIWrite(0x2A, 	0x00000578, 1); 		// writing value 0x00000578 = 1400 = 0.0 to address 21 = 0x2A(D1)
		TMC5160_SPIWrite(0x2B, 	0x0000000A, 1); 		// writing value 0x0000000A = 10 = 0.0 to address 22 = 0x2B(VSTOP)
		TMC5160_SPIWrite(0x2C, 	0x00000000, 1); 		// writing value 0x00000000 = 0 = 0.0 to address 23 = 0x2C(TZEROWAIT)
		TMC5160_SPIWrite(0x2D, 	0xFFFF9C00, 1); 		// writing value 0xFFFF9C00 = 0 = 0.0 to address 24 = 0x2D(XTARGET)
		TMC5160_SPIWrite(0x33, 	0x00000000, 1); 		// writing value 0x00000000 = 0 = 0.0 to address 25 = 0x33(VDCMIN)
		TMC5160_SPIWrite(0x34, 	0x00000000, 1); 		// writing value 0x00000000 = 0 = 0.0 to address 26 = 0x34(SW_MODE)
		TMC5160_SPIWrite(0x38, 	0x00000000, 1); 		// writing value 0x00000000 = 0 = 0.0 to address 27 = 0x38(ENCMODE)
		TMC5160_SPIWrite(0x39, 	0x00000000, 1); 		// writing value 0x00000000 = 0 = 0.0 to address 28 = 0x39(X_ENC)
		TMC5160_SPIWrite(0x3A, 	0x00010000, 1); 		// writing value 0x00010000 = 65536 = 0.0 to address 29 = 0x3A(ENC_CONST)
		TMC5160_SPIWrite(0x3D, 	0x00000000, 1); 		// writing value 0x00000000 = 0 = 0.0 to address 30 = 0x3D(ENC_DEVIATION)
		TMC5160_SPIWrite(0x60, 	0xAAAAB554, 1); 		// writing value 0xAAAAB554 = 0 = 0.0 to address 31 = 0x60(MSLUT[0])
		TMC5160_SPIWrite(0x61, 	0x4A9554AA, 1); 		// writing value 0x4A9554AA = 1251300522 = 0.0 to address 32 = 0x61(MSLUT[1])
		TMC5160_SPIWrite(0x62, 	0x24492929, 1); 		// writing value 0x24492929 = 608774441 = 0.0 to address 33 = 0x62(MSLUT[2])
		TMC5160_SPIWrite(0x63, 	0x10104222, 1); 		// writing value 0x10104222 = 269500962 = 0.0 to address 34 = 0x63(MSLUT[3])
		TMC5160_SPIWrite(0x64, 	0xFBFFFFFF, 1); 		// writing value 0xFBFFFFFF = 0 = 0.0 to address 35 = 0x64(MSLUT[4])
		TMC5160_SPIWrite(0x65, 	0xB5BB777D, 1); 		// writing value 0xB5BB777D = 0 = 0.0 to address 36 = 0x65(MSLUT[5])
		TMC5160_SPIWrite(0x66, 	0x49295556, 1); 		// writing value 0x49295556 = 1227445590 = 0.0 to address 37 = 0x66(MSLUT[6])
		TMC5160_SPIWrite(0x67, 	0x00404222, 1); 		// writing value 0x00404222 = 4211234 = 0.0 to address 38 = 0x67(MSLUT[7])
		TMC5160_SPIWrite(0x68, 	0xFFFF8056, 1); 		// writing value 0xFFFF8056 = 0 = 0.0 to address 39 = 0x68(MSLUTSEL)
		TMC5160_SPIWrite(0x69, 	0x00F70000, 1); 		// writing value 0x00F70000 = 16187392 = 0.0 to address 40 = 0x69(MSLUTSTART)
		TMC5160_SPIWrite(0x6C, 	0x00410153, 1); 		// writing value 0x00410103 = 4260099 = 0.0 to address 41 = 0x6C(CHOPCONF)
		TMC5160_SPIWrite(0x6D, 	0x00000000, 1); 		// writing value 0x00000000 = 0 = 0.0 to address 42 = 0x6D(COOLCONF)
		TMC5160_SPIWrite(0x6E, 	0x00000000, 1); 		// writing value 0x00000000 = 0 = 0.0 to address 43 = 0x6E(DCCTRL)
		TMC5160_SPIWrite(0x70, 	0xC40C001E, 1); 		// writing value 0xC40C001E = 0 = 0.0 to address 44 = 0x70(PWMCONF)
}


void TMC5160_Basic_Rotate(uint8_t Mode) // 0=Velocity Mode + , 1=Velocity Mode -
{
	//getting started example from Doc, settings not altered from dev kit test
	//Velocity mode , using VMAX and AMAX

	TMC5160_SPIWrite(0x6C, 	0x00410153, 1); 		// writing value 0x00410103 = 4260099 = 0.0 to address 41 = 0x6C(CHOPCONF)
	TMC5160_SPIWrite(0x10, 	0x00070A03, 1); 		// writing value 0x00070A03 = 461315 = 0.0 to address 8 = 0x10(IHOLD_IRUN)
	TMC5160_SPIWrite(0x11, 	0x0000000A, 1); 		// writing value 0x0000000A = 10 = 0.0 to address 9 = 0x11(TPOWERDOWN)
	TMC5160_SPIWrite(0x00, 	0x00000008, 1); 		// writing value 0x00000008 = 8 = 0.0 to address 0 = 0x00(GCONF)
	TMC5160_SPIWrite(0x13, 	0x000001F4, 1); 		// writing value 0x00000000 = 0 = 0.0 to address 10 = 0x13(TPWMTHRS)


	TMC5160_SPIWrite(0x24, 	0x000015E0, 1); 		// writing value 0x000003E8 = 1 000 = 0.0 to address 16 = 0x24(A1)
	TMC5160_SPIWrite(0x25, 	0x00003200, 1); 		// writing value 0x00000000 = 50 000 = 0.0 to address 17 = 0x25(V1)
	TMC5160_SPIWrite(0x26, 	0x00003200, 1); 		// writing value 0x000009C4 = 500 = 0.0 to address 18 = 0x26(AMAX)
	TMC5160_SPIWrite(0x27, 	0x0000C800, 1); 		// writing value 0x00000000 = 200 000 = 0.0 to address 19 = 0x27(VMAX)
	TMC5160_SPIWrite(0x28, 	0x000002BC, 1); 		// writing value 0x000002BC = 700 = 0.0 to address 20 = 0x28(DMAX)
	TMC5160_SPIWrite(0x2A, 	0x00000578, 1); 		// writing value 0x00000578 = 1 400 = 0.0 to address 21 = 0x2A(D1)
	TMC5160_SPIWrite(0x2B, 	0x0000000A, 1); 		// writing value 0x0000000A = 10 = 0.0 to address 22 = 0x2B(VSTOP)

	if(Mode == 0)
	{
		TMC5160_SPIWrite(0x20, 	0x00000001, 1); 		// writing value 0x00000001 = 0 = 0.0 to address 13 = 0x20(RAMPMODE)VM +
	}

	else if(Mode == 1)
	{
		TMC5160_SPIWrite(0x20, 	0x00000002, 1); 		// writing value 0x00000002 = 0 = 0.0 to address 13 = 0x20(RAMPMODE) VM -
	}
}


void TMC5160_Rotate_To(uint32_t Position)
{
	uint32_t Actual_Position = 0;
	uint32_t Enc_Position[100];
	uint32_t Reg[100];
	int h = 0;

	TMC5160_SPIWrite(0x6C, 	0x00410153, 1); 		// writing value 0x00410103 = 4260099 = 0.0 to address 41 = 0x6C(CHOPCONF)
	TMC5160_SPIWrite(0x10, 	0x00070A03, 1); 		// writing value 0x00070A03 = 461315 = 0.0 to address 8 = 0x10(IHOLD_IRUN)
	TMC5160_SPIWrite(0x11, 	0x0000000A, 1); 		// writing value 0x0000000A = 10 = 0.0 to address 9 = 0x11(TPOWERDOWN)
	TMC5160_SPIWrite(0x00, 	0x00000008, 1); 		// writing value 0x00000008 = 8 = 0.0 to address 0 = 0x00(GCONF)
	TMC5160_SPIWrite(0x13, 	0x000001F4, 1); 		// writing value 0x00000000 = 0 = 0.0 to address 10 = 0x13(TPWMTHRS)


	TMC5160_SPIWrite(0x24, 	0x000003E8, 1); 		// writing value 0x000003E8 = 1 000 = 0.0 to address 16 = 0x24(A1)
	TMC5160_SPIWrite(0x25, 	0x0000C350, 1); 		// writing value 0x00000000 = 50 000 = 0.0 to address 17 = 0x25(V1)
	TMC5160_SPIWrite(0x26, 	0x000001F4, 1); 		// writing value 0x000009C4 = 500 = 0.0 to address 18 = 0x26(AMAX)
	TMC5160_SPIWrite(0x27, 	0x00030D40, 1); 		// writing value 0x00000000 = 200 000 = 0.0 to address 19 = 0x27(VMAX)
	TMC5160_SPIWrite(0x28, 	0x000002BC, 1); 		// writing value 0x000002BC = 700 = 0.0 to address 20 = 0x28(DMAX)
	TMC5160_SPIWrite(0x2A, 	0x00000578, 1); 		// writing value 0x00000578 = 1 400 = 0.0 to address 21 = 0x2A(D1)
	TMC5160_SPIWrite(0x2B, 	0x0000000A, 1); 		// writing value 0x0000000A = 10 = 0.0 to address 22 = 0x2B(VSTOP)
	TMC5160_SPIWrite(0x20, 	0x00000000, 1); 		// writing value 0x00000000 = 0 = 0.0 to address 13 = 0x20(RAMPMODE) MTP
	TMC5160_SPIWrite(0x2D, Position, 1); 			// writing value to address 24 = 0x2D(XTARGET)  1 lap
	TMC5160_SPIWrite(0x21,	0x00000000, 0);
	Actual_Position = TMC5160_SPIWrite(0x21,	0x00000000, 0);

	// enter loop to check if position is reached

	while(Actual_Position != Position) // || ~Actual_Position != Position
	{
		Actual_Position = TMC5160_SPIWrite(0x21, 0x00000000, 0); // read step counter position
		HAL_Delay(5);
		Enc_Position[h] = TMC5160_SPIWrite(0x39, 0x00000000, 0); // read encoder position
		HAL_Delay(5);
		h++;
		AMS5055_Get_Position(); // read Hall sensor position

		if(h == 100)
		{
			h = 0;
		}
	}
}

void TMC5160_Stop(void)
{
	TMC5160_SPIWrite(0x26, 	0x00000000, 1); 		// writing value 0x000009C4 = 500 = 0.0 to address 18 = 0x26(AMAX)
	TMC5160_SPIWrite(0x27, 	0x00000000, 1); 		// writing value 0x00000000 = 200 000 = 0.0 to address 19 = 0x27(VMAX)
	TMC5160_SPIWrite(0x24, 	0x00000000, 1); 		// writing value 0x000003E8 = 1 000 = 0.0 to address 16 = 0x24(A1)
	TMC5160_SPIWrite(0x25, 	0x00000000, 1); 		// writing value 0x00000000 = 50 000 = 0.0 to address 17 = 0x25(V1)
	TMC5160_SPIWrite(0x28, 	0x00000000, 1); 		// writing value 0x000002BC = 700 = 0.0 to address 20 = 0x28(DMAX)
	TMC5160_SPIWrite(0x2A, 	0x00000010, 1); 		// writing value 0x00000578 = 1 400 = 0.0 to address 21 = 0x2A(D1)
	TMC5160_SPIWrite(0x2B, 	0x00000010, 1); 		// writing value 0x0000000A = 10 = 0.0 to address 22 = 0x2B(VSTOP)
}



// Fill the shadow registers of hardware preset non-readable registers
// Only needed if you want to 'read' those registers e.g to display the value
// in the TMCL IDE register browser
void tmc5160_fillShadowRegisters(TMC5160TypeDef *tmc5160)
{
	// Check if we have constants defined
	if(ARRAY_SIZE(tmc5160_RegisterConstants) == 0)
		return;

	size_t i, j;
	for(i = 0, j = 0; i < TMC5160_REGISTER_COUNT; i++)
	{
		// We only need to worry about hardware preset, write-only registers
		// that have not yet been written (no dirty bit) here.
		if(tmc5160->registerAccess[i] != TMC_ACCESS_W_PRESET)
			continue;

		// Search the constant list for the current address. With the constant
		// list being sorted in ascended order, we can walk through the list
		// until the entry with an address equal or greater than i
		while(j < ARRAY_SIZE(tmc5160_RegisterConstants) && (tmc5160_RegisterConstants[j].address < i))
			j++;

		// Abort when we reach the end of the constant list
		if (j == ARRAY_SIZE(tmc5160_RegisterConstants))
			break;

		// If we have an entry for our current address, write the constant
		if(tmc5160_RegisterConstants[j].address == i)
		{
			tmc5160->config->shadowRegister[i] = tmc5160_RegisterConstants[j].value;
		}
	}
}

// Reset the TMC5160.
uint8_t tmc5160_reset(TMC5160TypeDef *tmc5160)
{
	if(tmc5160->config->state != CONFIG_READY)
		return false;

	// Reset the dirty bits and wipe the shadow registers
	size_t i;
	for(i = 0; i < TMC5160_REGISTER_COUNT; i++)
	{
		tmc5160->registerAccess[i] &= ~TMC_ACCESS_DIRTY;
		tmc5160->config->shadowRegister[i] = 0;
	}

	tmc5160->config->state        = CONFIG_RESET;
	tmc5160->config->configIndex  = 0;

	return true;
}

// Restore the TMC5160 to the state stored in the shadow registers.
// This can be used to recover the IC configuration after a VM power loss.
uint8_t tmc5160_restore(TMC5160TypeDef *tmc5160)
{
	if(tmc5160->config->state != CONFIG_READY)
		return false;

	tmc5160->config->state        = CONFIG_RESTORE;
	tmc5160->config->configIndex  = 0;

	return true;
}

// Change the values the IC will be configured with when performing a reset.
void tmc5160_setRegisterResetState(TMC5160TypeDef *tmc5160, const int32_t *resetState)
{
	size_t i;
	for(i = 0; i < TMC5160_REGISTER_COUNT; i++)
	{
		tmc5160->registerResetState[i] = resetState[i];
	}
}

// Register a function to be called after completion of the configuration mechanism
void tmc5160_setCallback(TMC5160TypeDef *tmc5160, tmc5160_callback callback)
{
	tmc5160->config->callback = (tmc_callback_config) callback;
}

// Helper function: Configure the next register.
static void writeConfiguration(TMC5160TypeDef *tmc5160)
{
	uint8_t *ptr = &tmc5160->config->configIndex;
	const int32_t *settings;

	if(tmc5160->config->state == CONFIG_RESTORE)
	{
		settings = tmc5160->config->shadowRegister;
		// Find the next restorable register
		while((*ptr < TMC5160_REGISTER_COUNT) && !TMC_IS_RESTORABLE(tmc5160->registerAccess[*ptr]))
		{
			(*ptr)++;
		}
	}
	else
	{
		settings = tmc5160->registerResetState;
		// Find the next resettable register
		while((*ptr < TMC5160_REGISTER_COUNT) && !TMC_IS_RESETTABLE(tmc5160->registerAccess[*ptr]))
		{
			(*ptr)++;
		}
	}

	if(*ptr < TMC5160_REGISTER_COUNT)
	{
		tmc5160_writeInt(tmc5160, *ptr, settings[*ptr]);
		(*ptr)++;
	}
	else // Finished configuration
	{
		if(tmc5160->config->callback)
		{
			((tmc5160_callback)tmc5160->config->callback)(tmc5160, tmc5160->config->state);
		}

		tmc5160->config->state = CONFIG_READY;
	}
}

// Call this periodically
void tmc5160_periodicJob(TMC5160TypeDef *tmc5160, uint32_t tick)
{
	if(tmc5160->config->state != CONFIG_READY)
	{
		writeConfiguration(tmc5160);
		return;
	}

	int32_t XActual;
	uint32_t tickDiff;

	// Calculate velocity v = dx/dt
	if((tickDiff = tick - tmc5160->oldTick) >= 5)
	{
		XActual = tmc5160_readInt(tmc5160, TMC5160_XACTUAL);
		// ToDo CHECK 2: API Compatibility - write alternative algorithm w/o floating point? (LH)
		tmc5160->velocity = (uint32_t) ((float32_t) ((XActual - tmc5160->oldX) / (float32_t) tickDiff) * (float32_t) 1048.576);

		tmc5160->oldX     = XActual;
		tmc5160->oldTick  = tick;
	}
}

// Rotate with a given velocity (to the right)
void tmc5160_rotate(TMC5160TypeDef *tmc5160, int32_t velocity)
{
	// Set absolute velocity
	tmc5160_writeInt(tmc5160, TMC5160_VMAX, abs(velocity));
	// Set direction
	tmc5160_writeInt(tmc5160, TMC5160_RAMPMODE, (velocity >= 0) ? TMC5160_MODE_VELPOS : TMC5160_MODE_VELNEG);
}

// Rotate to the right
void tmc5160_right(TMC5160TypeDef *tmc5160, uint32_t velocity)
{
	tmc5160_rotate(tmc5160, velocity);
}

// Rotate to the left
void tmc5160_left(TMC5160TypeDef *tmc5160, uint32_t velocity)
{
	tmc5160_rotate(tmc5160, -velocity);
}

// Stop moving
void tmc5160_stop(TMC5160TypeDef *tmc5160)
{
	tmc5160_rotate(tmc5160, 0);
}

// Move to a specified position with a given velocity
void tmc5160_moveTo(TMC5160TypeDef *tmc5160, int32_t position, uint32_t velocityMax)
{
	tmc5160_writeInt(tmc5160, TMC5160_RAMPMODE, TMC5160_MODE_POSITION);

	// VMAX also holds the target velocity in velocity mode.
	// Re-write the position mode maximum velocity here.
	tmc5160_writeInt(tmc5160, TMC5160_VMAX, velocityMax);

	tmc5160_writeInt(tmc5160, TMC5160_XTARGET, position);
}

// Move by a given amount with a given velocity
// This function will write the absolute target position to *ticks
void tmc5160_moveBy(TMC5160TypeDef *tmc5160, int32_t *ticks, uint32_t velocityMax)
{
	// determine actual position and add numbers of ticks to move
	*ticks += tmc5160_readInt(tmc5160, TMC5160_XACTUAL);

	tmc5160_moveTo(tmc5160, *ticks, velocityMax);
}

uint8_t tmc5160_consistencyCheck(TMC5160TypeDef *tmc5160)
{
	// Config has not yet been written -> it cant be consistent
	if(tmc5160->config->state != CONFIG_READY)
		return 0;

	// Check constant shadow registers consistent with actual registers
	for(size_t i = 0; i < TMC5160_REGISTER_COUNT; i++)
		if(tmc5160->config->shadowRegister[i] != tmc5160_readInt(tmc5160, i))
			return 1;

	// No inconsistency detected
	return 0;
}
