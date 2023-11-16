/*
 * SSM1ST2420HC.h
 *
 *  Created on: 3 nov. 2022
 *      Author: Wesley Goedegebuure
 *      Email: wesley@top-electronics.com
 */

#ifndef INC_SUMMIT_SSM1ST2420HC_H_
#define INC_SUMMIT_SSM1ST2420HC_H_

#include "AS5055A_Registers.h"
#include "stm32f0xx_hal.h"
#include "main.h"

typedef struct {
	uint32_t VSTART;
	uint32_t A1;
	uint32_t V1;
	uint32_t AMAX;
	uint32_t VMAX;
	uint32_t DMAX;
	uint32_t D1;
	uint32_t VSTOP;
} RampConfig;

typedef struct {
	uint32_t IRUN;
	uint32_t IHOLD;
} CurrentConfig;

/* TMC5160 functions */
void TMC5160_Basic_Init(CurrentConfig *Current);
/* Perform basic initialize of the TMC5160 and enable the DRV_Enable pin which controls the H-Bridge signals.
 * IRUN, IHOLD, full scale, see excel sheet for current estimate
 */

void TMC5160_Basic_Rotate(uint8_t Mode, RampConfig *Ramp);
/* Start stepper rotation in direction: "mode"
 * "mode" 0 or 1, 0 = right , 1 = left
 *  with Rampingprofile as set by RampConfig
 */
void TMC5160_Rotate_To(uint32_t Position, RampConfig *Ramp);
/* Rotate Stepper to "position"
 * "position": uint32_t (51200 = right, -51200 = left turn)
 * with Rampingprofile as set by RampConfig
 */
void TMC5160_Stop(void);
/* Stop all movement by writing VMAX = 0;
 */

void Drive_Enable(int state);
/* Enables the DRV_ENB pin, powering the motor and disabling free movement of the axis
 * "state": int (1 = enable drive , 0 = disable drive)
 */

uint32_t TMC5160_SPIWrite(uint8_t Address, uint32_t Value, int Action);
/* SPI "action" on register at "Address" for "Value" on the TMC5160
 * "action" 0 or 1, 0 = read, 1 = write
 * "address", see TMC5160 folder
 * "value", uint32_t value for the register
 */
uint16_t TMC_Get_Position();
/* Read the position register on the TMC5160 and convert the result (32 bit) to an angle (Degrees).
 */

void TMC5160_Init_Stallguard(int reset);
/* Initialize Stallguard
 * "reset" = 1 will reset the stall event and allow the motor to move again
 *
 * TODO: implement TSTEP, TPWMTHRS, TCOOLTHRS, THIGH needed?
 *  //TSTEP>- TPWMTHRS  , stealthchop PWM mode is enabled
 * 	//TCOOLTHRS >- TSTEP , stop on stall is enabled
 */

int TMC5160_Monitor_Stallguard(void);
/* monitor stallguard values
 * returns stallguard Flag value (1 = stall , 0 = no stall)
 */

void TMC5160_Init_Stealthchop(void);
/* Initialize Stealthchop
 */

/* AMS5055 functions */
void AMS5055_Basic_Init(void);
/* Wake up the AMS5055 and perform basic initialize, also performs a read angle to calibrate the output of AMS5055_get_Position().
 * only perform this function once
 */
uint16_t AMS5055_Get_Position();
/* Start a READ_ANGLE event and convert the result (12 bit) to an angle (Degrees).
 * TODO: check reliability
 */
uint8_t AMSParity(uint16_t value);
/* Calculate parity bit for SPI transaction
 * "value" the SPI transmit message
 */
uint16_t AMS5055_SPIWriteInt(uint16_t Address, int Action);
/* SPI "action" on register at "Address" for AMS5055
 * "action" 0 or 1, 0 = write , 1 = read
 * "address" , see AS5055A_registers.h
 */

/* Encoder functions */
uint16_t ENC_Get_Position(void);
/* Read the Encoder value register on TMC5160, and convert the result(32 bit) to an angle (Degrees)
 * TODO: check reliability
 */

/* GPIO functions */
void Toggle_OUT(int port,uint16_t time);
/* Toggle (24V) Output on "port" for duration of "time"
 * "port" 1 or 2, respectively OUT1 or OUT2
 * "time" duration in ms
 */
int Read_IN(int port);
/* Read input (5 to 24V) on "port"
 * "port" 1 or 2, respectively IN1 or IN2
 */
uint16_t Read_AIN(void);
/* Read analog input, absolute maximum = 5.0V
 * A voltage divider is used to scale voltage to MCU voltage.
 * (Input voltage x (15K / 10K+15K)) = voltage on MCU pin
 */

#endif /* INC_SUMMIT_SSM1ST2420HC_H_ */
