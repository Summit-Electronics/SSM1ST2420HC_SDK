/*
 * AS5055A.h
 *
 *  Created on: Apr 21, 2022
 *      Author: wesley
 */

#ifndef INC_AS5055A_REGISTERS_H_
#define INC_AS5055A_REGISTERS_H_

#define POR_OFF			0x3F22
#define SOFTWARE_RESET	0x3C00
#define MASTER_RESET	0x33A5
#define CLEAR_EF		0x3380
#define NOP 			0x0000
#define AGC				0x3FF8
#define ANGULAR_DATA	0x3FFF
#define ERROR_STATUS	0x335A
#define SYSTEM_CONFIG	0x3F20
#define CLEAR_ERROR_FLAG 0x3380
#define SOFTWARE_RESET_SPI 0x02


#define ANGULAR_DATA_READ 0xFFFF
#define AGC_DATA_READ 	0xFFF0
#define NOP_READ 		0x8000
#define SYSTEM_CONFIG_READ	0xBF20

#endif /* INC_AS5055A_REGISTERS_H_ */
