/*
 * TWI_master.h
 *
 * Created: 2016-03-09 14:28:14
 *  Author: et05cc8
 */ 
#define F_CPU 90000000UL

#ifndef TWI_MASTER_H
#define TWI_MASTER_H

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

// Prototype statements for functions found within this file.
void   I2CA_Init(void);
Uint16 I2CA_WriteData(volatile struct I2CMSG *msg);
Uint16 I2CA_ReadData(volatile struct I2CMSG *msg);
__interrupt void i2c_int1a_isr(void);


#define I2C_SLAVE_ADDR        0x68

extern Uint16 MPUFlag;		  // set to 1 when reading MPU data
extern volatile Uint16 sendControl;
extern volatile struct I2CMSG CurrentMsgPtr;				// Used in interrupts


#endif /* TWI_MASTER_H_ */
