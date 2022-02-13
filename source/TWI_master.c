/*
 * TWI_master.c
 *
 * Created: 2016-03-09 14:28:06
 *  Author: et05cc8
 */ 
#define F_CPU 90000000UL

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "MPU6050_GlobalHeader.h"     // Header Files for the MPU6050 Project
#include "flightControl_shared.h"
#include "GPIO_action.h"



// Global variables

volatile struct I2CMSG CurrentMsgPtr;				// Used in interrupts



Uint16 NOBZ 	   = 0;
Uint16 MPUFlag     = 0;

volatile Uint16	mpuCount	= 0;
volatile Uint16 sendControl = 0;
volatile Uint16 flagThree 	= 0;


void InitI2CGpio()
{

   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

//	GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up for GPIO28 (SDAA)
//	GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;    // Enable pull-up for GPIO29 (SCLA)

	GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;    // Enable pull-up for GPIO32 (SDAA)
	GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;	   // Enable pull-up for GPIO33 (SCLA)

/* Set qualification for selected pins to asynch only */
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.

//    GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (SDAA)
//    GpioCtrlRegs.GPAQSEL2.bit.GPIO29 = 3;  // Asynch input GPIO29 (SCLA)

	GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 3;  // Asynch input GPIO32 (SDAA)
	GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3;  // Asynch input GPIO33 (SCLA)

/* Configure I2C pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be I2C functional pins.
// Comment out other unwanted lines.

//	GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 2;   // Configure GPIO28 for SDAA operation
//	GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 2;   // Configure GPIO29 for SCLA operation

	GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;   // Configure GPIO32 for SDAA operation
	GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;   // Configure GPIO33 for SCLA operation

    EDIS;

	   // GPIO0 is input
	   EALLOW;
	   GpioCtrlRegs.GPAMUX2.bit.GPIO22 	= 0; 	// GPIO
	   GpioCtrlRegs.GPADIR.bit.GPIO22 	= 0; 	// input
	   GpioCtrlRegs.GPAQSEL2.bit.GPIO22 = 0; 	// XINT1 Synch to SYSCLKOUT only
	   EDIS;

	   //GPIO0 is XINT1
	   EALLOW;
	   GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 22;   // XINT1 is GPIO0
	   EDIS;

	   XIntruptRegs.XINT1CR.bit.POLARITY = 0;      // Rising edge interrupt

	   XIntruptRegs.XINT1CR.bit.ENABLE = 1;        // Enable XINT1

}

//===========================================================================
// End of file.
//===========================================================================


void I2CA_Init(void)
{

	InitI2CGpio();
	// Initialize I2C
	I2caRegs.I2CSAR = 0x0068;		// Slave address - EEPROM control code

	I2caRegs.I2CPSC.all = 6;		  // Prescaler - need 7-12 Mhz on module clk
	I2caRegs.I2CCLKL 	= 10;			// NOTE: must be non zero
	I2caRegs.I2CCLKH 	= 5;			// NOTE: must be non zero
	I2caRegs.I2CIER.all = 0x3C;	// Enable SCD & XRDYINT & RRDYINT & ARDY interrupts

	I2caRegs.I2CMDR.all = 0x0020;	// Take I2C out of reset
									// Stop I2C when suspended

	I2caRegs.I2CFFTX.all = 0x0000;	// Disable FIFO mode and TXFIFO
//  I2caRegs.I2CFFRX.all = 0x2040;	// Enable RXFIFO, clear RXFFINT,

	return;
}

Uint16 I2CA_WriteData(volatile struct I2CMSG *msg) {
	// Wait until the STP bit is cleared from any previous master communication.
	// Clearing of this bit by the module is delayed until after the SCD bit is
	// set. If this bit is not checked prior to initiating a new message, the
	// I2C could get confused.
	if (I2caRegs.I2CMDR.bit.STP == 1) {

		return I2C_STP_NOT_READY_ERROR;

	}

	// Setup slave address
	I2caRegs.I2CSAR = msg->SlaveAddress;

	// Check if bus busy
	if (I2caRegs.I2CSTR.bit.BB == 1) {

		return I2C_BUS_BUSY_ERROR;

	}

	// Setup number of bytes to send
	// MsgBuffer + Address
	I2caRegs.I2CCNT = (msg->NumOfBytes) + 1;

	// Setup data to send
	I2caRegs.I2CDXR = msg->MemoryLowAddr;
	NOBZ = 0;
	// Send start as master transmitter
	I2caRegs.I2CMDR.all = 0x6E20;

	CurrentMsgPtr.MsgStatus = I2C_MSGSTAT_WRITE_BUSY;

	return I2C_SUCCESS;
}

Uint16 I2CA_ReadData(volatile struct I2CMSG *msg) {
	// Wait until the STP bit is cleared from any previous master communication.
	// Clearing of this bit by the module is delayed until after the SCD bit is
	// set. If this bit is not checked prior to initiating a new message, the
	// I2C could get confused.
	if (I2caRegs.I2CMDR.bit.STP == 1) {

		return I2C_STP_NOT_READY_ERROR;

	}

	I2caRegs.I2CSAR = msg->SlaveAddress;

	if (msg->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP) {

		// Check if bus busy
		if (I2caRegs.I2CSTR.bit.BB == 1) {

			return I2C_BUS_BUSY_ERROR;

		}

		I2caRegs.I2CCNT = 1;
		NOBZ = 0;
//      I2caRegs.I2CDXR = msg->MemoryHighAddr;
		I2caRegs.I2CDXR = msg->MemoryLowAddr;
		I2caRegs.I2CMDR.all = 0x2620;		// Send data to setup EEPROM address
		// Update current message pointer and message status
		CurrentMsgPtr.MsgStatus = I2C_MSGSTAT_SEND_NOSTOP_BUSY;
	}

	return I2C_SUCCESS;
}

// ********** Interrupt Handlers ********** //
/****************************************************************************
This function is the Interrupt Service Routine (ISR), and called when the TWI interrupt is triggered;
that is whenever a TWI event has occurred. This function should not be called directly from the main
application.
****************************************************************************/
__interrupt void i2c_int1a_isr(void) {    // I2C-A


	SET_GPIO_17;

	Uint16 IntSource;

	// Read interrupt source
	IntSource = I2caRegs.I2CISRC.all;

	// Interrupt source = stop condition detected
	if (IntSource == I2C_SCD_ISRC) {
		// If completed message was writing data, reset msg to inactive state
		if (CurrentMsgPtr.MsgStatus == I2C_MSGSTAT_WRITE_BUSY) {

			if (flagThree == 1) {

				flagThree 	= 0;
				MPUFlag 	= 2;
				mpuCount++;

				if (mpuCount == 9) {

					sendControl = 1;
					mpuCount	= 0;

				}

				SET_GPIO_18;
				Cla1ForceTask4();
				// CLA is forced when 4 new quaternions are received!

			}
			CurrentMsgPtr.MsgStatus = I2C_MSGSTAT_INACTIVE;
		} else {
			// If a message receives a NACK during the address setup portion of the
			// EEPROM read, the code further below included in the register access ready
			// interrupt source code will generate a stop condition. After the stop
			// condition is received (here), set the message status to try again.
			// User may want to limit the number of retries before generating an error.
			if (CurrentMsgPtr.MsgStatus == I2C_MSGSTAT_SEND_NOSTOP_BUSY) {
				__asm("   ESTOP0");
			}

			// If completed message was reading EEPROM data, reset msg to inactive state
			// and read data from FIFO.
			else if (CurrentMsgPtr.MsgStatus == I2C_MSGSTAT_READ_BUSY) {
				CurrentMsgPtr.MsgStatus = I2C_MSGSTAT_INACTIVE;

				if (MPUFlag == 1) {
					Uint16 i;
					for (i = 0; i < CurrentMsgPtr.NumOfBytes; i++)
						mpu6050_fifoBuffer[i] = CurrentMsgPtr.MsgBuffer[i];

					//Reset FIFO!!!
					CurrentMsgPtr.MsgStatus 	= I2C_MSGSTAT_SEND_WITHSTOP;
					CurrentMsgPtr.SlaveAddress 	= I2C_SLAVE_ADDR;
					CurrentMsgPtr.NumOfBytes 	= 1;
					CurrentMsgPtr.MemoryLowAddr = MPU6050_RA_USER_CTRL;
					CurrentMsgPtr.MsgBuffer[0] 	= 0xE4;

					while (I2CA_WriteData(&CurrentMsgPtr) != I2C_SUCCESS);

					flagThree = 1;

				}

			}

		}

	}  // end of stop condition detected

	else if (IntSource == I2C_TX_ISRC) {
		I2caRegs.I2CDXR = CurrentMsgPtr.MsgBuffer[NOBZ];
		NOBZ++;
	}

	else if (IntSource == I2C_RX_ISRC) {
		CurrentMsgPtr.MsgBuffer[NOBZ] = I2caRegs.I2CDRR;
		NOBZ++;
	}

	// Interrupt source = Register Access Ready
	// This interrupt is used to determine when the EEPROM address setup portion of the
	// read data communication is complete. Since no stop bit is commanded, this flag
	// tells us when the message has been sent instead of the SCD flag. If a NACK is
	// received, clear the NACK bit and command a stop. Otherwise, move on to the read
	// data portion of the communication.
	else if (IntSource == I2C_ARDY_ISRC) {

		if (I2caRegs.I2CSTR.bit.NACK == 1) {

			I2caRegs.I2CMDR.bit.STP = 1;
			I2caRegs.I2CSTR.all 	= I2C_CLR_NACK_BIT;

		} else if (CurrentMsgPtr.MsgStatus == I2C_MSGSTAT_SEND_NOSTOP_BUSY) {

			CurrentMsgPtr.MsgStatus = I2C_MSGSTAT_RESTART;
			I2caRegs.I2CCNT = CurrentMsgPtr.NumOfBytes;	// Setup how many bytes to expect
			NOBZ = 0;
			I2caRegs.I2CMDR.all 	= 0x2C20;	// Send restart as master receiver
			CurrentMsgPtr.MsgStatus = I2C_MSGSTAT_READ_BUSY;

		}
	}  // end of register access ready

	else {
		// Generate some error due to invalid interrupt source
		__asm("   ESTOP0");
	}

	// Enable future I2C (PIE Group 8) interrupts
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;

	CLEAR_GPIO_17;

}
