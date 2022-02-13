//###########################################################################
// Description:
//! \addtogroup f2806x_example_cla_list
//! <h1> Finite Impulse Response Filter </h1>
//!
//! A 5 tap FIR filter is implemented in Task 1 of the
//! CLA.
//!
//! \b Watch \b Variables \n
//! - xResult - Result of the FIR operation
//! - xAdcInput - Simulated ADC input
//
//###########################################################################
// $TI Release: F2806x C/C++ Header Files and Peripheral Examples V141 $
// $Release Date: January 19, 2015 $
// $Copyright: Copyright (C) 2011-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "flightControl.h"
#include "DSP28x_Project.h"     // DSP28x Headerfile
#include "compileGuard.h"
#include "initMotors.h"
#include "initSCI.h"
#include "initADC.h"
#include "initCLA.h"
#include "initPIE.h"
#include "initCPU_timer.h"
#include "quad.h"
#include "global.h"
#include "MPU6050_GlobalHeader.h"     // Header Files for the MPU6050 Project
#include "GPIO_action.h"



//			  SSSSSS    CCCCCC	 IIII 			AAAA
//	  *		 SS    SS  CC    CC   II  		   AA  AA       *
//	 ***	 SS        CC         II  		  AA    AA     *** 		#########################################################################################
//	*****	  SSSSSS   CC         II  ======  AAAAAAAA    *****  	#########################################################################################
//	 ***	       SS  CC		  II  		  AA    AA     ***		#########################################################################################
//	  *		 SS    SS  CC    CC   II  		  AA    AA      *
//            SSSSSS    CCCCCC   IIII 		  AA    AA
#ifdef USE_FLASH
	#pragma CODE_SECTION(InitFlash, "ramfuncs")
	void InitFlash(void);
#endif

char *startMsg_A;

volatile Uint16 headerSCIA = 0;
volatile Uint16 tempHeaderSCIA = 0;
volatile Uint16 receivedByteA = 0;
volatile Uint16 oldReceivedByteA = 0;
volatile Uint16 headerByteCount = 0;
volatile Uint16 FC_ByteCount = 0;
volatile Uint16 GP_ByteCount = 0;
volatile Uint16 GR_ByteCount = 0;
volatile Uint16 GY_ByteCount = 0;

volatile Uint16 headerFlag = FALSE;
volatile Uint16 incommingData = FALSE;
volatile Uint16 tail = 0;

volatile Uint16 firstGain	= 0;

volatile Uint16 flightControlPayload[NBR_OF_FLIGHT_CONTROL_BYTES / 2] = { 0, 0,
		0, 0 };

volatile Uint16 gainBytesPitch[NBR_OF_GAIN_BYTES / 2] = { 0, 0, 0, 0, 0 };
volatile Uint16 gainBytesRoll[NBR_OF_GAIN_BYTES / 2] = { 0, 0, 0, 0, 0 };
volatile Uint16 gainBytesYaw[NBR_OF_GAIN_BYTES / 2] = { 0, 0, 0, 0, 0 };

Uint16 sendPitch = 0;
volatile Uint16 countii = 0;

//			  SSSSSS    CCCCCC	 IIII 		  PPPPPPP
//	  *		 SS    SS  CC    CC   II  		  PP    PP      *
//	 ***	 SS        CC         II  		  PP    PP     *** 		#########################################################################################
//	*****	  SSSSSS   CC         II  ======  PPPPPPP     *****  	#########################################################################################
//	 ***	       SS  CC		  II  		  PP	PP     ***		#########################################################################################
//	  *		 SS    SS  CC    CC   II  		  PP	PP      *
//            SSSSSS    CCCCCC   IIII 		  PPPPPPP

char *startMsg_B;

volatile Uint16 receivedByteB = 0;

//			  EEEEEEEE   PPPPPPP    WW      WW   MM     MM
//	  *		  EE         PP    PP   WW		WW   MMMM MMMM       *
//	 ***	  EE         PP    PP   WW      WW   MM MMM MM      *** 	#####################################################################################
//	*****	  EEEE       PPPPPPP    WW      WW   MM  M  MM     *****  	#####################################################################################
//	 ***	  EE         PP		    WW      WW   MM     MM      ***		#####################################################################################
//	  *		  EE         PP	         WW WW WW    MM     MM       *
//			  EEEEEEEE   PP           WW  WW     MM     MM

Uint16 EPWM_periodCount = 0;

int k = 0;

//  Main Function
void main(void) {

//	sendPitch.f = 0.0;
//	sendRoll.f  = 0.0;
//	sendYaw.f	= 0.0;



	startMsg_A = "SCI-A is alive!";		 // |
	startMsg_B = "SCI-B is alive!";		 // V Start up and debug phrase

	target_scale = 500 / MAX_ANGLE;

	InitSysCtrl();
	initPIE_vector();
	initCPU_timer();

#ifdef USE_FLASH
	//--- Copy all Flash sections that need to run from RAM (use memcpy() from RTS library)

	// Section secureRamFuncs contains user defined code that runs from CSM secured RAM
		memcpy(&secureRamFuncs_runstart, &secureRamFuncs_loadstart, (Uint32)&secureRamFuncs_loadsize);

	//--- Initialize the Flash and OTP
		InitFlash();
#endif

	initCLA();
	initMotors();
	initADC();
	scia_fifo_init();
	scib_fifo_init();
	InitSciGpio();
	I2CA_Init();
	initPIE();


	initMPU6050_DMP();

	scia_msg(startMsg_A);
	scib_msg(startMsg_B);

	Cla1ForceTask1();
	// To initialize CLA to CPU variables which can't be done from the main CPU...


	setGPIO_18_output();
	setGPIO_17_output();

	firstGain	= 0;

	for (;;) {

		if (sendControl == TRUE && sensor_ready == TRUE) {

#ifndef RADIO_DEBUG

			scia_xmit(_SEND_FLIGHT_CONTROL);

			if (firstGain < 7) scia_xmit(_INIT_FILTER_GAIN);

			if (firstGain == 7) {

				firstGain = 8;
				scia_xmit(_FILTER_GAIN_RECEIVED);

			}
#else
			scib_xmit(_SEND_FLIGHT_CONTROL);

			if (firstGain < 7) scib_xmit(_INIT_FILTER_GAIN);

			if (firstGain == 7) {

				firstGain = 8;
				scib_xmit(_FILTER_GAIN_RECEIVED);

			}
#endif

			sendControl = FALSE;

		}



#ifdef SCIA_DEBUG
		scia_msg(startMsg_A);
#endif

#ifdef SCIB_DEBUG
		scib_msg(startMsg_B);
#endif

#ifdef MPU6050

#endif

#ifdef SCI


#endif

	}
}

#ifdef USE_FLASH
	void InitFlash(void)
{
	asm(" EALLOW");									// Enable EALLOW protected register access
	FlashRegs.FPWR.bit.PWR = 3;						// Pump and bank set to active mode
	FlashRegs.FSTATUS.bit.V3STAT = 1;				// Clear the 3VSTAT bit
	FlashRegs.FSTDBYWAIT.bit.STDBYWAIT = 0x01FF;	// Sleep to standby transition cycles
	FlashRegs.FACTIVEWAIT.bit.ACTIVEWAIT = 0x01FF;	// Standby to active transition cycles
	FlashRegs.FBANKWAIT.bit.RANDWAIT = 3;			// Random access waitstates
	FlashRegs.FBANKWAIT.bit.PAGEWAIT = 3;			// Paged access waitstates
	FlashRegs.FOTPWAIT.bit.OTPWAIT = 5;				// OTP waitstates
	FlashRegs.FOPT.bit.ENPIPE = 1;					// Enable the flash pipeline
	asm(" EDIS");									// Disable EALLOW protected register access

// Force a complete pipeline flush to ensure that the write to the last register
// configured occurs before returning.  Safest thing is to wait 8 full cycles.

    asm(" RPT #6 || NOP");

} // end of InitFlash()
#endif

__interrupt void gyroADC_isr(void) {


	AdcRegs.ADCINTFLGCLR.bit.ADCINT3 = 1; 		//Clear ADCINT1 flag reinitialize for next SOC
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;   	// Acknowledge interrupt to PIE

}

__interrupt void cpu_timer0_isr(void) {

	CpuTimer0.InterruptCount++;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge this interrupt to receive more interrupts from group 1
}

__interrupt void cpu_timer1_isr(void) {

	CpuTimer1.InterruptCount++;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge this interrupt to receive more interrupts from group 1

}

__interrupt void cpu_timer2_isr(void) {

	CpuTimer2.InterruptCount++;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge this interrupt to receive more interrupts from group 1

}

__interrupt void motor1AB(void) {

#if _MOTORS

	if (sensor_ready == FALSE) {



		if (sensor_init < NO_SPIN_COUNT)
			setMotorsToMin();

		if (EPWM1A_SPIN_CONDITION)
			spinEPWM1A();
		if (EPWM1B_SPIN_CONDITION)
			spinEPWM1B();
		if (EPWM2A_SPIN_CONDITION)
			spinEPWM2B();
		if (EPWM3B_SPIN_CONDITION)
			spinEPWM2A();

		if (sensor_init > (NO_SPIN_COUNT + 4 * SPIN_COUNT))
			setMotorsToMin();

	}

#endif

	EPWM_periodCount++;

	if (EPWM_periodCount == 65535)
		EPWM_periodCount = EPWM_COUNT_RESET;

	EPwm1Regs.ETCLR.bit.INT = 1;
	// Acknowledge this interrupt to receive more interrupts from group 3
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void motor2AB(void) {		// Not runnint

	EPwm1Regs.ETCLR.bit.INT = 1;
	// Acknowledge this interrupt to receive more interrupts from group 3
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

#ifdef MPU6050

__interrupt void xint1_isr(void) {
	/*
	 * on interrupt set data availabe
	 */
	if (MPUFlag == 2) {
		MPUFlag = 1;
		CurrentMsgPtr.MsgStatus = I2C_MSGSTAT_SEND_NOSTOP;
		CurrentMsgPtr.SlaveAddress = I2C_SLAVE_ADDR;
		CurrentMsgPtr.NumOfBytes = 16;
		CurrentMsgPtr.MemoryLowAddr = MPU6050_RA_FIFO_R_W;

		while (I2CA_ReadData(&CurrentMsgPtr) != I2C_SUCCESS);

	}

	// Acknowledge this interrupt to get more from group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

#endif

#ifdef SCI
__interrupt void sciaRxFifoIsr(void) {



#ifdef RADIO_DEBUG
	receivedByteA = ScibRegs.SCIRXBUF.all;
#endif

#ifndef RADIO_DEBUG
	receivedByteA = SciaRegs.SCIRXBUF.all;
#endif


	receivedByteA = receivedByteA & 0x00FF; 	// SCI-buffer consists of two bytes. AND-away eventual unwanted
												// stuff (safety measure...)

	tempHeaderSCIA = oldReceivedByteA << 8 | receivedByteA;		// |
																// |
	headerByteCount++;											// |
																// |
	if (VALID_HEADER) {											// |
																// |
		headerSCIA    = tempHeaderSCIA;							// |
		FC_ByteCount  = 0;										// |
		GP_ByteCount  = 0;										// |
		GR_ByteCount  = 0;										// |
		GY_ByteCount  = 0;										// |
		headerFlag    = TRUE;										// |
		incommingData = FALSE;									// V Header detector. Always running...

	}

	if (IF_COM_ERROR) countii++;

	if (incommingData) {

		switch (headerSCIA) {

		case FLIGHT_CONTROL:// FLIGHT_CONTROL FLIGHT_CONTROL FLIGHT_CONTROL FLIGHT_CONTROL FLIGHT_CONTROL FLIGHT_CONTROL FLIGHT_CONTROL FLIGHT_CONTROL FLIGHT_CONTROL

			switch (FC_ByteCount) {
			case 0:
				flightControlPayload[0] = receivedByteA << 8;
				break;
			case 1:
				flightControlPayload[0] = flightControlPayload[0] | receivedByteA;
				break;
			case 2:
				flightControlPayload[1] = receivedByteA << 8;
				break;
			case 3:
				flightControlPayload[1] = flightControlPayload[1] | receivedByteA;
				break;
			case 4:
				flightControlPayload[2] = receivedByteA << 8;
				break;
			case 5:
				flightControlPayload[2] = flightControlPayload[2] | receivedByteA;
				break;
			case 6:
				flightControlPayload[3] = receivedByteA << 8;
				break;
			case 7:
				flightControlPayload[3] = flightControlPayload[3] | receivedByteA;
				break;
			case 8:
				tail = receivedByteA << 8;
				break;
			case 9:
				tail = tail | receivedByteA;
				break;
			default:
				break;
			}

			FC_ByteCount++;

			if (FC_ByteCount == 10 && tail == TAIL_FLIGHT_CONTROL) {

#if _CMD_TO_CLA
				thrust 		 = flightControlPayload[0];
				target_yaw   = flightControlPayload[1];
				target_pitch = flightControlPayload[2];
				target_roll  = flightControlPayload[3];
#endif
				received_control = TRUE;						// Should be set to false when data has been placed in the msgRAM to the CLA
				headerFlag       = FALSE;						// Time for a new header
				incommingData    = FALSE;

				FC_ByteCount = 0;
				GP_ByteCount = 0;
				GR_ByteCount = 0;
				GY_ByteCount = 1;
				headerSCIA   = 0;

			} else if ((FC_ByteCount == 10) && !(tail == TAIL_FLIGHT_CONTROL)) {

				received_control = FALSE;		// Currupt tail...
				headerFlag       = FALSE;		// Time for a new header
				incommingData    = FALSE;

				FC_ByteCount = 0;
				GP_ByteCount = 0;
				GR_ByteCount = 0;
				GY_ByteCount = 0;
				headerSCIA   = 0;

			}

			break;

		case FILTER_GAIN_PITCH: // PITCH_G PITCH_G PITCH_G PITCH_G PITCH_G PITCH_G PITCH_G PITCH_G PITCH_G PITCH_G PITCH_G PITCH_G PITCH_G PITCH_G PITCH_G PITCH_G PITCH_G PITCH_G

			switch (GP_ByteCount) {
			case 0:
				gainBytesPitch[0] = receivedByteA << 8;
				break;
			case 1:
				gainBytesPitch[0] = gainBytesPitch[0] | receivedByteA;
				break;
			case 2:
				gainBytesPitch[1] = receivedByteA << 8;
				break;
			case 3:
				gainBytesPitch[1] = gainBytesPitch[1] | receivedByteA;
				break;
			case 4:
				gainBytesPitch[2] = receivedByteA << 8;
				break;
			case 5:
				gainBytesPitch[2] = gainBytesPitch[2] | receivedByteA;
				break;
			case 6:
				gainBytesPitch[3] = receivedByteA << 8;
				break;
			case 7:
				gainBytesPitch[3] = gainBytesPitch[3] | receivedByteA;
				break;
			case 8:
				gainBytesPitch[4] = receivedByteA << 8;
				break;
			case 9:
				gainBytesPitch[4] = gainBytesPitch[4] | receivedByteA;
				break;
			case 10:
				gainBytesPitch[5] = receivedByteA << 8;
				break;
			case 11:
				gainBytesPitch[5] = gainBytesPitch[5] | receivedByteA;
				break;
			case 12:
				gainBytesPitch[6] = receivedByteA << 8;
				break;
			case 13:
				gainBytesPitch[6] = gainBytesPitch[6] | receivedByteA;
				break;
			case 14:
				tail = receivedByteA << 8;
				break;
			case 15:
				tail = tail | receivedByteA;
				break;
			default:
				break;
			}

			GP_ByteCount++;

			if (GP_ByteCount == 16 && tail == TAIL_FILTER_GAIN_PITCH) {

#if _CMD_TO_CLA
				uint_Kp_pitch        = gainBytesPitch[0];
				uint_Ki_pitch        = gainBytesPitch[1];
				uint_Kd_pitch 		 = gainBytesPitch[2];
				uint_Kp_ff_pitch     = gainBytesPitch[3];
				uint_gain_gyro_pitch = gainBytesPitch[4];
				thrust 		 	     = gainBytesPitch[5];
				trim_pitch	 	     = gainBytesPitch[6];
#endif

				received_gain_pitch = TRUE;		// Should be set to false when data has been placed in the msgRAM to the CLA
				firstGain			= firstGain | 0x1;
				headerFlag          = FALSE;	// Time for a new header
				incommingData       = FALSE;


				FC_ByteCount = 0;
				GP_ByteCount = 0;
				GR_ByteCount = 0;
				GY_ByteCount = 0;
				headerSCIA   = 0;

			} else if (GP_ByteCount == 16 && tail != TAIL_FILTER_GAIN_PITCH) {

				received_gain_pitch = FALSE;		// Currupt tail...
				headerFlag          = FALSE;		// Time for a new header
				incommingData       = FALSE;

				FC_ByteCount = 0;
				GP_ByteCount = 0;
				GR_ByteCount = 0;
				GY_ByteCount = 0;
				headerSCIA   = 0;

			}

			break;

		case FILTER_GAIN_ROLL: // ROLL_G ROLL_G ROLL_G ROLL_G ROLL_G ROLL_G ROLL_G ROLL_G ROLL_G ROLL_G ROLL_G ROLL_G ROLL_G ROLL_G ROLL_G ROLL_G ROLL_G ROLL_G

			switch (GR_ByteCount) {
			case 0:
				gainBytesRoll[0] = receivedByteA << 8;
				break;
			case 1:
				gainBytesRoll[0] = gainBytesRoll[0] | receivedByteA;
				break;
			case 2:
				gainBytesRoll[1] = receivedByteA << 8;
				break;
			case 3:
				gainBytesRoll[1] = gainBytesRoll[1] | receivedByteA;
				break;
			case 4:
				gainBytesRoll[2] = receivedByteA << 8;
				break;
			case 5:
				gainBytesRoll[2] = gainBytesRoll[2] | receivedByteA;
				break;
			case 6:
				gainBytesRoll[3] = receivedByteA << 8;
				break;
			case 7:
				gainBytesRoll[3] = gainBytesRoll[3] | receivedByteA;
				break;
			case 8:
				gainBytesRoll[4] = receivedByteA << 8;
				break;
			case 9:
				gainBytesRoll[4] = gainBytesRoll[4] | receivedByteA;
				break;
			case 10:
				gainBytesRoll[5] = receivedByteA << 8;
				break;
			case 11:
				gainBytesRoll[5] = gainBytesRoll[5] | receivedByteA;
				break;
			case 12:
				gainBytesRoll[6] = receivedByteA << 8;
				break;
			case 13:
				gainBytesRoll[6] = gainBytesRoll[6] | receivedByteA;
				break;
			case 14:
				tail = receivedByteA << 8;
				break;
			case 15:
				tail = tail | receivedByteA;
				break;
			default:
				break;
			}

			GR_ByteCount++;

			if (GR_ByteCount == 16 && tail == TAIL_FILTER_GAIN_ROLL) {

#if _CMD_TO_CLA
				uint_Kp_roll        = gainBytesRoll[0];
				uint_Ki_roll        = gainBytesRoll[1];
				uint_Kd_roll 		= gainBytesRoll[2];
				uint_Kp_ff_roll     = gainBytesRoll[3];
				uint_gain_gyro_roll = gainBytesRoll[4];
				thrust 		 	    = gainBytesRoll[5];
				trim_roll   	    = gainBytesRoll[6];
#endif

				received_gain_roll  = TRUE;	// Should be set to false when data has been placed in the msgRAM to the CLA
				firstGain			= firstGain | 0x2;
				headerFlag          = FALSE;	// Time for a new header
				incommingData       = FALSE;

				FC_ByteCount = 0;
				GP_ByteCount = 0;
				GR_ByteCount = 0;
				GY_ByteCount = 0;
				headerSCIA   = 0;

			} else if (GR_ByteCount == 16 && tail != TAIL_FILTER_GAIN_ROLL) {

				received_gain_roll  = FALSE;		// Currupt tail...
				headerFlag          = FALSE;		// Time for a new header
				incommingData       = FALSE;

				FC_ByteCount = 0;
				GP_ByteCount = 0;
				GR_ByteCount = 0;
				GY_ByteCount = 0;
				headerSCIA   = 0;

			}

			break;

		case FILTER_GAIN_YAW:

			switch (GY_ByteCount) {
			case 0:
				gainBytesYaw[0] = receivedByteA << 8;
				break;
			case 1:
				gainBytesYaw[0] = gainBytesYaw[0] | receivedByteA;
				break;
			case 2:
				gainBytesYaw[1] = receivedByteA << 8;
				break;
			case 3:
				gainBytesYaw[1] = gainBytesYaw[1] | receivedByteA;
				break;
			case 4:
				gainBytesYaw[2] = receivedByteA << 8;
				break;
			case 5:
				gainBytesYaw[2] = gainBytesYaw[2] | receivedByteA;
				break;
			case 6:
				gainBytesYaw[3] = receivedByteA << 8;
				break;
			case 7:
				gainBytesYaw[3] = gainBytesYaw[3] | receivedByteA;
				break;
			case 8:
				gainBytesYaw[4] = receivedByteA << 8;
				break;
			case 9:
				gainBytesYaw[4] = gainBytesYaw[4] | receivedByteA;
				break;
			case 10:
				gainBytesYaw[5] = receivedByteA << 8;
				break;
			case 11:
				gainBytesYaw[5] = gainBytesYaw[5] | receivedByteA;
				break;
			case 12:
				gainBytesYaw[6] = receivedByteA << 8;
				break;
			case 13:
				gainBytesYaw[6] = gainBytesYaw[6] | receivedByteA;
				break;
			case 14:
				tail = receivedByteA << 8;
				break;
			case 15:
				tail = tail | receivedByteA;
				break;
			default:
				break;
			}

			GY_ByteCount++;

			if (GY_ByteCount == 16 && tail == TAIL_FILTER_GAIN_YAW) {

#if _CMD_TO_CLA
				uint_Kp_yaw        = gainBytesYaw[0];
				uint_Ki_yaw        = gainBytesYaw[1];
				uint_Kd_yaw 	   = gainBytesYaw[2];
				uint_Kp_ff_yaw     = gainBytesYaw[3];
				uint_gain_gyro_yaw = gainBytesYaw[4];
				thrust 		 	   = gainBytesYaw[5];
				trim_yaw		   = gainBytesYaw[6];
#endif

				received_gain_yaw   = TRUE;		// Should be set to false when data has been placed in the msgRAM to the CLA
				firstGain			= firstGain | 0x4;
				headerFlag          = FALSE;	// Time for a new header
				incommingData       = FALSE;

				FC_ByteCount = 0;
				GP_ByteCount = 0;
				GR_ByteCount = 0;
				GY_ByteCount = 0;
				headerSCIA   = 0;

			} else if (GY_ByteCount == 16 && tail != TAIL_FILTER_GAIN_YAW) {

				received_gain_yaw   = FALSE;		// Currupt tail...
				headerFlag          = FALSE;		// Time for a new header
				incommingData       = FALSE;

				FC_ByteCount = 0;
				GP_ByteCount = 0;
				GR_ByteCount = 0;
				GY_ByteCount = 0;
				headerSCIA   = 0;

			}

			break;

		default:

			break;

		}

	}

	if (headerFlag)
		incommingData = TRUE;

	oldReceivedByteA = receivedByteA;

#ifdef RADIO_DEBUG
	ScibRegs.SCIFFRX.bit.RXFFOVRCLR = 1;   	// Clear Overflow flag
	ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;// Clear Interrupt flag

	PieCtrlRegs.PIEACK.all |= 0x100;// Issue PIE ack
#endif

#ifndef RADIO_DEBUG
	SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;   	// Clear Overflow flag
	SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;	// Clear Interrupt flag

	PieCtrlRegs.PIEACK.all |= 0x100;		// Issue PIE ack
#endif
}

__interrupt void sciaTxFifoIsr(void) {

}

__interrupt void scibRxFifoIsr(void) {

	receivedByteB = ScibRegs.SCIRXBUF.all;

	ScibRegs.SCIFFRX.bit.RXFFOVRCLR = 1;   	// Clear Overflow flag
	ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;	// Clear Interrupt flag

	PieCtrlRegs.PIEACK.all |= 0x100;		// Issue PIE ack
}

__interrupt void scibTxFifoIsr(void) {

}
#endif

//###########################################################################
// CLA ISRs
//###########################################################################
__interrupt void cla1_task1_isr(void) {

	PieCtrlRegs.PIEACK.bit.ACK11 = 1;

}

__interrupt void cla1_task2_isr(void) {
	PieCtrlRegs.PIEACK.bit.ACK11 = 1;
}

__interrupt void cla1_task3_isr(void) {
	PieCtrlRegs.PIEACK.bit.ACK11 = 1;
}

__interrupt void cla1_task4_isr(void) {

	CLEAR_GPIO_18;

	if (sensor_init < 4001) {

		sensor_init++;
		if (sensor_init == 4000)
			sensor_ready = TRUE;

	}



	PieCtrlRegs.PIEACK.bit.ACK11 = 1;
}

__interrupt void cla1_task5_isr(void) {

	PieCtrlRegs.PIEACK.bit.ACK11 = 1;
}

__interrupt void cla1_task6_isr(void) {
	PieCtrlRegs.PIEACK.bit.ACK11 = 1;
}

__interrupt void cla1_task7_isr(void) {
	PieCtrlRegs.PIEACK.bit.ACK11 = 1;
}

__interrupt void cla1_task8_isr(void) {
	PieCtrlRegs.PIEACK.bit.ACK11 = 1;
}
