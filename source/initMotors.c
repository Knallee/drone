/*
 * ePWM.c
 *
 *  Created on: 25 apr 2015
 *      Author: knalle
 */
#include "DSP28x_Project.h"
#include "initMotors.h"
#include "global.h"

void initMotors(void) {
	EALLOW;

	/* Disable internal pull-up for the selected output pins
	 for reduced power consumption */
	// Pull-ups can be enabled or disabled by the user.
	// This will enable the pullups for the specified pins.
	// Comment out other unwanted lines.
	GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    					// Disable pull-up on GPIO2 (EPWM1A)
	GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    					// Disable pull-up on GPIO3 (EPWM1B) Motor1B

	/* Configure EPwm-2 pins using GPIO regs*/
	// This specifies which of the possible GPIO pins will be EPWM2 functional pins.
	// Comment out other unwanted lines.
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   					// Configure GPIO2 as EPWM1A
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   					// Configure GPIO3 as EPWM1B



	EPwm1Regs.ETSEL.bit.SOCAEN = 1;							// Enable SOC on A group
	EPwm1Regs.ETSEL.bit.SOCASEL = 4;						// Select SOC from CMPA on upcount
	EPwm1Regs.ETPS.bit.SOCAPRD = 1;							// Generate pulse on 1st event

	// Setup TBCLK
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; 				// Count up
	EPwm1Regs.TBPRD = MotorPWMPeriod;       				// Set timer period
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;    				// Disable phase loading
	EPwm1Regs.TBPHS.half.TBPHS = 0x0000;       				// Phase is 0
	EPwm1Regs.TBCTR = 0x0000;                  				// Clear counter
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = MOTOR_HSP_CLK_DIV; 		// Clock ratio to SYSCLKOUT
	EPwm1Regs.TBCTL.bit.CLKDIV = MOTOR_CLK_DIV;

	// Setup shadow register load on ZERO
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	// Set actions
	EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;            			// Set PWM1A on Zero
	EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;     				// Clear PWM1A on event A, up count

	EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;            			// Set PWM1B on Zero
	EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;     				// Clear PWM1B on event B, up count

	// Interrupt where we will change the Compare Values
	EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     			// Select INT on Zero event
	EPwm1Regs.ETSEL.bit.INTEN = 1;                			// Enable INT
	EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;           			// Generate INT on 3rd event






	/* Disable internal pull-up for the selected output pins
	 for reduced power consumption */
	// Pull-ups can be enabled or disabled by the user.
	// This will enable the pullups for the specified pins.
	// Comment out other unwanted lines.
	GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    					// Disable pull-up on GPIO2 (EPWM2A)
	GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    					// Disable pull-up on GPIO3 (EPWM2B)

	/* Configure EPwm-2 pins using GPIO regs*/
	// This specifies which of the possible GPIO pins will be EPWM2 functional pins.
	// Comment out other unwanted lines.
	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   					// Configure GPIO2 as EPWM2A
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   					// Configure GPIO3 as EPWM2B



	EPwm2Regs.ETSEL.bit.SOCAEN = 1;							// Enable SOC on A group
	EPwm2Regs.ETSEL.bit.SOCASEL = 4;						// Select SOC from CMPA on upcount
	EPwm2Regs.ETPS.bit.SOCAPRD = 1;							// Generate pulse on 1st event

	// Setup TBCLK
	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; 				// Count up
	EPwm2Regs.TBPRD = MotorPWMPeriod;       				// Set timer period
	EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;    				// Disable phase loading
	EPwm2Regs.TBPHS.half.TBPHS = 0x0000;       				// Phase is 0
	EPwm2Regs.TBCTR = 0x0000;                  				// Clear counter
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = MOTOR_HSP_CLK_DIV; 		// Clock ratio to SYSCLKOUT
	EPwm2Regs.TBCTL.bit.CLKDIV = MOTOR_CLK_DIV;

	// Setup shadow register load on ZERO
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	// Set actions
	EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;            			// Set PWM1A on Zero
	EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;     				// Clear PWM1A on event A, up count

	EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;            			// Set PWM1B on Zero
	EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;     				// Clear PWM1B on event B, up count

	// Interrupt where we will change the Compare Values
	EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     			// Select INT on Zero event
	EPwm2Regs.ETSEL.bit.INTEN = 1;                			// Enable INT
	EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;           			// Generate INT on 3rd event



	EDIS;
}

void InitEPwm3(void) {

	// Setup TBCLK
	EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
	EPwm3Regs.TBPRD = EPWM3_TIMER_TBPRD;       // Set timer period
	EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
	EPwm3Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
	EPwm3Regs.TBCTR = 0x0000;                  // Clear counter
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = EPWM3_TIMER_HSP_CLK_DIV; // Clock ratio to SYSCLKOUT
	EPwm3Regs.TBCTL.bit.CLKDIV = EPWM3_TIMER_CLK_DIV;

	// Setup shadow register load on ZERO
	EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	// Set actions
	EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A on Zero
	EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;     // Clear PWM1A on event A, up count

	EPwm3Regs.AQCTLB.bit.ZRO = AQ_SET;            // Set PWM1B on Zero
	EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;     // Clear PWM1B on event B, up count

	// Interrupt where we will change the Compare Values
	EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
	EPwm3Regs.ETSEL.bit.INTEN = 1;                // Enable INT
	EPwm3Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

}




void spinEPWM1A(void) {

	EPwm1Regs.CMPA.half.CMPA = MOTOR_SPIN;
	EPwm1Regs.CMPB			 = MIN_MOTOR;

	EPwm2Regs.CMPA.half.CMPA = MIN_MOTOR;
	EPwm2Regs.CMPB 			 = MIN_MOTOR;

}
void spinEPWM1B(void) {

	EPwm1Regs.CMPA.half.CMPA = MIN_MOTOR;
	EPwm1Regs.CMPB			 = MOTOR_SPIN;

	EPwm2Regs.CMPA.half.CMPA = MIN_MOTOR;
	EPwm2Regs.CMPB 			 = MIN_MOTOR;

}
void spinEPWM2A() {

	EPwm1Regs.CMPA.half.CMPA = MIN_MOTOR;
	EPwm1Regs.CMPB			 = MIN_MOTOR;

	EPwm2Regs.CMPA.half.CMPA = MOTOR_SPIN;
	EPwm2Regs.CMPB 			 = MIN_MOTOR;

}
void spinEPWM2B() {

	EPwm1Regs.CMPA.half.CMPA = MIN_MOTOR;
	EPwm1Regs.CMPB			 = MIN_MOTOR;

	EPwm2Regs.CMPA.half.CMPA = MIN_MOTOR;
	EPwm2Regs.CMPB 			 = MOTOR_SPIN;

}

void setMotorsToMin() {

	EPwm1Regs.CMPA.half.CMPA = MIN_MOTOR;
	EPwm1Regs.CMPB			 = MIN_MOTOR;

	EPwm2Regs.CMPA.half.CMPA = MIN_MOTOR;
	EPwm2Regs.CMPB 			 = MIN_MOTOR;

}






