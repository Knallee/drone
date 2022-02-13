/*
 * initADC.c
 *
 *  Created on: 24 jan 2016
 *      Author: knalle
 */


#include "DSP28x_Project.h"
#include "initADC.h"



void initADC(void) {

	InitAdc();  // Not the same, bad naming sorry ;)

// Configure ADC
	EALLOW;

	AdcRegs.ADCCTL2.bit.ADCNONOVERLAP 	= 1;	// Enable non-overlap mode
	AdcRegs.ADCCTL1.bit.INTPULSEPOS 	= 1;	// ADCINT1 trips after AdcResults latch


	AdcRegs.INTSEL1N2.bit.INT1SEL 	= 1;  		// setup EOC1 to trigger ADCINT1 to fire
	AdcRegs.INTSEL1N2.bit.INT1E 	= 0;		// Enabled ADCINT1
	AdcRegs.INTSEL1N2.bit.INT1CONT 	= 0;		// Disable ADCINT1 Continuous mode

	AdcRegs.INTSEL1N2.bit.INT2SEL 	= 1;  		// setup EOC2 to trigger ADCINT2 to fire
	AdcRegs.INTSEL1N2.bit.INT2E 	= 0;		// Enabled ADCINT2
	AdcRegs.INTSEL1N2.bit.INT2CONT 	= 0;		// Disable ADCINT2 Continuous mode

	AdcRegs.INTSEL3N4.bit.INT3SEL 	= 1;
	AdcRegs.INTSEL3N4.bit.INT3E 	= 1;
	AdcRegs.INTSEL3N4.bit.INT3CONT 	= 0;

	AdcRegs.INTSEL3N4.bit.INT4SEL 	= 1;
	AdcRegs.INTSEL3N4.bit.INT4E 	= 0;
	AdcRegs.INTSEL3N4.bit.INT4CONT 	= 0;

	AdcRegs.INTSEL5N6.bit.INT5SEL 	= 1;
	AdcRegs.INTSEL5N6.bit.INT5E 	= 0;
	AdcRegs.INTSEL5N6.bit.INT5CONT 	= 0;





	AdcRegs.ADCSOC0CTL.bit.CHSEL = CH_GYRO_PITCH;
	AdcRegs.ADCSOC1CTL.bit.CHSEL = CH_GYRO_ROLL;
	AdcRegs.ADCSOC2CTL.bit.CHSEL = CH_GYRO_YAW;
//	AdcRegs.ADCSOC3CTL.bit.CHSEL = ADC_ACC_X;
//	AdcRegs.ADCSOC4CTL.bit.CHSEL = ADC_ACC_Y;
//	AdcRegs.ADCSOC5CTL.bit.CHSEL = ADC_ACC_Z;



	AdcRegs.ADCSOC0CTL.bit.TRIGSEL = MOTOR_EPWM1; 	// set SOC0 start trigger on CPU timer 0, due to round-robin SOC0 converts first then SOC1
	AdcRegs.ADCSOC1CTL.bit.TRIGSEL = MOTOR_EPWM1; 	// set SOC1 start trigger on CPU timer 0, due to round-robin SOC0 converts first then SOC1
	AdcRegs.ADCSOC2CTL.bit.TRIGSEL = MOTOR_EPWM1; 	// set SOC2 start trigger on CPU timer 0, due to round-robin SOC0 converts first then SOC1
//	AdcRegs.ADCSOC3CTL.bit.TRIGSEL = MOTOR_EPWM1;	// set SOC3 start trigger on CPU timer 0, due to round-robin SOC0 converts first then SOC1
//	AdcRegs.ADCSOC4CTL.bit.TRIGSEL = MOTOR_EPWM1; 	// set SOC4 start trigger on CPU timer 0, due to round-robin SOC0 converts first then SOC1
//	AdcRegs.ADCSOC5CTL.bit.TRIGSEL = MOTOR_EPWM1; 	// set SOC4 start trigger on CPU timer 0, due to round-robin SOC0 converts first then SOC1




	AdcRegs.ADCSOC0CTL.bit.ACQPS = 6; 		// set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	AdcRegs.ADCSOC1CTL.bit.ACQPS = 6; 		// set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	AdcRegs.ADCSOC2CTL.bit.ACQPS = 6;		// set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
//	AdcRegs.ADCSOC3CTL.bit.ACQPS = 6;		// set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
//	AdcRegs.ADCSOC4CTL.bit.ACQPS = 6; 		// set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
//	AdcRegs.ADCSOC5CTL.bit.ACQPS = 6; 		// set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)




	EDIS;


}
