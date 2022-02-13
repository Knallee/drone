/*
 * GPIO_action.c
 *
 *  Created on: 31 maj 2016
 *      Author: knalle
 */

#include "GPIO_action.h"
#include "DSP28x_Project.h"

void setGPIO_18_output() {

	EALLOW;
	GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO18  = 1;
	EDIS;

}

void setGPIO_17_output() {

	EALLOW;
	GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO17  = 1;
	EDIS;

}


