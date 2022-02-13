/*
 * GPIO_action.h
 *
 *  Created on: 31 maj 2016
 *      Author: knalle
 */

#ifndef GPIO_ACTION_H_
#define GPIO_ACTION_H_

#include "DSP28x_Project.h"

#define SET_GPIO_18 	GpioDataRegs.GPASET.bit.GPIO18    = 1
#define CLEAR_GPIO_18 	GpioDataRegs.GPACLEAR.bit.GPIO18  = 1
#define TOGGLE_GPIO_18 	GpioDataRegs.GPATOGGLE.bit.GPIO18 = 1

#define SET_GPIO_17 	GpioDataRegs.GPASET.bit.GPIO17    = 1
#define CLEAR_GPIO_17 	GpioDataRegs.GPACLEAR.bit.GPIO17  = 1
#define TOGGLE_GPIO_17 	GpioDataRegs.GPATOGGLE.bit.GPIO17 = 1


void setGPIO_18_output(void);
void setGPIO_17_output(void);

//void toggleGIPO_17(void);
//void toggleGIPO_18(void);

#endif /* GPIO_ACTION_H_ */
