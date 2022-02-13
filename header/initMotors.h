/*
 * ePWM.h
 *
 *  Created on: 25 apr 2015
 *      Author: knalle
 */

#ifndef INITMOTORS_H_
#define INITMOTORS_H_

#include "DSP28x_Project.h"

void initMotors(void);

void setMotorsToMin(void);

void spinEPWM1A(void);
void spinEPWM1B(void);
void spinEPWM2A(void);
void spinEPWM2B(void);

// If anything regarding the motor pwm prescaler OR period time is changes, the ADC will
// change sampling frequency hence correction must be made in the control algorithm
// if one does not want to change Kp, Ki Kd.

#define MotorPWMPeriod  			28125		// 28125  	// Period register, 28125 = 2.5 ms
#define MOTOR_CLK_DIV				TB_DIV4	    // TB_DIV4 	// TBCLK = SYSCLKOUT / (MOTOR_HSP_CLK_DIV * MOTOR_CLK_DIV) = 11.25 MHz
#define MOTOR_HSP_CLK_DIV			TB_DIV2



// Spare Epwm --------------------------------------------------------------------
#define EPWM3_TIMER_TBPRD  			50000  				// Period register, 1 cycle = 177.78 ns
#define EPWM3_TIMER_CLK_DIV			TB_DIV2	// TBCLK = SYSCLKOUT / (HSP_CLK_DIV × CLK_DIV)
#define EPWM3_TIMER_HSP_CLK_DIV		TB_DIV2


#define EPWM1A_SPIN_CONDITION (sensor_init > NO_SPIN_COUNT) && (sensor_init < (NO_SPIN_COUNT+SPIN_COUNT))
#define EPWM1B_SPIN_CONDITION (sensor_init > (NO_SPIN_COUNT+SPIN_COUNT)) && (sensor_init < (NO_SPIN_COUNT+2*SPIN_COUNT))
#define EPWM2A_SPIN_CONDITION (sensor_init > (NO_SPIN_COUNT+2*SPIN_COUNT)) && (sensor_init < (NO_SPIN_COUNT+3*SPIN_COUNT))
#define EPWM3B_SPIN_CONDITION (sensor_init > (NO_SPIN_COUNT+3*SPIN_COUNT)) && (sensor_init < (NO_SPIN_COUNT+4*SPIN_COUNT))

#define SPIN_COUNT		70
#define NO_SPIN_COUNT 	800
#define MOTOR_SPIN 		12000



#endif
