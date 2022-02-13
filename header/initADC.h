/*
 * initADC.h
 *
 *  Created on: 24 jan 2016
 *      Author: knalle
 */

#include "global.h"

#ifndef INITADC_H_
#define INITADC_H_

void initADC(void);

#define CPU_TIMER0_TRIG0 	1
#define CPU_TIMER1_TRIG1	2
#define	CPU_TIMER2_TRIG2	3
#define MOTOR_EPWM1			5


#define	CH_GYRO_PITCH	0
#define CH_GYRO_ROLL	1
#define CH_GYRO_YAW		2

#define ADC_GYRO_PITCH		AdcResult.ADCRESULT0
#define ADC_GYRO_ROLL		AdcResult.ADCRESULT1
#define ADC_GYRO_YAW		AdcResult.ADCRESULT2
#define ADC_ACC_X			AdcResult.ADCRESULT3
#define ADC_ACC_Y			AdcResult.ADCRESULT4
#define ADC_ACC_Z			AdcResult.ADCRESULT5


#endif /* INITADC_H_ */
