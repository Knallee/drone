/*
 * flightControl.h
 *
 *  Created on: 18 maj 2016
 *      Author: knalle
 */

#ifndef FLIGHTCONTROL_H_
#define FLIGHTCONTROL_H_

// Automatic stability control #####################

// Global defines



#define MAX_ANGLE 		30.0			// Max allowed angle
#define MAX_I			1000			// Max I-term
#define	MAX_OUT     	3000			// Max allowed output for the control filters
#define	_THRUST_SCALE	18.0

#define _THRUST_ADDITION 		0
#define _NEAR_TAKE_OFF			620
#define _PITCH_ROLL_STICK_SCALE	0.5
#define _TRIM_SCALE				0.1

// #### Roll #######################################
#define KP_ROLL			30
#define KI_ROLL			1
#define KD_ROLL			10
#define KP_FF_ROLL		2
#define ROLL_FF_GAIN	0.8


// #### Pitch #######################################

#define KP_PITCH		30.0
#define KI_PITCH		1
#define KD_PITCH		10.0
#define KP_FF_PITCH		2
#define PITCH_FF_GAIN	0.8


// #### Yaw #######################################

#define _YAW_TARGET_SCALE	0.05
#define _100_MS				19			// The MPU sends 4 quaternion data every 5th millisecond, CLA task 4 is trigger at this frequency
#define KP_YAW				10
#define KI_YAW				0
#define KD_YAW				3
#define KP_FF_YAW			1
#define YAW_FF_GAIN			0.0



#endif /* FLIGHTCONTROL_H_ */
