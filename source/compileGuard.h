/*
 * compileGuard.h
 *
 *  Created on: 3 okt 2015
 *      Author: knalle
 */

#ifndef COMPILEGUARD_H_
#define COMPILEGUARD_H_

#define ON 		1
#define OFF		0
#define FALSE	0
#define	TRUE	1



// Serial Com #########################################
#define SCI			ON

#define MPU6050


//#define ALL_PID
#define _PITCH_PID		ON
#define _ROLL_PID		ON
#define _YAW_PID		ON
//#define HARD_CAL	// Uncomment to perfrom a calibration of max and min
					// control signal to the motor control unit

#define _RADIO			ON
#define _CMD_TO_CLA		ON		// Assigne the CPU to CLA varibles with the received data
#define _REMOTE_TUNE	ON		// Ability to tune the control filters from the remote control
#define _MOTORS			ON
//#define SCIA_DEBUG					 // Countinously sending "SCI-A is alive"
//#define SCIB_DEBUG					 // Countinously sending "SCI-B is alive"

//#define RADIO_DEBUG
#define USE_FLASH


#endif /* COMPILEGUARD_H_ */
