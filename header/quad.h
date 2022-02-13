/*
 * quad.h
 *
 *  Created on: 24 jan 2016
 *      Author: knalle
 */

#include "global.h"


#ifndef QUAD_H_
#define QUAD_H_

#define CLA_PROG_ENABLE      0x0001
#define CLARAM0_ENABLE       0x0010
#define CLARAM1_ENABLE       0x0020
#define CLARAM2_ENABLE       0x0040
#define CLA_RAM0CPUE         0x0100
#define CLA_RAM1CPUE         0x0200
#define CLA_RAM2CPUE         0x0400

#define HARDWARE 1

// Constants
#define OSCCLK	     (10000000UL) 	         // < Internal OSC frequency


// Prototype statements.
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void gyroADC_isr(void);
__interrupt void motor1AB(void);
__interrupt void motor2AB(void);
__interrupt void sciaRxFifoIsr(void);
__interrupt void sciaTxFifoIsr(void);
__interrupt void scibRxFifoIsr(void);
__interrupt void scibTxFifoIsr(void);
__interrupt void xint1_isr(void);


// Include the test header file whose name is based on the test name
// which is defined by the macro TEST on the command line
#include "flightControl_shared.h"


__interrupt void cla1_task1_isr(void);
__interrupt void cla1_task2_isr(void);
__interrupt void cla1_task3_isr(void);
__interrupt void cla1_task4_isr(void);
__interrupt void cla1_task5_isr(void);
__interrupt void cla1_task6_isr(void);
__interrupt void cla1_task7_isr(void);
__interrupt void cla1_task8_isr(void);

//
// These are defined by the linker file and used to copy
// the CLA code from its load address to its run address
// in CLA program memory
//
extern Uint16 Cla1funcsLoadStart;
extern Uint16 Cla1funcsLoadEnd;
extern Uint16 Cla1funcsLoadSize;
extern Uint16 Cla1funcsRunStart;
extern Uint16 Cla1Prog_Start;
//
// These are defined by the linker file and used to copy
// the CLA math tables from its load address to its run address
// into one of the CLA data RAMs
//
extern Uint16 Cla1mathTablesLoadStart;
extern Uint16 Cla1mathTablesLoadEnd;
extern Uint16 Cla1mathTablesLoadSize;
extern Uint16 Cla1mathTablesRunStart;


#endif /* QUAD_H_ */
