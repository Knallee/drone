/*
 * initPIE.c
 *
 *  Created on: 8 jun 2016
 *      Author: knalle
 */



#include "initPIE.h"

void initPIE_vector() {

	//  Step 2: Initialize PIE control
	/*  Intialize PIE control, disable all interrupts and
	 * then copy over the PIE Vector table from BootROM to RAM
	 */
	DINT;
	InitPieCtrl();
	IER = 0x00000000;
	IFR = 0x00000000;
	InitPieVectTable();

	/*  Assign user defined ISR to the PIE vector table */
	EALLOW;
	PieVectTable.CLA1_INT1 = &cla1_task1_isr;
	PieVectTable.CLA1_INT2 = &cla1_task2_isr;
	PieVectTable.CLA1_INT3 = &cla1_task3_isr;
	PieVectTable.CLA1_INT4 = &cla1_task4_isr;
	PieVectTable.CLA1_INT5 = &cla1_task5_isr;
	PieVectTable.CLA1_INT6 = &cla1_task6_isr;
	PieVectTable.CLA1_INT7 = &cla1_task7_isr;
	PieVectTable.CLA1_INT8 = &cla1_task8_isr;

	PieVectTable.TINT0 = &cpu_timer0_isr;
	PieVectTable.TINT1 = &cpu_timer1_isr;
	PieVectTable.TINT2 = &cpu_timer2_isr;

	PieVectTable.ADCINT3 = &gyroADC_isr;

	PieVectTable.EPWM1_INT = &motor1AB;
	PieVectTable.EPWM2_INT = &motor2AB;

#ifdef	MPU6050

	PieVectTable.I2CINT1A = &i2c_int1a_isr;
	PieVectTable.XINT1    = &xint1_isr;

#endif

#ifdef SCI

#ifndef RADIO_DEBUG

	PieVectTable.SCITXINTA = &sciaTxFifoIsr;	// Tx
	PieVectTable.SCIRXINTA = &sciaRxFifoIsr;	// Rx

	PieVectTable.SCIRXINTB = &scibRxFifoIsr;	// Rx
	PieVectTable.SCITXINTB = &scibTxFifoIsr;	// Tx

#endif

#ifdef RADIO_DEBUG

	PieVectTable.SCIRXINTB = &sciaRxFifoIsr;	// Rx
	PieVectTable.SCITXINTB = &sciaTxFifoIsr;// Tx

#endif

#endif

}

void initPIE() {

	/*  Enable CLA interrupts at the group and subgroup levels */

	PieCtrlRegs.PIEIER11.all	   = 0xFFFF;   	// All CLA interrupt
	PieCtrlRegs.PIEIER10.bit.INTx3 = 1;			// ADC int 3 interrupt
	PieCtrlRegs.PIEIER1.bit.INTx7  = 1;			// Timer interrupt
	PieCtrlRegs.PIEIER3.bit.INTx1  = 1;    		// ePWM1
	PieCtrlRegs.PIEIER3.bit.INTx2  = 1;    		// ePWM2
	PieCtrlRegs.PIEIER9.bit.INTx1  = 1;    		// PIE Group 9, INT1 ~ SCIA RX interrupt
	PieCtrlRegs.PIEIER9.bit.INTx3  = 1;     	// PIE Group 9, INT3 ~ SCIB RX interrupt

#ifdef MPU6050
	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;          // Enable the PIE block
	PieCtrlRegs.PIEIER1.bit.INTx4 = 1;          // Enable PIE Group 1 INT4
	PieCtrlRegs.PIEIER8.bit.INTx1 = 1;			// Enable I2C interrupt 1 in the PIE: Group 8 interrupt 1
#endif

	IER = (M_INT11);   // CLA interrupt.

	// Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
	// which is connected to CPU-Timer 1, and CPU int 14, which is connected
	// to CPU-Timer 2:
	IER |= M_INT1;		// External interrupt 1 & 2.
	IER |= M_INT3;		// PWM interrupt.
	IER |= M_INT8;		// I2C interrupt.
	IER |= M_INT9;		// SCI interrupt.
	IER |= M_INT10;		// ADC 1-8 interrupt.
	IER |= M_INT13;		// Don't remember.
	IER |= M_INT14;		// Don't remember.
	EINT;
	// Enable Global interrupt INTM
	ERTM;
	// Enable Global realtime interrupt DBGM

}
