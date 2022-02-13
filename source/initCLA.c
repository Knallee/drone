/*
 * initCLA.c
 *
 *  Created on: 8 jun 2016
 *      Author: knalle
 */

#include "DSP28x_Project.h"
#include "quad.h"



void initCLA() {


	/*  Compute all CLA task vectors */
	EALLOW;

#ifdef USE_FLASH
//--- Copy the CLA program code from its load address to the CLA program
//--- memory (using memcpy() from RTS library).
	memcpy(&cla1Funcs_runstart, &cla1Funcs_loadstart, (Uint32)&cla1Funcs_loadsize);

//--- Memory Configuration
	Cla1Regs.MMEMCFG.bit.PROGE = 1;     // Configure as CLA program memory
	Cla1Regs.MMEMCFG.bit.RAM1E = 1;     // Configure as CLA data memory (C compiler scratchpad)
#endif

	Cla1Regs.MVECT1 = (Uint16) ((Uint32) &Cla1Task1 - (Uint32) &Cla1Prog_Start);
	Cla1Regs.MVECT2 = (Uint16) ((Uint32) &Cla1Task2 - (Uint32) &Cla1Prog_Start);
	Cla1Regs.MVECT3 = (Uint16) ((Uint32) &Cla1Task3 - (Uint32) &Cla1Prog_Start);
	Cla1Regs.MVECT4 = (Uint16) ((Uint32) &Cla1Task4 - (Uint32) &Cla1Prog_Start);
	Cla1Regs.MVECT5 = (Uint16) ((Uint32) &Cla1Task5 - (Uint32) &Cla1Prog_Start);
	Cla1Regs.MVECT6 = (Uint16) ((Uint32) &Cla1Task6 - (Uint32) &Cla1Prog_Start);
	Cla1Regs.MVECT7 = (Uint16) ((Uint32) &Cla1Task7 - (Uint32) &Cla1Prog_Start);
	Cla1Regs.MVECT8 = (Uint16) ((Uint32) &Cla1Task8 - (Uint32) &Cla1Prog_Start);


	//  Step 3 : Mapping CLA tasks
	/*  All tasks are enabled and will be started by an ePWM trigger
	 *  Map CLA program memory to the CLA and enable software breakpoints
	 */

	Cla1Regs.MPISRCSEL1.bit.PERINT1SEL = CLA_INT1_NONE;
	Cla1Regs.MPISRCSEL1.bit.PERINT2SEL = CLA_INT2_NONE;
	Cla1Regs.MPISRCSEL1.bit.PERINT3SEL = CLA_INT3_NONE;		// CLA_INT3_ADCINT3
	Cla1Regs.MPISRCSEL1.bit.PERINT4SEL = CLA_INT4_NONE;	//change to a PWM with 5ms period
	Cla1Regs.MPISRCSEL1.bit.PERINT5SEL = CLA_INT5_NONE;
	Cla1Regs.MPISRCSEL1.bit.PERINT6SEL = CLA_INT6_NONE;
	Cla1Regs.MPISRCSEL1.bit.PERINT7SEL = CLA_INT7_NONE;
	Cla1Regs.MPISRCSEL1.bit.PERINT8SEL = CLA_INT8_NONE;

	Cla1Regs.MIER.all = M_INT1 | M_INT3 | M_INT4 | M_INT5;

	/* Switch the CLA program space to the CLA and enable software forcing
	 * Also switch over CLA data ram 0,1 and 2
	 * CAUTION: The RAMxCPUE bits can only be enabled by writing to the register
	 * and not the individual bit field. Furthermore, the status of these bitfields
	 * is not reflected in either the watch or register views - they always read as
	 * zeros. This is a known bug and the user is advised to test CPU accessibilty
	 * first before proceeding
	 */


	Cla1Regs.MMEMCFG.all = CLA_PROG_ENABLE | CLARAM0_ENABLE | CLARAM1_ENABLE | CLARAM2_ENABLE | CLA_RAM1CPUE;// FtoK ****---------------> Why enable all CLA RAM??
	Cla1Regs.MCTL.bit.IACKE = 1;

	EDIS;

}
