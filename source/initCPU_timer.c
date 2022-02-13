/*
 * initCPU_timer.c
 *
 *  Created on: 8 jun 2016
 *      Author: knalle
 */



#include "initCPU_timer.h"



void initCPU_timer() {

	EDIS;

	// Step 4. Initialize the Device Peripheral. This function can befound in F2806x_CpuTimers.c
	InitCpuTimers();   // Initialize the Cpu Timers

	// Overflow interrupt
	// 90MHz CPU Freq, 1 second Period (in uSeconds)

//								 F_CPU     time (us)
	ConfigCpuTimer(&CpuTimer0, 		90, 	1000000);
	ConfigCpuTimer(&CpuTimer1, 		90, 	1000000);
	ConfigCpuTimer(&CpuTimer2, 		90, 	1000000);

	// To ensure precise timing, use write-only instructions to write to the entire register. Therefore, if any
	// of the configuration bits are changed in ConfigCpuTimer and InitCpuTimers (in F2806x_CpuTimers.h), the
	// below settings must also be updated.

	CpuTimer0Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0
	CpuTimer1Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0
	CpuTimer2Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0

	EALLOW;
}
