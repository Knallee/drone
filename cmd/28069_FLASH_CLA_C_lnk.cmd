/*
//###########################################################################
//
// FILE:    28069_RAM_CLA_C_lnk.cmd
//
// TITLE:   Linker Command File For F28069 examples that run out of RAM
//
//          This ONLY includes all SARAM blocks on the F28069 device.
//          This does not include flash or OTP.
//
//          Keep in mind that L0,L1,L2,L3 and L4 are protected by the code
//          security module.
//
//          What this means is in most cases you will want to move to
//          another memory map file which has more memory defined.
//
//###########################################################################
// $TI Release: F2806x C/C++ Header Files and Peripheral Examples V141 $ 
// $Release Date: January 19, 2015 $ 
// $Copyright: Copyright (C) 2011-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################
*/

/* ======================================================
// For Code Composer Studio V2.2 and later
// ---------------------------------------
// In addition to this memory linker command file,
// add the header linker command file directly to the project.
// The header linker command file is required to link the
// peripheral structures to the proper locations within
// the memory map.
//
// The header linker files are found in <base>\F2806x_headers\cmd
//
// For BIOS applications add:      F2806x_Headers_BIOS.cmd
// For nonBIOS applications add:   F2806x_Headers_nonBIOS.cmd
========================================================= */

/* ======================================================
// For Code Composer Studio prior to V2.2
// --------------------------------------
// 1) Use one of the following -l statements to include the
// header linker command file in the project. The header linker
// file is required to link the peripheral structures to the proper
// locations within the memory map                                    */

/* Uncomment this line to include file only for non-BIOS applications */
/* -l F2806x_Headers_nonBIOS.cmd */

/* Uncomment this line to include file only for BIOS applications */
/* -l F2806x_Headers_BIOS.cmd */

/* 2) In your project add the path to <base>\F2806x_headers\cmd to the
   library search path under project->build options, linker tab,
   library search path (-i).
/*========================================================= */

/* Define the memory block start/length for the F2806x
   PAGE 0 will be used to organize program sections
   PAGE 1 will be used to organize data sections

   Notes:
         Memory blocks on F28069 are uniform (ie same
         physical memory) in both PAGE 0 and PAGE 1.
         That is the same memory region should not be
         defined for both PAGE 0 and PAGE 1.
         Doing so will result in corruption of program
         and/or data.

         Contiguous SARAM memory blocks can be combined
         if required to create a larger memory block.
*/

-heap 0x400
-stack 0x400

/* Define size for the CLA scratchpad area that will be used
   by the CLA compiler for local symbols and temporary variables; Also force
   references to the special symbols that mark the scratchpad area. */
//CLA_SCRATCHPAD_SIZE = 0x300; //----------------------------------------------Why not working??????
--undef_sym=__cla_scratchpad_end
--undef_sym=__cla_scratchpad_start


MEMORY
{
PAGE 0 :	/* Program Memory */

   BEGIN_M0           : origin = 0x000000, length = 0x000002     /* Part of M0SARAM - used for "Boot to M0" bootloader mode */
//   RAMM0      		: origin = 0x000050, length = 0x0003B0

   L3DPSARAM          : origin = 0x009000, length = 0x001000     /* L3 DPSARAM, CSM secure, CLA Prog RAM */
   L4SARAM            : origin = 0x00A000, length = 0x002000     /* L4 SARAM, CSM secure */
   OTP            (R) : origin = 0x3D7800, length = 0x000400     /* OTP */
   FLASH_ABCDEFGH (R) : origin = 0x3D8000, length = 0x01FF80     /* FLASH, All sectors combined */
   CSM_RSVD       (R) : origin = 0x3F7F80, length = 0x000076     /* Part of FLASH Sector A - reserved when CSM is in use */
   BEGIN_FLASH    (R) : origin = 0x3F7FF6, length = 0x000002     /* Part of FLASH Sector A - used for "Jump to flash" bootloader mode */
   PASSWORDS      (R) : origin = 0x3F7FF8, length = 0x000008     /* Part of FLASH Sector A - CSM password locations */
   FPUTABLES      (R) : origin = 0x3FD860, length = 0x0006A0     /* Part of Boot ROM */
   IQTABLES       (R) : origin = 0x3FDF00, length = 0x000B50     /* Part of Boot ROM */
   IQTABLES2      (R) : origin = 0x3FEA50, length = 0x00008C     /* Part of Boot ROM */
   IQTABLES3      (R) : origin = 0x3FEADC, length = 0x0000AA     /* Part of Boot ROM */
   RESET          (R) : origin = 0x3FFFC0, length = 0x000002     /* part of Boot ROM */


PAGE 1 :	/* Data Memory */

   M0SARAM            : origin = 0x000002, length = 0x0003FE     /* M0 SARAM */
   M1SARAM            : origin = 0x000400, length = 0x000400     /* M1 SARAM */
   CLAMSGRAM1         : origin = 0x001480, length = 0x000080     /* Part of PF0 - CLA to CPU Message RAM */
   CLAMSGRAM2         : origin = 0x001500, length = 0x000080     /* Part of PF0 - CPU to CLA Message RAM */
   L0DPSARAM          : origin = 0x008000, length = 0x000800     /* L0 DPSARAM, CSM secure, CLA Data RAM 2 */
   L1DPSARAM          : origin = 0x008800, length = 0x000400     /* L1 DPSARAM, CSM secure, CLA Data RAM 0 */
   L2DPSARAM          : origin = 0x008C00, length = 0x000400     /* L2 DPSARAM, CSM secure, CLA Data RAM 1 */
   L5DPSARAM          : origin = 0x00C000, length = 0x002000     /* L5 DPSARAM, DMA RAM 0 */
   L6DPSARAM          : origin = 0x00E000, length = 0x002000     /* L6 DPSARAM, DMA RAM 1 */
   RAML7L8            : origin = 0x010000, length = 0x004000	 /* L7-8 DPSARAM, DMA RAM 2-3 */
}


SECTIONS
{

/*** Compiler Required Sections ***/

  /* Program memory (PAGE 0) sections */
   .text              : > FLASH_ABCDEFGH,        PAGE = 0
   .cinit             : > FLASH_ABCDEFGH,        PAGE = 0
   .const             : > FLASH_ABCDEFGH,        PAGE = 0
   .econst            : > FLASH_ABCDEFGH,        PAGE = 0
   .pinit             : > FLASH_ABCDEFGH,        PAGE = 0
   .reset             : > RESET,                 PAGE = 0, TYPE = DSECT  /* We are not using the .reset section */
   .switch            : > FLASH_ABCDEFGH,        PAGE = 0

  /* Data Memory (PAGE 1) sections */
   .bss               : > M0SARAM,               PAGE = 1
   .ebss              : > M0SARAM,               PAGE = 1
   .cio               : > M0SARAM,               PAGE = 1
   .stack             : > RAML7L8,               PAGE = 1
   .sysmem            : > RAML7L8,               PAGE = 1
   .esysmem           : > RAML7L8,               PAGE = 1

   /*** User Defined Sections ***/
   codestart          : > BEGIN_FLASH,           PAGE = 0                /* Used by file CodeStartBranch.asm */
   dmaMemBufs         : > L5DPSARAM,             PAGE = 1                /* Link to DMA accessible memory */
   csm_rsvd           : > CSM_RSVD,              PAGE = 0                /* Used by file Passwords.asm */
   passwords          : > PASSWORDS,             PAGE = 0                /* Used by file Passwords.asm */
   ClaToCpuMsgRAM     : > CLAMSGRAM1,            PAGE = 1                /* Link to PF0 - CLA Message RAM */
   CpuToClaMsgRAM     : > CLAMSGRAM2,            PAGE = 1                /* Link to PF0 - CLA Message RAM */

      /* Section secureRamFuncs used by file Flash.c. */
   ramfuncs		      :   LOAD = FLASH_ABCDEFGH, PAGE = 0                /* Load to flash, run from CSM secure RAM */
                          RUN = L4SARAM,         PAGE = 0
                          LOAD_START(_secureRamFuncs_loadstart),
                          LOAD_SIZE(_secureRamFuncs_loadsize),
                          RUN_START(_secureRamFuncs_runstart)

   /* Section Cla1Prog used by file Cla.c */
   Cla1Prog           :   LOAD = FLASH_ABCDEFGH, PAGE = 0                /* Load to flash, run from CLA Prog RAM */
                          RUN_START(_Cla1Prog_Start)
                          RUN = L3DPSARAM,       PAGE = 0
                          LOAD_START(_cla1Funcs_loadstart),
                          LOAD_SIZE(_cla1Funcs_loadsize),
                          RUN_START(_cla1Funcs_runstart)

   CLAscratch         : { *.obj(CLAscratch)                              /* Scratchpad memory for the CLA C Compiler */
                        . += CLA_SCRATCHPAD_SIZE;
                        *.obj(CLAscratch_end)
                        } > L0DPSARAM,           PAGE = 1


   IQmathTables       : > IQTABLES,              PAGE = 0, TYPE = NOLOAD
   IQmath             : > FLASH_ABCDEFGH,        PAGE = 0
   
   /* Allocate FPU math areas: */
   FPUmathTables      : > FPUTABLES, 			 PAGE = 0, TYPE = NOLOAD
   
   Cla1ToCpuMsgRAM  : > CLAMSGRAM1,   	  PAGE = 1
   CpuToCla1MsgRAM  : > CLAMSGRAM2,  	  PAGE = 1
   Cla1DataRam0		: > L1DPSARAM,		  PAGE = 1
   Cla1DataRam1		: > L2DPSARAM,		  PAGE = 1
   Cla1DataRam2		: > L0DPSARAM,		  PAGE = 1

   CLA1mathTables	: LOAD = FLASH_ABCDEFGH, PAGE = 0                /* Load to flash, run from CSM secure RAM */
                      RUN = L0DPSARAM,       PAGE = 1
                      LOAD_START(_Cla1mathTablesLoadStart),
                      LOAD_SIZE(_Cla1mathTablesLoadSize),
                      RUN_START(_Cla1mathTablesRunStart)


  /* Uncomment the section below if calling the IQNexp() or IQexp()
      functions from the IQMath.lib library in order to utilize the
      relevant IQ Math table in Boot ROM (This saves space and Boot ROM
      is 1 wait-state). If this section is not uncommented, IQmathTables2
      will be loaded into other memory (SARAM, Flash, etc.) and will take
      up space, but 0 wait-state is possible.
   */
   /*
   IQmathTables2    : > IQTABLES2, PAGE = 0, TYPE = NOLOAD
   {

              IQmath.lib<IQNexpTable.obj> (IQmathTablesRam)

   }
   */
   /* Uncomment the section below if calling the IQNasin() or IQasin()
      functions from the IQMath.lib library in order to utilize the
      relevant IQ Math table in Boot ROM (This saves space and Boot ROM
      is 1 wait-state). If this section is not uncommented, IQmathTables2
      will be loaded into other memory (SARAM, Flash, etc.) and will take
      up space, but 0 wait-state is possible.
   */
   /*
   IQmathTables3    : > IQTABLES3, PAGE = 0, TYPE = NOLOAD
   {

              IQmath.lib<IQNasinTable.obj> (IQmathTablesRam)

   }
   */

}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
