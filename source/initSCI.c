/*
 * sci.c
 *
 *  Created on: 25 apr 2015
 *      Author: knalle
 */

#include "initSCI.h"
#include "GPIO_action.h"
#include "compileGuard.h"



#define _9600_BUAD
//#define _19200_BUAD
//#define _38400_BUAD
//#define _57600_BUAD
//#define _115200_BUAD		//  Remeber to change the settings in the radio link as well ;)

#if SCI


//			  SSSSSS    CCCCCC	 IIII 			AAAA
//	  *		 SS    SS  CC    CC   II  		   AA  AA       *
//	 ***	 SS        CC         II  		  AA    AA     *** 		#########################################################################################
//	*****	  SSSSSS   CC         II  ======  AAAAAAAA    *****  	#########################################################################################
//	 ***	       SS  CC		  II  		  AA    AA     ***		#########################################################################################
//	  *		 SS    SS  CC    CC   II  		  AA    AA      *
//            SSSSSS    CCCCCC   IIII 		  AA    AA



void scia_fifo_init() {

	SciaRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
									// No parity,8 char bits,
									// async mode, idle-line protocol
	SciaRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
									// Disable RX ERR, SLEEP, TXWAKE

	SciaRegs.SCICTL2.bit.TXINTENA   = 0;
	SciaRegs.SCICTL2.bit.RXBKINTENA = 1;

//	BRR = (LSPCLK/(BaudRate x 8 ))-1

#ifdef _9600_BUAD

	SciaRegs.SCIHBAUD    = 0x0001;  // 9600 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK). |
	SciaRegs.SCILBAUD    = 0x0024;  //                                              v 0x1 concatenated with 0x24 = 0x124 = 0b100100100 = 292

#endif

#ifdef _19200_BUAD

	SciaRegs.SCIHBAUD    =0x0000;  // 19200 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK). |
	SciaRegs.SCILBAUD    =0x0091;  //                                               v 0x91 = 145

#endif

#ifdef _38400_BUAD

	SciaRegs.SCIHBAUD    =0x0000;  // 38400 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK). |
	SciaRegs.SCILBAUD    =0x0048;  //                                               v 0x48 = 72

#endif

#ifdef _57600_BUAD

	SciaRegs.SCIHBAUD    =0x0000;  // 57600 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK). |
	SciaRegs.SCILBAUD    =0x0030;  //                                               v 0x30 = 48

#endif

#ifdef _115200_BUAD

	SciaRegs.SCIHBAUD    =0x0000;  // 115200 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK). |
	SciaRegs.SCILBAUD    =0x0017;  //                                                v 0x17 = 23

#endif

	SciaRegs.SCICCR.bit.LOOPBKENA = 0; // Enable loop back

	SciaRegs.SCIFFTX.all = 0xC121;


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//
//				   15          14          13          12          11          10          9           8          7          6            5           4           3           2           1          0
//			   __________  __________  __________  __________  __________  __________  __________  __________  __________  __________  __________  __________  __________  __________  __________  __________
//			  |          ||  RXFFOVF ||  RXFIF0  ||          ||          ||          ||          ||          ||  RXFFINT ||  RXFFINT ||          ||          ||          ||          ||          ||          |
//	 SCIFFRX  | RXFFOF   ||   CLR    ||  Reset   || RXFIFST4 || RXFIFST3 || RXFIFST2 || RXFIFST1 || RXFIFST0 ||    Flag  ||   CLR    || RXFFIENA || RXFFIL4  || RXFFIL3  || RXFFIL2  || RXFFIL1  || RXFFIL0  |
//			  |__________||__________||__________||__________||__________||__________||__________||__________||__________||__________||__________||__________||__________||__________||__________||__________|
//
//	 0x0121 -      0 			0			0			0			0			0			0			1			0			0			1			0			0			0			0			1
//
//   0x0424 -      0 			0			0			0			0			1			0			0			0			0			1			0			0			1			0			0
//
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx


//	TXFFST4-0 	00000 Transmit FIFO is empty.
//				00001 Transmit FIFO has 1 words
//				00010 Transmit FIFO has 2 words
//				00011 Transmit FIFO has 3 words
//				00100 Transmit FIFO has 4 words
//
//	RXFFST4−0 	00000 Receive FIFO is empty
//				00001 Receive FIFO has 1 word
//				00010 Receive FIFO has 2 words
//				00011 Receive FIFO has 3 words
//				00100 Receive FIFO has 4 words, the maximum allowed




	SciaRegs.SCIFFRX.all = 0x0121;

	SciaRegs.SCIFFCT.all = 0x00;

	SciaRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset

	SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 1;
	SciaRegs.SCIFFRX.bit.RXFIFORESET  = 1;

}

void scia_xmit(int a)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {}
    SciaRegs.SCITXBUF=a;

}

void scia_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scia_xmit(msg[i]);
        i++;
    }
}


//			  SSSSSS    CCCCCC	 IIII 		  PPPPPPP
//	  *		 SS    SS  CC    CC   II  		  PP    PP      *
//	 ***	 SS        CC         II  		  PP    PP     *** 		#########################################################################################
//	*****	  SSSSSS   CC         II  ======  PPPPPPP     *****  	#########################################################################################
//	 ***	       SS  CC		  II  		  PP	PP     ***		#########################################################################################
//	  *		 SS    SS  CC    CC   II  		  PP	PP      *
//            SSSSSS    CCCCCC   IIII 		  PPPPPPP

void scib_fifo_init() {

	ScibRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
									// No parity,8 char bits,
									// async mode, idle-line protocol
	ScibRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
									// Disable RX ERR, SLEEP, TXWAKE

	ScibRegs.SCICTL2.bit.TXINTENA   = 0;
	ScibRegs.SCICTL2.bit.RXBKINTENA = 1;


#ifdef _9600_BUAD

	ScibRegs.SCIHBAUD    = 0x0001;  // 9600 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK). |
	ScibRegs.SCILBAUD    = 0x0024;  //                                              v 0x1 concatenated with 0x24 = 0x124 = 0b100100100 = 292

#endif

#ifdef _19200_BUAD

	ScibRegs.SCIHBAUD    =0x0000;  // 19200 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK). |
	ScibRegs.SCILBAUD    =0x0091;  //                                               v 0x91 = 145

#endif

#ifdef _38400_BUAD

	ScibRegs.SCIHBAUD    =0x0000;  // 38400 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK). |
	ScibRegs.SCILBAUD    =0x0048;  //                                               v 0x48 = 72

#endif

#ifdef _57600_BUAD

	ScibRegs.SCIHBAUD    =0x0000;  // 57600 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK). |
	ScibRegs.SCILBAUD    =0x0030;  //                                               v 0x30 = 48

#endif


#ifdef _115200_BUAD

	ScibRegs.SCIHBAUD    = 0x0000;  // 115200 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK). |
	ScibRegs.SCILBAUD    = 0x0017;  //                                                v 0x17 = 23

#endif

	ScibRegs.SCICCR.bit.LOOPBKENA = 0; // Enable loop back

	ScibRegs.SCIFFTX.all = 0xC121;
	ScibRegs.SCIFFRX.all = 0x0121;

	ScibRegs.SCIFFCT.all = 0x00;

	ScibRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset

	ScibRegs.SCIFFTX.bit.TXFIFOXRESET = 1;
	ScibRegs.SCIFFRX.bit.RXFIFORESET  = 1;

}

void scib_xmit(int a) {



    while (ScibRegs.SCIFFTX.bit.TXFFST != 0) {}
    ScibRegs.SCITXBUF=a;

}

void scib_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scib_xmit(msg[i]);
        i++;
    }
}

#endif
