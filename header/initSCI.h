/*
 * sci.h
 *
 *  Created on: 25 apr 2015
 *      Author: knalle
 */

#ifndef SCI_H_
#define SCI_H_

#include "DSP28x_Project.h"
#include "compileGuard.h"
//#include "ISR.h"

//			 RRRRRR 	EEEEEEEE   CCCCCC	EEEEEEEE  IIII	VV	   VV  EEEEEEEE
//	  *		 RR   RR    EE        CC    CC  EE         II 	VV     VV  EE
//	 ***	 RR   RR	EE        CC        EE         II 	VV     VV  EE
//	*****	 RRRRRR     EEEE      CC        EEEE       II 	VV     VV  EEEE
//	 ***	 RR  RR	    EE        CC		EE         II	 VV   VV   EE
//	  *		 RR   RR    EE        CC    CC  EE         II     VV VV    EE
//			 RR    RR   EEEEEEEE   CCCCCC   EEEEEEEE  IIII     VVV     EEEEEEEE
//
//
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx



//			      Flight control ~
//				       0xFFFE   	  |		 Thrust	 	   ||	  Yaw target	 ||	   Pitch target	   ||	Roll target  	 ||	       Tail	 	   |
//				_________  _________  V_________  _________VV_________  _________VV_________  _________VV_________  _________VV_________  _________V
//			   |         ||         | |         ||         ||         ||         ||         ||         ||         ||         ||         ||         |
//	12 Bytes   | Header1 || Header2 | | byte1.1 || byte2.1 || byte1.2 || byte2.2 || byte1.3 || byte2.3 || byte1.4 || byte2.4 ||  Tail1  || 	Tail2  |
//			   |_________||_________| |_________||_________||_________||_________||_________||_________||_________||_________||_________||_________|
//
//
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//
//									         Pitch/	           		Pitch/				  Pitch/			  Pitch/			    Pitch/         	      Pitch/         	    Pitch/
//								             Roll/ 		       		Roll/                 Roll/               Roll/                 Roll/                 Roll/                 Roll/
//				  Pitch ~ 0xFDFC		     Yaw         		   	Yaw					  Yaw				  Yaw			        Yaw                   Yaw                   Yaw
//			      Roll  ~ 0xFBFA		     P gain 	           	I gain 	              D gain 		      Pff gain              Gyro gain             Thrust                Offset Trim
//				  Yaw   ~ 0xF9F8	 |	  				  ||					||					  ||                    ||        	          ||        	        ||        	          ||	    Tail	    |
//			   _________  _________  V_________  _________VV_________  _________VV_________  _________VV_________  _________VV_________  _________VV_________  _________VV_________  _________VV_________  _________V
//			  |         ||         | |         ||         ||         ||         ||         ||         ||         ||         ||         ||         ||         ||         ||         ||         ||         ||         |
//	14 Bytes  | Header1 || Header2 | | byte1.1 || byte2.1 || byte1.2 || byte2.2 || byte1.3 || byte2.3 || byte1.4 || byte2.4 || byte1.5 || byte2.5 || byte1.6 || byte2.6 || byte1.7 || byte2.7 ||  Tail1  ||  Tail2  |
//			  |_________||_________| |_________||_________||_________||_________||_________||_________||_________||_________||_________||_________||_________||_________||_________||_________||_________||_________|
//
//
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

#define FIRST_BYTE_OF_HEADER  	 ((receivedByteA == HEADER1_FLIGHT_CONTROL) || (receivedByteA == HEADER1_FILTER_GAIN_PITCH) || (receivedByteA == HEADER1_FILTER_GAIN_ROLL) || (receivedByteA == HEADER1_FILTER_GAIN_YAW))
#define SECOND_BYTE_OF_HEADER 	 ((receivedByteA == HEADER2_FLIGHT_CONTROL) || (receivedByteA == HEADER2_FILTER_GAIN_PITCH) || (receivedByteA == HEADER2_FILTER_GAIN_ROLL) || (receivedByteA == HEADER2_FILTER_GAIN_YAW))

//#define VALID_HEADER			 ((headerSCI_A == FLIGHT_CONTROL) || (headerSCI_A == FILTER_GAIN_PITCH) || (headerSCI_A == FILTER_GAIN_ROLL) || (headerSCI_A == FILTER_GAIN_YAW))
//#define NOT_VALID_HEADER		 ((headerSCI_A != FLIGHT_CONTROL) && (headerSCI_A != FILTER_GAIN_PITCH) && (headerSCI_A != FILTER_GAIN_ROLL) && (headerSCI_A != FILTER_GAIN_YAW))

#define VALID_HEADER			 ((tempHeaderSCIA == FLIGHT_CONTROL) || (tempHeaderSCIA == FILTER_GAIN_PITCH) || (tempHeaderSCIA == FILTER_GAIN_ROLL) || (tempHeaderSCIA == FILTER_GAIN_YAW))
#define NOT_VALID_HEADER		 ((tempHeaderSCIA != FLIGHT_CONTROL) && (tempHeaderSCIA != FILTER_GAIN_PITCH) && (tempHeaderSCIA != FILTER_GAIN_ROLL) && (tempHeaderSCIA != FILTER_GAIN_YAW))

#define VALID_DATA				 ((tempData != FLIGHT_CONTROL) && (tempData != FILTER_GAIN_PITCH) && (tempData != FILTER_GAIN_ROLL) && (tempData != FILTER_GAIN_YAW))

#define NO_PART_OF_HEADER 		 (!SECOND_BYTE_OF_HEADER) // 1

#define IF_COM_ERROR		     !((receivedByteA == 0x0002) || (receivedByteA == 0x0031) || (receivedByteA == 0x0044) || (receivedByteA == 0x0026) || (receivedByteA == 0x001E) || (receivedByteA == 0x00FF) || (receivedByteA == 0x00FE)  || (receivedByteA == 0x00F7) || (receivedByteA == 0x0012))

//#define _RECEIVE_FIRST_GAIN		 !((firstGainPitch == FALSE) && (firstGainRoll == FALSE) && (firstGainYaw == FALSE))

#ifndef RADIO_DEBUG
#define FLIGHT_CONTROL					0xFFFE
#define HEADER1_FLIGHT_CONTROL			0xFF
#define HEADER2_FLIGHT_CONTROL			0xFE
#define TAIL_FLIGHT_CONTROL				0xF712

#define FILTER_GAIN_PITCH				0xFDFC
#define HEADER1_FILTER_GAIN_PITCH		0xFD
#define HEADER2_FILTER_GAIN_PITCH		0xFC
#define TAIL_FILTER_GAIN_PITCH			0xF612		//

#define FILTER_GAIN_ROLL				0xFBFA
#define HEADER1_FILTER_GAIN_ROLL		0xFB
#define HEADER2_FILTER_GAIN_ROLL		0xFA
#define TAIL_FILTER_GAIN_ROLL			0xF512		//

#define FILTER_GAIN_YAW					0xF9F8
#define HEADER1_FILTER_GAIN_YAW			0xF9
#define HEADER2_FILTER_GAIN_YAW			0xF8
#define TAIL_FILTER_GAIN_YAW			0xF412		//


#define INITIATE_TRANSMISSION			0xFFFD
#define _SEND_FLIGHT_CONTROL			0xFF
#define _INIT_FILTER_GAIN				0xFD
#define _FILTER_GAIN_RECEIVED			0xFC
#endif

#ifdef RADIO_DEBUG
#define FLIGHT_CONTROL					0x6162 		// ab
#define HEADER1_FLIGHT_CONTROL			0x0061		// a
#define HEADER2_FLIGHT_CONTROL			0x0062		// b
#define TAIL_FLIGHT_CONTROL				0x6969		// "i << 8" | i


#define FILTER_GAIN_PITCH				0x6364		// cd
#define HEADER1_FILTER_GAIN_PITCH		0x0063		// c
#define HEADER2_FILTER_GAIN_PITCH		0x0064		// d
#define TAIL_FILTER_GAIN_PITCH			0x6A6A		// "j << 8" | j

#define FILTER_GAIN_ROLL				0x6566		// ef
#define HEADER1_FILTER_GAIN_ROLL		0x0065		// e
#define HEADER2_FILTER_GAIN_ROLL		0x0066		// f
#define TAIL_FILTER_GAIN_ROLL			0x6B6B		// "k << 8" | k

#define FILTER_GAIN_YAW					0x6768		// gh
#define HEADER1_FILTER_GAIN_YAW			0x0067		// g
#define HEADER2_FILTER_GAIN_YAW			0x0068		// h
#define TAIL_FILTER_GAIN_YAW			0x6C6C		// "l << 8" | l
#endif



#define NBR_OF_GAIN_BYTES				14
#define NBR_OF_FLIGHT_CONTROL_BYTES		8

void scia_fifo_init(void);
void scia_xmit(int a);
void scia_msg(char *msg);

void scib_fifo_init(void);
void scib_xmit(int a);
void scib_msg(char *msg);



#endif /* SCI_H_ */
