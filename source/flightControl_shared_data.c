//###########################################################################
// Description:
//
// Declare shared memory variables and assign them to specific CLA-accessible
// memory locations
//
//! \addtogroup f2806x_example_cla_list
//! \b Memory \b Allocation \n
//!  - CLA1 Data RAM 1 (RAML2)
//!    - fCoeffs - Filter Coefficients
//!    - fDelayLine - Delay line memory elements
//!  - CLA1 to CPU Message RAM
//!    - xResult - Result of the FIR operation
//!  - CPU to CLA1 Message RAM
//!    - xAdcInput - Simulated ADC input
//
//###########################################################################
// $TI Release: F2806x C/C++ Header Files and Peripheral Examples V141 $
// $Release Date: January 19, 2015 $
// $Copyright: Copyright (C) 2011-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################


#include "flightControl.h"
#include "DSP28x_Project.h"
// Include the test header file whose name is based on the test name
// which is defined by the macro TEST on the command line
#include "flightControl_shared.h"
#include "compileGuard.h"
#include "global.h"

//GLobal Data
 //Ensure that all data is placed in the data rams

//Task 1 (C) Variables

//Task 2 (C) Variables

//Task 3 (C) Variables

//Task 4 (C) Variables


//			MM     MM   PPPPPPP    UU    UU
//	  		MMMM MMMM   PP    PP   UU    UU
//	*****	MM MMM MM   PP    PP   UU    UU   *****		#########################################################################################################
//	*****	MM  M  MM   PPPPPPP    UU    UU   *****  	#########################################################################################################
//	*****	MM     MM   PP		   UU    UU   ***** 	#########################################################################################################
//	  		MM     MM   PP	       UU    UU
//			MM     MM   PP          UUUUUU

// MPU CPU to CLA varibales ##################################################################

#pragma DATA_SECTION(mpu6050_fifoBuffer,"CpuToCla1MsgRAM");
Uint16 mpu6050_fifoBuffer[16];

#pragma DATA_SECTION(biasFlag,"CpuToCla1MsgRAM")
Uint16 biasFlag = 0;


// MPU CLA data RAM varibales ##################################################################
#pragma DATA_SECTION(bias,"Cla1DataRam2")
float  bias = 0.0;

#pragma DATA_SECTION(roll,"Cla1DataRam2")
float roll = 0.0;

#pragma DATA_SECTION(pitch,"Cla1DataRam2")
float pitch = 0.0;

#pragma DATA_SECTION(yaw,"Cla1DataRam2")
float yaw = 0.0;



#pragma DATA_SECTION(oldYaw,"Cla1DataRam1")
float oldYaw = 0.0;

#pragma DATA_SECTION(tempYaw,"Cla1DataRam1")
float tempYaw = 0.0;

#pragma DATA_SECTION(yawCount,"Cla1DataRam1")
int16 yawCount = 0;

// MPU CLA data to CPU varibales ##################################################################

#pragma DATA_SECTION(pitchData,"Cla1ToCpuMsgRAM")				//
Uint16 pitchData;

#pragma DATA_SECTION(rollData,"Cla1ToCpuMsgRAM")				//
Uint16 rollData;

#pragma DATA_SECTION(yawData,"Cla1ToCpuMsgRAM")					//
Uint16 yawData;

#pragma DATA_SECTION(dataOffset,"Cla1ToCpuMsgRAM")				//
float dataOffset;



//			FFFFFFFF   IIII   LL		TTTTTTTT	EEEEEEEE   RRRRRRR 	   SSSSSS
//	 		FF		    II    LL           TT       EE         RR    RR   SS    SS
//	*****	FF		    II    LL           TT       EE         RR    RR	  SS         *****	  ##################################################################
//	*****	FFFF	    II    LL           TT       EEEE       RRRRRRR     SSSSSS    *****    ##################################################################
//	*****	FF		    II	  LL      	   TT		EE         RR  RR	        SS   *****	  ##################################################################
//	   		FF		    II    LL           TT       EE         RR   RR    SS    SS
//			FF		   IIII   LLLLLLLL     TT       EEEEEEEE   RR    RR    SSSSSS

// "Global" filter CPU to CLA varibales ##################################################################

#pragma DATA_SECTION(received_control,"CpuToCla1MsgRAM")	//
Uint16 received_control = FALSE;							// Flaging that CPU received flight control from remote

#pragma DATA_SECTION(thrust,"CpuToCla1MsgRAM")	    		//
Uint16 thrust = 500.0;									    // Thrust is given by the operator

#pragma DATA_SECTION(thrust_scale,"CpuToCla1MsgRAM")	    //
Uint16 thrust_scale = _THRUST_SCALE;							//Thrust scaling factor

#pragma DATA_SECTION(sensor_ready,"CpuToCla1MsgRAM")		//
Uint16 sensor_ready = 0;						  			// Set to TRUE when yaw drift is eliminated

#pragma DATA_SECTION(sensor_init,"CpuToCla1MsgRAM")			//
Uint16 sensor_init = 0;										// Number of times the CLA task 4 has been executed

#pragma DATA_SECTION(target_scale,"CpuToCla1MsgRAM")		//
float target_scale = 0;						  				// Max angle ±MAX_ANGLE°, the macro MAX_ANGLE is defined in global.h







// "Global" filter CLA data RAM varibales ##################################################################

#pragma DATA_SECTION(target_norm,"Cla1DataRam2")			//
Uint16 target_norm = 500;									// Normalizing target into range of ±500

#pragma DATA_SECTION(thrust_CLA,"Cla1DataRam2")				//
Uint16 thrust_CLA = 500; 										//

#pragma DATA_SECTION(thrust_bias,"Cla1DataRam2")			//
float thrust_bias = 0;										// Normalized thrust given by the operator

#pragma DATA_SECTION(h,"Cla1DataRam2")						//
float h = MPU_RATE_S;										// The trapezoidal step size

#pragma DATA_SECTION(h_inv,"Cla1DataRam2")					// Multiplicative inverse of the step size (to avoid a division in the D-path of the filter,
float h_inv = MPU_RATE_HZ;									// a division take 9 clock cycles)

#pragma DATA_SECTION(one_half,"Cla1DataRam2")				//
float one_half = 0.5;							    		// (To avoid a division in the I-path of the filter)

#pragma DATA_SECTION(max_out,"Cla1DataRam2")				//
float max_out = MAX_OUT;									// Max allowed output for the control filters

#pragma DATA_SECTION(mean_count,"Cla1DataRam2")				//
float mean_count = 0; 										//






//					PPPPPPP    IIII   TTTTTTTT   CCCCCC	   HH    HH
//			  *		PP    PP    II       TT     CC    CC   HH    HH     *
//			 ***	PP    PP    II       TT     CC         HH    HH    *** 		#################################################################################
//          *****	PPPPPPP     II       TT     CC         HHHHHHHH   *****  	#################################################################################
//			 ***	PP	  ..	II		 TT		CC		   HH    HH    ***		#################################################################################
//			  *		PP	  ..    II       TT     CC    CC   HH    HH     *
//					PP.....    IIII      TT      CCCCCC    HH    HH



// Pitch CPU to CLA varibales ===================================================================================================================================

#pragma DATA_SECTION(received_gain_pitch,"CpuToCla1MsgRAM")		//
Uint16 received_gain_pitch = FALSE;								// Flaging that CPU received filter gain from remote

#pragma DATA_SECTION(target_pitch,"CpuToCla1MsgRAM")			//
Uint16 target_pitch = 500;										// Target pitch value received from user

#pragma DATA_SECTION(uint_Kp_pitch,"CpuToCla1MsgRAM")			//
Uint16 uint_Kp_pitch = 0;										// Kp sent from remote

#pragma DATA_SECTION(uint_Ki_pitch,"CpuToCla1MsgRAM")			//
Uint16 uint_Ki_pitch = 0;										// Ki sent from remote

#pragma DATA_SECTION(uint_Kd_pitch,"CpuToCla1MsgRAM")			//
Uint16 uint_Kd_pitch = 0;										// Kd sent from remote

#pragma DATA_SECTION(uint_Kp_ff_pitch,"CpuToCla1MsgRAM")		//
Uint16 uint_Kp_ff_pitch = 0;									// Kp_ff sent from remote

#pragma DATA_SECTION(uint_gain_gyro_pitch,"CpuToCla1MsgRAM")	//
Uint16 uint_gain_gyro_pitch = 0;								// Gyro gain sent from remote

#pragma DATA_SECTION(trim_pitch,"CpuToCla1MsgRAM")				//
Uint16 trim_pitch;												// Max allowed output for the control filters


// Pitch CLA to CPU varibales ===================================================================================================================================



// --------------------------------------------------------------------------------------------------------------------------------------------------------------






// Pitch CLA data RAM varibales =================================================================================================================================================

//#pragma DATA_SECTION(motor1AB_value,"Cla1DataRam2")		//
//Uint16 motor1AB_value = 0;								// Target pitch value received from user

#pragma DATA_SECTION(ADC_gyro_pitch_sum,"Cla1DataRam2")		//
Uint16 ADC_gyro_pitch_sum = 0; 								//

#pragma DATA_SECTION(target_pitch_CLA,"Cla1DataRam2")		//
float target_pitch_CLA = 500;

#pragma DATA_SECTION(trim_pitch_f,"Cla1DataRam2")			//
float trim_pitch_f = 500;

#pragma DATA_SECTION(Kp_pitch,"Cla1DataRam2")				//
float Kp_pitch = KP_PITCH;									// Proportional gain

#pragma DATA_SECTION(Ki_pitch,"Cla1DataRam2")				//
float Ki_pitch = KI_PITCH;									// Integral gain

#pragma DATA_SECTION(Kd_pitch,"Cla1DataRam2")				//
float Kd_pitch = KD_PITCH;									// Derivative gain

#pragma DATA_SECTION(Kp_ff_pitch,"Cla1DataRam2")			//
float Kp_ff_pitch = KP_FF_PITCH;							// P gain of second controller

#pragma DATA_SECTION(gain_gyro_pitch,"Cla1DataRam2")		//
float gain_gyro_pitch = PITCH_FF_GAIN;						// Gyro gain (angle velocity)



#pragma DATA_SECTION(error_pitch,"Cla1DataRam2")			//
float error_pitch = 0;										// Current error

#pragma DATA_SECTION(error_old_pitch,"Cla1DataRam2")		//
float error_old_pitch = 0;									// Old error

#pragma DATA_SECTION(P_pitch,"Cla1DataRam2")				//
float P_pitch = 0;											// Proportional part

#pragma DATA_SECTION(I_partial_pitch,"Cla1DataRam2")		//
float I_partial_pitch = 0;									// Current integral slice



#pragma DATA_SECTION(I_pitch,"Cla1DataRam2")				//
float I_pitch = 0;											// Sum of previous integral slices

#pragma DATA_SECTION(I_term_max_pitch,"Cla1DataRam2")		//
float I_term_max_pitch = MAX_I;								// Clamping the I-term to a predefined max (global.h)

#pragma DATA_SECTION(D_pitch,"Cla1DataRam2")				//
float D_pitch = 0;											// Derivative part

#pragma DATA_SECTION(pitch_old,"Cla1DataRam2")				//
float pitch_old = 0;							    		// Old pitch value, used for calculate the derivative



#pragma DATA_SECTION(out_first_pitch,"Cla1DataRam2")		// Pitch filter output
float out_first_pitch = 0;

#pragma DATA_SECTION(out_second_pitch,"Cla1DataRam2")		// Pitch filter output
float out_second_pitch = 0;

#pragma DATA_SECTION(gyro_unbiased_pitch,"Cla1DataRam2")	//
float gyro_unbiased_pitch = 1;								// Max allowed output for the control filters

#pragma DATA_SECTION(mean_offset_pitch,"Cla1DataRam2")		//
float mean_offset_pitch;									// Max allowed output for the control filters




//					 RRRRRRR       OOOOOO	  LL		 LL
//			  *		 RR    RR     OO    OO	  LL         LL      	   *
//			 ***	 RR    RR    OO		 OO	  LL         LL      	  *** 	#########################################################################################
//			*****	 RRRRRRR     OO		 OO	  LL         LL      	 *****  #########################################################################################
//			 ***	 RR   RR	 OO		 OO	  LL         LL      	  ***	#########################################################################################
//			  *		 RR    RR     OO	OO	  LL         LL      	   *
//					 RR     RR     OOOOOO	  LLLLLLLL   LLLLLLLL



// roll CPU to CLA varibales ===================================================================================================================================


#pragma DATA_SECTION(received_gain_roll,"CpuToCla1MsgRAM")		//
Uint16 received_gain_roll = FALSE;								// Flaging that CPU received filter gain from remote

#pragma DATA_SECTION(target_roll,"CpuToCla1MsgRAM")				//
Uint16 target_roll = 500;										// Target roll value received from user

#pragma DATA_SECTION(uint_Kp_roll,"CpuToCla1MsgRAM")			//
Uint16 uint_Kp_roll = 0;										// Kp sent from remote

#pragma DATA_SECTION(uint_Ki_roll,"CpuToCla1MsgRAM")			//
Uint16 uint_Ki_roll = 0;										// Ki sent from remote

#pragma DATA_SECTION(uint_Kd_roll,"CpuToCla1MsgRAM")			//
Uint16 uint_Kd_roll = 0;										// Kd sent from remote

#pragma DATA_SECTION(uint_Kp_ff_roll,"CpuToCla1MsgRAM")			//
Uint16 uint_Kp_ff_roll = 0;										// Kp_ff sent from remote

#pragma DATA_SECTION(uint_gain_gyro_roll,"CpuToCla1MsgRAM")		//
Uint16 uint_gain_gyro_roll = 0;									// Gyro gain sent from remote

#pragma DATA_SECTION(trim_roll,"CpuToCla1MsgRAM")				//
Uint16 trim_roll;												// Max allowed output for the control filters



// roll CLA to CPU varibales ===================================================================================================================================



// --------------------------------------------------------------------------------------------------------------------------------------------------------------






// roll CLA data RAM varibales =================================================================================================================================================


#pragma DATA_SECTION(ADC_gyro_roll_sum,"Cla1DataRam2")		//
Uint16 ADC_gyro_roll_sum = 0; 								//

#pragma DATA_SECTION(target_roll_CLA,"Cla1DataRam2")		//
float target_roll_CLA = 500;

#pragma DATA_SECTION(trim_roll_f,"Cla1DataRam2")			//
float trim_roll_f = 500;

#pragma DATA_SECTION(Kp_roll,"Cla1DataRam2")				//
float Kp_roll = KP_ROLL;									// Proportional gain

#pragma DATA_SECTION(Ki_roll,"Cla1DataRam2")				//
float Ki_roll = KI_ROLL;									// Integral gain

#pragma DATA_SECTION(Kd_roll,"Cla1DataRam2")				//
float Kd_roll = KD_ROLL;									// Derivative gain

#pragma DATA_SECTION(Kp_ff_roll,"Cla1DataRam2")				//
float Kp_ff_roll = KP_FF_ROLL;								// P gain of second controller

#pragma DATA_SECTION(gain_gyro_roll,"Cla1DataRam2")			//
float gain_gyro_roll = ROLL_FF_GAIN;						// Gyro gain (angle velocity)



#pragma DATA_SECTION(error_roll,"Cla1DataRam2")				//
float error_roll = 0;										// Current error

#pragma DATA_SECTION(error_old_roll,"Cla1DataRam2")			//
float error_old_roll = 0;									// Old error

#pragma DATA_SECTION(P_roll,"Cla1DataRam2")					//
float P_roll = 0;											// Proportional part

#pragma DATA_SECTION(I_partial_roll,"Cla1DataRam2")			//
float I_partial_roll = 0;									// Current integral slice



#pragma DATA_SECTION(I_roll,"Cla1DataRam2")					//
float I_roll = 0;											// Sum of previous integral slices

#pragma DATA_SECTION(I_term_max_roll,"Cla1DataRam2")		//
float I_term_max_roll = MAX_I;								// Clamping the I-term to a predefined max (global.h)

#pragma DATA_SECTION(D_roll,"Cla1DataRam2")					//
float D_roll = 0;											// Derivative part

#pragma DATA_SECTION(roll_old,"Cla1DataRam2")				//
float roll_old = 0;							    			// Old roll value, used for calculate the derivative



#pragma DATA_SECTION(out_first_roll,"Cla1DataRam2")			// roll filter output
float out_first_roll = 0;

#pragma DATA_SECTION(out_second_roll,"Cla1DataRam2")		// roll filter output
float out_second_roll = 0;

#pragma DATA_SECTION(gyro_unbiased_roll,"Cla1DataRam2")		//
float gyro_unbiased_roll = 1;								// Max allowed output for the control filters

#pragma DATA_SECTION(mean_offset_roll,"Cla1DataRam2")		//
float mean_offset_roll;										// Max allowed output for the control filters







//					YY    YY	AAA     WW		WW  ..	  ..
//			  *		 YY  YY    AA  AA   WW      WW  ....  ..     *
//			 ***	  YYYY    AA    AA  WW      WW  .. .. ..    ***	  ############################################################################################
//			*****	   YY     AAAAAAAA  WW      WW  ..  ....   *****  ############################################################################################
//			 ***	   YY     AA    AA   WW WW WW   ..   ...    ***	  ############################################################################################
//			  *		   YY     AA    AA    WW  WW    ..    ..     *



// yaw CPU to CLA varibales ===================================================================================================================================

#pragma DATA_SECTION(received_gain_yaw,"CpuToCla1MsgRAM")	//
Uint16 received_gain_yaw = FALSE;							// Flaging that CPU received filter gain from remote

#pragma DATA_SECTION(target_yaw,"CpuToCla1MsgRAM")			//
Uint16 target_yaw = 500;									// Target yaw value received from user

#pragma DATA_SECTION(uint_Kp_yaw,"CpuToCla1MsgRAM")			//
Uint16 uint_Kp_yaw = 0;										// Kp sent from remote

#pragma DATA_SECTION(uint_Ki_yaw,"CpuToCla1MsgRAM")			//
Uint16 uint_Ki_yaw = 0;										// Ki sent from remote

#pragma DATA_SECTION(uint_Kd_yaw,"CpuToCla1MsgRAM")			//
Uint16 uint_Kd_yaw = 0;										// Kd sent from remote

#pragma DATA_SECTION(uint_Kp_ff_yaw,"CpuToCla1MsgRAM")		//
Uint16 uint_Kp_ff_yaw = 0;									// Kp_ff sent from remote

#pragma DATA_SECTION(uint_gain_gyro_yaw,"CpuToCla1MsgRAM")	//
Uint16 uint_gain_gyro_yaw = 0;								// Gyro gain sent from remote

#pragma DATA_SECTION(trim_yaw,"CpuToCla1MsgRAM")			//
Uint16 trim_yaw;											// Max allowed output for the control filters


// yaw CLA to CPU varibales ===================================================================================================================================



// --------------------------------------------------------------------------------------------------------------------------------------------------------------






// yaw CLA data RAM varibales =================================================================================================================================================

#pragma DATA_SECTION(ADC_gyro_yaw_sum,"Cla1DataRam2")		//
Uint16 ADC_gyro_yaw_sum = 0; 								//

#pragma DATA_SECTION(yaw_count,"Cla1DataRam2")				//
Uint16 yaw_count = 0; 										//

#pragma DATA_SECTION(yaw_target_scale,"Cla1DataRam2")		//
Uint16 yaw_target_scale = _YAW_TARGET_SCALE;

#pragma DATA_SECTION(trim_yaw_f,"Cla1DataRam2")			//
float trim_yaw_f = 500;

#pragma DATA_SECTION(target_yaw_CLA,"Cla1DataRam2")			//
float target_yaw_CLA = 0;

#pragma DATA_SECTION(Kp_yaw,"Cla1DataRam2")					//
float Kp_yaw = KP_YAW;										// Proportional gain

#pragma DATA_SECTION(Ki_yaw,"Cla1DataRam2")					//
float Ki_yaw = KI_YAW;										// Integral gain

#pragma DATA_SECTION(Kd_yaw,"Cla1DataRam2")					//
float Kd_yaw = KD_YAW;										// Derivative gain

#pragma DATA_SECTION(Kp_ff_yaw,"Cla1DataRam2")				//
float Kp_ff_yaw = KP_FF_YAW;								// P gain of second contyawer

#pragma DATA_SECTION(gain_gyro_yaw,"Cla1DataRam2")			//
float gain_gyro_yaw = YAW_FF_GAIN;							// Gyro gain (angle velocity)



#pragma DATA_SECTION(error_yaw,"Cla1DataRam2")				//
float error_yaw = 0;										// Current error

#pragma DATA_SECTION(error_old_yaw,"Cla1DataRam2")			//
float error_old_yaw = 0;									// Old error

#pragma DATA_SECTION(P_yaw,"Cla1DataRam2")					//
float P_yaw = 0;											// Proportional part

#pragma DATA_SECTION(I_partial_yaw,"Cla1DataRam2")			//
float I_partial_yaw = 0;									// Current integral slice



#pragma DATA_SECTION(I_yaw,"Cla1DataRam2")					//
float I_yaw = 0;											// Sum of previous integral slices

#pragma DATA_SECTION(I_term_max_yaw,"Cla1DataRam2")		    //
float I_term_max_yaw = MAX_I;								// Clamping the I-term to a predefined max (global.h)

#pragma DATA_SECTION(D_yaw,"Cla1DataRam2")					//
float D_yaw = 0;											// Derivative part

#pragma DATA_SECTION(yaw_old,"Cla1DataRam2")				//
float yaw_old = 0;							    			// Old yaw value, used for calculate the derivative



#pragma DATA_SECTION(out_first_yaw,"Cla1DataRam2")			// yaw filter output
float out_first_yaw = 0;

#pragma DATA_SECTION(out_second_yaw,"Cla1DataRam2")			// yaw filter output
float out_second_yaw = 0;

#pragma DATA_SECTION(gyro_unbiased_yaw,"Cla1DataRam2")		//
float gyro_unbiased_yaw = 0;								// Max allowed output for the control filters

#pragma DATA_SECTION(mean_offset_yaw,"Cla1DataRam2")		//
float mean_offset_yaw;										// Max allowed output for the control filters


//Task 5 (C) Variables

//Task 6 (C) Variables

//Task 7 (C) Variables

//Task 8 (C) Variables

//Common (C) Variables
//#pragma DATA_SECTION(CLAasinTable,"CLA1mathTables");
//float CLAasinTable[]={
//0.0, 1.0, 0.0,
//0.000000636202, 0.999877862610, 0.007815361896,
//0.000005099694, 0.999510644409, 0.015647916155,
//0.000017268312, 0.998895919094, 0.023514960332,
//0.000041121765, 0.998029615282, 0.031434003631,
//0.000080794520, 0.996905974725, 0.039422875916,
//0.000140631089, 0.995517492804, 0.047499840611,
//0.000225244584, 0.993854840311, 0.055683712914,
//0.000339579512, 0.991906765146, 0.063993984848,
//0.000488979852, 0.989659972212, 0.072450958820,
//0.000679263611, 0.987098979366, 0.081075891529,
//0.000916805182, 0.984205946802, 0.089891150305,
//0.001208627040, 0.980960476685, 0.098920384204,
//0.001562502549, 0.977339379243, 0.108188712551,
//0.001987071928, 0.973316400729, 0.117722933997,
//0.002491973784, 0.968861907789, 0.127551759665,
//0.003087995053, 0.963942521723, 0.137706074532,
//0.003787242692, 0.958520694794, 0.148219231941,
//0.004603341138, 0.952554219267, 0.159127386977,
//0.005551660294, 0.945995657913, 0.170469875522,
//0.006649579796, 0.938791682505, 0.182289647088,
//0.007916796475, 0.930882303984, 0.194633761132,
//0.009375683410, 0.922199974574, 0.207553958472,
//0.011051710808, 0.912668537890, 0.221107321885,
//0.012973941175, 0.902201997769, 0.235357042896,
//0.015175614174, 0.890703070035, 0.250373315541,
//0.017694840102, 0.878061473098, 0.266234382514,
//0.020575425537, 0.864151902887, 0.283027765009,
//0.023867860513, 0.848831624374, 0.300851714968,
//0.027630504055, 0.831937595031, 0.319816937941,
//0.031931014547, 0.813283013821, 0.340048646894,
//0.036848083955, 0.792653161200, 0.361689022958,
//0.042473551274, 0.769800358920, 0.384900179460,
//0.048914992206, 0.744437830278, 0.409867752228,
//0.056298910750, 0.716232177740, 0.436805274317,
//0.064774696786, 0.684794109766, 0.465959540059,
//0.074519565699, 0.649666934178, 0.497617226179,
//0.085744766889, 0.610312179660, 0.532113122767,
//0.098703445606, 0.566091493186, 0.569840443472,
//0.113700678529, 0.516243664372, 0.611263845480,
//0.131106395009, 0.459855210927, 0.656936015611,
//0.151372169232, 0.395822366759, 0.707518998893,
//0.175053263659, 0.322801460177, 0.763811905770,
//0.202837883870, 0.239143420888, 0.826787304376,
//0.235586468765, 0.142806299514, 0.897639596948,
//0.274385149825, 0.031236880585, 0.977850174820,
//0.320619535938, -0.098791845166, 1.069276441800,
//0.376078169620, -0.251407364538, 1.174275392129,
//0.443100143614, -0.431959397725, 1.295878193174,
//0.524789871827, -0.647485610469, 1.438041695773,
//0.625336471263, -0.907400624736, 1.606018804842,
//0.750500589935, -1.224540947101, 1.806917563896,
//0.908377657341, -1.616794995066, 2.050569262035,
//1.110633894185, -2.109729648039, 2.350920816737,
//1.374584721437, -2.740985157716, 2.728353889708,
//1.726848242753, -3.567962877198, 3.213722960014,
//2.210117561056, -4.682006534082, 3.855770086891,
//2.896554011854, -6.236312386687, 4.735651038017,
//3.916505715382, -8.505488022524, 5.997790945975,
//5.526855868703, -12.026617159136, 7.922628470498,
//8.298197116322, -17.983705080358, 11.123941286820,
//13.741706072449, -29.488929624542, 17.203344479111,
//27.202707817485, -57.466598393615, 31.741016484669,
//83.158101335898, -171.803399517566, 90.149831709374
//};
//
//#pragma DATA_SECTION(CLAatan2Table,"CLA1mathTables");
//float CLAatan2Table[]={
//0.000000000000, 1.000040679675, -0.007811069750,
//-0.000003807022, 1.000528067772, -0.023410345493,
//-0.000018968310, 1.001498568106, -0.038941197075,
//-0.000052831927, 1.002943680965, -0.054358563160,
//-0.000112417996, 1.004850785788, -0.069618206649,
//-0.000204295885, 1.007203293243, -0.084677028723,
//-0.000334469493, 1.009980843501, -0.099493367628,
//-0.000508272606, 1.013159547023, -0.114027278430,
//-0.000730276035, 1.016712263449, -0.128240790343,
//-0.001004207980, 1.020608913557, -0.142098138715,
//-0.001332888717, 1.024816818801, -0.155565969259,
//-0.001718180431, 1.029301062569, -0.168613512667,
//-0.002160952612, 1.034024867135, -0.181212728329,
//-0.002661063159, 1.038949980222, -0.193338416410,
//-0.003217354977, 1.044037065160, -0.204968298146,
//-0.003827667546, 1.049246088868, -0.216083064706,
//-0.004488862702, 1.054536702192, -0.226666395507,
//-0.005196863579, 1.059868607573, -0.236704947279,
//-0.005946705500, 1.065201909527, -0.246188315613,
//-0.006732597422, 1.070497443994, -0.255108971022,
//-0.007547992413, 1.075717083222, -0.263462171840,
//-0.008385665599, 1.080824013499, -0.271245856476,
//-0.009237797924, 1.085782983693, -0.278460517693,
//-0.010096064139, 1.090560523211, -0.285109061650,
//-0.010951723407, 1.095125128557, -0.291196654447,
//-0.011795711029, 1.099447418304, -0.296730558920,
//-0.012618729873, 1.103500256784, -0.301719964310,
//-0.013411340199, 1.107258847274, -0.306175811325,
//-0.014164046711, 1.110700795887, -0.310110614947,
//-0.014867381831, 1.113806147717, -0.313538287176,
//-0.015511984298, 1.116557397059, -0.316473961655,
//-0.016088672395, 1.118939473766, -0.318933821948,
//-0.016588511229, 1.120939707956, -0.320934934991,
//-0.017002873645, 1.122547775367, -0.322495091022,
//-0.017323494499, 1.123755625749, -0.323632651068,
//-0.017542518140, 1.124557396633, -0.324366402860,
//-0.017652539065, 1.124949314810, -0.324715425833,
//-0.017646635823, 1.124929587773, -0.324698965683,
//-0.017518398344, 1.124498287266, -0.324336318787,
//-0.017261948920, 1.123657226955, -0.323646726613,
//-0.016871957164, 1.122409836098, -0.322649280139,
//-0.016343649294, 1.120761030918, -0.321362834119,
//-0.015672812166, 1.118717085258, -0.319805931024,
//-0.014855792451, 1.116285501852, -0.317996734279,
//-0.013889491441, 1.113474885484, -0.315952970460,
//-0.012771355912, 1.110294819035, -0.313691879968,
//-0.011499365513, 1.106755743329, -0.311230175714,
//-0.010072017131, 1.102868841531, -0.308584009287,
//-0.008488306656, 1.098645928648, -0.305768944066,
//-0.006747708579, 1.094099346648, -0.302799934724,
//-0.004850153815, 1.089241865523, -0.299691312610,
//-0.002796006119, 1.084086590517, -0.296456776409,
//-0.000586037441, 1.078646875666, -0.293109387609,
//0.001778597452, 1.072936243727, -0.289661570229,
//0.004296386804, 1.066968312440, -0.286125114351,
//0.006965487894, 1.060756727064, -0.282511182960,
//0.009783753017, 1.054315099009, -0.278830321668,
//0.012748755207, 1.047656950440, -0.275092470943,
//0.015857813546, 1.040795664582, -0.271306980411,
//0.019108017893, 1.033744441476, -0.267482624928,
//0.022496252887, 1.026516258989, -0.263627622089,
//0.026019221162, 1.019123838651, -0.259749650860,
//0.029673465636, 1.011579616186, -0.255855871106,
//0.033455390838, 1.003895716322, -0.251952943763,
//0.037361283212, 0.996083931611, -0.248047051426
//};
//


