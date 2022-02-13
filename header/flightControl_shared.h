//###########################################################################
// Description:
//
// Declare shared variables
//
//###########################################################################
// $TI Release: F2806x C/C++ Header Files and Peripheral Examples V141 $
// $Release Date: January 19, 2015 $
// $Copyright: Copyright (C) 2011-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#ifndef FIR32_SHARED_H_
#define FIR32_SHARED_H_
#ifdef __cplusplus
extern "C" {
#endif



#define TABLE_SIZE		64
#define TABLE_SIZE_M_1	TABLE_SIZE-1

#ifdef USE_FLASH
//---------------------------------------------------------------------------
// Global symbols defined in the linker command file
//
extern Uint16 cla1Funcs_loadstart;
extern Uint16 cla1Funcs_loadsize;
extern Uint16 cla1Funcs_runstart;
extern Uint16 secureRamFuncs_loadstart;
extern Uint16 secureRamFuncs_loadsize;
extern Uint16 secureRamFuncs_runstart;
extern Uint16 hwi_vec_loadstart;
extern Uint16 hwi_vec_loadsize;
extern Uint16 hwi_vec_runstart;
extern Uint16 trcdata_loadstart;
extern Uint16 trcdata_loadsize;
extern Uint16 trcdata_runstart;
extern Uint16 Cla1Prog_Start;
#endif

//Task 1 (C) Variables ====================================================================================

//Task 2 (C) Variables ====================================================================================

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

extern Uint16 mpu6050_fifoBuffer[];

extern Uint16 biasFlag;


// MPU CLA data RAM varibales ##################################################################

extern float roll,
			 pitch,
			 yaw,
			 oldYaw,
			 tempYaw;

extern int16 yawCount;

extern float  bias;

// MPU CLA data to CPU varibales ##################################################################

extern Uint16 	pitchData,
				rollData,
				yawData;

extern float	dataOffset;

//			FFFFFFFF   IIII   LL		TTTTTTTT	EEEEEEEE   RRRRRR 	   SSSSSS
//	 		FF		    II    LL           TT       EE         RR   RR    SS    SS
//	*****	FF		    II    LL           TT       EE         RR   RR	  SS         *****	  ####################################################################
//	*****	FFFF	    II    LL           TT       EEEE       RRRRRR      SSSSSS    *****    ####################################################################
//	*****	FF		    II	  LL      	   TT		EE         RR  RR	        SS   *****	  ####################################################################
//	   		FF		    II    LL           TT       EE         RR   RR    SS    SS
//			FF		   IIII   LLLLLLLL     TT       EEEEEEEE   RR    RR    SSSSSS

// "Global" filter CPU to CLA varibales ##################################################################

extern Uint16 	received_control,
				thrust,
				thrust_scale,
				sensor_ready,
				sensor_init;

extern float  	target_scale;

// "Global" filter CLA data RAM varibales ##################################################################

extern Uint16 	target_norm,
				thrust_CLA;

extern float 	thrust_bias,
				h,
				h_inv,
				one_half,
				max_out;


//					PPPPPPP    IIII   TTTTTTTT   CCCCCC	   HH    HH
//			  *		PP    PP    II       TT     CC    CC   HH    HH     *
//			 ***	PP    PP    II       TT     CC         HH    HH    *** 		##################################################################################
//          *****	PPPPPPP     II       TT     CC         HHHHHHHH   *****  	##################################################################################
//			 ***	PP	  ..	II		 TT		CC		   HH    HH    ***		##################################################################################
//			  *		PP	  ..    II       TT     CC    CC   HH    HH     *
//					PP.....    IIII      TT      CCCCCC    HH    HH




// Pitch CPU to CLA varibales ====================================================================================================================================

extern Uint16 	target_pitch,
				received_gain_pitch,
				uint_Kp_pitch,
				uint_Ki_pitch,
				uint_Kd_pitch,
				uint_Kp_ff_pitch,
				uint_gain_gyro_pitch,
				trim_pitch;






// --------------------------------------------------------------------------------------------------------------------------------------------------------------


// Pitch CLA to CPU varibales ===================================================================================================================================



// --------------------------------------------------------------------------------------------------------------------------------------------------------------

// Pitch CLA data RAM varibales =================================================================================================================================

extern Uint16 	ADC_gyro_pitch_sum;

extern float	Kp_pitch,
				Ki_pitch,
				Kd_pitch,
				Kp_ff_pitch,
				gain_gyro_pitch;

extern float	trim_pitch_f,
				target_pitch_CLA,
				error_pitch,
				error_old_pitch,
				P_pitch,
				I_partial_pitch,
				I_pitch,
				I_term_max_pitch,
				D_pitch,
				pitch_old,
				out_first_pitch,
				out_second_pitch,
				mean_offset_pitch,
				mean_count,
				gyro_unbiased_pitch;



//					 RRRRRRR       OOOOOO	  LL		 LL
//			  *		 RR    RR     OO    OO	  LL         LL      	   *
//			 ***	 RR    RR    OO		 OO	  LL         LL      	  *** 	#########################################################################################
//			*****	 RRRRRRR     OO		 OO	  LL         LL      	 *****  #########################################################################################
//			 ***	 RR   RR	 OO		 OO	  LL         LL      	  ***	#########################################################################################
//			  *		 RR    RR     OO	OO	  LL         LL      	   *
//					 RR     RR     OOOOOO	  LLLLLLLL   LLLLLLLL




// Roll CPU to CLA varibales ====================================================================================================================================

extern Uint16 	target_roll,
				received_gain_roll,
				uint_Kp_roll,
				uint_Ki_roll,
				uint_Kd_roll,
				uint_Kp_ff_roll,
				uint_gain_gyro_roll,
				trim_roll;

extern float	Kp_roll,
				Ki_roll,
				Kd_roll;




// --------------------------------------------------------------------------------------------------------------------------------------------------------------


// Roll CLA to CPU varibales ===================================================================================================================================



// --------------------------------------------------------------------------------------------------------------------------------------------------------------

// Roll CLA data RAM varibales =================================================================================================================================

extern Uint16 	ADC_gyro_roll_sum;

extern float	trim_roll_f,
				target_roll_CLA,
				error_roll,
				error_old_roll,
				P_roll,
				I_partial_roll,
				I_roll,
				I_term_max_roll,
				D_roll,
				roll_old,
				out_first_roll,
				out_second_roll,
				mean_offset_roll,
				mean_count,
				roll_ff,
				Kp_ff_roll,
				gain_gyro_roll,
				gyro_unbiased_roll;






//					YY    YY	AAA     WW		WW  ..	  ..
//			  *		 YY  YY    AA  AA   WW      WW  ....  ..     *
//			 ***	  YYYY    AA    AA  WW      WW  .. .. ..    ***
//			*****	   YY     AAAAAAAA  WW      WW  ..  ....   *****
//	 		 ***	   YY     AA    AA   WW WW WW   ..   ...    ***
//	  		  *		   YY     AA    AA    WW  WW    ..    ..     *




// Yaw CPU to CLA varibales ====================================================================================================================================

extern Uint16 	target_yaw,
				received_gain_yaw,
				uint_Kp_yaw,
				uint_Ki_yaw,
				uint_Kd_yaw,
				uint_Kp_ff_yaw,
				uint_gain_gyro_yaw,
				trim_yaw;

extern float	Kp_yaw,
				Ki_yaw,
				Kd_yaw;




// --------------------------------------------------------------------------------------------------------------------------------------------------------------


// Yaw CLA to CPU varibales ===================================================================================================================================



// --------------------------------------------------------------------------------------------------------------------------------------------------------------

// Yaw CLA data RAM varibales =================================================================================================================================

extern Uint16 	ADC_gyro_yaw_sum,
				yaw_count,
				yaw_target_scale;

extern float	trim_yaw_f,
				target_yaw_CLA,
				error_yaw,
				error_old_yaw,
				P_yaw,
				I_partial_yaw,
				I_yaw,
				I_term_max_yaw,
				D_yaw,
				yaw_old,
				out_first_yaw,
				out_second_yaw,
				mean_offset_yaw,
				mean_count,
				yaw_ff,
				Kp_ff_yaw,
				gain_gyro_yaw,
				gyro_unbiased_yaw;

//Task 5 (C) Variables

//Task 6 (C) Variables

//Task 7 (C) Variables

//Task 8 (C) Variables

//Common (C) Variables
//extern float CLAasinTable[];
//extern float CLAatan2Table[];

//CLA C Tasks
__interrupt void Cla1Task1();
__interrupt void Cla1Task2();
__interrupt void Cla1Task3();
__interrupt void Cla1Task4();
__interrupt void Cla1Task5();
__interrupt void Cla1Task6();
__interrupt void Cla1Task7();
__interrupt void Cla1Task8();

#ifdef __cplusplus     // FtoK --------------------------------------------> Why here???
}
#endif /* extern "C" */
#endif /*FIR32_SHARED_H_*/
