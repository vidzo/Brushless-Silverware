#include "defines.h"
#include "hardware.h"

// unused config and hardware settings found in miscellaneous.c file
//**********************************************************************************************************************
//***********************************************RECEIVER SETTINGS******************************************************

// ------------- Radio protocol selection
// ************* Select only one
#define RX_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND
//#define RX_BAYANG_BLE_APP
//#define RX_NRF24_BAYANG_TELEMETRY
//#define RX_SBUS                              // Optional define EXTERNAL_RX in hardware.c
//#define RX_IBUS                              // Define EXTERNAL_RX in hardware.c
//#define RX_CRSF                              // Define EXTERNAL_RX in hardware.c - Requires tbs firmware v2.88 or newer for failsafe to operate properly
//#define RX_SUMD                              // Define EXTERNAL_RX in hardware.c
//#define RX_DSMX_2048                         // Define EXTERNAL_RX in hardware.c
//#define RX_DSM2_1024                         // Define EXTERNAL_RX in hardware.c

// ------------- Rate in deg/sec
#define MAX_RATE 720.0
#define MAX_RATEYAW 720.0
// ************* Max angle for level mode
#define LEVEL_MAX_ANGLE 70.0f
// ************* Low rates multiplier if rates are assigned to a channel
#define LOW_RATES_MULTI 0.5f
// ************* Max rate used by level pid ( limit )
#define LEVEL_MAX_RATE 360

// ------------- Transmitter Type Selection
//#define USE_STOCK_TX
#define USE_DEVO
//#define USE_MULTI

// ********************************************SWITCH SELECTION*********************************************************

// ------------- CHAN_ON - on always ( all protocols)
// ************* CHAN_OFF - off always ( all protocols)
// ************* Aux channels are selectable as CHAN_5 through CHAN_12 for DEVO and through CHAN_13 (but no CHAN_11) for MULTIMODULE users
// ************* Toy transmitter mapping is CHAN_5 (rates button), CHAN_6 (stick gestures RRD/LLD), 
// ************* CHAN_7 (headfree button), CHAN_8 (roll trim buttons), CHAN_9 (pitch trim buttons)
// ************* All Defines for channels can be found in defines.h file

// ------------- Assign feature to auxiliary channel.  NOTE - Switching on LEVELMODE is required for any leveling modes to 
// ************* be active.  With LEVELMODE active - MCU will apply RACEMODE if racemode channel is on, HORIZON if horizon 
// ************* channel is on, or racemodeHORIZON if both channels are on - and will be standard LEVELMODE if neither 
// ************* racemode or horizon are switched on.
#define LEVELMODE CHAN_6
#define RACEMODE  CHAN_OFF
#define HORIZON   CHAN_OFF
#define RATES CHAN_ON
#define LEDS_ON CHAN_ON

// ------------- EXPO from 0.00 to 1.00 , 0 = no exp
// ************* Positive = less sensitive near center 
#define ACRO_EXPO_ROLL 0.80
#define ACRO_EXPO_PITCH 0.80
#define ACRO_EXPO_YAW 0.60

#define ANGLE_EXPO_ROLL 0.35
#define ANGLE_EXPO_PITCH 0.0
#define ANGLE_EXPO_YAW 0.35

// ------------- Idle up-Arm switch
// ************* idle up will behave like betaflight airmode, comment out to disable. 
// ************* Throttle must drop below this value if arming feature is enabled for arming to take place.  MIX_INCREASE_THROTTLE_3 
//if enabled will also not activate on the ground untill this threshold is passed during takeoff for safety and better staging behavior.
//#define ARMING CHAN_5
//#define IDLE_UP CHAN_5
//#define IDLE_THR 0.05f
//#define THROTTLE_SAFETY .10f

// ------------- ANALOG AUX CHANNELS
// ************* For some protocols, use Tx channels as auxiliary analog values
// ************* Bayang with analog aux protocol (Tx optional mode enabled in modified Multimodule and DeviationTx) has two analog channels available:
// ************* Deviation: channels 13 and 14
// ************* Multimodule: channels 14 and 15
// Sbus and DSM can use analog values from any channel
// comment to disable
//#define USE_ANALOG_AUX
// Select analog feature for each channel
// comment to disable
//#define ANALOG_RATE_MULT CHAN_13
//#define ANALOG_MAX_ANGLE CHAN_14
//#define ANALOG_RP_P  CHAN_13 // Adjust Roll and Pitch together
//#define ANALOG_RP_I  CHAN_13
//#define ANALOG_RP_D  CHAN_14
//#define ANALOG_RP_PD CHAN_14 // Adjust Roll and Pitch P & D together
//#define ANALOG_R_P   CHAN_13 // Adjust Roll only
//#define ANALOG_R_I   CHAN_13
//#define ANALOG_R_D   CHAN_14
//#define ANALOG_P_P   CHAN_13 // Adjust Pitch only
//#define ANALOG_P_I   CHAN_13
//#define ANALOG_P_D   CHAN_14
//#define ANALOG_Y_P   CHAN_13 // Adjust Yaw only
//#define ANALOG_Y_I   CHAN_13
//#define ANALOG_Y_D   CHAN_14
// The following define can always be left uncommented. It just gathers all analog aux PID settings together into one define.
#if defined USE_ANALOG_AUX && (defined ANALOG_R_P || defined ANALOG_R_I || defined ANALOG_R_D || defined ANALOG_P_P || defined ANALOG_P_I || defined ANALOG_P_D || defined ANALOG_Y_P || defined ANALOG_Y_I || defined ANALOG_Y_D || defined ANALOG_RP_P || defined ANALOG_RP_I || defined ANALOG_RP_D || defined ANALOG_RP_PD)
    #define ANALOG_AUX_PIDS
#endif

// ------------- Automatically remove center bias in toy tx ( needs throttle off for 1 second )
//#define STOCK_TX_AUTOCENTER
// ************* Start in level mode for toy tx.
//#define AUX1_START_ON




//**********************************************************************************************************************
//************************************************PID SETTINGS**********************************************************

// ------------- Activate dual PIDs mode (silverAG)
//#define ENABLE_DUAL_PIDS
//#define PID_SET_CHANGE CHAN_7 //channel used to switch between PID sets

// ------------- First PID set (used as set 1 in dual PID mode or as default set in single PID mode)
//Set 1
#define PIDKP1  { 5.00e-2 , 5.00e-2 , 2.50e-1 }
#define PIDKI1  { 1.20e-1 , 1.20e-1 , 1.20e-1 }
#define PIDKD1  { 2.00e-1 , 2.00e-1 , 0.0e-1 }

// second PID set (used as set 2 in dual PID mode - has no function in single PID mode)
//Set2
#define PIDKP2  { 2.50e-2 , 2.50e-2 , 1.30e-1 }
#define PIDKI2  { 0.80e-1 , 0.80e-1 , 0.60e-1 }
#define PIDKD2  { 1.00e-1 , 1.00e-1 , 0.0e-1 }

// ------------- Enables use of stick accelerator and stick transition for d term lpf 1 & 2
// ************* Define PIDPROFILE CHAN For switching stickAccelerator & stickTransition profiles on pid.c page
//#define ADVANCED_PID_CONTROLLER
//#define PIDPROFILE CHAN_9                

// ------------- Invert yaw pid for "PROPS OUT" configuration
//#define INVERT_YAW_PID

// ------------- Removes roll and pitch bounce back after flips (Credit to Joe Lucid)
//#define TRANSIENT_WINDUP_PROTECTION

// ------------- Voltage compensation to increase handling at low battery
// ************* Levelmode_PID_attenuation isused to prevent oscillations in angle modes with pid_voltage_compensation enabled due to high pids
//#define PID_VOLTAGE_COMPENSATION
//#define LEVELMODE_PID_ATTENUATION 0.90f

// ------------- Send PID values in the telemetry data.
//#define DISPLAY_PID_VALUES


//**********************************************************************************************************************
//***********************************************FILTER SETTINGS********************************************************

// ------------- Select the appropriate filtering set for your craft's gyro, D-term, and motor output or select CUSTOM_FILTERING to pick your own values.  
// ************* If your throttle does not want to drop crisply and quickly when you lower the throttle stick, then move to a stronger filter set
// ************* Filter strength selections outlined in miscellaneous.c file

//#define WEAK_FILTERING
//#define STRONG_FILTERING
//#define VERY_STRONG_FILTERING
#define CUSTOM_FILTERING
//#define BETA_FILTERING

#ifdef BETA_FILTERING  //*** ABOVE 100 ADJUST IN INCRIMENTS OF 20, BELOW 100 ADJUST IN INCRIMENTS OF 10, nothing coded beyond 500hz
//Select Gyro Filter Type *** Select Only One type
#define KALMAN_GYRO
//#define PT1_GYRO

//Select Gyro Filter Cut Frequency
#define GYRO_FILTER_PASS1 HZ_90
#define GYRO_FILTER_PASS2 HZ_90

// ------------- D term low pass filter type
// ************* 
//Select D Term Filter Cut Frequency *** Select Only one
#define DTERM_LPF_2ND_HZ 100
//#define DTERM_LPF_1ST_HZ 70
#endif



#ifdef CUSTOM_FILTERING
//Select Gyro Filter Type *** Select Only One type *** ABOVE 100 ADJUST IN INCRIMENTS OF 20, BELOW 100 ADJUST IN INCRIMENTS OF 10, nothing coded beyond 500hz
#define KALMAN_GYRO
//#define PT1_GYRO

// ------------- Gyro low pass filter ( iir )
//Select Gyro Filter Cut Frequency
#define GYRO_FILTER_PASS1 HZ_90
#define GYRO_FILTER_PASS2 HZ_90
//#define SOFT_LPF_NONE

// ------------- Gyro LPF filter frequency
// gyro filter 0 = 250hz delay 0.97mS
// gyro filter 1 = 184hz delay 2.9mS
// gyro filter 2 = 92hz delay 3.9mS
// gyro filter 3 = 41hz delay 5.9mS (Default)
#define GYRO_LOW_PASS_FILTER 0

// ------------- D term low pass filter type
// ************* 
//Select D Term Filter Cut Frequency *** Select Only one
#define DTERM_LPF_2ND_HZ 100
//#define DTERM_LPF_1ST_HZ 70
#endif


//**********************************************************************************************************************
//***********************************************MOTOR OUTPUT SETTINGS**************************************************

#ifdef CUSTOM_FILTERING
// ------------- Enable motor output filter - select and adjust frequency
// motorfilter1: hanning 3 sample fir filter
// motorfilter2: 1st lpf, 0.2 - 0.6 , 0.6 = less filtering
#define MOTOR_FILTER2_ALPHA MFILT1_HZ_90
//#define MOTOR_KAL HZ_70
//#define MOTOR_KAL_2ND HZ_90
//#define MOTOR_FILTER
//#define MOTOR_FILTER2_ALPHA 0.2
#endif


// ------------- Clip feedforward attempts to resolve issues that occur near full throttle
//#define CLIP_FF

// ------------- Torque boost is a highly eperimental feature.  it is a lpf D term on motor outputs that will accelerate the response
// ************* of the motors when the command to the motors is changing by increasing or decreasing the voltage thats sent.  It differs
// ************* from throttle transient compensation in that it acts on all motor commands - not just throttle changes.  this feature
// ************* is very noise sensative so D term specifically has to be lowered and gyro/d filtering may need to be increased.
// ************* reccomendation right now is to leave boost at or below 2, drop your p gains a few points, then cut your D in half and 
// ************* retune it back up to where it feels good.  I'm finding about 60 to 65% of my previous D value seems to work.
//#define TORQUE_BOOST 0.5

// ------------- Makes throttle feel more poppy - can intensify small throttle imbalances visible in FPV if factor is set too high
//#define THROTTLE_TRANSIENT_COMPENSATION 
// If the quad resets , or for brushless ,try a lower value
//#define THROTTLE_TRANSIENT_COMPENSATION_FACTOR 7.0 

// ------------- Mix lower throttle reduces thrust imbalances by reducing throttle proportionally to the adjustable reduction percent
// ************* Mix increase throttle increases the authority of the pid controller at lowest throttle values like airmode when combined with idle up
// ************* Mix3 has a stronger effect and works better with brushless
//#define MIX_LOWER_THROTTLE
//#define MIX_THROTTLE_REDUCTION_PERCENT 10
//#define MIX_INCREASE_THROTTLE

//#define MIX_LOWER_THROTTLE_3
//#define MIX_INCREASE_THROTTLE_3
//#define MIX_THROTTLE_INCREASE_MAX 0.2f

// ------------- Joelucid's scaling mixer throttle code
//#define MIX_SCALING

// ------------- Markus Gritsch's Brushless motor curve. Creates a motor curve to compensate for the PID controller
// and nonlinearity of motor thrust
//#define THRUST_LINEARISATION

// ------------- Throttle angle compensation in level mode
//#define AUTO_THROTTLE


//**********************************************************************************************************************
//***********************************************VOLTAGE SETTINGS*******************************************************

// ------------- Do not start software if battery is too low, flashes 2 times repeatedly at startup
//#define STOP_LOWBATTERY

// ------------- Voltage to start warning
#define VBATTLOW 3.5

// ------------- Compensation for battery voltage vs throttle drop
#define VDROP_FACTOR 0.7
// ************* Calculate above factor automatically
#define AUTO_VDROP_FACTOR

// ------------- Voltage hysteresis in volts
#define HYST 0.10

//**********************************************************************************************************************
//****************************************************MISC SETTINGS*****************************************************


// ------------- 0 - 7 - power for telemetry
#define TX_POWER 7

// ------------- LED brightness in-flight ( solid lights only)
// ************* 0- 15 range
#define LED_BRIGHTNESS 15

// ------------- External buzzer - pins in hardware.h
// ************* External buzzer channel define to enable switch control
//#define BUZZER_ENABLE CHAN_OFF

// ------------- Quad beeps using motors if failsafe occurs or lost signal (5 sec timeout)
// ************* Can only be used with DSHOT_DRIVER_BETA
//#define MOTOR_BEEPS

// ------------- Comment out to disable pid tuning gestures
#define PID_GESTURE_TUNING
#define COMBINE_PITCH_ROLL_PID_TUNING

// ------------- Flash saving features
//#define DISABLE_GESTURES2

// ------------- Flash save method
// ************* Flash_save 1: pids + accel calibration
// ************* Flash_save 2: accel calibration to option bytes
#define FLASH_SAVE1
//#define FLASH_SAVE2

// ------------- Enable inverted flight code ( brushless only ). Comment in //#define BIDIRECTIONAL in drv_dshot.c file
//#define INVERTED_ENABLE
//#define FN_INVERTED CH_OFF //for brushless only

// ------------- Transmitter stick adjustable deadband for roll/pitch/yaw
// ************* .01f = 1% of stick range - comment out to disable
//#define STICKS_DEADBAND .01f

//**********************************************************************************************************************
//****************************************************TESTING CONFIG****************************************************
// ------------- Disable motors for testing
//#define NOMOTORS
// ************* Throttle direct to motors for thrust measure
//#define MOTORS_TO_THROTTLE
//#define MOTORS_TO_THROTTLE_MODE CHAN_8

// ------------- Accelerometer telemetry which displays max G. 
// ************* Units are G * 10 e.g readout of 60 is 6 G *above gravity* Gravity not included. 
// ************* The value is in the rx reception field, only even numbers displayed. Value is held for 6 seconds
// ************* Only works with bayang_protocol_telemetry, bayang_protocol_telemetry_autobind and nrf24_bayang_telemetry
//#define ACC_TELEMETRY

// ------------- Accelerometer telemetry which displays max G
// ************* Code written by Markus Gritsch for Devo z axis logging 
// ************* The value is displayed in Volt1 telemetry box reception on Devo screen.
// ************* Only works with bayang_protocol_telemetry, bayang_protocol_telemetry_autobind and nrf24_bayang_telemetry
// ************* Logging will not work when quad is in air while using #define GYRO_SYNC3. Set to Channel 12 of TX, change
// ************* channel in respective protocol 
// ************* Only works with bayang_protocol_telemetry, bayang_protocol_telemetry_autobind and nrf24_bayang_telemetry
//#define Z_AXIS_LOGGING


// ------------- Telemetry option to view CPU load 
// ************* Allows the viewing of CPU load. Uses VBatt telemetry to display MCU loop time usage. 
// ************* Try not to exceed 1.0v (1000 us/1 ms)
// ************* Only works with bayang_protocol_telemetry, bayang_protocol_telemetry_autobind and nrf24_bayang_telemetry
//#define CPU_LOAD_WATCH CHAN_OFF


//**********************************************************************************************************************
//********************************************************BETA TESTING**************************************************
// *************This is a new section that will allow certain beta testing features to be tested, some activated by the stick gesture
// *************auxillary channel. Those features, if defined with stick gestures - the quad will power up with these features off.  To activate -  
// *************use the following stick gesture on the pitch/roll stick RIGHT-RIGHT-DOWN (leds will blink). To deactivate - 
// *************stick gesture LEFT-LEFT-DOWN. Other features are defined by CHAN. Please test the features you are interested in below and give feedback!!!

// ------------- SPECIAL TEST MODE TO CHECK TRANSMITTER STICK THROWS
// ************* This define will allow you to check if your radio is reaching 100% throws entering <RIGHT-RIGHT-DOWN> gesture
// ************* will disable throttle and will rapid blink the led when sticks are moved to 100% throws
// ************* entering <LEFT-LEFT-DOWN> will return the quad to normal operation.
//#define STICK_TRAVEL_CHECK

//#define STICK_TRAVEL_CHECK
//#define SWITCHABLE_FEATURE_1
#ifdef SWITCHABLE_FEATURE_1
//linked to gesture RRR & saved to flash with DDD
//toggles state of variable int flash_feature_1
#endif


//#############################################################################################################################
//#############################################################################################################################
// debug / other things
// this should not be usually changed
//#############################################################################################################################
//#############################################################################################################################

#define DISABLE_FLIP_SEQUENCER
#define STARTFLIP CHAN_OFF

// ------------- Level mode "manual" trims ( in degrees)
// Pitch positive forward
// Roll positive right
#define TRIM_PITCH 0.0
#define TRIM_ROLL 0.0

// ------------- Loop time in uS
// This affects soft gyro lpf frequency if used
#define LOOPTIME 1000

// ------------- Failsafe time in uS
#define FAILSAFETIME 1000000  // one second

// ------------- Lower throttle when battery below threshold - forced landing low voltage cutoff
//#define LVC_LOWER_THROTTLE
#define LVC_LOWER_THROTTLE_VOLTAGE 3.30
#define LVC_LOWER_THROTTLE_VOLTAGE_RAW 2.70
#define LVC_LOWER_THROTTLE_KP 3.0

// ------------- Enable motors if pitch / roll controls off center (at zero throttle)
// Possible values: 0 / 1
// Use in acro build only
#define ENABLESTIX 0
#define ENABLESTIX_TRESHOLD 0.3
#define ENABLESTIX_TIMEOUT 1e6

// ------------- Overclock to 64Mhz
//#define ENABLE_OVERCLOCK

#define PWMFREQ 32000
#define MOTOR_CURVE_NONE

// ------------- Limit minimum motor output to a value (0.0 - 1.0)
//#define MOTOR_MIN_ENABLE
#define MOTOR_MIN_VALUE 0.05

#pragma diag_warning 1035 , 177 , 4017
#pragma diag_error 260


// define logic - do not change
///////////////
// used for pwm calculations
#ifdef ENABLE_OVERCLOCK
#define SYS_CLOCK_FREQ_HZ 64000000
#else
#define SYS_CLOCK_FREQ_HZ 48000000
#endif

#ifdef WEAK_FILTERING
#define KALMAN_GYRO
#define GYRO_FILTER_PASS1 HZ_90
#define  DTERM_LPF_2ND_HZ 100
#define MOTOR_FILTER2_ALPHA MFILT1_HZ_90
#define GYRO_LOW_PASS_FILTER 0
#endif

#ifdef STRONG_FILTERING
#define KALMAN_GYRO
#define GYRO_FILTER_PASS1 HZ_80
#define MOTOR_FILTER2_ALPHA MFILT1_HZ_80
#define DTERM_LPF_2ND_HZ 90
#define GYRO_LOW_PASS_FILTER 0
#endif

#ifdef VERY_STRONG_FILTERING
#define KALMAN_GYRO
#define GYRO_FILTER_PASS1 HZ_70
#define MOTOR_FILTER2_ALPHA MFILT1_HZ_70
#define  DTERM_LPF_2ND_HZ 80
#define GYRO_LOW_PASS_FILTER 0
#endif

#ifdef BETA_FILTERING
	#if (!defined(KALMAN_GYRO) && !defined(PT1_GYRO)) || (!defined(GYRO_FILTER_PASS1) && !defined(GYRO_FILTER_PASS2))
		#define SOFT_LPF_NONE
	#endif
#endif

#ifdef BETA_FILTERING
#define GYRO_LOW_PASS_FILTER 0
#endif


