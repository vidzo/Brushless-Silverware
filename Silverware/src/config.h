#include "defines.h"
#include "hardware.h"

// unused config and hardware settings found in miscellaneous.c file
//**********************************************************************************************************************
//***********************************************RECEIVER SETTINGS******************************************************

// *************Radio protocol selection
// *************select only one
#define RX_BAYANG_PROTOCOL_TELEMETRY
//#define RX_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND
//#define RX_BAYANG_BLE_APP
//#define RX_SBUS
//#define RX_NRF24_BAYANG_TELEMETRY

// *************rate in deg/sec
#define MAX_RATE 720.0
#define MAX_RATEYAW 720.0
// *************max angle for level mode
#define MAX_ANGLE_HI 70.0f
// ************* low rates multiplier if rates are assigned to a channel
#define LOW_RATES_MULTI 0.5f
// max rate used by level pid ( limit )
#define LEVEL_MAX_RATE 360

// *************Transmitter Type Selection
//#define USE_STOCK_TX
#define USE_DEVO
//#define USE_MULTI

// ********************************************SWITCH SELECTION*********************************************************

// *************CHAN_ON - on always ( all protocols)
// *************CHAN_OFF - off always ( all protocols)
// *************Aux channels are selectable as CHAN_5 through CHAN_10 for DEVO and MULTIMODULE users
// *************Toy transmitter mapping is CHAN_5 (rates button), CHAN_6 (stick gestures RRD/LLD), 
// *************CHAN_7 (headfree button), CHAN_8 (roll trim buttons), CHAN_9 (pitch trim buttons)
// *************All Defines for channels can be found in defines.h file

//*************Assign feature to auxiliary channel.  NOTE - Switching on LEVELMODE is required for any leveling modes to 
//*************be active.  With LEVELMODE active - MCU will apply RACEMODE if racemode channel is on, HORIZON if horizon 
//*************channel is on, or racemodeHORIZON if both channels are on - and will be standard LEVELMODE if neither 
//*************racemode or horizon are switched on.
#define LEVELMODE CHAN_6
#define RACEMODE  CHAN_OFF
#define HORIZON   CHAN_OFF
#define RATES CHAN_ON
#define LEDS_ON CHAN_ON

// *************transmitter stick adjustable deadband for roll/pitch/yaw
// *************.01f = 1% of stick range - comment out to disable
//#define STICKS_DEADBAND .01f

// *************EXPO from 0.00 to 1.00 , 0 = no exp
// *************positive = less sensitive near center 
#define DISABLE_EXPO
#define EXPO_XY 0.8
#define EXPO_YAW 0.6

//*************Idle up-Arm switch
//************* idle up will behave like betaflight airmode, comment out to disable
//#define ARMING CHAN_5
//#define IDLE_UP CHAN_5
//#define IDLE_THR 0.05f

// *************automatically remove center bias in toy tx ( needs throttle off for 1 second )
//#define STOCK_TX_AUTOCENTER
// *************start in level mode for toy tx.
//#define AUX1_START_ON


//**********************************************************************************************************************
//************************************************PID SETTINGS**********************************************************

// activate dual PIDs mode (silverAG)
//#define ENABLE_DUAL_PIDS
//#define PID_SET_CHANGE CHAN_7 //channel used to switch between PID sets

// first PID set (used as set 1 in dual PID mode or as default set in single PID mode)
//Set 1
#define PIDKP1  { 5.00e-2 , 5.00e-2 , 2.50e-1 }
#define PIDKI1  { 1.20e-1 , 1.20e-1 , 1.20e-1 }
#define PIDKD1  { 2.00e-1 , 2.00e-1 , 0.0e-1 }

// second PID set (used as set 2 in dual PID mode - has no function in single PID mode)
//Set2
#define PIDKP2  { 2.50e-2 , 2.50e-2 , 1.30e-1 }
#define PIDKI2  { 0.80e-1 , 0.80e-1 , 0.60e-1 }
#define PIDKD2  { 1.00e-1 , 1.00e-1 , 0.0e-1 }

// *************invert yaw pid for "PROPS OUT" configuration
//#define INVERT_YAW_PID

// *************Voltage compensation to increase handling at low battery
//#define PID_VOLTAGE_COMPENSATION

// Removes roll and pitch bounce back after flips
//#define TRANSIENT_WINDUP_PROTECTION


//**********************************************************************************************************************
//***********************************************FILTER SETTINGS********************************************************


// *************gyro low pass filter ( iir )
// *************set only one below - kalman, 1st order, or second order - and adjust frequency
//**************ABOVE 100 ADJUST IN INCRIMENTS OF 20, BELOW 100 ADJUST IN INCRIMENTS OF 10

//#define SOFT_KALMAN_GYRO KAL1_HZ_90
//#define SOFT_LPF_1ST_HZ 80
//#define SOFT_LPF_2ND_HZ 80
//#define SOFT_LPF_1ST_023HZ
//#define SOFT_LPF_1ST_043HZ
//#define SOFT_LPF_1ST_100HZ
//#define SOFT_LPF_2ND_043HZ
//#define SOFT_LPF_2ND_088HZ
//#define SOFT_LPF_4TH_088HZ
//#define SOFT_LPF_4TH_160HZ
//#define SOFT_LPF_4TH_250HZ
#define SOFT_LPF_NONE

// *************Gyro LPF filter frequency
// gyro filter 0 = 250hz delay 0.97mS
// gyro filter 1 = 184hz delay 2.9mS
// gyro filter 2 = 92hz delay 3.9mS
// gyro filter 3 = 41hz delay 5.9mS (Default)
#define GYRO_LOW_PASS_FILTER 3

// *************D term low pass filter type - set only one below and adjust frequency if adjustable filter is used
// *************1st order adjustable, second order adjustable, or 3rd order fixed (non adjustable)
//#define DTERM_LPF_1ST_HZ 100
//#define DTERM_LPF_2ND_HZ 100
//#define DTERM_LPF3_88
//#define NORMAL_DTERM
#define NEW_DTERM
//#define MAX_FLAT_LPF_DIFF_DTERM


//**********************************************************************************************************************
//***********************************************MOTOR OUTPUT SETTINGS**************************************************


// *************enable motor output filter - select and adjust frequency
// motorfilter1: hanning 3 sample fir filter
// motorfilter2: 1st lpf, 0.2 - 0.6 , 0.6 = less filtering
//#define MOTOR_FILTER2_ALPHA MFILT1_HZ_90
//#define MOTOR_KAL KAL1_HZ_70
//#define MOTOR_FILTER
//#define MOTOR_FILTER2_ALPHA 0.2

// *************clip feedforward attempts to resolve issues that occur near full throttle
//#define CLIP_FF

// *************torque boost is a highly eperimental feature.  it is a lpf D term on motor outputs that will accelerate the response
// *************of the motors when the command to the motors is changing by increasing or decreasing the voltage thats sent.  It differs
// *************from throttle transient compensation in that it acts on all motor commands - not just throttle changes.  this feature
// *************is very noise sensative so D term specifically has to be lowered and gyro/d filtering may need to be increased.
// *************reccomendation right now is to leave boost at or below 2, drop your p gains a few points, then cut your D in half and 
// *************retune it back up to where it feels good.  I'm finding about 60 to 65% of my previous D value seems to work.
//#define TORQUE_BOOST 0.5

// *************makes throttle feel more poppy - can intensify small throttle imbalances visible in FPV if factor is set too high
//#define THROTTLE_TRANSIENT_COMPENSATION 
// if the quad resets , or for brushless ,try a lower value
//#define THROTTLE_TRANSIENT_COMPENSATION_FACTOR 7.0 

// *************mix lower throttle reduces thrust imbalances by reducing throttle proportionally to the adjustable reduction percent
// *************mix increase throttle increases the authority of the pid controller at lowest throttle values like airmode when combined with idle up
// *************mix3 has a stronger effect and works better with brushless
//#define MIX_LOWER_THROTTLE
//#define MIX_THROTTLE_REDUCTION_PERCENT 10
//#define MIX_INCREASE_THROTTLE

//#define MIX_LOWER_THROTTLE_3
//#define MIX_INCREASE_THROTTLE_3

// *************throttle angle compensation in level mode
//#define AUTO_THROTTLE


//**********************************************************************************************************************
//***********************************************VOLTAGE SETTINGS*******************************************************

// *************do not start software if battery is too low, flashes 2 times repeatedly at startup
//#define STOP_LOWBATTERY

// *************voltage to start warning
#define VBATTLOW 3.5

// *************compensation for battery voltage vs throttle drop
#define VDROP_FACTOR 0.7
// *************calculate above factor automatically
#define AUTO_VDROP_FACTOR

// *************voltage hysteresis in volts
#define HYST 0.10


//**********************************************************************************************************************
//****************************************************MISC SETTINGS*****************************************************


// *************0 - 7 - power for telemetry
#define TX_POWER 7

// *************led brightness in-flight ( solid lights only)
// *************0- 15 range
#define LED_BRIGHTNESS 15

// *************external buzzer - pins in hardware.h
// *************external buzzer channel define to enable switch control
//#define BUZZER_ENABLE CHAN_OFF

// *************quad beeps using motors if failsafe occurs or lost signal (5 sec timeout)
// *************can only be used with DSHOT_DRIVER_BETA
//#define MOTOR_BEEPS

// *************Comment out to disable pid tuning gestures
#define PID_GESTURE_TUNING
#define COMBINE_PITCH_ROLL_PID_TUNING

// *************Flash saving features
//#define DISABLE_GESTURES2

// *************flash save method
// *************flash_save 1: pids + accel calibration
// *************flash_save 2: accel calibration to option bytes
#define FLASH_SAVE1
//#define FLASH_SAVE2

// *************enable inverted flight code ( brushless only ). Comment in //#define BIDIRECTIONAL in drv_dshot.c file
//#define INVERTED_ENABLE
//#define FN_INVERTED CH_OFF //for brushless only


//**********************************************************************************************************************
//****************************************************TESTING CONFIG****************************************************
// disable motors for testing
//#define NOMOTORS
// throttle direct to motors for thrust measure
//#define MOTORS_TO_THROTTLE
//#define MOTORS_TO_THROTTLE_MODE MULTI_CHAN_8

// *************SPECIAL TEST MODE TO CHECK TRANSMITTER STICK THROWS
// *************This define will override the stick gesture controlled aux channel and allow you to check if your radio is reaching 100% throws
// *************entering <RIGHT-RIGHT-DOWN> gesture will disable throttle and will rapid blink the led when sticks are moved to 100% throws
// *************entering <LEFT-LEFT-DOWN> will return the quad to normal operation.
//#define STICK_TRAVEL_CHECK


//#############################################################################################################################
//#############################################################################################################################
// debug / other things
// this should not be usually changed
//#############################################################################################################################
//#############################################################################################################################

#define DISABLE_FLIP_SEQUENCER
#define STARTFLIP CHAN_OFF

// level mode "manual" trims ( in degrees)
// pitch positive forward
// roll positive right
#define TRIM_PITCH 0.0
#define TRIM_ROLL 0.0

// loop time in uS
// this affects soft gyro lpf frequency if used
#define LOOPTIME 1000

// failsafe time in uS
#define FAILSAFETIME 1000000  // one second

// *************lower throttle when battery below threshold - forced landing low voltage cutoff
//#define LVC_LOWER_THROTTLE
#define LVC_LOWER_THROTTLE_VOLTAGE 3.30
#define LVC_LOWER_THROTTLE_VOLTAGE_RAW 2.70
#define LVC_LOWER_THROTTLE_KP 3.0

// enable motors if pitch / roll controls off center (at zero throttle)
// possible values: 0 / 1
// use in acro build only
#define ENABLESTIX 0
#define ENABLESTIX_TRESHOLD 0.3
#define ENABLESTIX_TIMEOUT 1e6

// overclock to 64Mhz
//#define ENABLE_OVERCLOCK

#define PWMFREQ 32000
#define MOTOR_CURVE_NONE

// limit minimum motor output to a value (0.0 - 1.0)
//#define MOTOR_MIN_ENABLE
#define MOTOR_MIN_VALUE 0.05

#pragma diag_warning 1035 , 177 , 4017
#pragma diag_error 260

//Throttle must drop below this value if arming feature is enabled for arming to take place.  MIX_INCREASE_THROTTLE_3 if enabled
//will also not activate on the ground untill this threshold is passed during takeoff for safety and better staging behavior.
//#define THROTTLE_SAFETY .10f


// define logic - do not change
///////////////
// used for pwm calculations
#ifdef ENABLE_OVERCLOCK
#define SYS_CLOCK_FREQ_HZ 64000000
#else
#define SYS_CLOCK_FREQ_HZ 48000000
#endif


