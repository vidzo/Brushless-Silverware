//ADC scalefactor explained by Ian444
//As mcRich implied the scalefactor needs to be adjusted slightly for each board if you want exact voltages. I usually tune the scalefactor using the voltage readout via telemetry on a Devo. 
//I’m not sure if the Taranis displays the voltage, but if it can, here is how to do it.
//Say voltage of batt is 4.03V and the tx displays 4.09V. Get the calculator out, 4.03/4.09 x 0.001364 = 0.001344, so set 0.001344 for scalefactor.
//So to summarise:(actual batt voltage/reported batt voltage) x current scalefactor = new scalefactor

// HARDWARE PINS SETTING
//
// do not change hardware pins below
// make sure you don't set SWDIO or SWDCLK pins (programming pins)
// if you do, you lose board programmability without a reset pin
//
// example: pin "PB2" ( port b , pin 2 )
// pin: GPIO_Pin_2
// port: GPIOB


// setting procedure:
// set led number zero, led aux number zero
// uncomment  DISABLE_SPI_PINS and DISABLE_PWM_PINS
// this will prevent any pins to be used, so that there are no conflicts
// set pins starting with leds, in order to see working status
// do not set PA13 , PA14 (stm32f031) as this will break the programming interface
// to disable led pins set number to zero

// the error codes indicate a failure that prevents normal operation
// led flash codes - the quad will not fly / bind if flashing a code
// 2 - low battery at powerup - if enabled by config.h "#define STOP_LOWBATTERY" 
// 3 - radio chip not found
// 4 - Gyro not found - maybe i2c speed
// 5 - clock , intterrupts , systick - this should not come up
// 6 - loop time issue - if loop time exceeds 20mS
// 7 - i2c error  - triggered by hardware i2c driver only
// 8 - i2c error main loop  - triggered by hardware i2c driver only


//Gyro orientation choices
// gyro orientation
// the expected orientation is with the dot in the front-left corner
// use this to rotate to the correct orientation 
// rotations performed in order
// note, the motors don't get rotated,
// so they have to be referenced to the new gyro position
//Gyro orientation choices found in miscellaneous.c 

//define SENSOR_ROTATE_45_CCW
//define SENSOR_ROTATE_45_CW
//define SENSOR_ROTATE_90_CW
//define SENSOR_ROTATE_90_CCW
//define SENSOR_ROTATE_180
//define SENSOR_FLIP_180

// gyro filter 4 = 20hz
// gyro filter 5 = 10hz
// gyro filter 6 = 5hz
// gyro filter 7 = 3600hz delay 0.17mS

// pwm pin initialization
// enable the pwm pins to be used here ( multiple pins ok)
//#define PWM_PA0
//#define PWM_PA1
//#define PWM_PA2
//#define PWM_PA3
//#define PWM_PA5
//#define PWM_PA8
//#define PWM_PA9
//#define PWM_PA10
//#define PWM_PA11
//#define PWM_PB0

// PWM PINS DEFINITIONS 

// pwm pins disable
// disable all pwm pins / function
//#define DISABLE_PWM_PINS
// Assingment of pin to motor
// Assign one pin to one motor
// pins PA0 - PA11 , PB0 , PB1


//LED_NUMBER HARDWARE PARAMETERS SET BY DEFINE
//LED1PIN GPIO HARDWARE PARAMETERS SET BY DEFINE
//LED1PORT GPIO HARDWARE PARAMETERS SET BY DEFINE

// invert - leds turn on when high
// HARDWARE PARAMETERS SET BY DEFINE

// softi2c pins definitons:
// sda - out/in , sck - out

// i2c driver to use ( dummy - disables i2c )
// hardware i2c used PB6 and 7 by default ( can also use PA9 and 10)


// for boards without a SCL pullup - E011 ( nonstandard i2c )
// HARDWARE PARAMETERS SET BY DEFINE

// pins for hw i2c , select one only
// select pins PB6 and PB7 OR select pins PA9 and PA10

// SPI PINS DEFINITONS ( for radio ic )
// MOSI , CLK , SS - outputs , MISO input
// HARDWARE PARAMETERS SET BY DEFINE

//spi type
//#define SOFTSPI_3WIRE
//#define SOFTSPI_4WIRE
//#define SOFTSPI_NONE
//Determined by board type define. HARDWARE PARAMETERS SET BY DEFINE

//**************************************unused settings from config****************************************************

//copy and paste back to config.c file to regain functionality


// *************Radio protocol selection
// *************select only one
//#define RX_CG023_PROTOCOL
//#define RX_H7_PROTOCOL
//#define RX_BAYANG_PROTOCOL
//#define RX_BAYANG_PROTOCOL_BLE_BEACON
//#define RX_CX10BLUE_PROTOCOL

// *************switch for fpv / other, requires fet
// *************comment out to disable
//#define FPV_ON CHAN_ON

// *************motor curve to use - select one
// *************the pwm frequency has to be set independently
//#define MOTOR_CURVE_6MM_490HZ
//#define MOTOR_CURVE_85MM_8KHZ
//#define MOTOR_CURVE_85MM_32KHZ
//#define BOLDCLASH_716MM_8K
//#define BOLDCLASH_716MM_24K

// debug things ( debug struct and other)
//#define DEBUG
// rxdebug structure
//#define RXDEBUG


#ifdef MOTOR_BEEPS
#ifdef USE_ESC_DRIVER
#warning "MOTOR BEEPS_WORKS WITH BRUSHED MOTORS ONLY"
#endif
#endif



