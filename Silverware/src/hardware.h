// HARDWARE PIN SETTINGS INFORMATION IN MISCELLANEAOUS.C
//***LED FLASH ERROR CODES ARE FOUND IN MISCELLANEAOUS.C

//**********************************************************************************************************************
//***********************************************BOARD SELECTION********************************************************

// *************DEFINE FLIGHT CONTROLLER HARDWARE - SELECT ONLY ONE 
// *************uncomment BWHOOP define for bwhoop, bwhoop pro, E011C Santa Edition, and Beta FPV Lite Flight Controllers

// *******NOTE: CHANGE THE MOTOR PIN ASSIGNMENT IN ACCORDANCE WITH YOUR BOARD AND ESC BEFORE TESTING & FLIGHT***********

#define BWHOOP
//#define E011
//#define H8MINI_BLUE_BOARD
//#define SILVERLIGHT
//#define DEDICATED_BOARD

//**********************************************************************************************************************
//*******************************************ESC DRIVER SETTINGS********************************************************

// ------------- ESC driver = servo type signal for brushless esc
// ************* Dshot driver = esc signal from gate of FET only
#define USE_DSHOT_DMA_DRIVER
//#define USE_DSHOT_DRIVER_BETA // Can not use Overclock option in config.h 
//#define USE_ESC_DRIVER



// ------------- FC must have MOSFETS and motor pulldown resistors removed. 
// ************* Use in conjunction with either USE_ESC_DRIVER or USE_DSHOT_DRIVER_BETA  
// MAY NOT WORK WITH ALL ESCS
//#define USE_SERIAL_4WAY_BLHELI_INTERFACE

//**********************************************************************************************************************
//*******************************************MOTOR PINS SELECTION*******************************************************

// ------------- Assingment of pin to motor to processor pin
// back-left motor ( motor 0 )
#define MOTOR0_PIN_PB1
// front-left motor ( motor 1 )
#define MOTOR1_PIN_PA4
// back-right motor ( motor 2 )
#define MOTOR2_PIN_PA6
// front-right motor ( motor 3 )
#define MOTOR3_PIN_PA7


//**********************************************************************************************************************
//*****************************************HARDWARE SETTINGS************************************************************

// ------------- Select this for faster gyro read. Must use HARDWARE_I2C
#define SIXAXIS_READ_DMA
// ************* Sixaxis DMA BETA. Define channels to compare the sync
#define GYRO_SYNC1 CHAN_OFF
#define GYRO_SYNC2 CHAN_OFF
#define GYRO_SYNC3 CHAN_ON // works only when LEVELMODE off and not onground

// ------------- Select this for SPI radio.
// ************* Buzzer GPIO may need to be reassigned to another pin
//#define EXTERNAL_RX

// ------------- Automatic voltage telemetry correction/calibration factor - change the values below if voltage telemetry 
// ************* is inaccurate
#define ACTUAL_BATTERY_VOLTAGE 4.20
#define REPORTED_TELEMETRY_VOLTAGE 4.20


// -------------BUZZER pin settings - buzzer active "high"
// ************* SWDAT and SWCLK pins OK here
// GPIO_Pin_13 // SWDAT - GPIO_Pin_14 // SWCLK 
#define BUZZER_PIN       GPIO_Pin_x 
// *************B uzzer radio channel selected in config.h
// ************* x (micro)seconds after loss of tx or low bat before buzzer starts
#define BUZZER_DELAY     30e6 

// ------------- Disable lvc functions
//#define DISABLE_LVC

// ------------- Gyro orientation if board is rotated in a non-default orientation
// ************* GYRO ORIENTATION CHOICES DISPLAYED IN MISCELLANEAOUS.C - place define below

// ------------- Disable the check for known gyro that causes the 4 times flash
//#define DISABLE_GYRO_CHECK

// ------------- Check for radio chip ( 3 times flash = not found)
#define RADIO_CHECK

// ------------- RGB led type ws2812 - ws2813
// ************* numbers over 8 could decrease performance
#define RGB_LED_NUMBER 0
#define RGB_LED_DMA
// pin / port for the RGB led ( programming port ok )
#define RGB_PIN GPIO_Pin_11
#define RGB_PORT GPIOA

// ------------- Pin for fpv switch ( turns off at failsafe )
// ************* GPIO_Pin_13 // SWDAT - GPIO_Pin_14 // SWCLK  
// ************* if programming pin, will not flash after bind
#define FPV_PIN GPIO_Pin_13
#define FPV_PORT GPIOA 

//***************************************************************************************************************************
//***************************************************************************************************************************
//****************************************** END OF USER SETTINGS************************************************************
//***************************************************************************************************************************
//***************************************************************************************************************************

#ifdef  BWHOOP
#define LED_NUMBER 2
#define LED1PIN GPIO_Pin_2
#define LED1PORT GPIOA
#define LED1_INVERT
#define LED2_INVERT
#define GYRO_ID_2 0x98 // new id
#define USE_SOFTWARE_I2C
#define SENSOR_ROTATE_90_CW
#define BUZZER_PIN_PORT  GPIOA

#ifdef EXTERNAL_RX
#define SERIAL_RX_SPEKBIND_BINDTOOL_PIN GPIO_Pin_3
#define SERIAL_RX_PIN GPIO_Pin_14
#define SERIAL_RX_PORT GPIOA
#define SERIAL_RX_SOURCE GPIO_PinSource14
#define SERIAL_RX_CHANNEL GPIO_AF_1
#define SOFTSPI_NONE
//dummy spi placeholders
#define SPI_MOSI_PIN GPIO_Pin_x
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_y
#define SPI_CLK_PORT GPIOA
#define SPI_SS_PIN GPIO_Pin_z
#define SPI_SS_PORT GPIOA
#else
#define SOFTSPI_3WIRE
#define SPI_MOSI_PIN GPIO_Pin_0
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_1
#define SPI_CLK_PORT GPIOF
#define SPI_SS_PIN GPIO_Pin_0
#define SPI_SS_PORT GPIOF
#define RADIO_XN297L
#endif
#endif

#ifdef E011
#define LED_NUMBER 2
#define LED1PIN GPIO_Pin_2
#define LED1PORT GPIOA
#define LED1_INVERT
#define LED2_INVERT
#define GYRO_ID_2 0x98 // new id
#define SENSOR_ROTATE_90_CW
#define SOFTI2C_PUSHPULL_CLK
#define USE_SOFTWARE_I2C
#define BUZZER_PIN_PORT  GPIOA

#ifdef EXTERNAL_RX
#define SERIAL_RX_SPEKBIND_BINDTOOL_PIN GPIO_Pin_3
#define SERIAL_RX_PIN GPIO_Pin_14
#define SERIAL_RX_PORT GPIOA
#define SERIAL_RX_SOURCE GPIO_PinSource14
#define SERIAL_RX_CHANNEL GPIO_AF_1
#define SOFTSPI_NONE
//dummy spi placeholders
#define SPI_MOSI_PIN GPIO_Pin_x
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_y
#define SPI_CLK_PORT GPIOA
#define SPI_SS_PIN GPIO_Pin_z
#define SPI_SS_PORT GPIOA
#else
#define SOFTSPI_3WIRE
#define SPI_MOSI_PIN GPIO_Pin_0
#define SPI_MOSI_PORT GPIOF
#define SPI_CLK_PIN GPIO_Pin_1
#define SPI_CLK_PORT GPIOF
#define SPI_SS_PIN GPIO_Pin_0
#define SPI_SS_PORT GPIOA
#define RADIO_XN297L
#endif
#endif

#ifdef H8MINI_BLUE_BOARD
#define LED_NUMBER 1
#define LED1PIN GPIO_Pin_1
#define LED1PORT GPIOF
#define GYRO_ID_2 0x78 // common h8 gyro
#define USE_HARDWARE_I2C
#define SENSOR_ROTATE_180
#define BUZZER_PIN_PORT  GPIOA

#ifdef EXTERNAL_RX
#define SERIAL_RX_SPEKBIND_BINDTOOL_PIN GPIO_Pin_3
#define SERIAL_RX_PIN GPIO_Pin_14
#define SERIAL_RX_PORT GPIOA
#define SERIAL_RX_SOURCE GPIO_PinSource14
#define SERIAL_RX_CHANNEL GPIO_AF_1
#define SOFTSPI_NONE
//dummy spi placeholders
#define SPI_MOSI_PIN GPIO_Pin_x
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_y
#define SPI_CLK_PORT GPIOA
#define SPI_SS_PIN GPIO_Pin_z
#define SPI_SS_PORT GPIOA
#else
#define SOFTSPI_3WIRE
#define SPI_MOSI_PIN GPIO_Pin_1
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_2
#define SPI_CLK_PORT GPIOA
#define SPI_SS_PIN GPIO_Pin_3
#define SPI_SS_PORT GPIOA
#define RADIO_XN297L
#endif
#endif

#ifdef SILVERLIGHT
#define LED_NUMBER 1
#define LED1PIN GPIO_Pin_1
#define LED1PORT GPIOF
#define USE_HARDWARE_I2C
#define GYRO_ID_2 0x98 // common h8 gyro
//#define SENSOR_ROTATE_180
#define BUZZER_PIN_PORT  GPIOA

#ifdef EXTERNAL_RX
#define SERIAL_RX_SPEKBIND_BINDTOOL_PIN GPIO_Pin_3
#define SERIAL_RX_PIN GPIO_Pin_14
#define SERIAL_RX_PORT GPIOA
#define SERIAL_RX_SOURCE GPIO_PinSource14
#define SERIAL_RX_CHANNEL GPIO_AF_1
#define SOFTSPI_NONE
//dummy spi placeholders
#define SPI_MOSI_PIN GPIO_Pin_x
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_y
#define SPI_CLK_PORT GPIOA
#define SPI_SS_PIN GPIO_Pin_z
#define SPI_SS_PORT GPIOA
#else
#define SOFTSPI_3WIRE
#define SPI_MOSI_PIN GPIO_Pin_1
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_2
#define SPI_CLK_PORT GPIOA
#define SPI_SS_PIN GPIO_Pin_3
#define SPI_SS_PORT GPIOA
#define RADIO_XN297L
#endif
#endif

#ifdef DEDICATED_BOARD
#define LED_NUMBER 1
#define LED1PIN GPIO_Pin_0
#define LED1PORT GPIOF
//#define LED1_INVERT
#define GYRO_ID_2 0x98 // new id
#define USE_HARDWARE_I2C
#define SENSOR_ROTATE_180
#define BUZZER_PIN_PORT  GPIOF

#ifdef EXTERNAL_RX
#define SERIAL_RX_SPEKBIND_BINDTOOL_PIN GPIO_Pin_3
#define SERIAL_RX_PIN GPIO_Pin_14
#define SERIAL_RX_PORT GPIOA
#define SERIAL_RX_SOURCE GPIO_PinSource14
#define SERIAL_RX_CHANNEL GPIO_AF_1
#define SOFTSPI_NONE
//dummy spi placeholders
#define SPI_MOSI_PIN GPIO_Pin_w
#define SPI_MOSI_PORT GPIOA
#define SPI_MISO_PIN GPIO_Pin_x
#define SPI_MISO_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_y
#define SPI_CLK_PORT GPIOA
#define SPI_SS_PIN GPIO_Pin_z
#define SPI_SS_PORT GPIOA
#else
#define SOFTSPI_4WIRE
#define SPI_MOSI_PIN GPIO_Pin_1
#define SPI_MOSI_PORT GPIOA
#define SPI_MISO_PIN GPIO_Pin_0
#define SPI_MISO_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_2
#define SPI_CLK_PORT GPIOA
#define SPI_SS_PIN GPIO_Pin_3
#define SPI_SS_PORT GPIOA
#define RADIO_XN297L
#endif
#endif

#define LED2PIN GPIO_Pin_3
#define LED2PORT GPIOA
// I2C speed: fast = no delays 
// slow1 = for i2c without pull-up resistors
// slow2 = i2c failsafe speed
#define SOFTI2C_SPEED_FAST
//#define SOFTI2C_SPEED_SLOW1
//#define SOFTI2C_SPEED_SLOW2
// hardware i2c speed ( 1000, 400 , 200 , 100Khz)
#define HW_I2C_SPEED_FAST2
//#define HW_I2C_SPEED_FAST
//#define HW_I2C_SPEED_SLOW1
//#define HW_I2C_SPEED_SLOW2
#define BATTERYPIN GPIO_Pin_5
#define BATTERYPORT GPIOA
#define BATTERY_ADC_CHANNEL ADC_Channel_5
//#define HW_I2C_PINS_PB67
#define HW_I2C_PINS_PA910
#define SOFTI2C_SDAPIN GPIO_Pin_10
#define SOFTI2C_SDAPORT GPIOA
#define SOFTI2C_SCLPIN GPIO_Pin_9
#define SOFTI2C_SCLPORT GPIOA
#define SOFTI2C_GYRO_ADDRESS 0x68
//#define SOFTI2C_GYRO_ADDRESS 0x69
// gyro ids for the gyro check
#define GYRO_ID_1 0x68
// GYRO_ID_2 HARDWARE PARAMETERS SET BY DEFINE
#define GYRO_ID_3 0x7D
#define GYRO_ID_4 0x72
//#define RADIO_XN297
//#define RADIO_XN297L

#ifdef SIXAXIS_READ_DMA
#define USE_HARDWARE_I2C 
#undef USE_SOFTWARE_I2C
#endif

#define PWM_PA4
#define PWM_PA6
#define PWM_PA7
#define PWM_PB1

// Change this factor to get a correct battery voltage. 
// Information on how to increase accuracy can be found in miscellaneous.c file
#define ADC_SCALEFACTOR 0.001364


