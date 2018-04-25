[![Build Status](https://travis-ci.org/silver13/BoldClash-BWHOOP-B-03.svg?branch=master)](https://travis-ci.org/silver13/BoldClash-BWHOOP-B-03)

## Yet's Brushless Bwhoop B03 Silverware modified to work on BWHOOP, E011, H8mini_blue_board, Sliverlight and Dedicated brushless board. Based on the work of Silverxxx and NotFastEnuf.
Last major update 15.4.18

### New layout for config.h that includes PID tuning. *Setpoint weight is still in pid.c file.

There are two main steps to prep this file for flash.  The first will be in config.h and the second will be in pid.c file.

STEP 1:  Open config.h with keil and make edits to set your selections in seven categories.

         _Hardware Selection_ - select bwhoop, e011, H8mini_blue_board based hardware, silverlight and dedicated_board, radio protocol
         
         _Receiver Settings_ - set rates, transmitter type, expo and assign features to auxiliary channel switches *Channel defines can be found in defines.h file
                              
		 _PID settings_ - Choose base PID, select dual PIDs or single PID through define,invert yaw based on prop rotation choice *PID tuned through gestures, Silvervise app with selected Bayang_BLE_APP protocol will allow you to see PID numbers					  
		
		 _Filter Settings_ -set gyro filter and D term filter frequencies and orders					  
         
	     _Motor Output Settings_ - motor filter frequency, features related to motor output
		 
		 _Voltage Settings_ - low voltage cutoff and other voltage related settings
         
		 _Additional Features_ - remaining features 
         
STEP 2:  Open hardware.h and make edits to set your brushless settings

         _ESC Driver Selection_ - select ESC driver (Oneshot) or Dshot according to ESCs, Blheli pass through driver *Additional setting changes may be required in drv_dhsot.c or drv_esc.c in accordance with the way you are connecting your board to the ESC. Information is found here http://sirdomsen.diskstation.me/dokuwiki/doku.php?id=brushless_setup and http://sirdomsen.diskstation.me/dokuwiki/doku.php?id=more_brushless_setup_info

		 _Motor Pins Selection_ - select motor pins based on ESC
		 
		 _Battery Voltage Settings_ - set ADC to give correct battery voltage setting for telemetry and LVC *Can be viewed in debug if using without telemetry. Values to be changed to get battery voltage explained in miscellaneous.c file and wiki
		 
		 _Misc Settings_ - telemetry transmitter power, set buzzer time for buzzer *http://sirdomsen.diskstation.me/dokuwiki/doku.php?id=adding_a_buzzer
		 
## Current Experimental Features

### Added 25.4.18
 - Added JazzMac's experimental DShot DMA driver *EXPERIMENTAL*
 
 ### Added 22.4.18
 - Re-added motor beeps for use with Dshot driver for lost quad

### Added 24.3.18
 - PID Voltage compensation added. 

### Added 24.3.18
 - Joelucid's I term Yaw fix. Fix to stop yaw drifts in extreme moves. Can be commented in/out to test for differences (Credit to joelucid for code)
 - Added back the original Silverware filters under filter settings. Allows users to test various tuning options
 - Added Dual PID which can be switched on a channel. Single PID can be enabled if user prefers (Credit to SilverAG for code)
 - Added automatic voltage telemetry correction/calibration factor.

## Bikemike's Blheli 4way Interface Support(from Betaflight) 15.03.18

This commit enables flashing and configuring Blheli using the Silverware FC. **May not work with all ESCs**

 - FC must have MOSFETS and motor pulldown resistors removed
 - In hardware.h file comment in either define USE_ESC_DRIVER or define USE_DSHOT_DRIVER_BETA coupled with define USE_SERIAL_4WAY_BLHELI_INTERFACE
 - Quad must be onground and not armed
 - Connect TX/RX of a usb TTL adapter/Arduino to DAT/CLK and GND to GND in Blheli suite, select the 4way-IF (D) interface
 - Connect and configure/flash ESCs as usual
 - After disconnecting, normal FC operation should resume
 
 Credit to Bikemike https://www.rcgroups.com/forums/showpost.php?p=38505698&postcount=11950


 - E011 or Bwhoop Selection:  define your board type and hardware settings are automatically selected for E011, Bwhoop, bwhoop pro, E011c,  and beta fpv lite
 - Radio Type Selection:  Renamed aux channels as chan_5 through chan_10 and proper mapping is now controlled by defining your transmitter type.  Simply select devo, multi (for taranis or other multimodule), or stock (for toy tx) and assign the features you want to chan_#
 - Racemode:  flight mode with leveling on roll, acro on pitch, and yaw axis rotates about gravity vector.  Can be activated on an aux channel when in level mode.
 - Racemode Horizon: same as above with horizon on roll.
 - Horizon: leveling when upright, will flip or roll.
 - Kalman Gyro Filter:  adjustable gyro filter that's very similar to a 1st order filter but to me feels a little faster and a little weaker.  You decide...
 - Kalman Motor Filter:  adjustable motor output filter.  Same evaluation as the gyro version.
 - Idle up/Arming on aux channel:  idle speed is also adjustable.  Paired with mix increase throttle feature it behaves like airmode
 - Sticks Deadband:  adjustable deadband for transmitter inputs to compensate for gimbals that don't perfectly center
 - Motor Filter Frequencies:  motor filters are adjustable by a frequency instead of a decimal value.
 - AUTOBIND Has been added.  Set a fixed ID in your TX.  Stick gesture comand up-up-up to turn on autobind. Quad will flash once.  Stick gesture Down-Down-Down will save your fixed ID.  Unplug battery and plug in to verify.  No more power cycling TX to bind. Currently only works with RX_BAYANG_TELEMETRY_AUTOBIND protol

_ Please find me here for feedback and bugs; https://www.rcgroups.com/forums/showthread.php?3052088-Silverware-NFE-Yets-Brushless-Fork


## SilverWare - BoldClash BWHOOP B-03 version

_Also works with B03 Pro_

Currently does not account for centered throttle! ( no altitude hold ) It uses normal throttle, low = motors off. That means it should be used with Devo / multimodule or another tx.

Programming connector is a Microjst 1.25 pitch connector. Don't wire positive to st-link as you may overcharge the battery if connected or damage the st-link.

Programming instructions ( uses St-link Utility program ):
https://www.rcgroups.com/forums/showthread.php?2876797-Boldclash-bwhoop-B-03-opensource-firmware

Compiling instructions ( uses Keil uVision IDE):
https://www.rcgroups.com/forums/showthread.php?2877480-Compilation-instructions-for-silverware#post37391059

### Level / acro mode change
By default "gestures" are used to change modes, move the stick *left-left-down* for acro mode , and *right-right-down* for level mode.

### Pid gestures and save
Pid gestures allow the pilot to change the acro mode pids by a percentage, and such tune the quadcopter without a computer. The new pids can be saved so that they will be restored after the quad is power cycled.The new pids will remain active if saved, until the pid values in file pid.c are changed. If the values are not changed, flashing the quad will not erase the pids unless the erase command is manually issued when programming.

*See also*
http://sirdomsen.diskstation.me/dokuwiki/doku.php?id=pidgesture

### Accel calibration / pid save
The gesture for accel calibration is down - down - down. If pids have been changed using the respective gestures since the last powerup or save, the pids will be saved instead.

### Telemetry
DeviationTx and multimodule+taranis can support telemetry, this requires no changes from the defaults on the quad. For devo, when selecting the Bayang protocol, hit Enter to see options, and enable telemetry there. Telemetry currently contains received number of data and telemetry packets, and 2 voltages, battery raw voltage and compensated voltage ( against voltage drop )

For multimodule, you need to add telemetry ( as an option ) to the bayang protocol in the protocol table ( configuration file ). Note the telemetry protocol won't work with stock quads.

*See also*
http://sirdomsen.diskstation.me/dokuwiki/doku.php?id=telemetry

### Android App telemetry
The Android app "SilverVISE" by SilverAG (not me) is able to receive telemetry from the quad using BLE packets. To use, the app protocol should be set in the quadcopter. The app will also show the pids.

*More information*
http://sirdomsen.diskstation.me/dokuwiki/doku.php?id=silvervise

### Linux/gcc version
The gcc version compiles close to 16k, and may need turning off features in order to make it fit. It's also possible to flash up to 32K with changes. Read __install.md__ for additional information. 

The boldclash settings may need changes to work with gcc well. Turn off the software lpf filters except the last 2, the loop time is longer, at 1400, which affects the frequency of other soft lpf filters. You could set loop time to 1500 but it may not be needed.

### Wiki
http://sirdomsen.diskstation.me/dokuwiki/doku.php?id=start

### 01.18
* 2 new D term filters, 1st and 2nd order with custom frequency
* improved led flash timing
* the throttle smooth feature is gone 
* nrf24 support added, bayang protocol + telemetry
* softi2c support for 1 pullup only ( e011 )

### 29.08.17
* moved flash save to 31k
* added 2 looptime independent soft lpf
* added lowerthrottlemix3 from h8
* added motor filter 2 replacing original,much better performance
* pid save has own led flash now
* cleanup of unused features: headless and some other


### xx.06.17
* pid gestures - pids can be changed by gestures at the field
* pid save - new pids are saved to flash by the d-d-d gesture

### 02.05.17
* level pid is tuned more aggressive
* added filter between level pid and  acro pid
* fixed issue where "Overclock" option broke level mode (i2c) - overclock can be used on the bwhoop now
* removed level mode I term - this was not usually used
* minor cleanup and efficiency
* GCC autovdrop now functional
* LVC throttle lowering disabled by default just in case it malfunctions

### initial changes

* autovdrop improved, and better compensation
* new measured motor curve at 24K pwm
* added a new way of lowering throttle below a certain voltage
* lvc now flashes below an "uncompensated" voltage, too, just in case
