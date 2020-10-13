/*
The MIT License (MIT)

Copyright (c) 2016 silverx

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include "project.h"

#include "binary.h"
#include "sixaxis.h"
#include "drv_time.h"
#include "util.h"
#include "config.h"
#include "led.h"
#include "drv_serial.h"
#include "defines.h"

#include "drv_i2c.h"

#include <math.h>
#include <stdio.h>
#include <inttypes.h>

#include "debug.h"


#ifdef SIXAXIS_READ_DMA_HW_1_3
// this works only on newer boards (non mpu-6050)
// on older boards the hw gyro setting controls the acc as well
#define ACC_LOW_PASS_FILTER		5
#define TICK1US								(SYS_CLOCK_FREQ_HZ*1e-6f)

#ifdef SIXAXIS_READ_DMA
	
	#ifndef USE_HARDWARE_I2C
		#warning "I2C DMA must use Hardware I2C"
	#endif
	
	#if defined(ENABLE_OVERCLOCK) && !(defined(HW_I2C_SPEED_FAST_OC) || defined(HW_I2C_SPEED_FAST2_OC) )
 		#warning "HW_I2C_SPEED_FAST(2)_OC not set"
	#endif
	
	#if !defined(ENABLE_OVERCLOCK) && !(defined(HW_I2C_SPEED_FAST) || defined(HW_I2C_SPEED_FAST2) )
 		#warning "HW_I2C_SPEED_FAST(2) not set"
	#endif
	
	#if	defined(HW_I2C_SPEED_FAST2) || defined(HW_I2C_SPEED_FAST2_OC)
		#define	SIXAXIS_READ_TIME		271
		#define	GYRO_READ_TIME			152
	#else
		#define SIXAXIS_READ_TIME		500
		#warning "*** Only 1Mbps supported ***"
	#endif	

	#define SIXAXIS_READ_PERIOD1			((1000-SIXAXIS_READ_TIME*2) * TICK1US)
	#define SIXAXIS_READ_PERIOD2			((1000-SIXAXIS_READ_TIME*1) * TICK1US)
	#define GYRO_READ_PERIOD					((1000-   GYRO_READ_TIME*1) * TICK1US)

volatile uint16_t	i2c_dma_phase 				=	0;			//	0:data no ready	1:new data available
volatile uint16_t	i2c_dma_count 				=	0;			//	0:read count, if 4 times the same data, force output
volatile uint16_t	i2c_dma_phase_count		=	0;
volatile uint8_t	i2c_rx_buffer_dma1[14];
volatile uint8_t	i2c_rx_buffer_dma2[14];
volatile uint8_t	i2c_rx_buffer_sync[6];
volatile uint16_t	sixaxis_read_period		=	SIXAXIS_READ_PERIOD1;
volatile float		sixaxis_read_period2	=	SIXAXIS_READ_PERIOD2;
extern char aux[AUXNUMBER];
extern int onground;
#endif

extern debug_type debug;
uint8_t i2c_rx_buffer[14];

extern int hw_i2c_sendheader( int, int );

// temporary fix for compatibility between versions
#ifndef GYRO_ID_1 
#define GYRO_ID_1 0x68 
#endif
#ifndef GYRO_ID_2
#define GYRO_ID_2 0x98
#endif
#ifndef GYRO_ID_3
#define GYRO_ID_3 0x7D
#endif
#ifndef GYRO_ID_4
#define GYRO_ID_4 0x72
#endif

void sixaxis_init( void)
{
// gyro soft reset
{	
	
	i2c_writereg(  107 , 128);
	 
 delay(40000);
	

// set pll to 1, clear sleep bit old type gyro (mpu-6050)	
	i2c_writereg(  107 , 1);
	
	int newboard = !(0x68 == i2c_readreg(117) );

    delay(100);
	
	i2c_writereg(  28, B00011000);	// 16G scale
	i2c_writereg(  25, B00000000);	// Sample Rate = Gyroscope Output Rate

    
// acc lpf for the new gyro type
//       0-6 ( same as gyro)
	if (newboard) i2c_writereg( 29, ACC_LOW_PASS_FILTER);
	
// gyro scale 2000 deg (FS =3)

	i2c_writereg( 27 , 24);
	
// Gyro DLPF low pass filter

	i2c_writereg( 26 , GYRO_LOW_PASS_FILTER);
	
}
	
#ifdef SIXAXIS_READ_DMA	
	////////////////////////////////////////////////////////////////////////
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	// TIM17 Periph clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);
	
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period =							sixaxis_read_period;
	TIM_TimeBaseStructure.TIM_Prescaler = 					0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 			0;
	TIM_TimeBaseStructure.TIM_CounterMode = 				TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);
	TIM_ARRPreloadConfig(TIM17, ENABLE);
	TIM_Cmd( TIM17, DISABLE );
	
	/* configure TIM17 interrupt */
  NVIC_InitStructure.NVIC_IRQChannel =						TIM17_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority =		0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = 				ENABLE;
  NVIC_Init( &NVIC_InitStructure );	
	TIM17->SR = 0;
	TIM_ITConfig( TIM17, TIM_IT_Update, ENABLE );
	
	DMA_InitTypeDef DMA_InitStructure;
	DMA_StructInit(&DMA_InitStructure);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	/* DMA1 Channe3 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel3);
	DMA_InitStructure.DMA_PeripheralBaseAddr = 		(uint32_t)&I2C1->RXDR;
	DMA_InitStructure.DMA_MemoryBaseAddr = 				(uint32_t)i2c_rx_buffer_dma1;
	DMA_InitStructure.DMA_DIR = 									DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 						14;
	DMA_InitStructure.DMA_PeripheralInc = 				DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = 						DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = 		DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = 				DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = 									DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = 							DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = 									DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);
	
	/* configure DMA1 Channel3 interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = 					DMA1_Channel2_3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 	(uint8_t)DMA_Priority_High;
	NVIC_InitStructure.NVIC_IRQChannelCmd = 			ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* enable DMA1 Channel3 transfer complete interrupt */
	DMA_ClearFlag( DMA1_FLAG_GL3 );
	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);	
#endif	
}

#ifdef SIXAXIS_READ_DMA
void sixaxis_read_start()
{	
	// delayed trigger next DMA by TIM17		
	TIM_SetCounter( TIM17, 0 );
	TIM_Cmd( TIM17, ENABLE );
}

void gyro_sync1()
{	// from sync off to on, limit TIM17 to shorten sync time 	
	if( sixaxis_read_period>(SIXAXIS_READ_PERIOD1+TICK1US*10) || sixaxis_read_period<(SIXAXIS_READ_PERIOD1-TICK1US*10) )
			sixaxis_read_period = SIXAXIS_READ_PERIOD1;

	if( i2c_dma_phase == 2) {			
		if( sixaxis_read_period < (SIXAXIS_READ_PERIOD1+TICK1US*10) )
				sixaxis_read_period +=  1;
				
		TIM17->ARR = sixaxis_read_period+TICK1US*3;
	} else {
		if( sixaxis_read_period > (SIXAXIS_READ_PERIOD1-TICK1US*10) )
				sixaxis_read_period -=   1;

		TIM17->ARR = sixaxis_read_period-TICK1US*3;			
	}
}

void gyro_sync2()
{ // from sync off to on, limit TIM17 to shorten sync time 	
	if( sixaxis_read_period2>(SIXAXIS_READ_PERIOD2+TICK1US*3) || sixaxis_read_period2<(SIXAXIS_READ_PERIOD2-TICK1US*3) )
			sixaxis_read_period2 = SIXAXIS_READ_PERIOD2;
	
	if( i2c_dma_phase == 1) {			
		if( sixaxis_read_period2 > (SIXAXIS_READ_PERIOD2-TICK1US*3) )
				sixaxis_read_period2 -=  (float)0.01;
			
		TIM17->ARR = sixaxis_read_period2-TICK1US*3;
	
	} else {
		if( sixaxis_read_period2 < (SIXAXIS_READ_PERIOD2+TICK1US*3) )
				sixaxis_read_period2 +=   50;

		TIM17->ARR = sixaxis_read_period2+TICK1US*3;			
	}
}

void gyro_sync3()
{ // from sync off to on, limit TIM17 to shorten sync time 	
	if( sixaxis_read_period2>(GYRO_READ_PERIOD+TICK1US*3) || sixaxis_read_period2<(GYRO_READ_PERIOD-TICK1US*3) )
			sixaxis_read_period2 = GYRO_READ_PERIOD;
	
	if( i2c_dma_phase == 1) {			
		if( sixaxis_read_period2 > (GYRO_READ_PERIOD-TICK1US*3) )
				sixaxis_read_period2 -=  (float)0.01;
			
		TIM17->ARR = sixaxis_read_period2-TICK1US*3;
	
	} else {
		if( sixaxis_read_period2 < (GYRO_READ_PERIOD+TICK1US*3) )
				sixaxis_read_period2 +=   50;

		TIM17->ARR = sixaxis_read_period2+TICK1US*3;			
	}
}

void DMA1_Channel2_3_IRQHandler(void)
{	
	DMA_Cmd(DMA1_Channel3, DISABLE);
	DMA_ClearFlag( DMA1_FLAG_GL3 );
	DMA_ClearITPendingBit(DMA1_IT_TC3);

	// if onground, gyro is steady and often the same, sync will fail and the time will be limited at boundary
	// if GYRO_SYNC off, TIM17 + read 14bytes one time in a loop	
	if( !aux[GYRO_SYNC1] && !aux[GYRO_SYNC2] && (aux[LEVELMODE] || aux[RACEMODE] || aux[HORIZON] || !aux[GYRO_SYNC3] || onground) ) {			
		for( int i=0;i<14;i++ ) 
			i2c_rx_buffer_dma2[i] = i2c_rx_buffer_dma1[i];
		
		i2c_dma_phase = 2;
		
		TIM17->ARR = (1000-SIXAXIS_READ_TIME-1)*TICK1US;		
		sixaxis_read_start();
		return;
	}		
	
	i2c_dma_count++;
	if( (i2c_rx_buffer_sync[0]==i2c_rx_buffer_dma1[8]) && (i2c_rx_buffer_sync[1]==i2c_rx_buffer_dma1[9]) && (i2c_rx_buffer_sync[2]==i2c_rx_buffer_dma1[10]) && (i2c_rx_buffer_sync[3]==i2c_rx_buffer_dma1[11]) && (i2c_rx_buffer_sync[4]==i2c_rx_buffer_dma1[12]) && (i2c_rx_buffer_sync[5]==i2c_rx_buffer_dma1[13]) ) {
		if( aux[GYRO_SYNC2] || aux[GYRO_SYNC3])
			i2c_dma_phase = 2;

	} else {	
			i2c_rx_buffer_sync[0] = i2c_rx_buffer_dma1[ 8];
			i2c_rx_buffer_sync[1] = i2c_rx_buffer_dma1[ 9];
			i2c_rx_buffer_sync[2] = i2c_rx_buffer_dma1[10];
			i2c_rx_buffer_sync[3] = i2c_rx_buffer_dma1[11];
			i2c_rx_buffer_sync[4] = i2c_rx_buffer_dma1[12];
			i2c_rx_buffer_sync[5] = i2c_rx_buffer_dma1[13];			
		
			for( int i=0;i<14;i++ ) 
				i2c_rx_buffer_dma2[i] = i2c_rx_buffer_dma1[i];
		
			i2c_dma_phase = i2c_dma_count;
	}

	if( i2c_dma_count < 2 && aux[GYRO_SYNC1] ) {
		// trigger I2C DMA
		DMA_ClearFlag( DMA1_FLAG_GL3 );
		DMA1_Channel3->CNDTR = 14;
	
		hw_i2c_sendheader( 59 , 1 );
		//send restart + readaddress
		I2C_TransferHandling(I2C1, (0x68)<<1 , 14, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
	
		DMA_Cmd( DMA1_Channel3, ENABLE );
		I2C_DMACmd( I2C1, I2C_DMAReq_Rx, ENABLE );		
	} else {
		
		if(				aux[GYRO_SYNC1] ) gyro_sync1();
		else if(	aux[GYRO_SYNC2] ) gyro_sync2();
		else												gyro_sync3();

		i2c_dma_count = 0;		
		i2c_dma_phase = 2;
		sixaxis_read_start();
	}
}

void TIM17_IRQHandler(void)
{	
	TIM_Cmd( TIM17, DISABLE );
	TIM_ClearITPendingBit( TIM17, TIM_IT_Update );
	TIM17->SR = 0;
	
	// trigger I2C DMA
	DMA_ClearFlag( DMA1_FLAG_GL3 );
	
	if( (!aux[LEVELMODE] || aux[RACEMODE] || aux[HORIZON])  && (aux[GYRO_SYNC3] && !onground)  ) {
		DMA1_Channel3->CMAR = (uint32_t)(i2c_rx_buffer_dma1+8);
		DMA1_Channel3->CNDTR = 6;
		hw_i2c_sendheader( 67 , 1 );
		//send restart + readaddress
		I2C_TransferHandling(I2C1, (0x68)<<1 ,  6, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
	} else {
		DMA1_Channel3->CMAR = (uint32_t)i2c_rx_buffer_dma1;
		DMA1_Channel3->CNDTR = 14;
		hw_i2c_sendheader( 59 , 1 );
		//send restart + readaddress
		I2C_TransferHandling(I2C1, (0x68)<<1 , 14, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);	
	}		
	DMA_Cmd( DMA1_Channel3, ENABLE );
  I2C_DMACmd( I2C1, I2C_DMAReq_Rx, ENABLE );
}
#endif

int sixaxis_check( void)
{
	#ifndef DISABLE_GYRO_CHECK
	// read "who am I" register
	int id = i2c_readreg( 117 );

	#ifdef DEBUG
	debug.gyroid = id;
	#endif
	
	#ifdef SIXAXIS_READ_DMA
	sixaxis_read_start();
	#endif
	
	return (GYRO_ID_1==id||GYRO_ID_2==id||GYRO_ID_3==id||GYRO_ID_4==id );
	#else
	return 1;
	#endif
}

float accel[3];
float gyro[3];
float gyro_unfiltered[3];

float accelcal[3];
float gyrocal[3];

int calibration_done;

float lpffilter(float in, int num);
float lpffilter2(float in, int num);

void sixaxis_read(void)
{
	float gyronew[3];

#ifdef SIXAXIS_READ_DMA	
	uint32_t	time=gettime();
	// wait maximum a LOOPTIME for fresh data, if onground, more wait for flash save when doing calibration 
	while( i2c_dma_phase < 2 && (gettime()-time) < (LOOPTIME*(1+onground*100)) ) { }
	while( i2c_dma_phase < 2 ) {
		extern void failloop();
		failloop(9);
	}
	
	__disable_irq();
	for( int i=0;i<14;i++ )
		i2c_rx_buffer[i] = i2c_rx_buffer_dma2[i];
	i2c_dma_phase = 0;
	__enable_irq();
	
#else	
	int data[14];		
		
	i2c_readdata( 59 , data , 14 );		
	for( int i=0;i<14;i++) i2c_rx_buffer[i] = (uint8_t)data[i];	
#endif		
	
#ifdef SENSOR_ROTATE_90_CW	         
        accel[0] = (int16_t) ((i2c_rx_buffer[2] << 8) + i2c_rx_buffer[3]);
        accel[1] = -(int16_t) ((i2c_rx_buffer[0] << 8) + i2c_rx_buffer[1]);
        accel[2] = (int16_t) ((i2c_rx_buffer[4] << 8) + i2c_rx_buffer[5]);
#else
        
	accel[0] = -(int16_t) ((i2c_rx_buffer[0] << 8) + i2c_rx_buffer[1]);
	accel[1] = -(int16_t) ((i2c_rx_buffer[2] << 8) + i2c_rx_buffer[3]);
	accel[2] = (int16_t) ((i2c_rx_buffer[4] << 8) + i2c_rx_buffer[5]);  
        
#endif
  

#ifdef SENSOR_ROTATE_90_CW_deleted
		{//
		float temp = accel[1];
		accel[1] = accel[0];
		accel[0] = -temp;	
		}
#endif       
        
// this is the value of both cos 45 and sin 45 = 1/sqrt(2)
#define INVSQRT2 0.707106781f
		
#ifdef SENSOR_ROTATE_45_CCW
		{
		float temp = accel[0];
		accel[0] = (accel[0] * INVSQRT2 + accel[1] * INVSQRT2);
		accel[1] = -(temp * INVSQRT2 - accel[1] * INVSQRT2);	
		}
#endif
        
#ifdef SENSOR_ROTATE_45_CW
		{
		float temp = accel[1];
		accel[1] = (accel[1] * INVSQRT2 + accel[0] * INVSQRT2);
		accel[0] = -(temp * INVSQRT2 - accel[0] * INVSQRT2);	
		}
#endif
	
		
#ifdef SENSOR_ROTATE_90_CCW
		{
		float temp = accel[1];
		accel[1] = -accel[0];
		accel[0] = temp;	
		}
#endif
				
#ifdef SENSOR_ROTATE_180
		{
		accel[1] = -accel[1];
		accel[0] = -accel[0];	
		}
#endif		
		
#ifdef SENSOR_FLIP_180
		{
		accel[2] = -accel[2];
		accel[0] = -accel[0];	
		}
#endif	
//order
	gyronew[1] = (int16_t) ((i2c_rx_buffer[8] << 8) + i2c_rx_buffer[9]);
	gyronew[0] = (int16_t) ((i2c_rx_buffer[10] << 8) + i2c_rx_buffer[11]);
	gyronew[2] = (int16_t) ((i2c_rx_buffer[12] << 8) + i2c_rx_buffer[13]);


gyronew[0] = gyronew[0] - gyrocal[0];
gyronew[1] = gyronew[1] - gyrocal[1];
gyronew[2] = gyronew[2] - gyrocal[2];

#ifdef SENSOR_ROTATE_90_CW
		{
		float temp = gyronew[1];
		gyronew[1] = -gyronew[0];
		gyronew[0] = temp;	
		}
#endif
        
	
#ifdef SENSOR_ROTATE_45_CCW
		{
		float temp = gyronew[1];
		gyronew[1] = gyronew[0] * INVSQRT2 + gyronew[1] * INVSQRT2;
		gyronew[0] = gyronew[0] * INVSQRT2 - temp * INVSQRT2;	
		}
#endif

#ifdef SENSOR_ROTATE_45_CW
		{
		float temp = gyronew[0];
		gyronew[0] = gyronew[1] * INVSQRT2 + gyronew[0] * INVSQRT2;
		gyronew[1] = gyronew[1] * INVSQRT2 - temp * INVSQRT2;	
		}
#endif	        
		
		
#ifdef SENSOR_ROTATE_90_CCW
		{
		float temp = gyronew[1];
		gyronew[1] = gyronew[0];
		gyronew[0] = -temp;	
		}
#endif
		
					
#ifdef SENSOR_ROTATE_180
		{
		gyronew[1] = -gyronew[1];
		gyronew[0] = -gyronew[0];	
		}
#endif		
		
#ifdef SENSOR_FLIP_180
		{
		gyronew[1] = -gyronew[1];
		gyronew[2] = -gyronew[2];	
		}
#endif	

//gyronew[0] = - gyronew[0];
gyronew[1] = - gyronew[1];
gyronew[2] = - gyronew[2];

	for (int i = 0; i < 3; i++)
	  {
		  gyronew[i] = gyronew[i] * 0.061035156f * 0.017453292f;

			#ifndef SOFT_LPF_NONE

		#if defined (GYRO_FILTER_PASS2) && defined (GYRO_FILTER_PASS1)
			gyro[i] = lpffilter(gyronew[i], i);
			gyro[i] = lpffilter2(gyro[i], i);
		#endif

		#if defined (GYRO_FILTER_PASS1) && !defined(GYRO_FILTER_PASS2)
			gyro[i] = lpffilter(gyronew[i], i);
		#endif

		#if defined (GYRO_FILTER_PASS2) && !defined(GYRO_FILTER_PASS1)
			gyro[i] = lpffilter2(gyronew[i], i);
		#endif
#else
		  gyro[i] = gyronew[i];
#endif
			
			gyro_unfiltered[i] = gyronew[i];
			
	  }
}
	
void gyro_read( void)
{
int data[6];
	
i2c_readdata( 67 , data , 6 );
	
float gyronew[3];
	// order
gyronew[1] = (int16_t) ((data[0]<<8) + data[1]);
gyronew[0] = (int16_t) ((data[2]<<8) + data[3]);
gyronew[2] = (int16_t) ((data[4]<<8) + data[5]);

		
gyronew[0] = gyronew[0] - gyrocal[0];
gyronew[1] = gyronew[1] - gyrocal[1];
gyronew[2] = gyronew[2] - gyrocal[2];
	
	
		
#ifdef SENSOR_ROTATE_45_CCW
		{
		float temp = gyronew[1];
		gyronew[1] = gyronew[0] * INVSQRT2 + gyronew[1] * INVSQRT2;
		gyronew[0] = gyronew[0] * INVSQRT2 - temp * INVSQRT2;	
		}
#endif
        
#ifdef SENSOR_ROTATE_45_CW
		{
		float temp = gyronew[0];
		gyronew[0] = gyronew[1] * INVSQRT2 + gyronew[0] * INVSQRT2;
		gyronew[1] = gyronew[1] * INVSQRT2 - temp * INVSQRT2;	
		}
#endif
			
#ifdef SENSOR_ROTATE_90_CW
		{
		float temp = gyronew[1];
		gyronew[1] = -gyronew[0];
		gyronew[0] = temp;	
		}
#endif

				
#ifdef SENSOR_ROTATE_90_CCW
		{
		float temp = gyronew[1];
		gyronew[1] = gyronew[0];
		gyronew[0] = -temp;	
		}
#endif
	
					
#ifdef SENSOR_ROTATE_180
		{
		gyronew[1] = -gyronew[1];
		gyronew[0] = -gyronew[0];	
		}
#endif		
		
							
#ifdef SENSOR_FLIP_180
		{
		gyronew[1] = -gyronew[1];
		gyronew[2] = -gyronew[2];	
		}
#endif		
	


		
//gyronew[0] = - gyronew[0];
gyronew[1] = - gyronew[1];
gyronew[2] = - gyronew[2];
	
	
for (int i = 0; i < 3; i++)
	  {
		  gyronew[i] = gyronew[i] * 0.061035156f * 0.017453292f;
#ifndef SOFT_LPF_NONE

		#if defined (GYRO_FILTER_PASS2) && defined (GYRO_FILTER_PASS1)
			gyro[i] = lpffilter(gyronew[i], i);
			gyro[i] = lpffilter2(gyro[i], i);
		#endif

		#if defined (GYRO_FILTER_PASS1) && !defined(GYRO_FILTER_PASS2)
			gyro[i] = lpffilter(gyronew[i], i);
		#endif

		#if defined (GYRO_FILTER_PASS2) && !defined(GYRO_FILTER_PASS1)
			gyro[i] = lpffilter2(gyronew[i], i);
		#endif
#else
		  gyro[i] = gyronew[i];
#endif
			
			gyro_unfiltered[i] = gyronew[i];
	  }

}
 


#define CAL_TIME 2e6

#ifdef SIXAXIS_READ_DMA
void gyro_cal(void)
{
//int data[6];
float limit[3];	
unsigned long time = gettime();
unsigned long timestart = time;
unsigned long timemax = time;
unsigned long lastlooptime = time;

float gyro[3];	
	
	for( int i = 0 ; i < 3 ; i++) {
		limit[i] = gyrocal[i];
	}

// 2 and 15 seconds
	while( time - timestart < CAL_TIME  &&  time - timemax < 15e6 ) {	
		
		unsigned long looptime; 
		looptime = time - lastlooptime;
		lastlooptime = time;
		if ( looptime == 0 ) looptime = 1;

		//i2c_readdata(  67 , data , 6 );	
		//gyro[1] = (int16_t) ((data[0]<<8) + data[1]);
		//gyro[0] = (int16_t) ((data[2]<<8) + data[3]);
		//gyro[2] = (int16_t) ((data[4]<<8) + data[5]);
		
		sixaxis_read();
		gyro[1] = (int16_t) ((i2c_rx_buffer[8] << 8) + i2c_rx_buffer[9]);
		gyro[0] = (int16_t) ((i2c_rx_buffer[10] << 8) + i2c_rx_buffer[11]);
		gyro[2] = (int16_t) ((i2c_rx_buffer[12] << 8) + i2c_rx_buffer[13]);
		
/*		
if ( (time - timestart)%200000 > 100000) 
{
	ledon(B00000101);
	ledoff(B00001010);
}
else 
{
	ledon(B00001010);
	ledoff(B00000101);
}
*/
		#define GLOW_TIME 62500 
		static int brightness = 0;
		led_pwm( brightness);
		if((brightness&1)^((time - timestart)%GLOW_TIME > (GLOW_TIME>>1) )) {
			brightness++;
		}
		brightness&=0xF;

		for( int i = 0 ; i < 3 ; i++) {
			if ( gyro[i] > limit[i] )  limit[i] += 0.1f; // 100 gyro bias / second change
			if ( gyro[i] < limit[i] )  limit[i] -= 0.1f;
				
			limitf( &limit[i] , 800);
				
			if( fabsf(gyro[i]) > 100+ fabsf(limit[i]) ) {										
				timestart = gettime();
				brightness = 1;
			}	else {						
				lpf( &gyrocal[i] , gyro[i], lpfcalc( (float) looptime , 0.5 * 1e6) );
		}
	}

// receiver function
void checkrx( void);
checkrx();
            
		while ( (gettime() - time) < 1000 ) delay(10); 				
		time = gettime();
	}
	
	if ( time - timestart < CAL_TIME ) {
		for ( int i = 0 ; i < 3; i++) {
			gyrocal[i] = 0;
		}
	}
	
calibration_done = 1;
}
#else  // not using dma
void gyro_cal(void)
{
int data[6];
float limit[3];	
unsigned long time = gettime();
unsigned long timestart = time;
unsigned long timemax = time;
unsigned long lastlooptime = time;

float gyro[3];	
	
 for ( int i = 0 ; i < 3 ; i++)
			{
			limit[i] = gyrocal[i];
			}

// 2 and 15 seconds
while ( time - timestart < CAL_TIME  &&  time - timemax < 15e6 )
	{	
		
		unsigned long looptime; 
		looptime = time - lastlooptime;
		lastlooptime = time;
		if ( looptime == 0 ) looptime = 1;

	i2c_readdata(  67 , data , 6 );	

			
	gyro[1] = (int16_t) ((data[0]<<8) + data[1]);
	gyro[0] = (int16_t) ((data[2]<<8) + data[3]);
	gyro[2] = (int16_t) ((data[4]<<8) + data[5]);
		

/*		
if ( (time - timestart)%200000 > 100000) 
{
	ledon(B00000101);
	ledoff(B00001010);
}
else 
{
	ledon(B00001010);
	ledoff(B00000101);
}
*/
#define GLOW_TIME 62500 
static int brightness = 0;
led_pwm( brightness);
if ((brightness&1)^((time - timestart)%GLOW_TIME > (GLOW_TIME>>1) ))
{
brightness++;
}

brightness&=0xF;

		 for ( int i = 0 ; i < 3 ; i++)
			{

					if ( gyro[i] > limit[i] )  limit[i] += 0.1f; // 100 gyro bias / second change
					if ( gyro[i] < limit[i] )  limit[i] -= 0.1f;
				
					limitf( &limit[i] , 800);
				
					if ( fabsf(gyro[i]) > 100+ fabsf(limit[i]) ) 
					{										
						timestart = gettime();
						brightness = 1;
					}
					else
					{						
					lpf( &gyrocal[i] , gyro[i], lpfcalc( (float) looptime , 0.5 * 1e6) );
					}
			}

while ( (gettime() - time) < 1000 ) delay(10); 				
time = gettime();
	}
	
if ( time - timestart < CAL_TIME )
{
	for ( int i = 0 ; i < 3; i++)
	{
	gyrocal[i] = 0;

	}
}
	calibration_done = 1;
}
#endif

void acc_cal(void)
{
	accelcal[2] = 2048;
	for (int y = 0; y < 500; y++)
	  {
		  sixaxis_read();
		  for (int x = 0; x < 3; x++)
		    {
			    lpf(&accelcal[x], accel[x], 0.92);
		    }
		  gettime();	// if it takes too long time will overflow so we call it here

	  }
	accelcal[2] -= 2048;

#ifdef FLASH_SAVE2
	for (int x = 0; x < 3; x++)
	  {
		  limitf(&accelcal[x], 127);
	  }
#endif
      
#ifdef FLASH_SAVE1
	for (int x = 0; x < 3; x++)
	  {
		  limitf(&accelcal[x], 500);
	  }
#endif
}
#endif 


#ifdef SIXAXIS_READ_DMA_OVERSAMPLING

// this works only on newer boards (non mpu-6050)
// on older boards the hw gyro setting controls the acc as well
#define ACC_LOW_PASS_FILTER		5
#define TICK1US								(SYS_CLOCK_FREQ_HZ*1e-6f)

#ifdef SIXAXIS_READ_DMA
	
	#ifndef USE_HARDWARE_I2C
		#warning "I2C DMA must use Hardware I2C"
	#endif
	
	#if defined(ENABLE_OVERCLOCK) && !(defined(HW_I2C_SPEED_FAST_OC) )
 		#warning "HW_I2C_SPEED_FAST_OC not set"
	#endif
	
	#if !defined(ENABLE_OVERCLOCK) && !(defined(HW_I2C_SPEED_FAST2) )
 		#warning "HW_I2C_SPEED_FAST2 not set"
	#endif
	
	#if	!defined(HW_I2C_SPEED_FAST2) && !defined(HW_I2C_SPEED_FAST_OC)
		#warning "*** Only 1Mbps supported ***"
	#endif	
	
	#if (GYRO_LOW_PASS_FILTER != 0) || defined(SOFT_LPF_NONE)
		#undef	OVERSAMPLING
		#define OVERSAMPLING						1
	#endif	

	#if (GYRO_LOW_PASS_FILTER_2 != 0) || defined(SOFT_LPF_NONE_2)
		#undef	OVERSAMPLING_2
		#define OVERSAMPLING_2					1
	#endif	

	#define MAX_OVERSAMPLING					4

volatile uint16_t	i2c_dma_phase 				=	0;			//	0:data not ready	2:new gyro ready 3:new accel ready
volatile uint16_t	i2c_dma_sync_phase 		=	0;			//	
volatile uint16_t	i2c_dma_count 				=	0;			//	0:read count, if 4 times the same data, force output
volatile uint16_t	i2c_dma_phase_count		=	0;
volatile uint16_t filter_type						= 0;
volatile uint16_t	Oversampling					= 1;
volatile uint8_t	i2c_rx_buffer_dma1[14];
volatile uint8_t	i2c_rx_buffer_dma2[MAX_OVERSAMPLING][14];
volatile uint8_t	i2c_rx_buffer_sync[6];
volatile float		sixaxis_read_period		= LOOPTIME*TICK1US;
volatile int			lock_phase_time_shift	= 0;
extern char aux[AUXNUMBER];
extern int onground;
#endif

extern debug_type debug;
uint8_t i2c_rx_buffer[14];

extern int hw_i2c_sendheader( int, int );

// temporary fix for compatibility between versions
#ifndef GYRO_ID_1 
#define GYRO_ID_1 0x68 
#endif
#ifndef GYRO_ID_2
#define GYRO_ID_2 0x98
#endif
#ifndef GYRO_ID_3
#define GYRO_ID_3 0x7D
#endif
#ifndef GYRO_ID_4
#define GYRO_ID_4 0x72
#endif

// this is the value of both cos 45 and sin 45 = 1/sqrt(2)
#define INVSQRT2 0.707106781f

float accel[3];
float gyro[3];

float accelcal[3];
float gyrocal[3];
float gyro_unfiltered[3];

int calibration_done;

float lpffilter(float in, int num);
float lpffilter2(float in, int num);

void sixaxis_init( void)
{
// gyro soft reset
	
	
	i2c_writereg(  107 , 128);
	 
 delay(40000);
	

// set pll to 1, clear sleep bit old type gyro (mpu-6050)	
	i2c_writereg(  107 , 1);
	
	int newboard = !(0x68 == i2c_readreg(117) );

    delay(100);
	
	i2c_writereg(  28, B00011000);	// 16G scale
	i2c_writereg(  25, B00000000);	// Sample Rate = Gyroscope Output Rate

    
// acc lpf for the new gyro type
//       0-6 ( same as gyro)
	if (newboard) i2c_writereg( 29, ACC_LOW_PASS_FILTER);
	
// gyro scale 2000 deg (FS =3)

	i2c_writereg( 27 , 24);
	
// Gyro DLPF low pass filter

	i2c_writereg( 26 , GYRO_LOW_PASS_FILTER);

#ifdef SIXAXIS_READ_DMA	
	////////////////////////////////////////////////////////////////////////
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	// TIM17 Periph clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);
	
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period =							LOOPTIME*TICK1US;
	TIM_TimeBaseStructure.TIM_Prescaler = 					0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 			0;
	TIM_TimeBaseStructure.TIM_CounterMode = 				TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);
	TIM_ARRPreloadConfig(TIM17, ENABLE);
	TIM_Cmd( TIM17, DISABLE );
	
	/* configure TIM17 interrupt */
  NVIC_InitStructure.NVIC_IRQChannel =						TIM17_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority =		0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = 				ENABLE;
  NVIC_Init( &NVIC_InitStructure );	
	TIM17->SR = 0;
	TIM_ITConfig( TIM17, TIM_IT_Update, ENABLE );
	
	DMA_InitTypeDef DMA_InitStructure;
	DMA_StructInit(&DMA_InitStructure);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	/* DMA1 Channe3 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel3);
	DMA_InitStructure.DMA_PeripheralBaseAddr = 		(uint32_t)&I2C1->RXDR;
	DMA_InitStructure.DMA_MemoryBaseAddr = 				(uint32_t)i2c_rx_buffer_dma1;
	DMA_InitStructure.DMA_DIR = 									DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 						14;
	DMA_InitStructure.DMA_PeripheralInc = 				DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = 						DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = 		DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = 				DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = 									DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = 							DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = 									DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);
	
	/* configure DMA1 Channel3 interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = 					DMA1_Channel2_3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 	(uint8_t)DMA_Priority_High;
	NVIC_InitStructure.NVIC_IRQChannelCmd = 			ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* enable DMA1 Channel3 transfer complete interrupt */
	DMA_ClearFlag( DMA1_FLAG_GL3 );
	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);	
#endif	
}
#ifdef SIXAXIS_READ_DMA
#define LOCK_RANGE		3 //in us

void do_gyro_sync()
{ // from sync off to on, limit TIM17 to shorten sync time 	
	if( sixaxis_read_period>(LOOPTIME+LOCK_RANGE)*TICK1US || sixaxis_read_period<(LOOPTIME-LOCK_RANGE)*TICK1US )
			sixaxis_read_period = LOOPTIME*TICK1US;
	
	if( i2c_dma_sync_phase == 1) {			
		if( sixaxis_read_period > (LOOPTIME-LOCK_RANGE)*TICK1US )
				sixaxis_read_period -=  (float)0.01;
			
		lock_phase_time_shift = -TICK1US*3;
	} else {
		if( sixaxis_read_period < (LOOPTIME+LOCK_RANGE)*TICK1US )
				sixaxis_read_period +=   50;

		lock_phase_time_shift = TICK1US*3;;			
	}
}
void DMA1_Channel2_3_IRQHandler(void)
{	
	DMA_Cmd(DMA1_Channel3, DISABLE);
	DMA_ClearFlag( DMA1_FLAG_GL3 );
	DMA_ClearITPendingBit(DMA1_IT_TC3);

	i2c_dma_count++;
	// if onground, gyro is steady and often the same, sync will fail and the time will be limited at boundary
	// TIM17 + firstly read gyro 6B and then accel 6B in 1ms		
	if( i2c_dma_count == 1 ) {			
		if( aux[GYRO_SYNC] ) {	
				if( (i2c_rx_buffer_sync[0]==i2c_rx_buffer_dma1[8]) && (i2c_rx_buffer_sync[1]==i2c_rx_buffer_dma1[9]) && (i2c_rx_buffer_sync[2]==i2c_rx_buffer_dma1[10]) && (i2c_rx_buffer_sync[3]==i2c_rx_buffer_dma1[11]) && (i2c_rx_buffer_sync[4]==i2c_rx_buffer_dma1[12]) && (i2c_rx_buffer_sync[5]==i2c_rx_buffer_dma1[13]) ) {
					i2c_dma_sync_phase = 2;
				} else {	
					i2c_dma_sync_phase = 1;
					i2c_rx_buffer_sync[0] = i2c_rx_buffer_dma1[ 8];
					i2c_rx_buffer_sync[1] = i2c_rx_buffer_dma1[ 9];
					i2c_rx_buffer_sync[2] = i2c_rx_buffer_dma1[10];
					i2c_rx_buffer_sync[3] = i2c_rx_buffer_dma1[11];
					i2c_rx_buffer_sync[4] = i2c_rx_buffer_dma1[12];
					i2c_rx_buffer_sync[5] = i2c_rx_buffer_dma1[13];						
				}
		}
	}

	if( i2c_dma_count <= Oversampling ) {
		for( int i=8;i<14;i++ ) 
			i2c_rx_buffer_dma2[i2c_dma_count-1][i] = i2c_rx_buffer_dma1[i];
	}
	
	if( i2c_dma_count < Oversampling ) {
		// continue get gyro data 6Byte, oversampling
		hw_i2c_sendheader( 67 , 1 );
		//send restart + readaddress
		I2C_TransferHandling(I2C1, (0x68)<<1 , 6, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
		DMA1_Channel3->CMAR = (uint32_t)(i2c_rx_buffer_dma1+8);
		DMA1_Channel3->CNDTR = 6;
		DMA_Cmd( DMA1_Channel3, ENABLE );
		I2C_DMACmd( I2C1, I2C_DMAReq_Rx, ENABLE );			
		return;	
	}
	
	if( i2c_dma_count == Oversampling ) {
		i2c_dma_phase = 2;	

		// continue get acc data 6Byte
		hw_i2c_sendheader( 59 , 1 );
		//send restart + readaddress
		I2C_TransferHandling(I2C1, (0x68)<<1 , 6, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
		DMA1_Channel3->CMAR = (uint32_t)i2c_rx_buffer_dma1;
		DMA1_Channel3->CNDTR = 6;
		DMA_Cmd( DMA1_Channel3, ENABLE );
		I2C_DMACmd( I2C1, I2C_DMAReq_Rx, ENABLE );			
		return;
	} else {
		for( int i=0;i<6;i++ ) 
			i2c_rx_buffer_dma2[0][i] = i2c_rx_buffer_dma1[i];
		i2c_dma_phase = 3;
		i2c_dma_count = 0;
		
		if( aux[GYRO_SYNC] ) {
			do_gyro_sync();	
		} else {	
			sixaxis_read_period = (LOOPTIME+1)*TICK1US;
			lock_phase_time_shift = 0;
		}
		
		#ifdef FILTER2
			if( filter_type != aux[FILTER2] ) {
				filter_type = aux[FILTER2];
				if( filter_type ) {
					i2c_writereg( 26 , GYRO_LOW_PASS_FILTER_2);
				}
				else {
					i2c_writereg( 26 , GYRO_LOW_PASS_FILTER);
				}
			}
		#endif
	}
}
void TIM17_IRQHandler(void)
{	
	TIM_Cmd( TIM17, DISABLE );
	TIM_ClearITPendingBit( TIM17, TIM_IT_Update );
	TIM17->SR = 0;
	
	TIM17->ARR = sixaxis_read_period + lock_phase_time_shift;			
	// start next 1ms loop
	TIM_SetCounter( TIM17, 0 );
	TIM_Cmd( TIM17, ENABLE );
	
	// trigger I2C DMA
	DMA_ClearFlag( DMA1_FLAG_GL3 );		
	DMA1_Channel3->CMAR = (uint32_t)(i2c_rx_buffer_dma1+8);
	DMA1_Channel3->CNDTR = 6;
	hw_i2c_sendheader( 67 , 1 );
	//send restart + readaddress
	I2C_TransferHandling(I2C1, (0x68)<<1 , 6, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);		
	DMA_Cmd( DMA1_Channel3, ENABLE );
  I2C_DMACmd( I2C1, I2C_DMAReq_Rx, ENABLE );
}
void gyro_read_dma(void)
{
	float gyronew[3],gyronewbuf[MAX_OVERSAMPLING][3];

	uint32_t	time=gettime();
	// wait maximum a LOOPTIME for fresh data, if onground, more wait for flash save when doing calibration 
	while( i2c_dma_phase < 2 && (gettime()-time) < (LOOPTIME*(1+onground*200)) ) { }
	while( i2c_dma_phase < 2 ) {
		extern void failloop();
		failloop(9);
	}
	
	__disable_irq();
	for( int j=0;j<Oversampling;j++ ) {
		for( int i=8;i<14;i++ ) {
		  //order
			gyronewbuf[j][1] = (int16_t) ((i2c_rx_buffer_dma2[j][ 8] << 8) + i2c_rx_buffer_dma2[j][ 9]);
			gyronewbuf[j][0] = (int16_t) ((i2c_rx_buffer_dma2[j][10] << 8) + i2c_rx_buffer_dma2[j][11]);
			gyronewbuf[j][2] = (int16_t) ((i2c_rx_buffer_dma2[j][12] << 8) + i2c_rx_buffer_dma2[j][13]);
		}
	}
	//i2c_dma_phase = 0;
	__enable_irq();

	for( int j=0;j<Oversampling;j++ ) {
		#ifdef FILTER2
			if( aux[FILTER2] ) {
				#ifdef SOFT_LPF_NONE_2
					j = Oversampling-1;
					gyronew[0] = gyronewbuf[j][0];
					gyronew[1] = gyronewbuf[j][1];
					gyronew[2] = gyronewbuf[j][2];
				#else
					gyronew[0] = lpffilter(gyronewbuf[j][0], 0);
					gyronew[1] = lpffilter(gyronewbuf[j][1], 1);
					gyronew[2] = lpffilter(gyronewbuf[j][2], 2);
				#endif
				Oversampling = OVERSAMPLING_2;
			} else {
				#ifdef SOFT_LPF_NONE
					j = Oversampling-1;
					gyronew[0] = gyronewbuf[j][0];
					gyronew[1] = gyronewbuf[j][1];
					gyronew[2] = gyronewbuf[j][2];
				#else
					gyronew[0] = lpffilter(gyronewbuf[j][0], 0);
					gyronew[1] = lpffilter(gyronewbuf[j][1], 1);
					gyronew[2] = lpffilter(gyronewbuf[j][2], 2);
				#endif
				Oversampling = OVERSAMPLING;					
			}
		#else	
			#ifdef SOFT_LPF_NONE
				j = Oversampling-1;
				gyronew[0] = gyronewbuf[j][0];
				gyronew[1] = gyronewbuf[j][1];
				gyronew[2] = gyronewbuf[j][2];
			#else
				gyronew[0] = lpffilter(gyronewbuf[j][0], 0);
				gyronew[1] = lpffilter(gyronewbuf[j][1], 1);
				gyronew[2] = lpffilter(gyronewbuf[j][2], 2);
			#endif
			Oversampling = OVERSAMPLING;
		#endif	
	}
	
gyronew[0] = gyronew[0] - gyrocal[0];
gyronew[1] = gyronew[1] - gyrocal[1];
gyronew[2] = gyronew[2] - gyrocal[2];

#ifdef SENSOR_ROTATE_90_CW
		{
		float temp = gyronew[1];
		gyronew[1] = -gyronew[0];
		gyronew[0] = temp;	
		}
#endif        
	
#ifdef SENSOR_ROTATE_45_CCW
		{
		float temp = gyronew[1];
		gyronew[1] = gyronew[0] * INVSQRT2 + gyronew[1] * INVSQRT2;
		gyronew[0] = gyronew[0] * INVSQRT2 - temp * INVSQRT2;	
		}
#endif

#ifdef SENSOR_ROTATE_45_CW
		{
		float temp = gyronew[0];
		gyronew[0] = gyronew[1] * INVSQRT2 + gyronew[0] * INVSQRT2;
		gyronew[1] = gyronew[1] * INVSQRT2 - temp * INVSQRT2;	
		}
#endif	        
				
#ifdef SENSOR_ROTATE_90_CCW
		{
		float temp = gyronew[1];
		gyronew[1] = gyronew[0];
		gyronew[0] = -temp;	
		}
#endif
					
#ifdef SENSOR_ROTATE_180
		{
		gyronew[1] = -gyronew[1];
		gyronew[0] = -gyronew[0];	
		}
#endif		
		
#ifdef SENSOR_FLIP_180
		{
		gyronew[1] = -gyronew[1];
		gyronew[2] = -gyronew[2];	
		}
#endif	

//gyronew[0] = - gyronew[0];
gyronew[1] = - gyronew[1];
gyronew[2] = - gyronew[2];

gyro[0] = gyronew[0] * 0.061035156f * 0.017453292f;
gyro[1] = gyronew[1] * 0.061035156f * 0.017453292f;
gyro[2] = gyronew[2] * 0.061035156f * 0.017453292f;
}
void accel_read_dma(void)
{	
	//uint32_t	time=gettime();
	// wait maximum a LOOPTIME for fresh data, if onground, more wait for flash save when doing calibration 
	//while( i2c_dma_phase > 2 && (gettime()-time) < (LOOPTIME*(1+onground*200)) ) { }
	//while( i2c_dma_phase > 2 ) {
	//	extern void failloop();
	//	failloop(9);
  //}
	
	__disable_irq();
	for( int i=0;i<6;i++ )
		i2c_rx_buffer[i] = i2c_rx_buffer_dma2[0][i];
	i2c_dma_phase = 0;
	__enable_irq();
	
#ifdef SENSOR_ROTATE_90_CW	         
        accel[0] = (int16_t) ((i2c_rx_buffer[2] << 8) + i2c_rx_buffer[3]);
        accel[1] = -(int16_t) ((i2c_rx_buffer[0] << 8) + i2c_rx_buffer[1]);
        accel[2] = (int16_t) ((i2c_rx_buffer[4] << 8) + i2c_rx_buffer[5]);
#else
        
	accel[0] = -(int16_t) ((i2c_rx_buffer[0] << 8) + i2c_rx_buffer[1]);
	accel[1] = -(int16_t) ((i2c_rx_buffer[2] << 8) + i2c_rx_buffer[3]);
	accel[2] = (int16_t) ((i2c_rx_buffer[4] << 8) + i2c_rx_buffer[5]);  
        
#endif

#ifdef SENSOR_ROTATE_90_CW_deleted
		{//
		float temp = accel[1];
		accel[1] = accel[0];
		accel[0] = -temp;	
		}
#endif       
		
#ifdef SENSOR_ROTATE_45_CCW
		{
		float temp = accel[0];
		accel[0] = (accel[0] * INVSQRT2 + accel[1] * INVSQRT2);
		accel[1] = -(temp * INVSQRT2 - accel[1] * INVSQRT2);	
		}
#endif
        
#ifdef SENSOR_ROTATE_45_CW
		{
		float temp = accel[1];
		accel[1] = (accel[1] * INVSQRT2 + accel[0] * INVSQRT2);
		accel[0] = -(temp * INVSQRT2 - accel[0] * INVSQRT2);	
		}
#endif
		
#ifdef SENSOR_ROTATE_90_CCW
		{
		float temp = accel[1];
		accel[1] = -accel[0];
		accel[0] = temp;	
		}
#endif
				
#ifdef SENSOR_ROTATE_180
		{
		accel[1] = -accel[1];
		accel[0] = -accel[0];	
		}
#endif		
		
#ifdef SENSOR_FLIP_180
		{
		accel[2] = -accel[2];
		accel[0] = -accel[0];	
		}
#endif	
}
#endif

int sixaxis_check( void)
{
	#ifndef DISABLE_GYRO_CHECK
	// read "who am I" register
	int id = i2c_readreg( 117 );

	#ifdef DEBUG
	debug.gyroid = id;
	#endif
	
	#ifdef SIXAXIS_READ_DMA
		TIM17->ARR = LOOPTIME*TICK1US;			
		// start next 1ms loop
		TIM_SetCounter( TIM17, 0 );
		TIM_Cmd( TIM17, ENABLE );
	#endif
	
	return (GYRO_ID_1==id||GYRO_ID_2==id||GYRO_ID_3==id||GYRO_ID_4==id );
	#else
	return 1;
	#endif
}
void sixaxis_read(void)
{
	float gyronew[3];

#ifdef SIXAXIS_READ_DMA	
	uint32_t	time=gettime();
	// wait maximum a LOOPTIME for fresh data, if onground, more wait for flash save when doing calibration 
	while( i2c_dma_phase < 3 && (gettime()-time) < (LOOPTIME*(1+onground*500)) ) { }
	while( i2c_dma_phase < 3 ) {
		extern void failloop();
		failloop(9);
	}
	
	__disable_irq();
	for( int i=0;i<14;i++ )
		i2c_rx_buffer[i] = i2c_rx_buffer_dma2[0][i];
	i2c_dma_phase = 0;
	__enable_irq();
	
#else	
	int data[14];		
		
	i2c_readdata( 59 , data , 14 );		
	for( int i=0;i<14;i++) i2c_rx_buffer[i] = (uint8_t)data[i];	
#endif		
	
#ifdef SENSOR_ROTATE_90_CW	         
        accel[0] = (int16_t) ((i2c_rx_buffer[2] << 8) + i2c_rx_buffer[3]);
        accel[1] = -(int16_t) ((i2c_rx_buffer[0] << 8) + i2c_rx_buffer[1]);
        accel[2] = (int16_t) ((i2c_rx_buffer[4] << 8) + i2c_rx_buffer[5]);
#else
        
	accel[0] = -(int16_t) ((i2c_rx_buffer[0] << 8) + i2c_rx_buffer[1]);
	accel[1] = -(int16_t) ((i2c_rx_buffer[2] << 8) + i2c_rx_buffer[3]);
	accel[2] = (int16_t) ((i2c_rx_buffer[4] << 8) + i2c_rx_buffer[5]);  
        
#endif
  

#ifdef SENSOR_ROTATE_90_CW_deleted
		{//
		float temp = accel[1];
		accel[1] = accel[0];
		accel[0] = -temp;	
		}
#endif       
		
#ifdef SENSOR_ROTATE_45_CCW
		{
		float temp = accel[0];
		accel[0] = (accel[0] * INVSQRT2 + accel[1] * INVSQRT2);
		accel[1] = -(temp * INVSQRT2 - accel[1] * INVSQRT2);	
		}
#endif
        
#ifdef SENSOR_ROTATE_45_CW
		{
		float temp = accel[1];
		accel[1] = (accel[1] * INVSQRT2 + accel[0] * INVSQRT2);
		accel[0] = -(temp * INVSQRT2 - accel[0] * INVSQRT2);	
		}
#endif
	
		
#ifdef SENSOR_ROTATE_90_CCW
		{
		float temp = accel[1];
		accel[1] = -accel[0];
		accel[0] = temp;	
		}
#endif
				
#ifdef SENSOR_ROTATE_180
		{
		accel[1] = -accel[1];
		accel[0] = -accel[0];	
		}
#endif		
		
#ifdef SENSOR_FLIP_180
		{
		accel[2] = -accel[2];
		accel[0] = -accel[0];	
		}
#endif	
//order
	gyronew[1] = (int16_t) ((i2c_rx_buffer[8] << 8) + i2c_rx_buffer[9]);
	gyronew[0] = (int16_t) ((i2c_rx_buffer[10] << 8) + i2c_rx_buffer[11]);
	gyronew[2] = (int16_t) ((i2c_rx_buffer[12] << 8) + i2c_rx_buffer[13]);


gyronew[0] = gyronew[0] - gyrocal[0];
gyronew[1] = gyronew[1] - gyrocal[1];
gyronew[2] = gyronew[2] - gyrocal[2];

#ifdef SENSOR_ROTATE_90_CW
		{
		float temp = gyronew[1];
		gyronew[1] = -gyronew[0];
		gyronew[0] = temp;	
		}
#endif
        
	
#ifdef SENSOR_ROTATE_45_CCW
		{
		float temp = gyronew[1];
		gyronew[1] = gyronew[0] * INVSQRT2 + gyronew[1] * INVSQRT2;
		gyronew[0] = gyronew[0] * INVSQRT2 - temp * INVSQRT2;	
		}
#endif

#ifdef SENSOR_ROTATE_45_CW
		{
		float temp = gyronew[0];
		gyronew[0] = gyronew[1] * INVSQRT2 + gyronew[0] * INVSQRT2;
		gyronew[1] = gyronew[1] * INVSQRT2 - temp * INVSQRT2;	
		}
#endif	        
		
		
#ifdef SENSOR_ROTATE_90_CCW
		{
		float temp = gyronew[1];
		gyronew[1] = gyronew[0];
		gyronew[0] = -temp;	
		}
#endif
		
					
#ifdef SENSOR_ROTATE_180
		{
		gyronew[1] = -gyronew[1];
		gyronew[0] = -gyronew[0];	
		}
#endif		
		
#ifdef SENSOR_FLIP_180
		{
		gyronew[1] = -gyronew[1];
		gyronew[2] = -gyronew[2];	
		}
#endif	

//gyronew[0] = - gyronew[0];
gyronew[1] = - gyronew[1];
gyronew[2] = - gyronew[2];

	for (int i = 0; i < 3; i++)
	  {
		  gyronew[i] = gyronew[i] * 0.061035156f * 0.017453292f;
			
			#ifndef SOFT_LPF_NONE

		#if defined (GYRO_FILTER_PASS2) && defined (GYRO_FILTER_PASS1)
			gyro[i] = lpffilter(gyronew[i], i);
			gyro[i] = lpffilter2(gyro[i], i);
		#endif

		#if defined (GYRO_FILTER_PASS1) && !defined(GYRO_FILTER_PASS2)
			gyro[i] = lpffilter(gyronew[i], i);
		#endif

		#if defined (GYRO_FILTER_PASS2) && !defined(GYRO_FILTER_PASS1)
			gyro[i] = lpffilter2(gyronew[i], i);
		#endif
#else
		  gyro[i] = gyronew[i];
#endif
			
			gyro_unfiltered[i] = gyronew[i];
			
	  }
}
	
void gyro_read( void)
{
int data[6];
	
i2c_readdata( 67 , data , 6 );
	
float gyronew[3];
	// order
gyronew[1] = (int16_t) ((data[0]<<8) + data[1]);
gyronew[0] = (int16_t) ((data[2]<<8) + data[3]);
gyronew[2] = (int16_t) ((data[4]<<8) + data[5]);

		
gyronew[0] = gyronew[0] - gyrocal[0];
gyronew[1] = gyronew[1] - gyrocal[1];
gyronew[2] = gyronew[2] - gyrocal[2];
	
	
		
#ifdef SENSOR_ROTATE_45_CCW
		{
		float temp = gyronew[1];
		gyronew[1] = gyronew[0] * INVSQRT2 + gyronew[1] * INVSQRT2;
		gyronew[0] = gyronew[0] * INVSQRT2 - temp * INVSQRT2;	
		}
#endif
        
#ifdef SENSOR_ROTATE_45_CW
		{
		float temp = gyronew[0];
		gyronew[0] = gyronew[1] * INVSQRT2 + gyronew[0] * INVSQRT2;
		gyronew[1] = gyronew[1] * INVSQRT2 - temp * INVSQRT2;	
		}
#endif
			
#ifdef SENSOR_ROTATE_90_CW
		{
		float temp = gyronew[1];
		gyronew[1] = -gyronew[0];
		gyronew[0] = temp;	
		}
#endif

				
#ifdef SENSOR_ROTATE_90_CCW
		{
		float temp = gyronew[1];
		gyronew[1] = gyronew[0];
		gyronew[0] = -temp;	
		}
#endif
	
					
#ifdef SENSOR_ROTATE_180
		{
		gyronew[1] = -gyronew[1];
		gyronew[0] = -gyronew[0];	
		}
#endif		
		
							
#ifdef SENSOR_FLIP_180
		{
		gyronew[1] = -gyronew[1];
		gyronew[2] = -gyronew[2];	
		}
#endif		
	


		
//gyronew[0] = - gyronew[0];
gyronew[1] = - gyronew[1];
gyronew[2] = - gyronew[2];
	
	
for (int i = 0; i < 3; i++)
	  {
		  gyronew[i] = gyronew[i] * 0.061035156f * 0.017453292f;
#ifndef SOFT_LPF_NONE

		#if defined (GYRO_FILTER_PASS2) && defined (GYRO_FILTER_PASS1)
			gyro[i] = lpffilter(gyronew[i], i);
			gyro[i] = lpffilter2(gyro[i], i);
		#endif

		#if defined (GYRO_FILTER_PASS1) && !defined(GYRO_FILTER_PASS2)
			gyro[i] = lpffilter(gyronew[i], i);
		#endif

		#if defined (GYRO_FILTER_PASS2) && !defined(GYRO_FILTER_PASS1)
			gyro[i] = lpffilter2(gyronew[i], i);
		#endif
#else
		  gyro[i] = gyronew[i];
#endif
			
			gyro_unfiltered[i] = gyronew[i];
	  }

}
 
 


#define CAL_TIME 2e6

#ifdef SIXAXIS_READ_DMA
void gyro_cal(void)
{
//int data[6];
float limit[3];	
unsigned long time = gettime();
unsigned long timestart = time;
unsigned long timemax = time;
unsigned long lastlooptime = time;

float gyro[3];	
	
	for( int i = 0 ; i < 3 ; i++) {
		limit[i] = gyrocal[i];
	}

// 2 and 15 seconds
	while( (time - timestart < CAL_TIME)  &&  (time - timemax < 15e6) ) {	
		
		unsigned long looptime; 
		looptime = time - lastlooptime;
		lastlooptime = time;
		if ( looptime == 0 ) looptime = 1;

		//i2c_readdata(  67 , data , 6 );	
		//gyro[1] = (int16_t) ((data[0]<<8) + data[1]);
		//gyro[0] = (int16_t) ((data[2]<<8) + data[3]);
		//gyro[2] = (int16_t) ((data[4]<<8) + data[5]);
		
		sixaxis_read();
		gyro[1] = (int16_t) ((i2c_rx_buffer[8] << 8) + i2c_rx_buffer[9]);
		gyro[0] = (int16_t) ((i2c_rx_buffer[10] << 8) + i2c_rx_buffer[11]);
		gyro[2] = (int16_t) ((i2c_rx_buffer[12] << 8) + i2c_rx_buffer[13]);
		
/*		
if ( (time - timestart)%200000 > 100000) 
{
	ledon(B00000101);
	ledoff(B00001010);
}
else 
{
	ledon(B00001010);
	ledoff(B00000101);
}
*/
		#define GLOW_TIME 62500 
		static int brightness = 0;
		led_pwm( brightness);
		if((brightness&1)^((time - timestart)%GLOW_TIME > (GLOW_TIME>>1) )) {
			brightness++;
		}
		brightness&=0xF;

		for( int i = 0 ; i < 3 ; i++) {
			if ( gyro[i] > limit[i] )  limit[i] += 0.1f; // 100 gyro bias / second change
			if ( gyro[i] < limit[i] )  limit[i] -= 0.1f;
				
			limitf( &limit[i] , 800);
				
			if( fabsf(gyro[i]) > 100+ fabsf(limit[i]) ) {										
				timestart = gettime();
				brightness = 1;
			}	else {						
				lpf( &gyrocal[i] , gyro[i], lpfcalc( (float) looptime , 0.5 * 1e6) );
		}
	}

// receiver function
void checkrx( void);
checkrx();

		//while ( (gettime() - time) < 1000 ) delay(10); 				
		time = gettime();
	}
	
	if ( time - timestart < CAL_TIME ) {
		for ( int i = 0 ; i < 3; i++) {
			gyrocal[i] = 0;
		}
	}

calibration_done = 1;
}


#else  // not using dma
void gyro_cal(void)
{
int data[6];
float limit[3];	
unsigned long time = gettime();
unsigned long timestart = time;
unsigned long timemax = time;
unsigned long lastlooptime = time;

float gyro[3];	
	
 for ( int i = 0 ; i < 3 ; i++)
			{
			limit[i] = gyrocal[i];
			}

// 2 and 15 seconds
while ( time - timestart < CAL_TIME  &&  time - timemax < 15e6 )
	{	
		
		unsigned long looptime; 
		looptime = time - lastlooptime;
		lastlooptime = time;
		if ( looptime == 0 ) looptime = 1;

	i2c_readdata(  67 , data , 6 );	

			
	gyro[1] = (int16_t) ((data[0]<<8) + data[1]);
	gyro[0] = (int16_t) ((data[2]<<8) + data[3]);
	gyro[2] = (int16_t) ((data[4]<<8) + data[5]);
		

/*		
if ( (time - timestart)%200000 > 100000) 
{
	ledon(B00000101);
	ledoff(B00001010);
}
else 
{
	ledon(B00001010);
	ledoff(B00000101);
}
*/
#define GLOW_TIME 62500 
static int brightness = 0;
led_pwm( brightness);
if ((brightness&1)^((time - timestart)%GLOW_TIME > (GLOW_TIME>>1) ))
{
brightness++;
}

brightness&=0xF;

		 for ( int i = 0 ; i < 3 ; i++)
			{

					if ( gyro[i] > limit[i] )  limit[i] += 0.1f; // 100 gyro bias / second change
					if ( gyro[i] < limit[i] )  limit[i] -= 0.1f;
				
					limitf( &limit[i] , 800);
				
					if ( fabsf(gyro[i]) > 100+ fabsf(limit[i]) ) 
					{										
						timestart = gettime();
						brightness = 1;
					}
					else
					{						
					lpf( &gyrocal[i] , gyro[i], lpfcalc( (float) looptime , 0.5 * 1e6) );
					}
			}

while ( (gettime() - time) < 1000 ) delay(10); 				
time = gettime();
	}
	
if ( time - timestart < CAL_TIME )
{
	for ( int i = 0 ; i < 3; i++)
	{
	gyrocal[i] = 0;

	}
}
	calibration_done = 1;

}

#endif

void acc_cal(void)
{
	accelcal[2] = 2048;
	for (int y = 0; y < 500; y++)
	  {
		  sixaxis_read();
		  for (int x = 0; x < 3; x++)
		    {
			    lpf(&accelcal[x], accel[x], 0.92);
		    }
		  gettime();	// if it takes too long time will overflow so we call it here

	  }
	accelcal[2] -= 2048;

#ifdef FLASH_SAVE2
	for (int x = 0; x < 3; x++)
	  {
		  limitf(&accelcal[x], 127);
	  }
#endif
      
#ifdef FLASH_SAVE1
	for (int x = 0; x < 3; x++)
	  {
		  limitf(&accelcal[x], 500);
	  }
#endif
}
#endif

  
#ifdef SIXAXIS_READ_DMA_MULTI
// this works only on newer boards (non mpu-6050)
// on older boards the hw gyro setting controls the acc as well
#define ACC_LOW_PASS_FILTER 5

extern debug_type debug;
uint8_t i2c_rx_buffer[14];
//volatile uint16_t i2c_dma_phase = 0;			//	0:idel	1:delay is counting	2:DMA triggered

// temporary fix for compatibility between versions
#ifndef GYRO_ID_1
#define GYRO_ID_1 0x68
#endif
#ifndef GYRO_ID_2
#define GYRO_ID_2 0x98
#endif
#ifndef GYRO_ID_3
#define GYRO_ID_3 0x7D
#endif
#ifndef GYRO_ID_4
#define GYRO_ID_4 0x72
#endif

void sixaxis_init( void)
{
// gyro soft reset


	i2c_writereg(07 , 128);

 delay(40000);


// set pll to 1, clear sleep bit old type gyro (mpu-6050)
	i2c_writereg( 107 , 1);

	int newboard = !(0x68 == i2c_readreg(117));

    delay(100);

	i2c_writereg( 28, B00011000);	// 16G scale
	i2c_writereg( 25, B00000000);	// Sample Rate = Gyroscope Output Rate


// acc lpf for the new gyro type
//       0-6 ( same as gyro)
	if (newboard) i2c_writereg( 29, ACC_LOW_PASS_FILTER);

// gyro scale 2000 deg (FS =3)

	i2c_writereg( 27 , 24);

// Gyro DLPF low pass filter

	i2c_writereg( 26 , GYRO_LOW_PASS_FILTER);

#ifdef SIXAXIS_READ_DMA
	#ifndef USE_HARDWARE_I2C
		#warning "I2C DMA must use Hardware I2C"
	#endif

	#if defined(ENABLE_OVERCLOCK) && !(defined(HW_I2C_SPEED_FAST_OC) || defined(HW_I2C_SPEED_FAST2_OC) )
 		#warning "HW_I2C_SPEED_FAST(2)_OC not set"
	#endif

	#if !defined(ENABLE_OVERCLOCK) && !(defined(HW_I2C_SPEED_FAST) || defined(HW_I2C_SPEED_FAST2) )
 		#warning "HW_I2C_SPEED_FAST(2) not set"
	#endif

	#if	defined(HW_I2C_SPEED_FAST2) || defined(HW_I2C_SPEED_FAST2_OC)
		#define	SIXAXIS_READ_TIME	280			// 270us + 10us as a tolerance
	#else
		#define SIXAXIS_READ_TIME	510			// 500us + 10us as a tolerance
	#endif

	#define SIXAXIS_READ_PERIOD		( (SYS_CLOCK_FREQ_HZ * ((LOOPTIME-SIXAXIS_READ_TIME)*1e-6f)) )

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	// TIM17 Periph clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period =							SIXAXIS_READ_PERIOD;
	TIM_TimeBaseStructure.TIM_Prescaler = 					0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 			0;
	TIM_TimeBaseStructure.TIM_CounterMode = 				TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);
	TIM_ARRPreloadConfig(TIM17, ENABLE);
	TIM_Cmd( TIM17, DISABLE );

	/* configure TIM17 interrupt */
  NVIC_InitStructure.NVIC_IRQChannel =						TIM17_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority =		0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = 				ENABLE;
  NVIC_Init( &NVIC_InitStructure );
	TIM_ClearITPendingBit( TIM17, TIM_IT_Update );
	TIM_ITConfig( TIM17, TIM_IT_Update, ENABLE );

	////////////////////////////////////////////////////////////////////////
	DMA_InitTypeDef DMA_InitStructure;

	DMA_StructInit(&DMA_InitStructure);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* DMA1 Channe3 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel3);
	DMA_InitStructure.DMA_PeripheralBaseAddr = 		(uint32_t)&I2C1->RXDR;
	DMA_InitStructure.DMA_MemoryBaseAddr = 				(uint32_t)i2c_rx_buffer;
	DMA_InitStructure.DMA_DIR = 									DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 						14;
	DMA_InitStructure.DMA_PeripheralInc = 				DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = 						DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = 		DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = 				DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = 									DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = 							DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = 									DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);
	
	
    TIM_SetCounter( TIM17, SIXAXIS_READ_PERIOD );
    TIM_Cmd( TIM17, ENABLE );
    while( !DMA_GetFlagStatus( DMA1_FLAG_TC3 ) ) { };
    DMA_Cmd( DMA1_Channel3, DISABLE );
    I2C_DMACmd( I2C1, I2C_DMAReq_Rx, DISABLE );

// 	}
#endif
}

#ifdef SIXAXIS_READ_DMA
extern int hw_i2c_sendheader( int, int );

void TIM17_IRQHandler(void)
{
	TIM_Cmd( TIM17, DISABLE );
	TIM_ClearITPendingBit( TIM17, TIM_IT_Update );

	DMA_ClearFlag( DMA1_FLAG_GL3 );
	DMA1_Channel3->CNDTR = 14;

	hw_i2c_sendheader( 59 , 1 );
	//send restart + readaddress
	I2C_TransferHandling(I2C1, (0x68)<<1 , 14, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

	DMA_Cmd( DMA1_Channel3, ENABLE );
  I2C_DMACmd( I2C1, I2C_DMAReq_Rx, ENABLE );


}
#endif

int sixaxis_check( void)
{
	#ifndef DISABLE_GYRO_CHECK
	// read "who am I" register
	int id = i2c_readreg( 117 );

	#ifdef DEBUG
	debug.gyroid = id;
	#endif

	return (GYRO_ID_1==id||GYRO_ID_2==id||GYRO_ID_3==id||GYRO_ID_4==id );
	#else
	return 1;
	#endif
}

float accel[3];
float gyro[3];

float accelcal[3];
float gyrocal[3];
float gyro_unfiltered[3];

int calibration_done;

float lpffilter(float in, int num);
float lpffilter2(float in, int num);
void sixaxis_read(void)
{
	float gyronew[3];

#ifdef SIXAXIS_READ_DMA


	//if DMA not ready, SIXAXIS_READ_TIME should be larger and make less delay for trigger DMA
	extern void failloop(int);
	uint32_t	time=gettime();
	while( !DMA_GetFlagStatus( DMA1_FLAG_TC3 ) && (gettime()-time) < LOOPTIME ) { } 	// wait maximum a LOOPTIME for I2C DMA to complete
		if( !DMA_GetFlagStatus( DMA1_FLAG_TC3 ) ) failloop(9);
  DMA_Cmd( DMA1_Channel3, DISABLE );
  I2C_DMACmd( I2C1, I2C_DMAReq_Rx, DISABLE );
	//i2c_dma_phase = 0;

	// delayed trigger next DMA by TIM17
	TIM_SetCounter( TIM17, 0 );
	TIM_Cmd( TIM17, ENABLE );

#else
	int data[14];

	i2c_readdata( 59, data, 14 );
	for( int i=0;i<14;i++) i2c_rx_buffer[i] = (uint8_t)data[i];
#endif
	#ifdef SIXAXIS_READ_DMA
#endif

#ifdef SENSOR_ROTATE_90_CW
        accel[0] = (int16_t) ((i2c_rx_buffer[2] << 8) + i2c_rx_buffer[3]);
        accel[1] = -(int16_t) ((i2c_rx_buffer[0] << 8) + i2c_rx_buffer[1]);
        accel[2] = (int16_t) ((i2c_rx_buffer[4] << 8) + i2c_rx_buffer[5]);
#else

	accel[0] = -(int16_t) ((i2c_rx_buffer[0] << 8) + i2c_rx_buffer[1]);
	accel[1] = -(int16_t) ((i2c_rx_buffer[2] << 8) + i2c_rx_buffer[3]);
	accel[2] = (int16_t) ((i2c_rx_buffer[4] << 8) + i2c_rx_buffer[5]);

#endif


#ifdef SENSOR_ROTATE_90_CW_deleted
		{//
		float temp = accel[1];
		accel[1] = accel[0];
		accel[0] = -temp;
		}
#endif

// this is the value of both cos 45 and sin 45 = 1/sqrt(2)
#define INVSQRT2 0.707106781f

#ifdef SENSOR_ROTATE_45_CCW
		{
		float temp = accel[0];
		accel[0] = (accel[0] * INVSQRT2 + accel[1] * INVSQRT2);
		accel[1] = -(temp * INVSQRT2 - accel[1] * INVSQRT2);
		}
#endif

#ifdef SENSOR_ROTATE_45_CW
		{
		float temp = accel[1];
		accel[1] = (accel[1] * INVSQRT2 + accel[0] * INVSQRT2);
		accel[0] = -(temp * INVSQRT2 - accel[0] * INVSQRT2);
		}
#endif


#ifdef SENSOR_ROTATE_90_CCW
		{
		float temp = accel[1];
		accel[1] = -accel[0];
		accel[0] = temp;
		}
#endif

#ifdef SENSOR_ROTATE_180
		{
		accel[1] = -accel[1];
		accel[0] = -accel[0];
		}
#endif

#ifdef SENSOR_FLIP_180
		{
		accel[2] = -accel[2];
		accel[0] = -accel[0];
		}
#endif
//order
	gyronew[1] = (int16_t) ((i2c_rx_buffer[8] << 8) + i2c_rx_buffer[9]);
	gyronew[0] = (int16_t) ((i2c_rx_buffer[10] << 8) + i2c_rx_buffer[11]);
	gyronew[2] = (int16_t) ((i2c_rx_buffer[12] << 8) + i2c_rx_buffer[13]);


gyronew[0] = gyronew[0] - gyrocal[0];
gyronew[1] = gyronew[1] - gyrocal[1];
gyronew[2] = gyronew[2] - gyrocal[2];

#ifdef SENSOR_ROTATE_90_CW
		{
		float temp = gyronew[1];
		gyronew[1] = -gyronew[0];
		gyronew[0] = temp;
		}
#endif


#ifdef SENSOR_ROTATE_45_CCW
		{
		float temp = gyronew[1];
		gyronew[1] = gyronew[0] * INVSQRT2 + gyronew[1] * INVSQRT2;
		gyronew[0] = gyronew[0] * INVSQRT2 - temp * INVSQRT2;
		}
#endif

#ifdef SENSOR_ROTATE_45_CW
		{
		float temp = gyronew[0];
		gyronew[0] = gyronew[1] * INVSQRT2 + gyronew[0] * INVSQRT2;
		gyronew[1] = gyronew[1] * INVSQRT2 - temp * INVSQRT2;
		}
#endif


#ifdef SENSOR_ROTATE_90_CCW
		{
		float temp = gyronew[1];
		gyronew[1] = gyronew[0];
		gyronew[0] = -temp;
		}
#endif


#ifdef SENSOR_ROTATE_180
		{
		gyronew[1] = -gyronew[1];
		gyronew[0] = -gyronew[0];
		}
#endif

#ifdef SENSOR_FLIP_180
		{
		gyronew[1] = -gyronew[1];
		gyronew[2] = -gyronew[2];
		}
#endif

//gyronew[0] = - gyronew[0];
gyronew[1] = - gyronew[1];
gyronew[2] = - gyronew[2];

	for (int i = 0; i < 3; i++)
	  {
		  gyronew[i] = gyronew[i] * 0.061035156f * 0.017453292f;
		
			#ifndef SOFT_LPF_NONE

		#if defined (GYRO_FILTER_PASS2) && defined (GYRO_FILTER_PASS1)
			gyro[i] = lpffilter(gyronew[i], i);
			gyro[i] = lpffilter2(gyro[i], i);
		#endif

		#if defined (GYRO_FILTER_PASS1) && !defined(GYRO_FILTER_PASS2)
			gyro[i] = lpffilter(gyronew[i], i);
		#endif

		#if defined (GYRO_FILTER_PASS2) && !defined(GYRO_FILTER_PASS1)
			gyro[i] = lpffilter2(gyronew[i], i);
		#endif
#else
		  gyro[i] = gyronew[i];
#endif
			
			gyro_unfiltered[i] = gyronew[i];
	  }
}

void gyro_read( void)
{
int data[6];

i2c_readdata( 67, data, 6);

float gyronew[3];
	// order
gyronew[1] = (int16_t) ((data[0]<<8) + data[1]);
gyronew[0] = (int16_t) ((data[2]<<8) + data[3]);
gyronew[2] = (int16_t) ((data[4]<<8) + data[5]);


gyronew[0] = gyronew[0] - gyrocal[0];
gyronew[1] = gyronew[1] - gyrocal[1];
gyronew[2] = gyronew[2] - gyrocal[2];



#ifdef SENSOR_ROTATE_45_CCW
		{
		float temp = gyronew[1];
		gyronew[1] = gyronew[0] * INVSQRT2 + gyronew[1] * INVSQRT2;
		gyronew[0] = gyronew[0] * INVSQRT2 - temp * INVSQRT2;
		}
#endif

#ifdef SENSOR_ROTATE_45_CW
		{
		float temp = gyronew[0];
		gyronew[0] = gyronew[1] * INVSQRT2 + gyronew[0] * INVSQRT2;
		gyronew[1] = gyronew[1] * INVSQRT2 - temp * INVSQRT2;
		}
#endif

#ifdef SENSOR_ROTATE_90_CW
		{
		float temp = gyronew[1];
		gyronew[1] = -gyronew[0];
		gyronew[0] = temp;
		}
#endif


#ifdef SENSOR_ROTATE_90_CCW
		{
		float temp = gyronew[1];
		gyronew[1] = gyronew[0];
		gyronew[0] = -temp;
		}
#endif


#ifdef SENSOR_ROTATE_180
		{
		gyronew[1] = -gyronew[1];
		gyronew[0] = -gyronew[0];
		}
#endif


#ifdef SENSOR_FLIP_180
		{
		gyronew[1] = -gyronew[1];
		gyronew[2] = -gyronew[2];
		}
#endif




//gyronew[0] = - gyronew[0];
gyronew[1] = - gyronew[1];
gyronew[2] = - gyronew[2];


for (int i = 0; i < 3; i++)
	  {
		  gyronew[i] = gyronew[i] * 0.061035156f * 0.017453292f;

#ifndef SOFT_LPF_NONE

		#if defined (GYRO_FILTER_PASS2) && defined (GYRO_FILTER_PASS1)
			gyro[i] = lpffilter(gyronew[i], i);
			gyro[i] = lpffilter2(gyro[i], i);
		#endif

		#if defined (GYRO_FILTER_PASS1) && !defined(GYRO_FILTER_PASS2)
			gyro[i] = lpffilter(gyronew[i], i);
		#endif

		#if defined (GYRO_FILTER_PASS2) && !defined(GYRO_FILTER_PASS1)
			gyro[i] = lpffilter2(gyronew[i], i);
		#endif
#else
		  gyro[i] = gyronew[i];
#endif
			
			gyro_unfiltered[i] = gyronew[i];
	  }

}



#define CAL_TIME 2e6

#ifdef SIXAXIS_READ_DMA
void gyro_cal(void)
{
//int data[6];
float limit[3];
unsigned long time = gettime();
unsigned long timestart = time;
unsigned long timemax = time;
unsigned long lastlooptime = time;

float gyro[3];

 for ( int i = 0 ; i < 3 ; i++)
			{
			limit[i] = gyrocal[i];
			}

// 2 and 15 seconds
while ( time - timestart < CAL_TIME  &&  time - timemax < 15e6 )
	{

		unsigned long looptime;
		looptime = time - lastlooptime;
		lastlooptime = time;
		if ( looptime == 0 ) looptime = 1;

	//i2c_readdata( 67, data, 6 );


	//gyro[1] = (int16_t) ((data[0]<<8) + data[1]);
	//gyro[0] = (int16_t) ((data[2]<<8) + data[3]);
	//gyro[2] = (int16_t) ((data[4]<<8) + data[5]);

		sixaxis_read();
		gyro[1] = (int16_t) ((i2c_rx_buffer[8] << 8) + i2c_rx_buffer[9]);
		gyro[0] = (int16_t) ((i2c_rx_buffer[10] << 8) + i2c_rx_buffer[11]);
		gyro[2] = (int16_t) ((i2c_rx_buffer[12] << 8) + i2c_rx_buffer[13]);

/*
if ( (time - timestart)%200000 > 100000)
{
	ledon(B00000101);
	ledoff(B00001010);
}
else
{
	ledon(B00001010);
	ledoff(B00000101);
}
*/
#define GLOW_TIME 62500
static int brightness = 0;
led_pwm( brightness);
if ((brightness&1)^((time - timestart)%GLOW_TIME > (GLOW_TIME>>1) ))
{
brightness++;
}

brightness&=0xF;

		 for ( int i = 0 ; i < 3 ; i++)
			{

					if ( gyro[i] > limit[i] )  limit[i] += 0.1f; // 100 gyro bias / second change
					if ( gyro[i] < limit[i] )  limit[i] -= 0.1f;

					limitf( &limit[i] , 800);

					if ( fabsf(gyro[i]) > 100+ fabsf(limit[i]) )
					{
						timestart = gettime();
						brightness = 1;
					}
					else
					{
					lpf( &gyrocal[i] , gyro[i], lpfcalc( (float) looptime , 0.5 * 1e6) );

					}

			}

// receiver function
void checkrx( void);
checkrx();			
			
while ( (gettime() - time) < 1000 ) delay(10);
time = gettime();

	}



if ( time - timestart < CAL_TIME )
{
	for ( int i = 0 ; i < 3; i++)
	{
	gyrocal[i] = 0;

	}

}

calibration_done = 1;

}

#else // not using DMA
void gyro_cal(void)
{
int data[6];
float limit[3];	
unsigned long time = gettime();
unsigned long timestart = time;
unsigned long timemax = time;
unsigned long lastlooptime = time;

float gyro[3];	
	
 for ( int i = 0 ; i < 3 ; i++)
			{
			limit[i] = gyrocal[i];
			}

// 2 and 15 seconds
while ( time - timestart < CAL_TIME  &&  time - timemax < 15e6 )
	{	
		
		unsigned long looptime; 
		looptime = time - lastlooptime;
		lastlooptime = time;
		if ( looptime == 0 ) looptime = 1;

	i2c_readdata(  67 , data , 6 );	

			
	gyro[1] = (int16_t) ((data[0]<<8) + data[1]);
	gyro[0] = (int16_t) ((data[2]<<8) + data[3]);
	gyro[2] = (int16_t) ((data[4]<<8) + data[5]);
		
/*		
if ( (time - timestart)%200000 > 100000) 
{
	ledon(B00000101);
	ledoff(B00001010);
}
else 
{
	ledon(B00001010);
	ledoff(B00000101);
}
*/
#define GLOW_TIME 62500 
static int brightness = 0;
led_pwm( brightness);
if ((brightness&1)^((time - timestart)%GLOW_TIME > (GLOW_TIME>>1) ))
{
brightness++;
}

brightness&=0xF;

		 for ( int i = 0 ; i < 3 ; i++)
			{

					if ( gyro[i] > limit[i] )  limit[i] += 0.1f; // 100 gyro bias / second change
					if ( gyro[i] < limit[i] )  limit[i] -= 0.1f;
				
					limitf( &limit[i] , 800);
				
					if ( fabsf(gyro[i]) > 100+ fabsf(limit[i]) ) 
					{										
						timestart = gettime();
						brightness = 1;
					}
					else
					{						
					lpf( &gyrocal[i] , gyro[i], lpfcalc( (float) looptime , 0.5 * 1e6) );
					}
			}

while ( (gettime() - time) < 1000 ) delay(10); 				
time = gettime();
	}
	
if ( time - timestart < CAL_TIME )
{
	for ( int i = 0 ; i < 3; i++)
	{
	gyrocal[i] = 0;

	}
}
	calibration_done = 1;
}
#endif


void acc_cal(void)
{
	accelcal[2] = 2048;
	for (int y = 0; y < 500; y++)
	  {
		  sixaxis_read();
		  for (int x = 0; x < 3; x++)
		    {
			    lpf(&accelcal[x], accel[x], 0.92);
		    }
		  gettime();	// if it takes too long time will overflow so we call it here

	  }
	accelcal[2] -= 2048;

#ifdef FLASH_SAVE2
	for (int x = 0; x < 3; x++)
	  {
		  limitf(&accelcal[x], 127);
	  }
#endif

#ifdef FLASH_SAVE1
	for (int x = 0; x < 3; x++)
	  {
		  limitf(&accelcal[x], 500);
	  }
#endif
}

#endif


