
#include "config.h"
#include "defines.h"

#ifdef KALMAN_GYRO
// kalman Q/R ratio for Q = 0.02
// loop time 1000Hz
#define	HZ_10	0.004078
#define	HZ_20	0.015952
#define	HZ_30	0.035546
#define	HZ_40	0.062984
#define	HZ_50	0.097857
#define	HZ_60	0.139957
#define	HZ_70	0.190992
#define	HZ_80	0.249072
#define	HZ_90	0.308894
#define	HZ_100	0.397188
#define	HZ_120	0.542488
#define	HZ_140	0.719026
#define	HZ_160	0.928746
#define	HZ_180	1.144837
#define	HZ_200	1.354386
#define	HZ_220	1.611742
#define	HZ_240	1.87532
#define	HZ_260	2.123421
#define	HZ_280	2.377006
#define	HZ_300	2.595641
#define	HZ_320	2.864404
#define	HZ_340	3.052077
#define	HZ_360	3.272997
#define	HZ_380	3.44942
#define	HZ_400	3.679173
#define	HZ_420	3.721861
#define	HZ_440	3.880844
#define	HZ_460	3.908564
#define	HZ_480	3.984022
#define	HZ_500	4.100000
#endif

#ifdef PT1_GYRO
#define	HZ_10	10
#define	HZ_20	20
#define	HZ_30	30
#define	HZ_40	40
#define	HZ_50	50
#define	HZ_60	60
#define	HZ_70	70
#define	HZ_80	80
#define	HZ_90	90
#define	HZ_100	100
#define	HZ_120	120
#define	HZ_140	140
#define	HZ_160	160
#define	HZ_180	180
#define	HZ_200	200
#define	HZ_220	220
#define	HZ_240	240
#define	HZ_260	260
#define	HZ_280	280
#define	HZ_300	300
#define	HZ_320	320
#define	HZ_340	340
#define	HZ_360	360
#define	HZ_380	380
#define	HZ_400	400
#define	HZ_420	420
#define	HZ_440	440
#define	HZ_460	460
#define	HZ_480	480
#define	HZ_500	500
#endif

#ifndef GYRO_FILTER_PASS1
  #define SOFT_LPF1_NONE
#endif

#ifndef GYRO_FILTER_PASS2
  #define SOFT_LPF2_NONE
#endif


#if defined PT1_GYRO && defined GYRO_FILTER_PASS1
	#define SOFT_LPF_1ST_PASS1 GYRO_FILTER_PASS1
extern "C" float lpfcalc( float sampleperiod , float filtertime);
extern "C" float lpfcalc_hz(float sampleperiod, float filterhz);
extern "C" void lpf( float *out, float in , float coeff);

static float alpha = 0.5;
extern float looptime;

void lpf_coeff()
{
 alpha = FILTERCALC( looptime , (1.0f/SOFT_LPF_1ST_PASS1) );       
}


class  filter_lpf1
{
    private:
		float lpf_last;
    public:
        filter_lpf1()
    {
      lpf_last = 0;   
    }
     float step( float in)
     {
       lpf ( &lpf_last , in , alpha); 
         
       return lpf_last;
     }
};

filter_lpf1 filter[3];
#endif



#if defined PT1_GYRO && defined GYRO_FILTER_PASS2
	#define SOFT_LPF_1ST_PASS2 GYRO_FILTER_PASS2
extern "C" float lpfcalc( float sampleperiod , float filtertime);
extern "C" float lpfcalc_hz(float sampleperiod, float filterhz);
extern "C" void lpf( float *out, float in , float coeff);

static float alpha2 = 0.5;
extern float looptime;

void lpf_coeff_pass2()
{
 alpha2 = FILTERCALC( looptime , (1.0f/SOFT_LPF_1ST_PASS2) );       
}


class  filter_lpf2
{
    private:
		float lpf_last;
    public:
        filter_lpf2()
    
{
      lpf_last = 0;   
    }
     float step( float in)
     {
       lpf ( &lpf_last , in , alpha2); 

       return lpf_last;
     }
};

filter_lpf2 filter2[3];
#endif



#if defined KALMAN_GYRO && defined GYRO_FILTER_PASS1
 #define SOFT_KALMAN_GYRO_PASS1 GYRO_FILTER_PASS1
class  filter_kalman
{
    private:
        float x_est_last ;
        float P_last ; 
        float Q;
        float R;
    public:
        filter_kalman()
        {
            Q = 0.02; 
            R = 0.1;

            #ifdef SOFT_KALMAN_GYRO_PASS1
            R = Q/(float)SOFT_KALMAN_GYRO_PASS1;
            #endif
        }
        float  step( float in )   
        {    

            //do a prediction 
            float x_temp_est = x_est_last; 
            float P_temp = P_last + Q; 

            float K = P_temp * (1.0f/(P_temp + R));
            float x_est = x_temp_est + K * (in - x_temp_est);  
            float P = (1- K) * P_temp; 
           
            //update our last's 
            P_last= P; 
            x_est_last = x_est; 

            return x_est;
        }
};       
filter_kalman filter[3];       
#endif

#if defined KALMAN_GYRO && defined GYRO_FILTER_PASS2
	#define SOFT_KALMAN_GYRO_PASS2 GYRO_FILTER_PASS2
class  filter_kalman2
{
    private:
        float x_est_last ;
        float P_last ; 
        float Q;
        float R;
    public:
        filter_kalman2()
        {
            Q = 0.02; 
            R = 0.1;

						#ifdef SOFT_KALMAN_GYRO_PASS2
					  R = Q/(float)SOFT_KALMAN_GYRO_PASS2;
            #endif
        }
        float  step( float in )   
        {    

            //do a prediction 
            float x_temp_est = x_est_last; 
            float P_temp = P_last + Q; 

            float K = P_temp * (1.0f/(P_temp + R));
            float x_est = x_temp_est + K * (in - x_temp_est);  
            float P = (1- K) * P_temp; 

            //update our last's 
            P_last= P; 
            x_est_last = x_est; 

            return x_est;
        }
};       
filter_kalman2 filter2[3];       
#endif



extern "C" float lpffilter( float in,int num )
{
#ifdef SOFT_LPF1_NONE
	return in;
	#else
    
   #ifdef SOFT_LPF_1ST_PASS1
    if ( num == 0 ) lpf_coeff();
    #endif
    
	      return filter[num].step(in );   
	      #endif
	
}


 extern "C" float lpffilter2( float in,int num )
{
	#ifdef SOFT_LPF2_NONE
	return in;
	#else

    #ifdef SOFT_LPF_1ST_PASS2
    if ( num == 0 ) lpf_coeff_pass2();
    #endif

	return filter2[num].step(in );   
	#endif

} 

// 16Hz hpf filter for throttle compensation
//High pass bessel filter order=1 alpha1=0.016 
class  FilterBeHp1
{
	public:
		FilterBeHp1()
		{
			v[0]=0.0;
		}
	private:
		float v[2];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = (9.521017968695103528e-1f * x)
				 + (0.90420359373902081668f * v[0]);
			return 
				 (v[1] - v[0]);
		}
};

FilterBeHp1 throttlehpf1;

extern "C" float throttlehpf( float in )
{
	return throttlehpf1.step(in );
}

 
// for TRANSIENT_WINDUP_PROTECTION feature
//Low pass bessel filter order=1 alpha1=0.023
class  FilterSP
{
	public:
		FilterSP()
		{
			v[0]=0.0;
		}
	private:
		float v[2];
	public:
		float step(float x) //class II
		{
			v[0] = v[1];
			v[1] = (6.749703162983405891e-2f * x)
				 + (0.86500593674033188218f * v[0]);
			return
				 (v[0] + v[1]);
		}
};

FilterSP spfilter[3];

extern "C" float splpf( float in,int num )
{

	return spfilter[num].step(in );
}



