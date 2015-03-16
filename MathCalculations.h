/**
 * @file 	MathCalculations.h
 * @author 	Alex Ryan
 * @date	MAR 16, 2015
 * @brief 	header for the calculation code moved to MathCalculations.c
 */

#ifndef MATHCALCULATIONS_H_
#define MATHCALCULATIONS_H_

#include <stdio.h>					//For printf
#include <stdint.h>
#include <c6x.h>					//generic include
#include <csl.h>					//generic csl include
#include <csl_gpio.h>				//GPIO support
#include <csl_mcbsp.h>				//for codec support
#include <csl_irq.h>				//interrupt support
#include <math.h>					//duh

#include "dsk6713.h"
#include "dsk6713_aic23.h"
#include "dsk6713_led.h"

#include "DebugTools.h"
#include "MathCalculations.h"
#include "time_stamper_master.h"

// sinc pulse normalized bandwidth
#define BW 0.0125 //100Hz@8KhzFs

// 2*N+1 is the number of samples in the sinc function
#define N (1 << 9) //512
#define N2 ((N*2)+1) //1025

// length of searching window in samples
#define M 60

// threshold value for searching window
#define T1 100000


// max lag for computing correlations
#define MAXLAG 200

// number of coarse delays to store
#define MAX_STORED_DELAYS_COARSE 16
#define MAX_STORED_DELAYS_FINE 	 16


// maximum sample value
#define MAXSAMP 32767; //short sample value (also -32768)

// define PI and INVPI
#define PI 3.14159265358979323846
#define INVPI 0.318309886183791


//Calculation Variables
extern float buf[M];       	// search buffer
extern float matchedFilterCosine[M];			// in-phase correlation buffer
extern float matchedFilterSine[M];       	// quadrature correlation buffer
extern float corr_max, corr_max_s, corr_max_c; // correlation variables
extern float corr_c[2*M];
extern float corr_s[2*M];
extern float s[2*M];
extern short corr_max_lag;
extern short bufindex;
extern float corrSumCosine,corrSumSine,corrSumIncoherent;
extern short i,j,k;				// Indices
extern double t,x,y;				// More Indices
extern float basebandSincRef[2*N+1];   		// baseband sinc pulse buffer
extern float recbuf[2*N+2*M]; 		// recording buffer
extern float downMixedCosine[2*N+2*M];     		// in-phase downmixed buffer
extern float downMixedSine[2*N+2*M];     		// quadrature downmixed buffer
extern short recbufindex;		//

extern volatile char local_carrier_phase;
extern char r;
extern double phase_correction_factor;
extern short max_samp;

extern volatile short recbuf_start_clock; // virtual clock counter for first sample in recording buffer



//Basic Math Helpers
double sin(double);
double cos(double);
double atan2(double,double);
float sumFloatArray(float*, short numElmts);
long sumIntArray(short* array, short numElmts);

//Setup Functions
void SetupTransmitModulatedSincPulseBuffer();
void SetupReceiveBasebandSincPulseBuffer();
void SetupReceiveTrigonometricMatchedFilters();

//Analysis functions
void runReceivedPulseBufferDownmixing();
void runReceviedSincPulseTimingAnalysis();

// Time Calc Functions
short calculateNewSynchronizationTimeSlave(short curTime, short delayEstimate);
short calculateNewResponseTimeMaster(short curTime, short delayEstimate);


#endif /* MATHCALCULATIONS_H_ */
