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
#include "ProjectDefinitions.h"


// length of searching window in samples
#define M 60

// threshold value for searching window
#define T1 100000

// max lag for computing correlations
#define MAXLAG 200

// maximum sample value
#define MAXSAMP 32767; //short sample value (also -32768)

// define PI and INVPI
#define PI 3.14159265358979323846
#define INVPI 0.318309886183791

//Delay estimates (here because they're used for all code)
extern int coarse_delay_estimate[MAX_STORED_DELAYS_COARSE];
extern float fine_delay_estimate[MAX_STORED_DELAYS_FINE];
extern short cde_index;
extern short fde_index;

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

extern volatile int recbuf_start_clock; // virtual clock counter for first sample in recording buffer



//Basic Math Helpers
double sin(double);
double cos(double);
double atan2(double,double);
float sumFloatArray(float*, short numElmts);
long sumIntArray(short* array, short numElmts);

//Setup Functions
void SetupTransmitModulatedSincPulseBuffer();
void setupTransmitBuffer(short* tBuffer, short halfBufLen, float sincBandwidth, float carrierFreq, float delay);
void SetupReceiveBasebandSincPulseBuffer();
void setupBasebandSincBuffer(float* buffer, short halfBufLen, float sincBandwidth);
void SetupReceiveTrigonometricMatchedFilters();
void setupQuadratureCarrierWaveFilterBuffer(float* inPhaseBuffer, float* quadraturebuffer, short bufLen, float cFreq);

//Analysis functions
void runReceivedPulseBufferDownmixing();
void quarterWavePulseDownmix(float* receiveBuf, float* dmCos, float* dmSin, short receiveBufSize);
void runReceviedSincPulseTimingAnalysis();

// Time Calc Functions
int calculateNewSynchronizationTimeSlaveCoarse(int curTime, int delayEstimate);
int calculateNewResponseTimeMasterCoarse(int curTime, int delayEstimate);

void calculateNewResponseBufferMaster(short* buffer, short bufferLen, float* fine_delay_estimate, short fde_index);
int calculateNewResponseTimeMasterFine(int vclock_counter, float* fine_delay_estimate, short fde_index);

void calculateNewVerifBufferSlave(short* buffer, short bufferLen, float* fine_delay_estimate, short fde_index);
int calculateNewSynchronizationTimeSlaveFine(int vclock_counter, float* fine_delay_estimate, short fde_index);

int GetPulseIndex(int VClockCurrent, int VClockCenterPulse, short halfBufLen);

#endif /* MATHCALCULATIONS_H_ */
