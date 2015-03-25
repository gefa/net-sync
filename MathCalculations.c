/**
 * @file 	MathCalculations.c
 * @author 	Alex Ryan
 * @date	MAR 16, 2015
 * @brief 	Math heavy code for calculations on stop/start/index times are here
 *
 * Moved all of the calculation code to this file
 *
 */

#include "MathCalculations.h"


//Calculation Variables
float buf[M];       	// search buffer
float matchedFilterCosine[M];			// in-phase correlation buffer
float matchedFilterSine[M];       	// quadrature correlation buffer
float corr_max, corr_max_s, corr_max_c; // correlation variables
float corr_c[2*M];
float corr_s[2*M];
float s[2*M];
short corr_max_lag;
short bufindex = 0;
float corrSumCosine,corrSumSine,corrSumIncoherent;
short i,j,k;				// Indices
double t,x,y;				// More Indices
float basebandSincRef[2*N+1];   		// baseband sinc pulse buffer
float recbuf[2*N+2*M]; 		// recording buffer
float downMixedCosine[2*N+2*M];     		// in-phase downmixed buffer
float downMixedSine[2*N+2*M];     		// quadrature downmixed buffer
short recbufindex = 0;		//

volatile char local_carrier_phase = 0;
char r = 0;
double phase_correction_factor;
short max_samp = 0;

volatile int recbuf_start_clock = 0; // virtual clock counter for first sample in recording buffer


/**
 * Calculates the delay estimates
 * Stores the results in the coarse_delay_estimate and fine_delay_estimate buffers
 */
void runReceviedSincPulseTimingAnalysis(){
	// this is where we apply the matched filter
	// we only do this over a limited range
	for (i=0;i<=(2*M-1);i++) {
		corr_c[i] = 0;
		corr_s[i] = 0;
		for (j=0;j<(2*N+1);j++) {
			corr_c[i] += basebandSincRef[j]*downMixedCosine[j+i];
			corr_s[i] += basebandSincRef[j]*downMixedSine[j+i];
		}
		s[i] = corr_c[i]*corr_c[i]+corr_s[i]*corr_s[i];  // noncoherent correlation metric
	}

	// now find the peak
	corr_max = 0;
	corr_max_lag = 0;
	for (i=0;i<=(2*M-1);i++) {
		if (s[i]>corr_max){
			corr_max = s[i];
			corr_max_lag = i;
		}
	}
	corr_max_c = corr_c[corr_max_lag];
	corr_max_s = corr_s[corr_max_lag];

	/*
	printf wrecks the real-time operation
	printf("Max lag: %d\n",corr_max_lag);
	printf("Coarse delay estimate: %d.\n",recbuf_start_clock+corr_max_lag);
 	 */


	//increment indices
	cde_index++;
	if(cde_index>=MAX_STORED_DELAYS_COARSE) cde_index = 0;
	fde_index++;
	if(fde_index>=MAX_STORED_DELAYS_FINE) fde_index = 0;
	//We do this here, so that the current indice that we are about to store is always the most recent.

	// store coarse delay estimates
	coarse_delay_estimate[cde_index] = recbuf_start_clock+corr_max_lag;

	// fine delay estimate
	y = (double) corr_max_s;
	x = (double) corr_max_c;
	phase_correction_factor = atan2(y,x)*2*INVPI; // phase
	r = (recbuf_start_clock+corr_max_lag) & 3; // compute remainder
	if (r==0)
		fine_delay_estimate[fde_index] = recbuf_start_clock+corr_max_lag+phase_correction_factor;
	else if (r==1)
		fine_delay_estimate[fde_index] = recbuf_start_clock+corr_max_lag+phase_correction_factor-1;
	else if (r==2) {
		if (phase_correction_factor>0)
			fine_delay_estimate[fde_index] = recbuf_start_clock+corr_max_lag+phase_correction_factor-2;
		else
			fine_delay_estimate[fde_index] = recbuf_start_clock+corr_max_lag+phase_correction_factor+2;
	}
	else if (r==3)
		fine_delay_estimate[fde_index] = recbuf_start_clock+corr_max_lag+phase_correction_factor+1;
	else
		printf("ERROR");

	// --- Calculations Finished ---
}

/**
 * Mixes the received waveform in recbuf down to baseband. ONLY WORKS at currently set center freq (1/2 of nyquist)
 */
void runReceivedPulseBufferDownmixing(){
	// downmix (had problems using sin/cos here so used a trick)
	/* The trick is based on the incoming frequency per sample being (n * pi/2), so every other sample goes to zero,
	 * while the non-zero components sin() multiplicative factor is unity/1 */
	for (i=0;i<(2*N+2*M);i+=4){
		downMixedCosine[i] = recbuf[i];
		downMixedSine[i] = 0;
 	}
	for (i=1;i<(2*N+2*M);i+=4){
		downMixedCosine[i] = 0;
		downMixedSine[i] = recbuf[i];
	}
	for (i=2;i<(2*N+2*M);i+=4){
		downMixedCosine[i] = -recbuf[i];
		downMixedSine[i] = 0;
	}
	for (i=3;i<(2*N+2*M);i+=4){
		downMixedCosine[i] = 0;
		downMixedSine[i] = -recbuf[i];
	}
}



/**
	Sets up the transmit buffer for the sinc pulse modulated at quarter sampling frequency
*/
void SetupTransmitModulatedSincPulseBuffer(){

	for (i=-N;i<=N;i++){
		x = i*BW;
		t = i*CBW;
		if (i!=0)
			y = cos(2*PI*t)*(sin(PI*x)/(PI*x)); // modulated sinc pulse at carrier freq = fs/4
		else
			y = 1;								//x = 0 case.
		tClockSincPulse[i+N] = y*32767;			//Both of these  may be modified in the future calculations anywas
		tVerifSincPulse[i+N] = y*32767;			//If FDE mode is on in order to do partial synchronization
	}
	//Half sample delayed
	for (i=-N;i<=N;i++){
		x = (i-0.5)*BW;
		t = (i-0.5)*0.25;
		y = cos(2*PI*t)*(sin(PI*x)/(PI*x)); // modulated sinc pulse at carrier freq = fs/4
		tVerifSincPulsePhased[i+N] = y*32767;
	}




}
/**
	Sets up the buffer used for matched filtering of the sinc pulse
*/
void SetupReceiveBasebandSincPulseBuffer(){
	for (i=-N;i<=N;i++){
		x = i*BW;
		if (i!=0)
			y = sin(PI*x)/(PI*x); // double
		else
			y = 1.0;
		basebandSincRef[i+N] = (float) y;
	}
}
/**
	Sets up the matched filter buffers which are used for matching and filtering of the incoming sines and cosines
*/
void SetupReceiveTrigonometricMatchedFilters(){
	for (i=0;i<M;i++){
		t = i*CBW;				// time
		y = cos(2*PI*t);		// cosine matched filter (double)
		matchedFilterCosine[i] = (float) y;		// cast and store
		y = sin(2*PI*t);		// sine matched filter (double)
		matchedFilterSine[i] = (float) y;     // cast and store
		buf[i] = 0;             // clear searching buffer
	}
}

/**
 * Looks at the received response time, the current time (to prevent trying to send immediately a fractional waveform,
 * and calculates the new mirrored center time around a clock tick upon which to center the response at
 */
short calculateNewResponseTimeMaster(short curTime, short delayEstimate){
	return 0; /** \todo */
}

/**
 *
 */
short calculateNewSynchronizationTimeSlave(short curTime, short delayEstimate){
	return 0; /** \todo */
}

/**
 *	Responds with the buffer index for transmitting the output synchronization sinc pulse
 *	Essentially looks at the virClockTransmitCenterSinc variable, and subtracts and wraps
 *
 *
 */
short GetSincPulseIndex(){
	return 0;
}

/**
 *	Responds with the buffer index for transmitting the output verification sinc pulse
 *
 *	Variables it looks at:
 *	virClockTransmitCenterVerify
 *	SMALL_VCLK_WRAP(vclock_counter)
 *	N 							(to find when to start/stop)
 *
 *
 */
short GetVerifPulseIndex(){
	return 0;
}

/**
 * Sums up an array of floats to produce the sum
 * @param array	The array to sum
 * @param numElmts Number of elements to sum
 * @return The sum
 *
 *
 */
float sumFloatArray(float* array, short numElmts){
	float sum = 0.0;
	short idx = 0;
	for(idx=0; idx < numElmts; idx++){
		sum += array[idx];
	}
	return sum;
}

/**
 * Sums an array of integers and returns the result
 * @param array pointer to array of values
 * @param numElmts
 * @return the sum of the array. In long format to prevent overflow
 */
long sumIntArray(short* array, short numElmts){
	long sum = 0;
	short idx = 0;
	for(idx=0; idx < numElmts; idx++){
		sum += (long)array[idx];
	}
	return sum;
}

/**
 * calculates the new time to transmit for the verification on the slave
 * @param curTime
 * @param delayEstimate
 * @return the center time of the vclock counter
 */
int calculateNewSynchronizationTimeSlaveCoarse(int curTime, int delayEstimate){
	return 0;
}
/**
 * Calculates the new center time to transmit at for the masters mirrored response to the slave
 * @param curTime
 * @param delayEstimate
 * @return the center time of vclock counter
 */
int calculateNewResponseTimeMasterCoarse(int curTime, int delayEstimate){
	return 0;
}


/**
 * Calculates the new master buffer given the prior delay estimates for responding around the center tick.
 * @param buffer
 * @param bufferLen
 * @param fine_delay_estimate
 * @param fde_index
 */
void calculateNewResponseBufferMaster(short* buffer, short bufferLen, float* fine_delay_estimate, short fde_index){

}


/**
 * Writes the verification waveform used to show synchronization with the master to the slaves buffer.
 * This is used for fine synchronization (subsample)
 * @param buffer				Bufer to write into
 * @param bufferLen				length of buffer (usually N2)
 * @param fine_delay_estimate	array of prior delay estimations to allow for linear extrapolation
 * @param fde_index				index of latest estimate
 */
void calculateNewVerifBufferSlave(short* buffer, short bufferLen, float* fine_delay_estimate, short fde_index){

}

/**
 *
 * @param vclock_counter
 * @param fine_delay_estimate
 * @param fde_index
 * @return the clock time to put into
 */
int calculateNewResponseTimeMasterFine(int vclock_counter, float* fine_delay_estimate, short fde_index){
	return 0;
}

/**
 *
 * @param vclock_counter		current counter time
 * @param fine_delay_estimate   array of prior delay estimates
 * @param fde_index				index of current estimate
 * @return
 */
int calculateNewSynchronizationTimeSlaveFine(int vclock_counter, float* fine_delay_estimate, short fde_index){
	return 0;
}

