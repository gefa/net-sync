/*************************************************************************
 *  Timestamp free synchronization
 *  Slave node code with coarse delay estimation
 *  DRB Apr 14, 2015
 *  Plays out a modulated sinc pulse on the left channel every 4 clock periods

 *  States:
 *  State -1: do nothing (not implemented yet)
 *  State 0: searching
 *  State 1: recording
 *  State 2: coarse/fine delay estimation
 *  State 3: set up adjusted virtual clock buffer?
 *
 *************************************************************************/

#define CHIP_6713 1

// length of searching window in samples
#define M 40

// threshold value for searching window
#define T1 100000

// sinc pulse normalized bandwidth
#define BW 0.0125

// 2*N+1 is the number of samples in the sinc function
#define N 512

// virtual clock counter
#define L 4096

// modulated sinc pulse buffer length (mostly zeros)
#define LL 16384//12288 //16384 // 3*4096 gives faster clock error correction update rate
						 // current code does not require LL to be a power of 2

#define CLK_BUF_SFTY_MARGIN 40
#define CALC_SFTY_MARGIN 300

// define PI and INVPI
#define PI 3.14159265358979323846
#define INVPI 0.318309886183791

#define CLOCK_WRAP(i) ((i)&(L-1)) // index wrapping macro
#define SINC_WRAP(i) ((i)&(LL-1))

// Fine estimate resolution
#define FER	100		//Number of fine delay buffers
#define BW 0.0125 	//100Hz@8k Fs baseband sinc frequency
#define CBW 0.25 	//2kHz@8k Fs  carrier frequency

// number of saves for testing/debugging
#define HISTORY_SAVES 256
#define SAVE_WRAP(i) ((i)&(HISTORY_SAVES-1))

#define LE_Num	8		// number of past fde's that the estimate is made upon
//int LE_Num = 2;

//States for easy reading
#define STATE_SEARCH	0
#define STATE_RECORD	1
#define STATE_CALC		2
#define STATE_SYNC		3

//gpio registers
#define GPIO_ENABLE_ADDRESS	0x01B00000
#define GPIO_DIRECTION_ADDRESS	0x01B00004
#define GPIO_VALUE_ADDRESS	0x01B00008

#include <stdio.h>
#include <c6x.h>
#include <csl.h>
#include <csl_gpio.h>	//GPIO support
#include <csl_mcbsp.h>
#include <csl_irq.h>
#include <math.h>

#include "dsk6713.h"
#include "dsk6713_aic23.h"
#include "dsk6713_led.h"

// ------------------------------------------
// start of variables
// ------------------------------------------
//Math variables for delay estimation
float buf[M];       // search buffer
float mfc[M];		// in-phase correlation buffer
float mfs[M];       // quadrature correlation buffer
float corr_max, corr_max_s, corr_max_c; // correlation variables
float corr_c[2*M];
float corr_s[2*M];
float s[2*M];
short corr_max_lag;
short bufindex = 0;
short i,j,imax, k, ell;
float zc, zs, z, zi, zq, zmax;
double t,x,y;
float recbuf[2*N+2*M]; // recording buffer
float si[2*N+1];  // in-phase sinc pulse
float sq[2*N+1];  // quadrature sinc pulse
short recbufindex = 0;
short recbuf_start_clock = 0; // virtual clock counter for first sample in recording buffer
short max_recbuf = 0;

//state control for delay correlation segments in ISR
volatile int state = 0;

//clock counter variables
volatile short vclock_counter = 0; // virtual clock counter
volatile int sincpulsebuf_counter = 0;  // sinc pulse buffer counter

//debug max_samp
short max_samp = 0;

//delay estimates
volatile short cde = 0;			// coarse delay estimate (integer)	from [0, LL)
volatile float pcf = 0.0;		// phase correction factor			from (-1, 1)
volatile float fde = 0.0;		// fine delay estimate				cde + pcf
volatile short integerOffset = 0;

volatile float clockOffset;		//Offset wrapped to [0, L) and adjusted for waveform offset, gives true "center" from vclock=0 to masters center

//float fde_past[LE_Num];
//short vclock_past[LE_Num];
//float fde_save[HISTORY_SAVES];    		 	// save fine delay estimates for testing/debugging
volatile float clockOffset_save[HISTORY_SAVES]; 		// save clock offset estimates for testing/debugging
volatile int   clockOffset_tick_save[HISTORY_SAVES]; //saves which "tick" the latest waveform is associated with
double ppm_estimate = 0.0;
volatile short save_index = 0;						//index for going through the delay calculation estimate buffers
short buffer_swap_index = L;  // initialize to L to prevent swapping until new buffer is ready
double one_over_beta;
double adjustedclockoffset;
double fractionalShift = 0.0;

volatile short regression_index = 0;
volatile float regressionSave[HISTORY_SAVES];

// check if clock sinc gets updated every window (every 4096 cycle)
#define DEBUG_INDEX 100
//#define DEBUG_WRAP(i) ((i)&(DEBUG_INDEX-1))
short clockUpdateDebug[2][DEBUG_INDEX];
short clockUpdateDebug_index = 0;
char clockUpdateFlag = 0;

volatile int infiniteClockTick = 0;					//infinite time counter, used for counting rollovers
volatile int lastInfiniteClockTick = 0;				//used to save the last clock tick for assocation of delays with said tick

//Condition flags for ISR
char newDelayEstimationFlag = 0;  // flag to tell ISR when delay estimates have been calculated
char needNewShiftedBufferFlag = 0;


DSK6713_AIC23_CodecHandle hCodec;							// Codec handle
DSK6713_AIC23_Config config = DSK6713_AIC23_DEFAULTCONFIG;  // Codec configuration with default settings

//waveform buffers
short sincpulsebuf[LL];  // sinc pulse buffer (left channel)
short clockbuf[L];  // clock buffer (temp "virtual" right channel)
short clockbuf_shifted[2][L]; //Shifted clock buffer (actual right channel)
short current_clockbuf = 0;	  //selects which clock buffer to play

#pragma DATA_SECTION(allMyDelayedWaveforms,".mydata")
far short allMyDelayedWaveforms[FER][2*N+1];	//fractionally delayed waveform buffer
// ------------------------------------------
// end of variables
// ------------------------------------------

double sin(double);
double cos(double);
interrupt void serialPortRcvISR(void);
void setupTransmitBuffer(	short tBuffer[],
							short halfBufLen,
							double sincBandwidth,
							double carrierFreq,
							double delay);

void runSystemOffsetCalculations();
void saveClockOffsetCalcs();


double getNextClockOffsetEstimate(); //wrapper for whatever other funciton call we want to get a new estimate
float LinearRegressionExtrapolation(int clockOffsetTickTarget);//multi-point guess
double offsetExtrapolation(); 									//simple linear fit based on last two data points


void updateOutputBuffer();
void updateOutputBufferWClockOffset(float clockOffset);


void SetupBuffers();
void SystemInit();



void main()
{

	SetupBuffers();


	for(i = 0; i < FER; i++){
		setupTransmitBuffer(&(allMyDelayedWaveforms[i][0]), N, BW, CBW, (((double) i) / FER));
	}

	//zero out history buffers
	for(i = 0; i < HISTORY_SAVES; i++){
		//fde_save[i] = 0.0;
		clockOffset_save[i] = 0;
		clockOffset_tick_save[i] = 0;
		regressionSave[i] = 0.0;
	}


	for (i = 0; i < DEBUG_INDEX; ++i)
	{
		clockUpdateDebug[0][i] = 0;
		clockUpdateDebug[1][i] = 0;
	}

	// initial value of -1 means, past data not valid
	// valid fde and vclock are always positive
	/*
	for(i=0;i<LE_Num;++i)
	{
		fde_past[i]=-1;
		vclock_past[i]=-1;
	}
	 */
	SystemInit();

	// main loop
	while(1)
	{
		//Need to perform the next buffer movement and clock offset for
		//Can only run when clockbuf is not in use that is N (512) samples around the vclock overflow point
		if ((needNewShiftedBufferFlag==1) && (vclock_counter > N + 5) && (vclock_counter < L - N - 30) && state!=STATE_SEARCH)
//			(((vclock_counter + N + CLK_BUF_SFTY_MARGIN)  < (int)(clockOffset)) ||	//greater than 10 counts
//			 ((vclock_counter) > ((int)(clockOffset) + N + CLK_BUF_SFTY_MARGIN))))   //before or after the prior mid point?
		{
			int nextClockTick = infiniteClockTick + 1; //get clock tick for next update cycle
			float nextEstimatedClockOffset = LinearRegressionExtrapolation(nextClockTick);

			//debug
			regression_index++;
			if(regression_index >= HISTORY_SAVES)
				regression_index = 0;
			regressionSave[regression_index] = nextEstimatedClockOffset;
			//end debug

//			// copy appropriate fractionally shifted clock buffer to shifted clock buffer
//			one_over_beta = ((double) LL)/((double) LL - ppm_estimate/61.03515625); // approx 1+ppm_estimate/1E6
//			adjustedclockoffset = clockOffset + ((double) 2*L)*one_over_beta;  // latency for first pulse is 2*L (xxx revisit)
//
//			while (adjustedclockoffset>((double) L))
//				adjustedclockoffset = adjustedclockoffset - (double) L;
//			integerOffset = (short) adjustedclockoffset; // casting from float to short just truncates, e.g., 3.99 becomes 3
//			fractionalShift = adjustedclockoffset - (double) integerOffset; // fractionalShift >= 0 and < 1 tells us how much of a sample is left
//
//			k = (short) (fractionalShift * (double) FER);  // this also truncates and should give result on 0,...,FER-1
//
//			//Convert to 0-centered waveform fractional-delayed waveform in clockbuf
//			for (i=-N;i<=N;i++)
//			{
//				if(current_clockbuf == 0)
//					clockbuf[1][CLOCK_WRAP(i)] = (short) allMyDelayedWaveforms[k][i+N];
//				else
//					clockbuf[0][CLOCK_WRAP(i)] = (short) allMyDelayedWaveforms[k][i+N];
//			}

			updateOutputBufferWClockOffset(nextEstimatedClockOffset);
			needNewShiftedBufferFlag = 0;
			clockUpdateFlag = 0;
		}

		//Need to perform delay estimate computation from received signal?
		if 		((newDelayEstimationFlag==1) )//&& (vclock_counter > N + 5) && (vclock_counter < L - N - 5))
//				(((vclock_counter + N + CALC_SFTY_MARGIN)  < (int)(clockOffset)) ||	//greater than 10 counts
//				 ((vclock_counter) > ((int)(clockOffset) + N + CALC_SFTY_MARGIN))))   //before or after the prior mid point?){
		{
			runSystemOffsetCalculations();
			saveClockOffsetCalcs();

//			// ppm estimator (quick and dirty)
//			// actual formula is
//			// (clockoffset_save[save_index]-clockoffset_save[save_index-1])/(LL)*1E6
//			// xxx this will not work if clock offset wraps
			if (save_index>=1) {
				ppm_estimate = (clockOffset_save[save_index]-clockOffset_save[SAVE_WRAP(save_index-1)])*61.03515625; //
			}
			//ppm_estimate = 4.32133674621582; // xxx temporary

			// when can we swap buffers?
			buffer_swap_index = 1;//clockOffset+L/2;  // midpoint of the silent part (I think)

			// tell the ISR the calculations are done
			newDelayEstimationFlag = 0;
		}
	}

	/** Old working while loop code
	 	while(1)
	{
		if ((state==STATE_CALC)&&(newDelayEstimationFlag==0)){  // TIME TO COMPUTE DELAY ESTIMATES
			//---- Start of delay computation ----
			runSystemOffsetCalculations();
			saveClockOffsetCalcs();
			//---- End of Delay computation ----

			// tell the ISR the calculations are done
			newDelayEstimationFlag = 1;
			state = STATE_SYNC; // next state

			//---- Start of synchronization waveform computation ----
			updateOutputBuffer();
		}
	}
	 */
}

interrupt void serialPortRcvISR()
{
	union {Uint32 combo; short channel[2];} codecIn;
	union {Uint32 combo; short channel[2];} codecOut;
	codecOut.combo = 0; //default to no sig

	codecIn.combo = MCBSP_read(DSK6713_AIC23_DATAHANDLE);
	// Note that right channel is in codecIn.channel[0]
	// Note that left channel is in codecIn.channel[1]

	// update virtual clock (cycles from 0 to L-1)
	vclock_counter++;
	if (vclock_counter>=L) {
		vclock_counter = 0; // clock tick occurred, wrap
		infiniteClockTick++;
		needNewShiftedBufferFlag = 1; //set flag for new delay shift neccesary
		clockUpdateFlag = 1;
	}

	// update sinc pulse counter (cycles from 0 to LL-1)
	// this is for sinc pulses from the slave to the master
	// sinc pulses from the slave to the master have their peak at
	// sincpulsebuf_counter = 0 (this makes clock offset calculations simple)
	sincpulsebuf_counter++;
	if (sincpulsebuf_counter>=LL) {
		sincpulsebuf_counter = 0; // wrap
	}

	// debug clock sinc update rate
	clockUpdateDebug_index ++;
	if (clockUpdateDebug_index>=DEBUG_INDEX)
		clockUpdateDebug_index = 0;

	if (clockUpdateFlag == 0){
	clockUpdateFlag = 1;
	clockUpdateDebug[0][clockUpdateDebug_index] = 1;
	clockUpdateDebug[1][clockUpdateDebug_index] = vclock_counter;

	//clockUpdateDebug[0][DEBUG_WRAP(clockUpdateDebug_index+1)] = clockUpdateDebug[1][DEBUG_WRAP(clockUpdateDebug_index+1)] = 0;
	}
	// end of debug clock sinc
//
//	// keep track of largest sample for diagnostics
//	if (codecIn.channel[0]>max_samp)
//		max_samp = codecIn.channel[0];

	if (state==STATE_SEARCH) {
        // put sample in searching buffer
		buf[bufindex] = (float) codecIn.channel[0];  // right channel

	    // compute incoherent correlation
	    zc = 0;
	    zs = 0;
		for(i=0;i<M;i++) {
			zc += mfc[i]*buf[i];
			zs += mfs[i]*buf[i];
		}
		z = zc*zc+zs*zs;

		if (z>T1) {  				// threshold exceeded?
			max_recbuf = 0;
			state = STATE_RECORD; 				// enter "recording" state (takes effect in next interrupt)
			//DSK6713_LED_toggle(0);	// toggle LED here for diagnostics
			// record time of first sample
			recbuf_start_clock = sincpulsebuf_counter-M; // should not be negative since we started counting when we launched the S->M sinc pulse
			recbufindex = M;		// start recording new samples at position M
			j = bufindex;			//
			for (i=0;i<M;i++){  	// copy samples from buf to first M elements of recbuf
				j++;   				// the first time through, this puts us at the oldest sample
				if (j>=M)
					j=0;
				recbuf[i] = buf[j];
				buf[j] = 0;  		// clear out searching buffer to avoid false trigger
			}
		}
		else {
			// increment and wrap pointer
		    bufindex++;
		    if (bufindex>=M)
		    	bufindex = 0;
		}
	}
	else if (state==STATE_RECORD) {

		// put sample in recording buffer
		recbuf[recbufindex] = (float) codecIn.channel[0];  // right channel
		recbufindex++;
		if (recbufindex>=(2*N+2*M)) {
			state = STATE_CALC;  		// buffer is full
			//DSK6713_LED_toggle(0);	// toggle LED here for diagnostics
			recbufindex = 0; 	// shouldn't be necessary
			newDelayEstimationFlag = 1;  // set flag that we're ready for estimation
			lastInfiniteClockTick = infiniteClockTick; //save the current tick when we finish recording as the latest "update"
		}
	}
	else if (state==STATE_CALC) {
		if (sincpulsebuf_counter == 0) //we're back to the start, go back to start state anyways.
			state = STATE_SEARCH;
	}

//	// when new buffer ready and when vclock is not touching the output bug
//	if(vclock_counter>N+5 && vclock_counter<L-N-5 && needNewShiftedBufferFlag==0)
//	{
//		// swap clockbuf arrays
//		short * temp 	 = clockbuf_ready;
//		clockbuf_ready	 	 = clockbuf_shifted;
//		clockbuf_shifted = temp;
//	}

	if(buffer_swap_index==1 && state!=STATE_CALC)
	{
		buffer_swap_index = 0;
		if (current_clockbuf==0)
			current_clockbuf = 1;
		else
			current_clockbuf = 0;
	}
	//codecOut.channel[1] = clockbuf_shifted[vclock_counter];  // slave clock signal (always played) xxx
	codecOut.channel[1] = clockbuf_shifted[current_clockbuf][vclock_counter];
	codecOut.channel[0] = sincpulsebuf[sincpulsebuf_counter];  // this initiates the sinc pulse exchange with the master

	MCBSP_write(DSK6713_AIC23_DATAHANDLE, codecOut.combo); // output L/R channels


}

/**
 * Calculates a transmission waveform to be placed into a buffer.
 *
 * Note that this buffer is zeroed at idx=[halfBufLen], and the total size is 2*halfBufLen + 1
 *
 * @param tBuffer		The buffer to put the waveform into
 * @param halfBufLen	The size of one half of the buffer (e.g. -N, to 0, to N)
 * @param sincBandwidth The bandwidth of the sinc pulse (e.g. 0.0125 BW)
 * @param carrierFreq	The frequency to modulate at 	(e.g. 0.25 CBW)
 * @param delay			the fractional delay in samples to allow for sync (e.g. 0.5 is one half sample delay)
 */
void setupTransmitBuffer(short tBuffer[], short halfBufLen, double sincBandwidth, double carrierFreq, double delay){

	//i is index per element, y is temp for output
	int idx;
	double x, y, t;
	double cosine, sine, denom;

	for (idx=-halfBufLen;idx<=halfBufLen;idx++){
		x = ((double)idx - delay) * sincBandwidth;
		t = ((double)idx - delay) * carrierFreq;
		if (x == 0.00000) //floating point check if delay is too close to 1 or 0 to keep division by zero from occurring
			y = 32767.0;
		else {
			cosine = cos(2*PI*t);
			sine = sin(PI*x);
			denom = (PI*x);
			y = (cosine * sine * 32767.0 / denom);
		}

		tBuffer[idx+halfBufLen] = (short)y;

	}
}

/**
 * sets up all buffers initially for use in calculation processing.
 */
void SetupBuffers(){
	// set up the cosine and sin matched filters for searching
	// also initialize searching buffer
	for (i=0;i<M;i++){
		t = i*0.25;				// time
		y = cos(2*PI*t);		// cosine matched filter (double)
		mfc[i] = (float) y;		// cast and store
		y = sin(2*PI*t);		// sine matched filter (double)
		mfs[i] = (float) y;     // cast and store
		buf[i] = 0;             // clear searching buffer
	}

	// initialize clock buffer
	for (i=0;i<L;i++)
		clockbuf[i] = clockbuf[i] = 0;

	// initialize sinc pulse buffer
	for (i=0;i<LL;i++)
		sincpulsebuf[i] = 0;

	// set up clock buffer and sinc pulse buffer
	// to play modulated sinc centered at zero
	for (i=-N;i<=N;i++){
		x = i*BW;
		if (i!=0) {
			t = i*0.25;
			y = 32767.0*cos(2*PI*t)*sin(PI*x)/(PI*x); // double
		}
		else {
			y = 32767.0;
		}

		j = i;
		if (j<0) {
			j += L; // wrap
		}
		clockbuf[j] = (short) y;
		j = i;
		if (j<0) {
			j += LL; // wrap
		}
		sincpulsebuf[j] = (short) y;
	}

	// set up inphase and quadrature sinc pulses for coarse and fine delay estimators
	j = 0;
	for (i=-N;i<=N;i++){
		x = i*BW;
		if (i!=0) {
			t = i*0.25;
			si[j] = (float) (cos(2*PI*t)*sin(PI*x)/(PI*x));
			sq[j] = (float) (sin(2*PI*t)*sin(PI*x)/(PI*x));
		}
		else {
			si[j] = 1.0;
			sq[j] = 0.0;
		}
		j++;
	}

}

void SystemInit(){
	DSK6713_init();		// Initialize the board support library, must be called first
	DSK6713_LED_init(); // initialize LEDs
    hCodec = DSK6713_AIC23_openCodec(0, &config);	// open codec and get handle

    //gpioInit();

	// Configure buffered serial ports for 32 bit operation
	// This allows transfer of both right and left channels in one read/write
	MCBSP_FSETS(SPCR1, RINTM, FRM);
	MCBSP_FSETS(SPCR1, XINTM, FRM);
	MCBSP_FSETS(RCR1, RWDLEN1, 32BIT);
	MCBSP_FSETS(XCR1, XWDLEN1, 32BIT);

	// set codec sampling frequency
	DSK6713_AIC23_setFreq(hCodec, DSK6713_AIC23_FREQ_8KHZ);

	// interrupt setup
	IRQ_globalDisable();			// Globally disables interrupts
	IRQ_nmiEnable();				// Enables the NMI interrupt
	IRQ_map(IRQ_EVT_RINT1,15);		// Maps an event to a physical interrupt
	IRQ_enable(IRQ_EVT_RINT1);		// Enables the event
	IRQ_globalEnable();				// Globally enables interrupts
}

/**
 * Runs the calculations that give the sample time of the masters clock response to our current zero timer for the sinc pulse output
 */
void runSystemOffsetCalculations(){
	// compute coarse delay estimate
	zmax = 0.0;				// maximum
	imax = 0;				// index of maximum
	for (i=0;i<(2*M-1);i++){  // lag index
		z = 0;
		for (j=0;j<(2*N+1);j++) {
			z+= si[j]*recbuf[i+j];  // correlation at lag i
		}
		if (abs(z)>zmax) {
			zmax = abs(z);  // store maximum
			imax = i;       // store index of maximum
		}
	}
	// coarse delay estimate (DRB: +N here because si is already shifted by N)
	cde = recbuf_start_clock + imax +N;  // coarse delay estimate
	// cde  is the number of samples elapsed since we launched the S->M sinc pulse

    // compute fine delay estimate
	zi = 0.0;  // in phase
	zq = 0.0;  // quadrature
	for (j=0;j<(2*N+1);j++) {
		zi+= si[j]*recbuf[imax+j];  // correlation at lag imax
		zq+= sq[j]*recbuf[imax+j];  // correlation at lag imax
	}
	if(isnan(zi) || isnan(zq))
		pcf = 0.0; //default to a working value
	else
		pcf = atan2(zq,zi)*(2*INVPI); // assumes wc = pi/2
	fde = (float) cde + pcf;

}

/**
 * Saves the current clock offset estimate after a delay estimation to the delay buffer.
 * This delay buffer will be used by the offset estimator code later for algorithmic regression analysis
 * to guess a new offset at time T
 */
void saveClockOffsetCalcs(){

	clockOffset = (fde + 1) * 0.5;
	while (clockOffset>((float) L))
		clockOffset = clockOffset - (float) L;

	// testing/debugging
	save_index++;
	if (save_index>=HISTORY_SAVES)  // wrap
		save_index = 0;
	//fde_save[save_index] = fde;
	clockOffset_save[save_index] = clockOffset;
	clockOffset_tick_save[save_index] = lastInfiniteClockTick; //saves the "clock tick" number when we found the signal

}

//void updateOutputBuffer(){
//	//Get integer part of delay for clock counter
//	int integerOffset = (int) clockOffset;
//
//	//get decimal portion
//	float fractionalOffset = clockOffset - (float) integerOffset;
//
//	//convert decimal portion to delay buffer index
//	int integralFractionalOffset = ((int)(fractionalOffset * FER)) % FER; //scales [0, 1) to [0, FRE - 1]
//
//	// need to wait until the vclock_counter does not point to a sinc signal
//	// so that the output sinc clock does not appear discontinuous
//	// wait until we reach the end + a few counts of the current output sync level
//	//while(vclock_counter != CLOCK_WRAP(integerOffset + N + 5));
//
//
////	//rezero all of clockbuf
////	for(i=0; i<L; i++){
////		clockbuf[i] = 0;
////	}
//	//Do initial copy from fractional delayed waveform buffer now into the zero centered clockbuf thingy
//	for (i=-N;i<=N;i++)
//	{
//		j = i;
//		if (j < 0) {
//			j += L; // wrap
//		}
//
//		//clockbuf[j] = (short) allMyDelayedWaveforms[integralFractionalOffset][i+N];
//		clockbuf[j] = (short) allMyDelayedWaveforms[integralFractionalOffset][i+N];
//	}
//
//	//Rewrite the current sinc buffer now
//	for (i=0;i<L;i++) {
//		//j = (short) clockoffset; // casting from float to short just truncates, e.g., 3.99 becomes 3
//		//j = (short) (clockoffset+0.5); // this rounds as long as clockoffset is positive, which it should be
//	  j = i+integerOffset;
//	  clockbuf_shifted[CLOCK_WRAP(j)] = clockbuf[i];
//	}
//
//
//}

/**
 * same as updateOutputBuffer code, but with a general given "clockOffset" argument
 * @param clockOffset the floating point offset from the local vclock=0 to use, instead of a global variable
 */
void updateOutputBufferWClockOffset(float clockOffsetToUse){
	//Get integer part of delay for clock counter
	float temp;
	if(isnan(clockOffsetToUse))
		temp = 0.0;
	else
		temp = clockOffsetToUse;
	integerOffset = (short) temp;
	//get decimal portion
	float fractionalOffset = clockOffsetToUse - (float) integerOffset;
	//convert decimal portion to delay buffer index
	int integralFractionalOffset = ((int)(fractionalOffset * FER)) % FER; //scales [0, 1) to [0, FER - 1]

	for (i=0;i<L;i++) {
		if (current_clockbuf==0)
			clockbuf_shifted[1][i] = 0;
		else
			clockbuf_shifted[0][i] = 0;
	}

//	//Convert to 0-centered waveform fractional-delayed waveform in clockbuf
//	for (i=-N;i<=N;i++)
//	{
//		if(current_clockbuf == 0)
//			clockbuf[1][CLOCK_WRAP(i)] = (short) allMyDelayedWaveforms[integralFractionalOffset][i+N];
//		else
//			clockbuf[0][CLOCK_WRAP(i)] = (short) allMyDelayedWaveforms[integralFractionalOffset][i+N];
//	}

	for (i=-N;i<=N;i++) {
		ell = integerOffset+i;
//		if (ell>=L) {
//			ell = ell-L;
//		}
		if (current_clockbuf==0) {
			clockbuf_shifted[1][CLOCK_WRAP(ell)] = (short) allMyDelayedWaveforms[integralFractionalOffset][i+N];  // write other buffer
		}
		else {
			clockbuf_shifted[0][CLOCK_WRAP(ell)] = (short) allMyDelayedWaveforms[integralFractionalOffset][i+N];  // write other buffer
		}
	}

//	//Convert to 0-centered waveform fractional-delayed waveform in clockbuf
//	for (i=-N;i<=N;i++)
//	{
//		clockbuf[CLOCK_WRAP(i)] = (short) allMyDelayedWaveforms[integralFractionalOffset][i+N];
//	}
//
//	//Rewrite the current sinc buffer now centered at the integer offset
//	for (i=0;i<L;i++) {
//		clockbuf_shifted[CLOCK_WRAP(i+integerOffset)] = clockbuf[i];
//	}
}

///**
// * Looks at the clock offset history, and the current vlock tick counter, and guesses what the current offset is
// * @return an estimated clock offset value to be used for the clock tick
// */
//
//double offsetExtrapolation(){
//
//	double extrapolatedValue;
//	int nextClockTick = infiniteClockTick + 1; //get clock tick for next update cycle
//
//	//most recent saved tick value
//	double 	lastClockOffset = clockOffset_save[save_index];
//	int 	lastClockOffsetTick = clockOffset_tick_save[save_index];
//
//	//prior saved tick value to extrapolate a line from
//	double 	lastLastClockOffset = clockOffset_save[SAVE_WRAP(save_index - 1)];
//	int 	lastLastClockOffsetTick = clockOffset_tick_save[SAVE_WRAP(save_index - 1)];
//
//	//repeat here if we need more points to reduce noise?
//
//
//
//	//Extrapolate a simple line based on two points
//	double slope = 	(lastClockOffset - lastLastClockOffset) /
//					(lastClockOffsetTick - lastLastClockOffsetTick);	//m
//
//	int tickDelta = nextClockTick - lastClockOffsetTick;				//delta-x
//
//	double offset = lastClockOffset;
//
//	extrapolatedValue = offset + (double) tickDelta * slope; 	//simple linear extrapolated value
//
//	return extrapolatedValue;
//
//}


//double LinearRegression(){
//	//				// unwrap points
//	//
//	//				// TODO
//	//				char slope[LE_Num]=0, prev_slope=0, wrap[LE_Num]=0;// assume both positive and negative slope
//	//
//	//				// see where the slope changes, it normally should change only once (with LE_Num as small as 4)
//	//				for (i=0;i<LE_Num-1;++i)
//	//				{
//	//					if(fde_past[i]<fde_past[i+1])
//	//						slope[i] = 1;
//	//					else
//	//						slope[i] = 0;
//	//				}
//	//				// wrap affects the cde as well
//
//					// function: fde= a + b*vclock (y = a + b*x)
//					//							    		n*sum(xy)-sum(x)*sum(y)
//					// b(slope) using linear regression = --------------------------
//					//										n*sum(x^2)-(sum(x))^2
//					// n = LE_Num
//					float xy=0, x=0, y=0, x2=0, xx2=0, b=0, fde_estimate=0;
//					for (i=0;i<LE_Num;++i)
//					{
//						xy += fde_past[i]*vclock_past[i];
//						y  += fde_past[i];
//						x  += vclock_past[i];
//						x2 += x*x;
//					}
//					xx2 = x*x;
//
//					b = (LE_Num*xy - x*y)/(LE_Num*x2 - xx2);
//					a = (y -b*x)/LE_Num;
//
//					//fde_estimate = ( fde_past[PAST_WRAP(past-1)] - b*vclock_past[PAST_WRAP(past-1)] )	+	b*vclock_counter;
//					fde_estimate = a + b*vclock_counter;


//					return fde_estimate;
//
////					clockOffset = (fde_estimate+1) * 0.5;
////					//clockOffset = fde * 0.5; //SHOULD BE LIKE THIS!
////					while (clockOffset>((float) L))
////						clockOffset = clockOffset - (float) L;
////
////					updateOutputBuffer();
////					estimateIndependentState = 0;
////					overflowCount = 0;
//}



/**
 *
 * @return a linear regression analysis extrapolated value
 */
float LinearRegressionExtrapolation(int clockOffsetTickTarget){

	float answer;

	float 	localClockOffsetSave[LE_Num];
	int 	localClockOffsetTickSave[LE_Num];
	for(i=0; i < LE_Num; i++){
		localClockOffsetSave[i] = clockOffset_save[SAVE_WRAP(save_index - (LE_Num-1) + i)];			//y
		localClockOffsetTickSave[i] = clockOffset_tick_save[SAVE_WRAP(save_index - (LE_Num-1) + i)]; 	//x
	}

	float xmean=0.0, ymean=0.0, sx=0.0, sy=0.0, sxy=0.0, a=0.0, b=0.0;

	//do means
	for(i=0; i<LE_Num; i++){
		xmean += localClockOffsetTickSave[i];
		ymean += localClockOffsetSave[i];
	}
	xmean /= (float)LE_Num;
	ymean /= (float)LE_Num;

	for(i=0; i<LE_Num; i++){
		sx += 	(localClockOffsetTickSave[i] - xmean)*(localClockOffsetTickSave[i] - xmean);	//x-error^2
		//sy += 	(localClockOffsetSave[i] - ymean)*(localClockOffsetSave[i] - ymean);			//y-error^2
		sxy +=	(localClockOffsetTickSave[i] - xmean)*(localClockOffsetSave[i] - ymean);		//x-error*y-error
	}
	if(sx != 0){
		b = sxy / sx;			//slope
		a =	ymean - b*xmean;	//y-intercept
	}
	else {
		b = 0.0;
		a = ymean;
	}

//	//--------------------------------------------------------------
//	// function: fde= a + b*vclock (y = a + b*x)
//	//							    		n*sum(xy)-sum(x)*sum(y)
//	// b(slope) using linear regression = --------------------------
//	//										n*sum(x^2)-(sum(x))^2
//	// n = LE_Num
//	float xy=0.0, x=0.0, y=0.0, x2=0.0, xx2=0.0, b=0.0, a=0.0;
//	for (i=0;i<LE_Num;++i)
//	{
//		xy += localClockOffsetSave[i]*localClockOffsetTickSave[i];
//		y  += localClockOffsetSave[i];
//		x  += localClockOffsetTickSave[i];
//		x2 += x*x;
//	}
//	xx2 = x*x;
//
//	b = (((float)LE_Num)*xy - x*y)/(((float)LE_Num)*x2 - xx2);
//	a = (y - b*x)/(float)LE_Num;
//	//--------------------------------------------------------------

	answer = (float) (		a + b*((float)clockOffsetTickTarget)	);

	//sanity check on the output here
	while (answer > (float)L)
		answer -= (float) L;
	while (answer < 0.0)
		answer += (float) L;
	if(isnan(answer))
		while(1); //lock up here for debug

	return answer;
}


////provides the latest clock offset estimate
///**
// * Wrapper for a call to a clock offset estimation function
// * @return next "clock offset" value
// */
//double getNextClockOffsetEstimate(int arg){
//	//return offsetExtrapolation(); //two point extrapolate
//	return LinearRegressionExtrapolation(arg);
//}

