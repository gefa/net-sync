/*************************************************************************
 *  Basic stereo loop code for C6713 DSK and AIC23 codec
 *  D. Richard Brown on 22-Aug-2011
 *  Based on code from "Real-Time Digital Signal Processing Based on TMS320C6000"
 *  by N. Kehtarnavaz and N. Kim.
 *
 *  Timestamping code
 *    - three states:
 *      - state 0: searching
 *      - state 1: recording
 *      - state 2: timestamp calculation
 *		- state 3: sinc response
 *    - looks for modulated sinc pulse
 *    - correlates to find nearest sample
 *    - uses carrier phase to refine timestamp
 *
 *	Current Revision: 	0.1
 *	Revision Name:		Hurr durr I'ma sheep
 *************************************************************************/


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

// ------------------------------------------
// start of variables
// ------------------------------------------

volatile int vclock_counter = 0; // virtual clock counter

//Output waveform buffers for clock and sync channels
short tClockSincPulse[N2];
short tVerifSincPulse[N2];

volatile int virClockTransmitCenterSinc = 0;			//center for sinc pulse according to vclock_counter
volatile int virClockTransmitCenterVerify = 0;		//center for the verification pulse on the second channel

volatile short virClockTransmitCenterSincIndex = 0;		//index for reading the synchronizing output buffer
volatile short virClockTransmitCenterVerifyIndex = 0;	//index for reading the output buffer

//Check whether the delay by N could cause the half sample,
//As the full buffer is 2N+1, instead of just 2N



int coarse_delay_estimate[MAX_STORED_DELAYS_COARSE];
float fine_delay_estimate[MAX_STORED_DELAYS_FINE];
short cde_index = 0;
short fde_index = 0;

//State Variables (transmit/receive/calculation)
#if (NODE_TYPE == MASTER_NODE)//if master, listen to slave first and then send the sinc back
extern volatile int state = STATE_SEARCHING;
#elif (NODE_TYPE == SLAVE_NODE)//if slave, send sinc and then wait for master's response
extern volatile int state = STATE_TRANSMIT;
#endif

// ISR combos
union {uint32_t combo; short channel[2];} tempOutput;
union {uint32_t combo; short channel[2];} tempInput;

// Handles
DSK6713_AIC23_CodecHandle hCodec;							// Codec handle
DSK6713_AIC23_Config config = DSK6713_AIC23_DEFAULTCONFIG;  // Codec configuration with default settings


// ------------------------------------------
// end of variables
// ------------------------------------------


interrupt void serialPortRcvISR(void);

void main()
{

	// reset coarse and fine delay estimate buffers
	for (i=0;i<MAX_STORED_DELAYS_COARSE;i++)
		coarse_delay_estimate[i] = 0;
		fine_delay_estimate[i] = 0.0;

	// set up the cosine and sin matched filters for searching
	// also initialize searching buffer
	SetupReceiveTrigonometricMatchedFilters();
	SetupReceiveBasebandSincPulseBuffer();
	SetupTransmitModulatedSincPulseBuffer();
	// -------- DSK Hardware Setup --------

	DSK6713_init();		// Initialize the board support library, must be called first
	DSK6713_LED_init(); // initialize LEDs
    hCodec = DSK6713_AIC23_openCodec(0, &config);	// open codec and get handle

	// Configure buffered serial ports for 32 bit operation
	// This allows transfer of both right and left channels in one read/write
	MCBSP_FSETS(SPCR1, RINTM, FRM);
	MCBSP_FSETS(SPCR1, XINTM, FRM);
	MCBSP_FSETS(RCR1, RWDLEN1, 32BIT);
	MCBSP_FSETS(XCR1, XWDLEN1, 32BIT);

	// set codec sampling frequency
	DSK6713_AIC23_setFreq(hCodec, DSK_SAMPLE_FREQ);

	//Example taken from TI forums
	//Setup GPIO
	gpioInit();

	//NOTE inf loop
	//gpioToggle();

	// interrupt setup
	IRQ_globalDisable();			// Globally disables interrupts
	IRQ_nmiEnable();				// Enables the NMI interrupt
	IRQ_map(IRQ_EVT_RINT1,15);		// Maps an event to a physical interrupt
	IRQ_enable(IRQ_EVT_RINT1);		// Enables the event
	IRQ_globalEnable();				// Globally enables interrupts

	// -------- End DSK Hardware Setup --------

	while(1)						// main loop
	{
		if (state!=STATE_CALCULATION) {
			//Do nothing
			//Maybe calculate the question to 42 if we have time
		}
		else if (state==STATE_CALCULATION){

			// -----------------------------------------------
			// this is where we estimate the time of arrival
			// -----------------------------------------------
			runReceivedPulseBufferDownmixing();
			runReceviedSincPulseTimingAnalysis();

			//Now we calculate the new center clock for verification or response sinc
			#if USE_FDE
					#if (NODE_TYPE==MASTER_NODE)
						calculateNewResponseTimeMaster(vclock_counter, coarse_delay_estimate[cde_index]);
					#elif (NODE_TYPE==SLAVE_NODE)
						calculateNewSynchronizationTimeSlave(vclock_counter, coarse_delay_estimate[cde_index]);
					#endif
			#else
				#if (NODE_TYPE==MASTER_NODE)
					calculateNewResponseTimeMaster(vclock_counter, coarse_delay_estimate[cde_index]);
				#elif (NODE_TYPE==SLAVE_NODE)
					calculateNewSynchronizationTimeSlave(vclock_counter, coarse_delay_estimate[cde_index]);
				#endif
			#endif
				/**
				 * \todo change the above to support A. floating point ability when FDE is turned on
				 * 									 B. actually assign the result to the new center points
				 */


			//Enter transmission state after reception
			state = STATE_TRANSMIT;
			while(state == STATE_TRANSMIT){
				//wait for ISR to timeout
			}

		}
	}
}


/**
 * @brief ISR for audio codec sample transceive
 *
 * Handles logic for taking in a sample from the audio codec, running processing, and then outputting
 * an appropriate sample straight back into the codec at the same time.
 */
interrupt void serialPortRcvISR()
{
	tempInput.combo = MCBSP_read(DSK6713_AIC23_DATAHANDLE);
	tempOutput.combo = 0; //Set to zero now for missed sets.
	// Note that right channel is in temp.channel[0]
	// Note that left channel is in temp.channel[1]

	//Clock counter wrap
	vclock_counter++; //Note! --- Not sure of the effects of moving the increment to the top
	//Code should be common to both
	if(vclock_counter >= LARGE_VCLK_MAX){
		vclock_counter = 0;
		ToggleDebugGPIO(0);
	}


	//Logic for different states is the same on both master and slave sides when expecting to receive
	if (state==		STATE_SEARCHING) {
		runSearchingStateCodeISR(); //Possibly timeout to transmit as slave
	}
	else if (state==STATE_RECORDING) {
		runRecordingStateCodeISR();
	}
	else if(state==	STATE_CALCULATION){
		runCalculationStateCodeISR();
	}
	else if(state==	STATE_TRANSMIT){
		runSincPulseTransmitISR();	//this function will change system state when it's done
	}
	else {
		//ERROR in STATE CODE LOGIC
	}
	//Always Outputs synchronized pulse on aux channel
		runVerifyPulseTransmitISR();
		

	//Write the output sample to the audio codec
	MCBSP_write(DSK6713_AIC23_DATAHANDLE, tempOutput.combo);

}

/**
	Sets up the transmit buffer for the sinc pulse modulated at quarter sampling frequency
*/
void SetupTransmitModulatedSincPulseBuffer(){

	for (i=-N;i<=N;i++){
		x = i*BW;
		t = i*0.25;
		if (i!=0)
			y = cos(2*PI*t)*(sin(PI*x)/(PI*x)); // modulated sinc pulse at carrier freq = fs/4
		else
			y = 1;								//x = 0 case.
		tClockSincPulse[i+N] = y*32767;			//Both of these  may be modified in the future calculations anywas
		tVerifSincPulse[i+N] = y*32767;			//If FDE mode is on in order to do partial synchronization
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
		t = i*0.25;				// time
		y = cos(2*PI*t);		// cosine matched filter (double)
		matchedFilterCosine[i] = (float) y;		// cast and store
		y = sin(2*PI*t);		// sine matched filter (double)
		matchedFilterSine[i] = (float) y;     // cast and store
		buf[i] = 0;             // clear searching buffer
	}
}


/**
 * Handles the searching for peak function of the incoming waveform
 */
void runSearchingStateCodeISR(){

		// put sample in searching buffer
	buf[bufindex] = (float) tempInput.channel[RECEIVE_SINC];  // right channel

	// increment and wrap pointer
	bufindex++;
	if (bufindex>=M)
		bufindex = 0;

	// compute incoherent correlation
	corrSumCosine = 0;
	corrSumSine = 0;
	for(i=0;i<M;i++) {
		corrSumCosine+= matchedFilterCosine[i]*buf[i];
		corrSumSine+= matchedFilterSine[i]*buf[i];
	}
	corrSumIncoherent = corrSumCosine*corrSumCosine+corrSumSine*corrSumSine;

	if (corrSumIncoherent>T1) {  // should make sure this runs in real-time
		state = STATE_RECORDING; // enter "recording" state (takes effect in next interrupt)
		recbuf_start_clock = vclock_counter - M; // virtual clock tick at at start of recording buffer
												 // (might be negative but doesn't matter)
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
}

/**
 * Saves the incoming waveform data to the receive buffer for later processing in the main loop
 */
void runRecordingStateCodeISR(){
	// put sample in recording buffer
	recbuf[recbufindex] = (float) tempInput.channel[RECEIVE_SINC];  // right channel
	recbufindex++;
	if (recbufindex>=(2*N+2*M)) {
		state = STATE_CALCULATION;  // buffer is full (stop recording)
		recbufindex = 0; // shouldn't be necessary
	}
}

/**
 * Placeholder to do nothing. Calculations are supposed to be ongoing in the main loop
 */
void runCalculationStateCodeISR(){
	//Do nothing here
}

/**
 * 	Transmits the synchronizing sinc pulse on the main channel to the other node.
 * 	The value for the center point should be properly calculated based on node type from the other functions
 * 	Is always centered at 0 for the slave, and the mirrored response for the master
 */
void runSincPulseTransmitISR(){
	short idxTemp = GetSincPulseIndex(vclock_counter, virClockTransmitCenterSinc);
	if(idxTemp != -1){
		tempOutput.channel[TRANSMIT_SINC] = tClockSincPulse[idxTemp];
	}
	else{
		tempOutput.channel[TRANSMIT_SINC] = 0;
	}
	if(idxTemp == N2) //We have reached the end! go to receive
		state=STATE_SEARCHING;
}

/**
 *	Adds the second sinc pulse which acts as the verification point. Is centered at 0 always for the master,
 *	and the new observed synchronized time pulse for the slave
 */
void runVerifyPulseTransmitISR(){
	short idxTemp = GetVerifPulseIndex(vclock_counter, SMALL_VCLK_WRAP(virClockTransmitCenterVerify));
	if(idxTemp != -1){
		tempOutput.channel[TRANSMIT_CLOCK] = tVerifSincPulse[idxTemp];
	}
	else{
		tempOutput.channel[TRANSMIT_CLOCK] = 0;
	}
}








