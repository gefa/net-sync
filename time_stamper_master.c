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
		- state 3: sinc response
 *    - looks for modulated sinc pulse
 *    - correlates to find nearest sample
 *    - uses carrier phase to refine timestamp
 *************************************************************************/

#define CHIP_6713 1

// length of searching window in samples
#define M 40

// threshold value for searching window
#define T1 100000

// sinc pulse normalized bandwidth
#define BW 0.0125

// 2*N+1 is the number of samples in the sinc function
#define N 500
#define N2 1000

// virtual clock counter
#define L 4000

#define CLOCK_WRAP(i) ((i) & (L - 1)) // index wrapping macro

// max lag for computing correlations
#define MAXLAG 200

// number of coarse delays to store
#define LL 50

//Response buffer size in samples
#define OUTPUT_BUF_SIZE (2*N+1)

// State definitions
#define STATE_SEARCHING 0
#define STATE_RECORDING 1
#define STATE_CALCULATION 2
#define STATE_RESPONSE 3

// define PI and INVPI
#define PI 3.14159265358979323846
#define INVPI 0.318309886183791

#include <stdio.h>
#include <c6x.h>
#include <csl.h>
#include <csl_mcbsp.h>
#include <csl_irq.h>
#include <math.h>

#include "dsk6713.h"
#include "dsk6713_aic23.h"
#include "dsk6713_led.h"

// ------------------------------------------
// start of variables
// ------------------------------------------
float buf[M];       // search buffer
float mfc[M];		// in-phase correlation buffer
float mfs[M];       // quadrature correlation buffer
float corr_max, corr_max_s, corr_max_c; // correlation variables
float corr_c[2*M];
float corr_s[2*M];
float s[2*M];
short corr_max_lag;
short bufindex = 0;
float zc,zs,z;
short i,j;
double t,x,y;
float bbsinc[2*N+1];   // baseband sinc pulse buffer
float recbuf[2*N+2*M]; // recording buffer
float yc[2*N+2*M];     // in-phase downmixed buffer
float ys[2*N+2*M];     // quadrature downmixed buffer
short recbufindex = 0;
int state = STATE_SEARCHING;
short vclock_counter = 0; // virtual clock counter
short vir_clock_history[LL];
short recbuf_start_clock = 0; // virtual clock counter for first sample in recording buffer
short coarse_delay_estimate[LL];
float fine_delay_estimate[LL];
short cde_index = 0;
char local_carrier_phase = 0;
char r = 0;
double phase_correction_factor;
short max_samp = 0;

short vir_clock_start;
short halfSinc;
short outputBuf[OUTPUT_BUF_SIZE];
volatile short response_done = 0; //not done yet
volatile short response_buf_idx = 0; //index for output buffer
volatile short response_buf_idx_max = OUTPUT_BUF_SIZE;
volatile short amSending = 0;
volatile short HighSample = 0;

DSK6713_AIC23_CodecHandle hCodec;							// Codec handle
DSK6713_AIC23_Config config = DSK6713_AIC23_DEFAULTCONFIG;  // Codec configuration with default settings
// ------------------------------------------
// end of variables
// ------------------------------------------

double sin(double);
double cos(double);
double atan2(double,double);

interrupt void serialPortRcvISR(void);

void main()
{

	// initialize coarse delay estimate buffer
	for (i=0;i<LL;i++)
		coarse_delay_estimate[i] = 0;

	for (i=0; i<LL;++i)
		vir_clock_history[i]=0;

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

	// set up baseband sinc pulse buffer
	for (i=-N;i<=N;i++){
		x = i*BW;
		if (i!=0)
			y = sin(PI*x)/(PI*x); // double
		else
			y = 1.0;
		bbsinc[i+N] = (float) y;
	}

//	// set up baseband sinc pulse output buffer //Will cheat and use bbsinc to keep em the same
//		for (i=-N;i<=N;i++){
//		x = i*BW;
//		if (i!=0)
//			y = sin(PI*x)/(PI*x); // double
//		else
//			y = 1.0;
//		outputBuf[i+N] = (float) y*32767;
//
//	}

	// set up modulated sinc pulse buffer
	for (i=-N;i<=N;i++){
		x = i*BW;
		t = i*0.25;
		if (i!=0)
			y = cos(2*PI*t)*(sin(PI*x)/(PI*x)); // modulated sinc pulse at carrier freq = fs/4
		else
			y = 1;
		outputBuf[i+N] = y*32767;
	}

	halfSinc = 2*N;

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
	DSK6713_AIC23_setFreq(hCodec, DSK6713_AIC23_FREQ_8KHZ);

	// interrupt setup
	IRQ_globalDisable();			// Globally disables interrupts
	IRQ_nmiEnable();				// Enables the NMI interrupt
	IRQ_map(IRQ_EVT_RINT1,15);		// Maps an event to a physical interrupt
	IRQ_enable(IRQ_EVT_RINT1);		// Enables the event
	IRQ_globalEnable();				// Globally enables interrupts

	while(1)						// main loop
	{
		if (state==STATE_CALCULATION) {

			//printf wrecks the real-time operation
			//printf("Buffer recorded: %d %f.\n",recbuf_start_clock,z);

			z = 0;  // clear correlation sum

			// -----------------------------------------------
			// this is where we estimate the time of arrival
			// -----------------------------------------------

			// downmix (had problems using sin/cos here so used a trick)
			for (i=0;i<(2*N+2*M);i+=4){
				yc[i] = recbuf[i];
				ys[i] = 0;
			}
			for (i=1;i<(2*N+2*M);i+=4){
				yc[i] = 0;
				ys[i] = recbuf[i];
			}
			for (i=2;i<(2*N+2*M);i+=4){
				yc[i] = -recbuf[i];
				ys[i] = 0;
			}
			for (i=3;i<(2*N+2*M);i+=4){
				yc[i] = 0;
				ys[i] = -recbuf[i];
			}

			// this is where we apply the matched filter
			// we only do this over a limited range
			for (i=0;i<=(2*M-1);i++) {
				corr_c[i] = 0;
				corr_s[i] = 0;
				for (j=0;j<(2*N+1);j++) {
					corr_c[i] += bbsinc[j]*yc[j+i];
					corr_s[i] += bbsinc[j]*ys[j+i];
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

			//printf wrecks the real-time operation
			//printf("Max lag: %d\n",corr_max_lag);
			//printf("Coarse delay estimate: %d.\n",recbuf_start_clock+corr_max_lag);

			// store coarse delay estimates
			coarse_delay_estimate[cde_index] = recbuf_start_clock+corr_max_lag;

			// fine delay estimate
			y = (double) corr_max_s;
			x = (double) corr_max_c;
			phase_correction_factor = atan2(y,x)*2*INVPI; // phase
			r = (recbuf_start_clock+corr_max_lag) & 3; // compute remainder
			if (r==0)
				fine_delay_estimate[cde_index] = recbuf_start_clock+corr_max_lag+phase_correction_factor;
			else if (r==1)
				fine_delay_estimate[cde_index] = recbuf_start_clock+corr_max_lag+phase_correction_factor-1;
			else if (r==2) {
				if (phase_correction_factor>0)
					fine_delay_estimate[cde_index] = recbuf_start_clock+corr_max_lag+phase_correction_factor-2;
				else
					fine_delay_estimate[cde_index] = recbuf_start_clock+corr_max_lag+phase_correction_factor+2;
			}
			else if (r==3)
				fine_delay_estimate[cde_index] = recbuf_start_clock+corr_max_lag+phase_correction_factor+1;
			else
				printf("ERROR");


			//state = STATE_SEARCHING; // back to searching state
			//New code! Setup for response, doublecheck math at some point, I think a sample or two may be off in timing
			response_done = 0; //not done yet
			response_buf_idx = 0; //index for output buffer

			//Wrap start timer around virtual clock origin.
			vir_clock_start = (L - CLOCK_WRAP(coarse_delay_estimate[cde_index]) - N2); //start time  //cde_index-1 -> take most recent estimate
																				//2N because course estimate points to beginning of sinc not the peak (check this?)
			//vir_clock_start = CLOCK_WRAP(coarse_delay_estimate[cde_index]);//for debugg

			//vir_clock_start += 192*vir_clock_start*0.000125;//outgoing sinc has 4096 virtual clock, incomming sinc has 8000 vir.clock
															// they are 96*2=192 samples off, accumulated error is therefore:
															// 192*virtual_clock_start/8000

			vir_clock_history[cde_index]=vclock_counter;																				   //without -1 at start, estimate is zero
		    short CurTime = vclock_counter;

		    if (vir_clock_start < 0)
				{
				vir_clock_start = CLOCK_WRAP( vir_clock_start );
				}
			/*
			if (vir_clock_start < 0)
				{
				vir_clock_start = CLOCK_WRAP( vir_clock_start );
				while(vclock_counter != 0);		//wait for virtual clock tick
				while(vclock_counter != 0);		//wait for virtual clock tick
				//while(vclock_counter != 0);		//wait for virtual clock tick

				//Debugging variable, doesn't do anything
				if(bug>=5)
					bug=0;
				bug++;

				}
			*/

			if(CurTime > vir_clock_start){
				while(vclock_counter != 0); //wait one additional tick because we've already passed previous starting point we need
			}
			while(vclock_counter != 0);		//wait for virtual clock tick

			state = STATE_RESPONSE; //set to response for the ISR to pick the appropriate path

			while(!response_done){} //Loop and wait here until the responding output code works
				state = STATE_SEARCHING; // back to searching state, after responding

			// increment coarse delay estimate index (same index for fine delay estimates)
			cde_index++;
			if (cde_index>=LL)
				cde_index = 0;
		}

	}
}

interrupt void serialPortRcvISR()
{
	union {Uint32 combo; short channel[2];} temp;
	union {Uint32 combo; short channel[2];} outputData;

	temp.combo = MCBSP_read(DSK6713_AIC23_DATAHANDLE);
	// Note that right channel is in temp.channel[0]
	// Note that left channel is in temp.channel[1]

	// keep track of largest sample for diagnostics
	if (temp.channel[0]>max_samp)
		max_samp = temp.channel[0];

	if(state==STATE_CALCULATION){ //Do nothing, since it originally did nothing. this loop is to ensure the appropriate output buffer is written.
		local_carrier_phase = ((char) vclock_counter) & 3;

	}

	else if (state==STATE_SEARCHING) {

        // put sample in searching buffer
		buf[bufindex] = (float) temp.channel[0];  // right channel

		// increment and wrap pointer
	    bufindex++;
	    if (bufindex>=M)
	    	bufindex = 0;

	    // compute incoherent correlation
	    zc = 0;
	    zs = 0;
		for(i=0;i<M;i++) {
			zc+= mfc[i]*buf[i];
			zs+= mfs[i]*buf[i];
		}
		z = zc*zc+zs*zs;

		if ((z>T1)&&(local_carrier_phase==0)) {  // xxx should make sure this runs in real-time
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

		local_carrier_phase = ((char) vclock_counter) & 3;

	}
	else if (state==STATE_RECORDING) {
		// put sample in recording buffer
		recbuf[recbufindex] = (float) temp.channel[0];  // right channel
		recbufindex++;
		if (recbufindex>=(2*N+2*M)) {
			state = STATE_CALCULATION;  // buffer is full (stop recording)
			recbufindex = 0; // shouldn't be necessary
		}

		local_carrier_phase = ((char) vclock_counter) & 3;


	}

	else if(state==STATE_RESPONSE){

		if(vclock_counter==vir_clock_start){ //Okay, we've reached the appropriate wrap around point where we should start sending the dataers
			amSending = 1;
		}
		if(amSending){ //write the buffered output waveform to the output file, adn increment the index counter
			outputData.channel[0] = outputBuf[response_buf_idx];

			response_buf_idx++;
		}
		if(response_buf_idx==response_buf_idx_max){
			response_done = 1;
			amSending = 0;
		}
		local_carrier_phase = ((char) vclock_counter) & 3;

	}

	// update virtual clock (cycles from 0 to L-1)
	vclock_counter++;
	if (vclock_counter>=L) {
		vclock_counter = 0; // wrap
		HighSample = 32000; //reset output clock pulse to highest value
	}
	else
	{
		HighSample = HighSample >> 1;
	}
	outputData.channel[1] = HighSample; //Left channel for debug, doesn't really do anything

	MCBSP_write(DSK6713_AIC23_DATAHANDLE, outputData.combo); // not necessary

}

