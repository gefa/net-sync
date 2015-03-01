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

//Because
#define CHIP_6713 1

//Board node definitions
#define MASTER_NODE 1
#define SLAVE_NODE 2

#define NODE_TYPE MASTER_NODE

// length of searching window in samples
#define M 60

// threshold value for searching window
#define T1 100000

// sinc pulse normalized bandwidth
#define BW 0.0125

// 2*N+1 is the number of samples in the sinc function
#define N (1 << 9) //512
#define N2 (N << 1) //1024

// virtual clock counter
#define L 4096

#define CLOCK_WRAP(i) ((i) & (L - 1)) // index wrapping macro

// max lag for computing correlations
#define MAXLAG 200

// number of coarse delays to store
#define MAX_STORED_DELAYS_COARSE 50
#define MAX_STORED_DELAYS_FINE 	 50

//Response buffer size in samples
#define OUTPUT_BUF_SIZE (2*N+1)

//Define channel numbers
#define CHANNEL_LEFT 0
#define CHANNEL_RIGHT 1

//Define master/slave channels
#define MASTER_RECEIVE_FROM_SLAVE_CHANNEL 	CHANNEL_LEFT
#define MASTER_TRANSMIT_CHANNEL_CLOCK_PULSE CHANNEL_RIGHT
#define MASTER_TRANSMIT_CHANNEL_TO_SLAVE	CHANNEL_LEFT

#define SLAVE_TRANSMIT_TO_MASTER_CHANNEL 	CHANNEL_LEFT
#define SLAVE_TRANSMIT_SYNC_CLOCK_CHANNEL	CHANNEL_RIGHT
#define SLAVE_RECEIVE_FROM_MASTER_CHANNEL	CHANNEL_RIGHT

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
float zc,zs,z;
short i,j,k;				//Indices
double t,x,y;				//More Indices
float bbsinc[2*N+1];   		// baseband sinc pulse buffer
float recbuf[2*N+2*M]; 		// recording buffer
float yc[2*N+2*M];     		// in-phase downmixed buffer
float ys[2*N+2*M];     		// quadrature downmixed buffer
short recbufindex = 0;		//

//System state variables
int state = STATE_SEARCHING;
short vclock_counter = 0; // virtual clock counter
short vir_clock_history[MAX_STORED_DELAYS_COARSE];
short recbuf_start_clock = 0; // virtual clock counter for first sample in recording buffer
short coarse_delay_estimate[MAX_STORED_DELAYS_COARSE];
float fine_delay_estimate[MAX_STORED_DELAYS_FINE];
short cde_index = 0;
short fde_index = 0;
char local_carrier_phase = 0;
char r = 0;
double phase_correction_factor;
short max_samp = 0;

short vir_clock_start;
short halfSinc;
short transmitSincPulseBuffer[OUTPUT_BUF_SIZE];
volatile short response_done = 0; 						//not done var for response state
volatile short response_buf_idx = 0; 					//index for output buffer
volatile short response_buf_idx_max = OUTPUT_BUF_SIZE;
volatile short MasterResponseSendingHuh = 0;			//control var for starting the sending of the response from master
volatile short ClockPulse = 0;							//Used for generating the master clock pulse output value

DSK6713_AIC23_CodecHandle hCodec;							// Codec handle
DSK6713_AIC23_Config config = DSK6713_AIC23_DEFAULTCONFIG;  // Codec configuration with default settings
// ------------------------------------------
// end of variables
// ------------------------------------------

double sin(double);
double cos(double);
double atan2(double,double);

interrupt void serialPortRcvISR(void);

//Helper function prototypes

void SetupTransmitModulatedSincPulseBuffer();
void SetupReceiveBasebandSincPulseBuffer();
void SetupReceiveTrigonometricMatchedFilters();

void main()
{

	// reset coarse and fine delay estimate buffers
	for (i=0;i<MAX_STORED_DELAYS_COARSE;i++)
		coarse_delay_estimate[i] = 0;
		fine_delay_estimate[i] = 0.0;

	for (i=0; i<MAX_STORED_DELAYS_COARSE;++i)
		vir_clock_history[i]=0;

	// set up the cosine and sin matched filters for searching
	// also initialize searching buffer
	SetupReceiveTrigonometricMatchedFilters();
	SetupReceiveBasebandSincPulseBuffer();
	SetupTransmitModulatedSincPulseBuffer();



	// ----- DSK Hardware Setup ------

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

	// ----- End DSK Hardware Setup -----

	while(1)						// main loop
	{
		if(NODE_TYPE == MASTER_NODE){	//Master control loop code
			if (state != STATE_CALCULATION) {//delete this
				//Do nothing
				//We are waiting for calculations to finish
			}
			else if (state==STATE_CALCULATION) {

				//printf wrecks the real-time operation
				//printf("Buffer recorded: %d %f.\n",recbuf_start_clock,z);

				int CurTime = vclock_counter;

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

				// No need to update indexes, we dont need history
				// increment coarse delay estimate index (same index for fine delay estimates)
				cde_index++;
				if (cde_index>=MAX_STORED_DELAYS_COARSE)
					cde_index = 0;
				fde_index++;
				if (fde_index>=MAX_STORED_DELAYS_FINE)
					fde_index = 0;

				// --- Calculations Finished ---


				// --- Prepare for Response State ---
				response_done = 0; //not done yet
				response_buf_idx = 0; //index for output buffer



				//Wrap start timer around virtual clock origin.
				vir_clock_start = CLOCK_WRAP(L - CLOCK_WRAP(coarse_delay_estimate[cde_index-1]) - N2); //start time  //cde_index-1 -> take most recent estimate
				//course delay estimate wraps with respect to L, I dont think that's good?


				if(CLOCK_WRAP(coarse_delay_estimate[cde_index-1])>vclock_counter)
					while(vclock_counter != 0) ;

				if(CurTime > vir_clock_start){
					while(vclock_counter != 0); //wait one additional tick because we've already passed previous starting point we need
					while(vclock_counter != 0); //wait one additional tick because we've already passed previous starting point we need
				}

				/*if(vir_clock_start<(L-N) && cur_vir>vclock_counter){
					while(vclock_counter != 0) ; //wait one additional tick because we've already passed previous starting point we need
				}*/

				while(vclock_counter != 0) ; //wait one additional tick because we've already passed previous starting point we need

				state = STATE_RESPONSE; //set to response for the ISR to pick the appropriate path

				while(!response_done) ; //Loop and wait here until the responding output code works
					state = STATE_SEARCHING; // back to searching state, after responding
			}


		}
		else if (NODE_TYPE == SLAVE_NODE){
			//Do nothing, we're the slave. All real calculations occur during the ISR
		}

	}
}

interrupt void serialPortRcvISR()
{
	union {Uint32 combo; short channel[2];} tempInput;
	union {Uint32 combo; short channel[2];} tempOutput;

	tempInput.combo = MCBSP_read(DSK6713_AIC23_DATAHANDLE);
	tempOutput.combo = 0; //Set to zero now for missed sets.
	// Note that right channel is in temp.channel[0]
	// Note that left channel is in temp.channel[1]



	if(NODE_TYPE == MASTER_NODE){


		if(state==STATE_CALCULATION){ //Do nothing, since it originally did nothing. this loop is to ensure the appropriate output buffer is written.
			//Do nothing
		}

		else if (state==STATE_SEARCHING) {

			// put sample in searching buffer
			buf[bufindex] = (float) tempInput.channel[MASTER_RECEIVE_FROM_SLAVE_CHANNEL];  // right channel

			// increment and wrap pointer
			bufindex++;
			if (bufindex>=M)
				bufindex = 0;

			// compute incoherent correlation
			zc = 0;
			zs = 0;
			for(i=0;i<M;i++) {
				zc+= matchedFilterCosine[i]*buf[i];
				zs+= matchedFilterSine[i]*buf[i];
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

		}
		else if (state==STATE_RECORDING) {
			// put sample in recording buffer
			recbuf[recbufindex] = (float) tempInput.channel[MASTER_RECEIVE_FROM_SLAVE_CHANNEL];  // right channel
			recbufindex++;
			if (recbufindex>=(2*N+2*M)) {
				state = STATE_CALCULATION;  // buffer is full (stop recording)
				recbufindex = 0; // shouldn't be necessary
			}

		}

		else if(state==STATE_RESPONSE){

			if(vclock_counter==vir_clock_start){ //Okay, we've reached the appropriate wrap around point where we should start sending the dataers
				MasterResponseSendingHuh = 1;
			}
			if(MasterResponseSendingHuh){ //write the buffered output waveform to the output file, adn increment the index counter
				tempOutput.channel[MASTER_TRANSMIT_CHANNEL_TO_SLAVE] = transmitSincPulseBuffer[response_buf_idx];
				response_buf_idx++;
			}
			if(response_buf_idx==response_buf_idx_max){
				response_done = 1;		//Used to break from STATE_RESPONSE in the main while loop and move on
				MasterResponseSendingHuh = 0;			//quits the sending part above
			}
		}

		// ---- Common ----- ----- -----
		local_carrier_phase = ((char) vclock_counter) & 3;

		vclock_counter++;
		// update virtual clock (cycles from 0 to L-1)
		if (vclock_counter>=L){
			vclock_counter = 0; // wrap
			ClockPulse = 32000; //reset output clock pulse to highest value
		}
		else{
			ClockPulse = 0;
		}

		tempOutput.channel[MASTER_TRANSMIT_CHANNEL_CLOCK_PULSE] = ClockPulse; //Left channel for debug, doesn't really do anything


	}

/*	//Run all interrupt routines for the slave node here
	else if (NODE_TYPE == SLAVE_NODE){

		// update virtual clock
		if (vclock_counter>=(2*L)){ //runs at 1/2x rate of master for clock pulses, might want to switch variables?
			vclock_counter = 0;
			//DSK6713_LED_toggle(0); // not sure if this runs in real-time
			tempOutput.channel[SLAVE_TRANSMIT_SYNC_CLOCK_CHANNEL] = 32000;
		}
		else {
			tempOutput.channel[SLAVE_TRANSMIT_SYNC_CLOCK_CHANNEL] = 0;
		}
		if ((pulse_counter>=-N)&&(pulse_counter<=N))
			tempOutput.channel[SLAVE_TRANSMIT_TO_MASTER_CHANNEL] = sincpulse[pulse_counter+N];
		else
			tempOutput.channel[SLAVE_TRANSMIT_TO_MASTER_CHANNEL] = 0;

		pulse_counter++;
		if (pulse_counter>=LMAX)
			pulse_counter = LMIN;

	}*/

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
		transmitSincPulseBuffer[i+N] = y*32767;
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
		bbsinc[i+N] = (float) y;
	}
}

/**

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

