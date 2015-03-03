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

//Because
#define CHIP_6713 1

//Board node definitions
#define MASTER_NODE 1
#define SLAVE_NODE 	2

//Node type - This changes whether setting 
#define NODE_TYPE MASTER_NODE

//Audio codec sample frequency
#define DSK_SAMPLE_FREQ DSK6713_AIC23_FREQ_8KHZ

// length of searching window in samples
#define M 60

// threshold value for searching window
#define T1 100000

// sinc pulse normalized bandwidth
#define BW 0.0125

// 2*N+1 is the number of samples in the sinc function
#define N (1 << 9) //512
#define N2 ((N<<1)+1) //1025

// virtual clock counter maximum
#define L (1 << 12)	//4096

#define SLAVE_PULSE_COUNTER_MIN (-L/2)
#define SLAVE_PULSE_COUNTER_MAX (L/2)

#define CLOCK_WRAP(i) ((i) & (L - 1)) // index wrapping macro

// max lag for computing correlations
#define MAXLAG 200

// number of coarse delays to store
#define MAX_STORED_DELAYS_COARSE 50
#define MAX_STORED_DELAYS_FINE 	 50

//Response buffer size in samples
#define OUTPUT_BUF_SIZE (2*N+1)

// maximum sample value
#define MAXSAMP 32767;

//Define channel numbers for audio codec union variable
#define CHANNEL_LEFT 0
#define CHANNEL_RIGHT 1

//Define master/slave channels
#define MASTER_TRANSMIT_CHANNEL_CLOCK_PULSE CHANNEL_RIGHT
#define MASTER_TRANSMIT_CHANNEL_TO_SLAVE	CHANNEL_LEFT

#define MASTER_RECEIVE_FROM_SLAVE_CHANNEL 	CHANNEL_LEFT

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
#if (NODE_TYPE == MASTER_NODE)
volatile int state = STATE_SEARCHING;
#elif (NODE_TYPE == SLAVE_NODE)
volatile int state = STATE_SEARCHING;
#endif

volatile short vclock_counter = 0; // virtual clock counter
volatile short recbuf_start_clock = 0; // virtual clock counter for first sample in recording buffer
short coarse_delay_estimate[MAX_STORED_DELAYS_COARSE];
float fine_delay_estimate[MAX_STORED_DELAYS_FINE];
short cde_index = 0;
short fde_index = 0;
volatile char local_carrier_phase = 0;
char r = 0;
double phase_correction_factor;
short max_samp = 0;

//Master sinc response variables
volatile short vir_clock_start; 
volatile short CurTime = 0;
short halfSinc;
short transmitSincPulseBuffer[OUTPUT_BUF_SIZE];
volatile short response_done = 0; 						//not done var for response state
volatile short response_buf_idx = 0; 					//index for output buffer
volatile short response_buf_idx_max = OUTPUT_BUF_SIZE;
volatile short MasterResponseSendingHuh = 0;			//control var for starting the sending of the response from master
volatile short ClockPulse = 0;							//Used for generating the master clock pulse output value
volatile short calculation_done = 0;		//debug
volatile short v_clk[3];					//debug

//Slave transmit variables
volatile short pulse_counter = SLAVE_PULSE_COUNTER_MIN;
volatile short slaveNewVClk = 0; 							//calculated new virtiual clock value with respect to the current running vclock counter
volatile short slaveNewVClkOff = 0;							//Unused
volatile short slaveNewVClkF = 0.0;							//Unused
volatile short slaveNewVClkFOff = 0.0;						//Unused

//ISR combos
union {Uint32 combo; short channel[2];} tempOutput;
union {Uint32 combo; short channel[2];} tempInput;

DSK6713_AIC23_CodecHandle hCodec;							// Codec handle
DSK6713_AIC23_Config config = DSK6713_AIC23_DEFAULTCONFIG;  // Codec configuration with default settings
// ------------------------------------------
// end of variables
// ------------------------------------------

//Math function prototypes need to be declared here to be run properly
double sin(double);
double cos(double);
double atan2(double,double);
float sumFloatArray(float*, short numElmts);


interrupt void serialPortRcvISR(void);

//Helper function prototypes
void SetupTransmitModulatedSincPulseBuffer();
void SetupReceiveBasebandSincPulseBuffer();
void SetupReceiveTrigonometricMatchedFilters();
void runReceivedPulseBufferDownmixing();

//State functions run during ISR
void runSearchingStateCodeISR();
void runRecordingStateCodeISR();
void runCalculationStateCodeISR();
void runResponseStateCodeISR();


//State functions run during while() loop
void runMasterResponseSincPulseTimingControl();
void runReceviedSincPulseTimingAnalysis();

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

	// interrupt setup
	IRQ_globalDisable();			// Globally disables interrupts
	IRQ_nmiEnable();				// Enables the NMI interrupt
	IRQ_map(IRQ_EVT_RINT1,15);		// Maps an event to a physical interrupt
	IRQ_enable(IRQ_EVT_RINT1);		// Enables the event
	IRQ_globalEnable();				// Globally enables interrupts
	
	// -------- End DSK Hardware Setup --------

	while(1)						// main loop
	{
		#if (NODE_TYPE==MASTER_NODE) //Master control loop code
			if (state != STATE_CALCULATION) {
				//Do nothing
				//Maybe calculate the question to 42 if we have time
			}
			else if (state==STATE_CALCULATION) {
				//printf wrecks the real-time operation
				//printf("Buffer recorded: %d %f.\n",recbuf_start_clock,z);

				z = 0;  // clear correlation sum
				// -----------------------------------------------
				// this is where we estimate the time of arrival
				// -----------------------------------------------
				runReceivedPulseBufferDownmixing();
				runReceviedSincPulseTimingAnalysis();
				// --- Prepare for Response State ---
				runMasterResponseSincPulseTimingControl();
								// increment delay estimate index (now different index for fine delay estimates)
				cde_index++;
				if (cde_index>=MAX_STORED_DELAYS_COARSE)
					cde_index = 0;
				fde_index++;
				if (fde_index>=MAX_STORED_DELAYS_FINE)
					fde_index = 0;
			}
		#elif (NODE_TYPE==SLAVE_NODE)
			if(state!=STATE_CALCULATION){
				//Still do nothing
				
			}
			else if (state==STATE_CALCULATION){
				
				runReceivedPulseBufferDownmixing();
				runReceviedSincPulseTimingAnalysis();
				
				//Now we calculate the new center clock because YOLO
				
				// increment delay estimate index (now different index for fine delay estimates)
				cde_index++;
				if (cde_index>=MAX_STORED_DELAYS_COARSE)
					cde_index = 0;
				fde_index++;
				if (fde_index>=MAX_STORED_DELAYS_FINE)
					fde_index = 0;
			}
			
		#endif


	}
}

interrupt void serialPortRcvISR()
{

	tempInput.combo = MCBSP_read(DSK6713_AIC23_DATAHANDLE);
	tempOutput.combo = 0; //Set to zero now for missed sets.
	// Note that right channel is in temp.channel[0]
	// Note that left channel is in temp.channel[1]
	
	vclock_counter++; //Note! --- Not sure of the effects of moving the increment to the top	
	//Clock counter wrap
	#if (NODE_TYPE==MASTER_NODE)
		if (vclock_counter>=(L)) {
			vclock_counter = 0; // wrap
			tempOutput.channel[MASTER_TRANSMIT_CHANNEL_CLOCK_PULSE] = 32000; //Left channel for debug, doesn't really do anything
		}
		else{
			tempOutput.channel[MASTER_TRANSMIT_CHANNEL_CLOCK_PULSE] = 0; //Left channel for debug, doesn't really do anything
		}
	
	#elif(NODE_TYPE==SLAVE_NODE)
		if (vclock_counter>=(2*L)){ //runs at 1/2x rate of master for clock pulses, might want to switch variables?
			vclock_counter = 0;
			tempOutput.channel[SLAVE_TRANSMIT_SYNC_CLOCK_CHANNEL] = 32000;
		}
		else{
			tempOutput.channel[SLAVE_TRANSMIT_SYNC_CLOCK_CHANNEL] = 0;
		}
	#endif
	//Extra debug test. Variable set during calculation loop
	if(calculation_done)
	{
		calculation_done=0;
		tempOutput.channel[SLAVE_TRANSMIT_SYNC_CLOCK_CHANNEL] = 15000;
	}
	// ---- Common ----- ----- ----- 
	local_carrier_phase = ((char) vclock_counter) & 3;

	
	//Run all interrupt routine logic for the master node here
	#if (NODE_TYPE==MASTER_NODE)
		if (state==STATE_SEARCHING) {
			runSearchingStateCodeISR();
		}
		else if (state==STATE_RECORDING) {
			runRecordingStateCodeISR();
		}
		else if(state==STATE_CALCULATION){ 
			runCalculationStateCodeISR();
		}
		else if(state==STATE_RESPONSE){
			runResponseStateCodeISR();
		}
		else {
			//ERROR in STATE CODE LOGIC
		}

	//Run all interrupt routines for the slave node here
	#elif (NODE_TYPE==SLAVE_NODE)
		//Control code for outputting sinc pulse to master for synchronization
		if ((pulse_counter>=-N)&&(pulse_counter<=N))
			tempOutput.channel[SLAVE_TRANSMIT_TO_MASTER_CHANNEL] = transmitSincPulseBuffer[pulse_counter+N];
		else
			tempOutput.channel[SLAVE_TRANSMIT_TO_MASTER_CHANNEL] = 0;
		pulse_counter++;
		if (pulse_counter>=SLAVE_PULSE_COUNTER_MAX)
			pulse_counter = SLAVE_PULSE_COUNTER_MIN;
		
		//Control code for states and receiving stuff
		if(state==STATE_SEARCHING) {
			runSearchingStateCodeISR();
		}
		else if(state==STATE_RECORDING){
			runCalculationStateCodeISR();
		}
		else if(state==STATE_CALCULATION){
			runCalculationStateCodeISR();
		}
		else if(state==STATE_RESPONSE){
			//No special response for slave
		}
		else {
			//ERROR in STATE CODE LOGIC
		}
		
		
	#endif

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
	
*/
void runMasterResponseSincPulseTimingControl(){
	// --- Prepare for Response State ---
	union {Uint32 combo; short channel[2];} temp;
	temp.combo = 0; //Set to zero now for missed sets.

	response_done = 0; //not done yet
	response_buf_idx = 0; //index for output buffer
	//Wrap start timer around virtual clock origin.
	vir_clock_start = CLOCK_WRAP(L - CLOCK_WRAP(coarse_delay_estimate[cde_index]) - N2); //start time  //cde_index-1 -> take most recent estimate
	//course delay estimate wraps with respect to L, I dont think that's good?

	calculation_done = 1;
	
	if(CLOCK_WRAP(coarse_delay_estimate[cde_index])>vclock_counter){//i dont think this triggers ever
		temp.channel[0] = 25000;
		MCBSP_write(DSK6713_AIC23_DATAHANDLE, temp.combo);

		if(vclock_counter==0){
			v_clk[0]=666;
			v_clk[0]=vclock_counter;
			while(vclock_counter != (L-1)) ;
		}else
			while(vclock_counter != 0) ;
	}

	if(CurTime > vir_clock_start){//i dont think this triggers ever
		temp.channel[0] = -15000;
		MCBSP_write(DSK6713_AIC23_DATAHANDLE, temp.combo);

		if(vclock_counter==0){
				v_clk[0]=666;
			v_clk[1]=vclock_counter;
			while(vclock_counter != (L-1)) ;
			while(vclock_counter != (L-1)) ;


		}
		else{
			while(vclock_counter != 0); //wait one additional tick because we've already passed previous starting point we need
			while(vclock_counter != 0); //wait one additional tick because we've already passed previous starting point we need
		}
	}

	while(vclock_counter != 0) ; //wait one additional tick because we've already passed previous starting point we need
	state = STATE_RESPONSE; //set to response for the ISR to pick the appropriate path
	while(!response_done) ; //Loop and wait here until the responding output code works
		state = STATE_SEARCHING; // back to searching state, after responding

}


void runSearchingStateCodeISR(){

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

void runRecordingStateCodeISR(){
	// put sample in recording buffer
	recbuf[recbufindex] = (float) tempInput.channel[MASTER_RECEIVE_FROM_SLAVE_CHANNEL];  // right channel
	recbufindex++;
	if (recbufindex>=(2*N+2*M)) {
		CurTime = vclock_counter;
		state = STATE_CALCULATION;  // buffer is full (stop recording)
		recbufindex = 0; // shouldn't be necessary
	}
}

void runCalculationStateCodeISR(){
	//Do nothing here
}


void runResponseStateCodeISR(){
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

void runReceviedSincPulseTimingAnalysis(){
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

	// --- Calculations Finished ---
}

void runReceivedPulseBufferDownmixing(){
	// downmix (had problems using sin/cos here so used a trick)
	// The trick is based on the incoming frequency per sample being (n * pi/2), so every other sample goes to zero.
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
}

float sumFloatArray(float* array, short numElmts){
	float sum = 0.0;
	short idx = 0;
	for(idx=0; idx < numElmts; idx++){
		sum += array[idx];
	}
	return sum;
}