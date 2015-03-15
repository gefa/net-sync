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
#define NODE_TYPE SLAVE_NODE

//Audio codec sample frequency
#define DSK_SAMPLE_FREQ DSK6713_AIC23_FREQ_8KHZ

// length of searching window in samples
#define M 60

// threshold value for searching window
#define T1 100000

// sinc pulse normalized bandwidth
#define BW 0.0125 //100Hz@8KhzFs

// 2*N+1 is the number of samples in the sinc function
#define N (1 << 9) //512
#define N2 ((N*2)+1) //1025

// virtual clock counter maximum
/*
#if (NODE_TYPE == MASTER_NODE)
#define VCLK_MAX (1<<12)	//4096
#elif (NODE_TYPE == SLAVE_NODE)
#define VCLK_MAX (1<<14) //4096
#endif
*/
#define VCLK_MAX (1<<14) //16384 counts, never reaches this
#define VCLK_MAX_WRAPPED (1<<12) //4096 counts

#define VCLK_WRAP(i) ((i)&(VCLK_MAX_WRAPPED-1))

//#define SLAVE_PULSE_COUNTER_MIN (-(VCLK_MAX*2))
//#define SLAVE_PULSE_COUNTER_MAX (VCLK_MAX*2)
//Don't matter now


// max lag for computing correlations
#define MAXLAG 200

// number of coarse delays to store
#define MAX_STORED_DELAYS_COARSE 16
#define MAX_STORED_DELAYS_FINE 	 16

//Response buffer size in samples
//#define OUTPUT_BUF_SIZE (2*N+1)
//Instead, use N2 for everything. EVERYTHING.

// maximum sample value
#define MAXSAMP 32767;

//Define channel numbers for audio codec union variable
#define CHANNEL_LEFT 0
#define CHANNEL_RIGHT 1

//Define master/slave channels
#define TRANSMIT_SINC	CHANNEL_LEFT	//Channel along which the transmission sinc pulse is sent
#define RECEIVE_SINC	CHANNEL_LEFT	//Corresponding receive channel on the other end
#define TRANSMIT_CLOCK	CHANNEL_RIGHT	//Verification clock output that centers along the zero-point of the vclock

// State definitions
#define STATE_SEARCHING 0
#define STATE_RECORDING 1
#define STATE_CALCULATION 2
#define STATE_TRANSMIT 3

// define PI and INVPI
#define PI 3.14159265358979323846
#define INVPI 0.318309886183791

//gpio registers
#define GPIO_ENABLE_ADDRESS		0x01B00000
#define GPIO_DIRECTION_ADDRESS	0x01B00004
#define GPIO_VALUE_ADDRESS		0x01B00008

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
float corrSumCosine,corrSumSine,corrSumIncoherent;
short i,j,k;				// Indices
double t,x,y;				// More Indices
float basebandSincRef[2*N+1];   		// baseband sinc pulse buffer
float recbuf[2*N+2*M]; 		// recording buffer
float downMixedCosine[2*N+2*M];     		// in-phase downmixed buffer
float downMixedSine[2*N+2*M];     		// quadrature downmixed buffer
short recbufindex = 0;		//

#if (NODE_TYPE == MASTER_NODE)//if master, listen to slave first and then send the sinc back
volatile int state = STATE_SEARCHING;
#elif (NODE_TYPE == SLAVE_NODE)//if slave, send sinc and then wait for master's response
volatile int state = STATE_TRANSMIT;
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
volatile short CurTime;
short halfSinc;
short tModulatedSincPulse[N2];
volatile short response_done = 0; 						//not done var for response state
//volatile short response_buf_idx = 0; 					//index for output buffer
//volatile short response_buf_idx_max = OUTPUT_BUF_SIZE;
volatile short amSending = 0;			//control var for starting the sending of the response from master
volatile short sinc_launch = 0;
volatile short sinc_roundtrip_time ;
volatile short vclock_offset ;
volatile short ClockPulse = 0;							//Used for generating the master clock pulse output value
//volatile short calculation_done = 0;		//debug
//volatile short v_clk[3];					//debug
volatile short virClockTransmitCenterSinc = 0;
volatile short virClockTransmitCenterVerify = 0;

//Slave transmit variables
//short pulse_counter = SLAVE_PULSE_COUNTER_MIN;

// ISR combos
union {uint32_t combo; short channel[2];} tempOutput;
union {uint32_t combo; short channel[2];} tempInput;

// Handles
DSK6713_AIC23_CodecHandle hCodec;							// Codec handle
DSK6713_AIC23_Config config = DSK6713_AIC23_DEFAULTCONFIG;  // Codec configuration with default settings

GPIO_Handle    hGpio; /* GPIO handle */

// ------------------------------------------
// end of variables
// ------------------------------------------

//Math function prototypes need to be declared here to be run properly
double sin(double);
double cos(double);
double atan2(double,double);
float sumFloatArray(float*, short numElmts);
long sumIntArray(short* array, short numElmts);

interrupt void serialPortRcvISR(void);

//Helper function prototypes
void SetupTransmitModulatedSincPulseBuffer();
void SetupReceiveBasebandSincPulseBuffer();
void SetupReceiveTrigonometricMatchedFilters();
void runReceivedPulseBufferDownmixing();
//void runSlaveSincPulseTimingUpdateCalcs();
void inline ToggleDebugGPIO(short IONum);
short isSincInSameWindowHuh(short curClock, short delayEstimate);
short GoodToSendIndex(short counterTime, short centerPoint);


//State functions run during ISR
void runSearchingStateCodeISR();
void runRecordingStateCodeISR();
void runCalculationStateCodeISR();
//void runResponseStateCodeISR();
//void runSlaveTransmitStateCodeISR();
void runSincPulseTransmitISR();
void runVerifyPulseTransmitISR();


//State functions run during while() loop
//void runMasterResponseSincPulseTimingControl();
void runReceviedSincPulseTimingAnalysis();

//New state functions for new code.
void calculateNewSynchronizationTimeSlave();
void calculateNewResponseTimeMaster();

//debug gpio function
void gpioInit();
void gpioToggle();

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
			ToggleDebugGPIO(1);
			runReceivedPulseBufferDownmixing();
			runReceviedSincPulseTimingAnalysis();
			ToggleDebugGPIO(1);
			//Now we calculate the new center clock

			#if (NODE_TYPE==MASTER_NODE)
				calculateNewResponseTimeMaster();
			#elif (NODE_TYPE==SLAVE_NODE)
				calculateNewSynchronizationTimeSlave();
			#endif

			state = STATE_TRANSMIT; //go to responding!

			while(state == STATE_TRANSMIT){
				//wait for ISR to timeout and switch state
			}

			//increment indices
			cde_index++;
			if(cde_index>=MAX_STORED_DELAYS_COARSE) cde_index = 0;
			fde_index++;
			if(fde_index>=MAX_STORED_DELAYS_FINE) fde_index = 0;
		}
	}
}

interrupt void serialPortRcvISR()
{

	tempInput.combo = MCBSP_read(DSK6713_AIC23_DATAHANDLE);
	tempOutput.combo = 0; //Set to zero now for missed sets.
	// Note that right channel is in temp.channel[0]
	// Note that left channel is in temp.channel[1]

	//Clock counter wrap
	vclock_counter++; //Note! --- Not sure of the effects of moving the increment to the top
	//Code should be common to both
	if(vclock_counter >= VCLK_MAX){
		vclock_counter = 0;
		ToggleDebugGPIO(0);
	}
	else{
		//Do nothing
	}

	//Logic for different states is the same on both master and slave sides when expecting to receive
	if (state==STATE_SEARCHING) {
		runSearchingStateCodeISR();
	}
	else if (state==STATE_RECORDING) {
		runRecordingStateCodeISR();
	}
	else if(state==STATE_CALCULATION){
		runCalculationStateCodeISR();
	}
	else if(state==STATE_TRANSMIT){
		runSincPulseTransmitISR();//this function will change state when it's done
	}
	else {
		//ERROR in STATE CODE LOGIC
	}
	runVerifyPulseTransmitISR();	//Outputs synchronized pulse on aux channel
		
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
		tModulatedSincPulse[i+N] = y*32767;
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
	Not sure what this does again now
*/
/*
void runMasterResponseSincPulseTimingControl(){
	// --- Prepare for Response State ---
	union {Uint32 combo; short channel[2];} temp;
	temp.combo = 0; //Set to zero now for missed sets.

	response_done = 0; //not done yet
	response_buf_idx = 0; //index for output buffer

#if (NODE_TYPE==MASTER_NODE)
	//Wrap start timer around virtual clock origin.
	vir_clock_start = CLOCK_WRAP(VCLK_MAX - CLOCK_WRAP(coarse_delay_estimate[cde_index]) - N2); //start time  //cde_index-1 -> take most recent estimate
	//course delay estimate wraps with respect to VCLK_MAX, I dont think that's good?

	if(CLOCK_WRAP(coarse_delay_estimate[cde_index])>vclock_counter){//i dont think this triggers ever
		//temp.channel[0] = 25000;
		//MCBSP_write(DSK6713_AIC23_DATAHANDLE, temp.combo);

		if(vclock_counter==0){
			while(vclock_counter != (VCLK_MAX-1)) ;
		}else
			while(vclock_counter != 0) ;
	}

	if(CurTime > vir_clock_start){//i dont think this triggers ever
		//temp.channel[0] = -15000;
		//MCBSP_write(DSK6713_AIC23_DATAHANDLE, temp.combo);
		ToggleDebugGPIO(0);



		if(vclock_counter==0){
				v_clk[0]=666;
			v_clk[1]=vclock_counter;
			while(vclock_counter != (VCLK_MAX-1)) ;
			while(vclock_counter != (VCLK_MAX-1)) ;


		}
		else{
			while(vclock_counter != 0); //wait one additional tick because we've already passed previous starting point we need
			while(vclock_counter != 0); //wait one additional tick because we've already passed previous starting point we need
		}
	}

	while(vclock_counter != 0) ; //wait one additional tick because we've already passed previous starting point we need

	state = STATE_TRANSMIT; //set to response for the ISR to pick the appropriate path

	while(state == STATE_TRANSMIT) ; //Loop and wait here until the responding output code works
#elif (NODE_TYPE==SLAVE_NODE)

	vir_clock_start = CLOCK_WRAP(CLOCK_WRAP(coarse_delay_estimate[cde_index])+ N); //start time  //cde_index-1 -> take most recent estimate

	while(vclock_counter != vir_clock_start) ;

	//temp.channel[TRANSMIT_CLOCK] = -15000;
	//MCBSP_write(DSK6713_AIC23_DATAHANDLE, temp.combo);



#endif
}
*/


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

	if (corrSumIncoherent>T1) {  // xxx should make sure this runs in real-time
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
		CurTime = vclock_counter;
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

/**
 * Mixes the received waveform in recbuf down to baseband. ONLY WORKS at currently set center freq (1/2 of nyquist)
 */
void runReceivedPulseBufferDownmixing(){
	// downmix (had problems using sin/cos here so used a trick)
	// The trick is based on the incoming frequency per sample being (n * pi/2), so every other sample goes to zero.
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
 * Initializes the GPIO registers to allow for toggling some of them to view the state change outputs
 */
void gpioInit()
{
	//--------------NOTE------------------
	//	FOR GPIOs TO WORK ON C6713 DSK SPECTRUM DIGITAL BOARD
	//	SWITCH 4 OF THE DIPSWITCH SW3 (NOT SW1!!!) HAS TO BE ON-CLOSED
	//--------------NOTE------------------

	GPIO_Config MyConfig = {
	0x00000000, /* gpgc */
	0x0000FFFF, /* gpen --*/
	0x00000000, /* gdir -*/
	0x00000000, /* gpval */
	0x00000000, /* gphm all interrupts disabled for io pins */
	0x00000000, /* gplm all interrupts to cpu or edma disabled  */
	0x00000000  /* gppol -- default state */
	};
	hGpio  = GPIO_open( GPIO_DEV0, GPIO_OPEN_RESET );
	GPIO_config(hGpio  , &MyConfig );
	/* Enables pins */
	GPIO_pinEnable (hGpio,GPIO_PIN0| GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5);//enable here or in MyConfig
	/* Sets Pin0, Pin1, and Pin2 as an output pins. */
	int Current_dir = GPIO_pinDirection(hGpio,GPIO_PIN0, GPIO_OUTPUT);
	Current_dir = GPIO_pinDirection(hGpio,GPIO_PIN1, GPIO_OUTPUT);
	Current_dir = GPIO_pinDirection(hGpio,GPIO_PIN2, GPIO_OUTPUT);
	Current_dir = GPIO_pinDirection(hGpio,GPIO_PIN3, GPIO_OUTPUT);
	Current_dir = GPIO_pinDirection(hGpio,GPIO_PIN4, GPIO_OUTPUT);
	Current_dir = GPIO_pinDirection(hGpio,GPIO_PIN5, GPIO_OUTPUT);
}

/** \brief Loops toggling two GPIO pins for debug test
 *
 *
 *
 */
void gpioToggle()
{
	while(1)
	{
		GPIO_pinWrite( hGpio, GPIO_PIN0, 0 );
		GPIO_pinWrite( hGpio, GPIO_PIN1, 0 );
		*((int*)GPIO_VALUE_ADDRESS) = 255;
		DSK6713_waitusec(500000);
		GPIO_pinWrite( hGpio, GPIO_PIN0, 1 );
		GPIO_pinWrite( hGpio, GPIO_PIN1, 1 );
		*((int*)GPIO_VALUE_ADDRESS) = 0;
		DSK6713_waitusec(500000);


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



/**	\brief Quickly toggles a set pin number high and then low again, for timing view
 *
 * @param IONum Pin number to toggle (0-3 currently)
 */
void inline ToggleDebugGPIO(short IONum){
	if (IONum == 0){
		GPIO_pinWrite(hGpio,GPIO_PIN0,1);
		GPIO_pinWrite(hGpio,GPIO_PIN0,0);
	}
	else if(IONum == 1){
		GPIO_pinWrite(hGpio,GPIO_PIN1, 1);
		GPIO_pinWrite(hGpio,GPIO_PIN1, 0);
	}
	else if(IONum == 2){
		GPIO_pinWrite(hGpio,GPIO_PIN2, 1);
		GPIO_pinWrite(hGpio,GPIO_PIN2, 0);
	}
	else if(IONum == 3){
		GPIO_pinWrite(hGpio,GPIO_PIN3, 1);
		GPIO_pinWrite(hGpio,GPIO_PIN3, 0);
	}
	else if(IONum == 4){
		GPIO_pinWrite(hGpio,GPIO_PIN4, 1);
		GPIO_pinWrite(hGpio,GPIO_PIN4, 0);
	}
	else if(IONum == 5){
		GPIO_pinWrite(hGpio,GPIO_PIN5, 1);
		GPIO_pinWrite(hGpio,GPIO_PIN5, 0);
	}
	else{
		//error
	}
}

/**
 * 	Transmits the synchronizing sinc pulse on the main channel to the other node.
 * 	The value for the center point should be properly calculated based on node type from the other functions
 * 	Is always centered at 0 for the slave, and the mirrored response for the master
 */
void runSincPulseTransmitISR(){

	short idxTemp = GoodToSendIndex(vclock_counter, virClockTransmitCenterSinc);

	if(idxTemp != -1){
		tempOutput.channel[TRANSMIT_SINC] = tModulatedSincPulse[idxTemp];
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
	short idxTemp = GoodToSendIndex(vclock_counter, virClockTransmitCenterVerify);

	if(idxTemp != -1){
		tempOutput.channel[TRANSMIT_CLOCK] = tModulatedSincPulse[idxTemp];
	}
	else{
		tempOutput.channel[TRANSMIT_CLOCK] = 0;
	}
}

/**
 * Responds with the proper index for the output buffer when given the transmitting center point, and the current time.
 * Will reply with an invalid value (-1) for the shorts if we are out of range. Logic should check for this
 * @param counterTime	Current time for the vclk counter in the interrupt
 * @param centerPoint	Highest point in the response sinc pulse (x=0) that is being sent
 * @return the index of the transmit buffer to send (0 to N2 - 1), or -1 if we're actually out of range
 */
short GoodToSendIndex(short counterTime, short centerPoint){
	if((centerPoint+N)<(centerPoint-N)){ //we are too close to the edge, it will wrap!

	}
	else{

	}
	return 0;
}

/**
 * Looks at the received response time, the current time (to prevent trying to send immediately a fractional waveform,
 * and calculates the new mirrored center time around a clock tick upon which to center the response at
 */
void calculateNewResponseTimeMaster(){
	virClockTransmitCenterSinc = 0; //placeholder
}

/**
 *
 */
void calculateNewSynchronizationTimeSlave(){
	virClockTransmitCenterVerify = 0; //placeholder
}

