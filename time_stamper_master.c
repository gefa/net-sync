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
//#define BW 0.0250

// 2*N+1 is the number of samples in the sinc function
#define N (1<<9) //512
#define N2 ((N<<1)+1) //1025

#define CALC_TIME	384		// measured on the scope
#define WIDTH		(2*N+1)
#define WIDTH15	(WIDTH + N)

// virtual clock counter maximum
#if (NODE_TYPE == MASTER_NODE)
#define VCLK_MAX (1<<12)	//4096
#elif (NODE_TYPE == SLAVE_NODE)
#define VCLK_MAX (1<<12) //4096
#endif

//#define SLAVE_PULSE_COUNTER_MIN (-(VCLK_MAX*2))
//#define SLAVE_PULSE_COUNTER_MAX (VCLK_MAX*2)

#define CLOCK_WRAP(i) ((i)&(VCLK_MAX-1)) // index wrapping macro
#define INDEX_WRAP(x) ((x)&((VCLK_MAX*BUF_SIZE)-1))

// max lag for computing correlations
#define MAXLAG 200

// number of coarse delays to store
#define MAX_STORED_DELAYS_COARSE 50
#define MAX_STORED_DELAYS_FINE 	 50

//Response buffer size in samples
#define OUTPUT_BUF_SIZE (2*N+1)
#define BUF_SIZE		4
// maximum sample value
#define MAXSAMP 32767;

//Define channel numbers for audio codec union variable
#define CHANNEL_LEFT 0
#define CHANNEL_RIGHT 1

//Define master/slave channels
#define TRANSMIT_SINC	CHANNEL_LEFT
#define RECEIVE_SINC	CHANNEL_LEFT
#define TRANSMIT_CLOCK	CHANNEL_RIGHT

// State definitions
#define STATE_SEARCHING 0
#define STATE_RECORDING 1
#define STATE_CALCULATION 2
#define STATE_TRANSMIT 3
#define STATE_SENDSINC 4

// define PI and INVPI
#define PI 3.14159265358979323846
#define INVPI 0.318309886183791

//gpio registers
#define GPIO_ENABLE_ADDRESS		0x01B00000
#define GPIO_DIRECTION_ADDRESS	0x01B00004
#define GPIO_VALUE_ADDRESS		0x01B00008

#define MAXDELAY 100	// this should be renamed to RESOLUTION_OF_FINE_DELAY_ESTIMATE
#define BW 0.0125 	//100Hz@8k Fs baseband sinc frequency
#define CBW 0.25 	//2kHz@8k Fs  carrier frequency

#include <stdio.h>
#include <c6x.h>
#include <csl.h>
#include <csl_mcbsp.h>
#include <csl_irq.h>
#include <csl_gpio.h>
#include <math.h>

#include <stdio.h>					//For printf
#include <c6x.h>					//generic include
#include <csl.h>					//generic csl include
#include <csl_gpio.h>
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
float tf,xf,yf;				// More Indices
float basebandSincRef[2*N+1];   		// baseband sinc pulse buffer
float recbuf[2*N+2*M]; 		// recording buffer
float downMixedCosine[2*N+2*M];     		// in-phase downmixed buffer
float downMixedSine[2*N+2*M];     		// quadrature downmixed buffer
volatile short recbufindex = 0;		//

#if (NODE_TYPE == MASTER_NODE)//if master, listen to slave first and then send the sinc back
volatile int state = STATE_SEARCHING;
#elif (NODE_TYPE == SLAVE_NODE)//if slave, send sinc and then wait for master's response
volatile int state = STATE_TRANSMIT;
#endif

volatile short vclock_counter = 0;	// virtual clock counter
volatile short vclock_complement = 0;	// VCLK_MAX - vclock_counter
volatile short max_recbuf = 0;
volatile short playback_scale = 1;
volatile short wait_count = 0;
volatile int myage = 0;
volatile short dedicated_clk = 0;	// make decision at fixed time after sinc peak center

volatile short recbuf_start_clock = 0; // virtual clock counter for first sample in recording buffer
short coarse_delay_estimate[MAX_STORED_DELAYS_COARSE];
float fine_delay_estimate[MAX_STORED_DELAYS_FINE];
short cde_index = 0;
short fde_index = 0;
volatile char local_carrier_phase = 0;
char r = 0;
double phase_correction_factor;
short max_samp = 0;

#if (NODE_TYPE == MASTER_NODE)
//volatile short ML[VCLK_MAX*BUF_SIZE];
//volatile short MR[VCLK_MAX*BUF_SIZE];
volatile short ML[VCLK_MAX*BUF_SIZE];//master response sinc
volatile short MR[VCLK_MAX*BUF_SIZE];//jus for debug
#elif (NODE_TYPE == SLAVE_NODE)
//volatile short SL[VCLK_MAX*BUF_SIZE];
//volatile short SR[VCLK_MAX*BUF_SIZE];
volatile short SR[VCLK_MAX];		//slave clock
volatile short SL[VCLK_MAX*BUF_SIZE];		//static outgoing-sinc from slave
#endif



volatile unsigned short run_head = 0;
volatile unsigned short run_head_sl = 0;
volatile unsigned short calc_head = 0;

//Master sinc response variables
volatile short vir_clock_start;
volatile short CurTime = 0;
short halfSinc;

//Output waveform buffers for clock and sync channels
short standardWaveformBuffer[N2];
short delayedWaveformBuffer[N2];
#pragma DATA_SECTION(allMyDelayedWaveforms,".mydata")
far short allMyDelayedWaveforms[MAXDELAY][N2];

short tModulatedSincPulse[OUTPUT_BUF_SIZE];
//#pragma DATA_SECTION(tModulatedSincPulse_delayed,".mydata")
//#pragma DATA_ALIGN (tModulatedSincPulse_delayed, 2)			//dont really need to align
far short tModulatedSincPulse_delayed[OUTPUT_BUF_SIZE];	//must be extern far according to http://www.ti.com/lit/ug/spru187o/spru187o.pdf page 136
																//not sure but all variable might be "far" by default
volatile short even = 1;
volatile short response_done = 0; 						//not done var for response state
volatile short response_buf_idx = 0; 					//index for output buffer
volatile short response_buf_idx_clk = 0; 					//another index for output buffer
volatile short response_buf_idx_max = OUTPUT_BUF_SIZE;
volatile short amSending = 0;			//control var for starting the sending of the response from master
volatile short amWaiting = 0;			//control var for starting the waiting process before master's response
volatile short sinc_launch = 0;
volatile short sinc_roundtrip_time ;
volatile short vclock_offset ;
volatile short ClockPulse = 0;							//Used for generating the master clock pulse output value
volatile short calculation_done = 0;		//debug
volatile short v_clk[3];					//debug
volatile short clk_flag = 0;

#define HISTORY	20
volatile short debug_history[4][HISTORY];
int index_history[4][HISTORY];
volatile float float_history[4][HISTORY];
//volatile short debug_history2[HISTORY];
//volatile short debug_history3[HISTORY];
int age=0;

//Slave transmit variables
//short pulse_counter = SLAVE_PULSE_COUNTER_MIN;

// ISR combos
union {Uint32 combo; short channel[2];} tempOutput;
union {Uint32 combo; short channel[2];} tempInput;
//union {Uint32 combo; short channel[2];} debugOutput;

DSK6713_AIC23_CodecHandle hCodec;							// Codec handle
DSK6713_AIC23_Config config = DSK6713_AIC23_DEFAULTCONFIG;  // Codec configuration with default settings

GPIO_Handle    hGpio; /* GPIO handle */

GPIO_Handle gpio_handle;
GPIO_Config gpio_config = {
    0x00000000, // gpgc = global control
    0x0000FFFF, // gpen = enable 0-15
    0x00000000, // gdir = all inputs
    0x00000000, // gpval = n/a
    0x00000000, // gphm all interrupts disabled for io pins
    0x00000000, // gplm all interrupts to cpu or edma disabled
    0x00000000  // gppol -- default state */
};

// ------------------------------------------
// end of variables
// ------------------------------------------

//Math function prototypes need to be declared here to be run properly
double sin(double);
double cos(double);
double atan2(double,double);
//float sumFloatArray(float*, short numElmts);

interrupt void serialPortRcvISR(void);

//Helper function prototypes
void SetupTransmitModulatedSincPulseBuffer();
void SetupTransmitModulatedSincPulseBufferDelayed();
void SetupTransmitModulatedSincPulseBufferDelayedFine(float fineDelay);
void setupTransmitBuffer(short tBuffer[], short halfBufLen, double sincBandwidth, double carrierFreq, double delay);
void SetupReceiveBasebandSincPulseBuffer();
void SetupReceiveTrigonometricMatchedFilters();
void runReceivedPulseBufferDownmixing();
//void runSlaveSincPulseTimingUpdateCalcs();
void ToggleDebugGPIO(short IONum);
short isSincInSameWindowHuh(short curClock, short delayEstimate);



//State functions run during ISR
void runSearchingStateCodeISR();
void runRecordingStateCodeISR();
void playRecordingStateCodeISR();
void runCalculationStateCodeISR();
void runResponseStateCodeISR();

void runResponseClkSinc();


//State functions run during while() loop
void runMasterResponseSincPulseTimingControl();
void runReceviedSincPulseTimingAnalysis();

//debug gpio function
void gpioInit();
void gpioToggle();

int led_prev=0;//not sure how to check led state so just keep local copy
void toggle_LED(int led)
{
	if(led_prev){
		DSK6713_LED_off(led);
		led_prev = 0;
	}else{
		DSK6713_LED_on(led);
		led_prev=1;
	}
}

void main()
{

	setupTransmitBuffer(standardWaveformBuffer, N, BW, CBW, 0.0);
	setupTransmitBuffer(delayedWaveformBuffer, N, BW, CBW, 0.0);

	for(i = 0; i < MAXDELAY; i++){
		setupTransmitBuffer(&(allMyDelayedWaveforms[i][0]), N, BW, CBW, ((double) i) / MAXDELAY);
	}


	// reset coarse and fine delay estimate buffers
	for (i=0;i<MAX_STORED_DELAYS_COARSE;i++)
		coarse_delay_estimate[i] = 0;
		fine_delay_estimate[i] = 0.0;

#if (NODE_TYPE == MASTER_NODE)
	for(i=0;i<VCLK_MAX*BUF_SIZE;++i)
		ML[i] = 0;
	for(i=0;i<VCLK_MAX*BUF_SIZE;++i)
		MR[i] = 0;
#elif (NODE_TYPE == SLAVE_NODE)
	for(i=0;i<VCLK_MAX;++i)
		SR[i] = 0;
	for(i=0;i<VCLK_MAX*BUF_SIZE;++i)
		SL[i] = 0;
	//populate SL with zero-delayed sinc
	for (i=-N;i<=N;i++)
		SL[INDEX_WRAP(i + N + (VCLK_MAX>>1))] =  allMyDelayedWaveforms[0][i + N];
#endif

	// set up the cosine and sin matched filters for searching
	// also initialize searching buffer
	SetupReceiveTrigonometricMatchedFilters();
	SetupReceiveBasebandSincPulseBuffer();
	SetupTransmitModulatedSincPulseBuffer();
	SetupTransmitModulatedSincPulseBufferDelayed();
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
		#if (NODE_TYPE==MASTER_NODE) //Master control loop code
			if (state != STATE_CALCULATION) {
				//Do nothing
				//Maybe calculate the question to 42 if we have time
			}
			else if (state==STATE_CALCULATION) {
				//printf wrecks the real-time operation
				//printf("Buffer recorded: %d %f.\n",recbuf_start_clock,corrSumIncoherent);
//				corrSumIncoherent = 0;  // clear correlation sum
//				// -----------------------------------------------
//				// this is where we estimate the time of arrival
//				// -----------------------------------------------
//				runReceivedPulseBufferDownmixing();
//				runReceviedSincPulseTimingAnalysis();
//
////				tempOutput.channel[TRANSMIT_SINC] = 15000;
////				tempOutput.channel[1] = 0;
////				MCBSP_write(DSK6713_AIC23_DATAHANDLE, tempOutput.combo);
//
//				// alternative way - portable code
//				volatile short tick_variable = vclock_counter;//variable tick
//				volatile short tick_center_point = CLOCK_WRAP((short)(fine_delay_estimate[fde_index]));//this does not need to be an array
//				volatile short complement_course_estimate = VCLK_MAX - tick_center_point;
//
//
//				// WE ASSUME THAT dedicated_clk > 0, FOR SOME MORE POWERFUL DSP-s IT MIGHT NOT BE THE CASE
//				volatile short dedicated_clk;
//
//				if(tick_center_point > tick_variable)
//				{
//					dedicated_clk = complement_course_estimate + tick_variable - N;
//				}else{
//					dedicated_clk = tick_variable - tick_center_point - N;
//				}
////
////				// wait for fixed time after sinc center peak
////				while(dedicated_clk <= WIDTH15)	;
////				volatile short tick_fixed = vclock_counter ;
//
//				volatile short tick_fixed = tick_variable ;
//
//				volatile short special_case = -1;
//
//				if(tick_fixed == 0){
//
//					special_case = 0;
////					// wait 3 overflows
////					while(vclock_counter != 0); //wait one additional tick
//
//				}else if(tick_fixed>0 && tick_fixed<=dedicated_clk){
//
//					special_case = 1;
////					// wait 2 overflows
////					while(vclock_counter != 0); //wait one additional tick
////					while(vclock_counter != 1); //wait for ISR
////					while(vclock_counter != 0); //wait one additional tick
//
//				}else if(tick_fixed>dedicated_clk){
//
//					special_case = 0;
////					// wait 1 overflows
////					while(vclock_counter != 0); //wait one additional tick
//
//				}/*else{
//
//					state=STATE_CALCULATION;
//					ToggleDebugGPIO(STATE_CALCULATION);
//
//				}*/
//
//				//vir_clock_start = CLOCK_WRAP(complement_course_estimate - N);// start half sinc before sinc center peak
//
//				volatile short tick_complement = VCLK_MAX - tick_variable;
//				//volatile short wait_ticks = tick_complement + dedicated_clk + tick_complement;
//
//				if(!special_case)
//				{
//					volatile short wait_ticks = tick_complement + dedicated_clk + tick_complement;
//					calc_head = INDEX_WRAP(run_head + wait_ticks);
//				}else{
//					volatile short wait_ticks = tick_complement + VCLK_MAX + complement_course_estimate - N;
//					calc_head = INDEX_WRAP(run_head + wait_ticks);
//				}
//
//				if(calc_head < run_head)
//				{
//					index_history[1][myage] = (int)calc_head;
//					index_history[2][myage] = (int)run_head;
//					index_history[2][myage] = (int)(calc_head - run_head);
//					myage++;
//					if(myage==HISTORY)
//						myage = 0;
//				}
//				// two cases
//				// where to put the sinc
//				// calc_head =
//
//				//for debug only... copy zero-delayed sinc to appropriate location
//				for (i=-N;i<=N;i++)
//					MR[INDEX_WRAP(calc_head + i + N)] =  allMyDelayedWaveforms[0][i + N];
//
//				//run recalculation, call that function below
//				SetupTransmitModulatedSincPulseBufferDelayedFine(fine_delay_estimate[fde_index]);
//
//				// no transmit state anymore, when done calculating jusmp to searchiong state
//				state=STATE_SEARCHING;
//				DSK6713_LED_off(STATE_CALCULATION);
//				DSK6713_LED_on(STATE_SEARCHING);
//
////				state = STATE_TRANSMIT; //set to response for the ISR to pick the appropriate path
////				ToggleDebugGPIO(STATE_TRANSMIT);
////				while(state == STATE_TRANSMIT) ; //Loop and wait here until the responding output code works


			}
		#elif (NODE_TYPE==SLAVE_NODE)
			//Do nothing, we're the slave. All real calculations occur during the ISR
			if(state!=STATE_CALCULATION){
				//Still do nothing
			}
			else if (state==STATE_CALCULATION){
				//printf wrecks the real-time operation
				//printf("Buffer recorded: %d %f.\n",recbuf_start_clock,corrSumIncoherent);
				corrSumIncoherent = 0;  // clear correlation sum
				// -----------------------------------------------
				// this is where we estimate the time of arrival
				// -----------------------------------------------
				runReceivedPulseBufferDownmixing();
				runReceviedSincPulseTimingAnalysis();
				// --- Prepare for Response State ---

				//Now we calculate the new center clock

//				//								whole # of clock overflows + delay_estimate
//				//		NOTE: delay_estimate needs to be wraped in some cases!
//				short sinc_roundtrip_time = ((short)(sinc_launch/VCLK_MAX))*VCLK_MAX + coarse_delay_estimate[cde_index];
//				//sinc_roundtrip_time = sinc_launch;
//				vclock_offset = sinc_roundtrip_time / 2;					// divide by two

				// alternative way - portable code
				volatile short tick_variable = vclock_counter;//variable tick
				volatile short tick_center_point = CLOCK_WRAP((short)(fine_delay_estimate[fde_index]));//this does not need to be an array

				//patch for error when tick_center_point=0 once in a while
				//if(tick_center_point!=0)
				{

				volatile short sinc_roundtrip_time;

				//if(tick_center_point < tick_variable)
					sinc_roundtrip_time = (sinc_launch)*VCLK_MAX + tick_center_point - (VCLK_MAX>>1);
				//else
				//	sinc_roundtrip_time = (sinc_launch-1)*VCLK_MAX + tick_center_point ;//- (VCLK_MAX>>1);

				//if(sinc_launch==0)
				//	sinc_launch=0;


				//if(tick_variable<tick_center_point)
				//	sinc_roundtrip_time -= VCLK_MAX;

				debug_history[0][age]=tick_center_point;
				debug_history[1][age]=tick_variable;
				debug_history[2][age]=sinc_launch;
				debug_history[3][age]=sinc_roundtrip_time;
				age++;
				if(age==HISTORY)
					age=0;


				vclock_offset = sinc_roundtrip_time>>1;//divide by 2
				vclock_offset = CLOCK_WRAP(vclock_offset-1); //Actually offsets properly
				//vclock_offset = CLOCK_WRAP(vclock_offset);

				SetupTransmitModulatedSincPulseBufferDelayedFine(fine_delay_estimate[fde_index]);

				while (vclock_counter != vclock_offset) ;//wait for master zero
				vclock_counter = VCLK_MAX; //correct the vclock

				}

				while(state == STATE_CALCULATION){//wait for ISR to timeout and switch state
//					debugOutput.channel[TRANSMIT_SINC] = sinc_roundtrip_time;
//					debugOutput.channel[0] = 0;
//					MCBSP_write(DSK6713_AIC23_DATAHANDLE, debugOutput.combo);
//					debugOutput.channel[TRANSMIT_SINC] = 0;
//					//printf("recorded: %d \n",sinc_launch);
//					MCBSP_write(DSK6713_AIC23_DATAHANDLE, debugOutput.combo);
				}
				// done, after 3 vitual clock overflows, ISR will timeout and go to STATE_TRANSMITTING

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

	//run_head = INDEX_WRAP(++run_head);

	vclock_counter++; //Note! --- Not sure of the effects of moving the increment to the top
	//Clock counter wrap
	#if (NODE_TYPE==MASTER_NODE)

		run_head = INDEX_WRAP(++run_head);

		if (vclock_counter>=(VCLK_MAX)) {
			vclock_counter = 0; // wrap
			if(state == STATE_CALCULATION)
				state = STATE_TRANSMIT;
			//tempOutput.channel[TRANSMIT_CLOCK] = 32000; //Left channel for debug, doesn't really do anything
			//clk_flag = 1;
		}
		else{
			tempOutput.channel[TRANSMIT_CLOCK] = 0; //Left channel for debug, doesn't really do anything
		}

		if(vclock_counter == (4096-512))
			clk_flag = 1;

//		tempOutput.channel[TRANSMIT_SINC] = ML[INDEX_WRAP(run_head)];
//		ML[INDEX_WRAP(run_head)] = 0;	// zero the buffer after we use it
//
//		//just for debug output zero-delayed clock sinc along with master's response sinc
//		tempOutput.channel[TRANSMIT_CLOCK] = MR[INDEX_WRAP(run_head)];
//		MR[INDEX_WRAP(run_head)] = 0;	// zero the buffer after we use it

		if(clk_flag)
			runResponseClkSinc();

	#elif(NODE_TYPE==SLAVE_NODE)

		run_head = CLOCK_WRAP(++run_head);
		//run_head_sl = INDEX_WRAP(++run_head_sl);

		if (vclock_counter>=(VCLK_MAX)){ //runs at 1/2x rate of master for clock pulses, might want to switch variables?
			vclock_counter = 0;
			//tempOutput.channel[TRANSMIT_CLOCK] = 32000;
			clk_flag = 1;
			sinc_launch++;
		}
		else{
			//tempOutput.channel[TRANSMIT_CLOCK] = 0;
		}

		// dynamicly delayed clock tick (sinc)
		//tempOutput.channel[TRANSMIT_CLOCK] = SR[CLOCK_WRAP(vclock_counter)];
		//SR[CLOCK_WRAP(vclock_counter)] = 0;	// zero the clock buffer after

		// incomming sinc from slave
		//tempOutput.channel[TRANSMIT_SINC] = SL[INDEX_WRAP(run_head_sl)];

		// update sinc start virtual clock

		if (sinc_launch>=4) {//x*VCLK_MAX, x dictates the timeout, 3 should be enough
			sinc_launch = 0; //
			state=STATE_TRANSMIT;//timeout reached, no sinc reflected from master, send sinc again
			//state=STATE_SEARCHING;
			DSK6713_LED_off(STATE_CALCULATION);
			DSK6713_LED_on(STATE_TRANSMIT);
			ToggleDebugGPIO(STATE_TRANSMIT);
		}

		if(clk_flag)
			runResponseClkSinc();


	#endif
//		if(coarse_delay_estimate[cde_index] == vclock_counter)
//			//ToggleDebugGPIO(0);
//		if(0 == vclock_counter)
//			ToggleDebugGPIO(1);



	//Run all interrupt routine logic for the master node here
	#if (NODE_TYPE==MASTER_NODE)
		if (state==STATE_SEARCHING) {
			runSearchingStateCodeISR();
		}
		else if (state==STATE_RECORDING) {
			//runRecordingStateCodeISR();
			recbuf[recbufindex] = (float) tempInput.channel[RECEIVE_SINC];  // right channel
			if (abs(recbuf[recbufindex])>max_recbuf) {

				max_recbuf = abs(recbuf[recbufindex]); // keep track of largest sample

			}
			recbufindex++;
			if (recbufindex==(2*N+2*M)) {
				//CurTime = vclock_counter;
				//recbufindex--;
				state = STATE_CALCULATION;  // buffer is full (stop recording)
				DSK6713_LED_off(STATE_RECORDING);
				DSK6713_LED_on(STATE_CALCULATION);
				ToggleDebugGPIO(STATE_CALCULATION);
				//recbufindex = 0; // shouldn't be necessary
				if (max_recbuf<2048)
					playback_scale = 0;  // don't send response (signal was too weak)
				else if (max_recbuf<4096)
					playback_scale = 8;  // reply and scale by 8
				else if (max_recbuf<8192)
					playback_scale = 4;  // reply and scale by 4
				else if (max_recbuf<16384)
					playback_scale = 2;  // reply and scale by 2
				else
					playback_scale = 1;  // no scaling
			}

			//vclock_complement = VCLK_MAX - vclock_counter;


		}
		else if(state==STATE_CALCULATION){// use if instead of "else if" because when state changes we jump in the next state in within the same ISR
			//runCalculationStateCodeISR();
//			if (vclock_counter == 0){	// wait for the tick
//				amWaiting = 1;
//			}
//			if (amWaiting)
//				if (vclock_counter == vclock_complement)	// wait for complement
//				{
//					state=STATE_TRANSMIT;//
//					DSK6713_LED_off(STATE_CALCULATION);
//					DSK6713_LED_on(STATE_TRANSMIT);
//					amWaiting = 0;
//				}
			wait_count++;//on overflow state changes

		}
		else if(state==STATE_TRANSMIT){// doesn't have transmit state, in fact it always transmits but mostly zeroes
			//runResponseStateCodeISR();
			//playRecordingStateCodeISR();
			wait_count--;
			if(wait_count==0){
				state = STATE_SENDSINC;
				recbufindex=(2*N+2*M);
			}
		}else if(state==STATE_SENDSINC){
			recbufindex--;
			if (recbufindex>=0) {
				tempOutput.channel[TRANSMIT_SINC] = playback_scale*recbuf[recbufindex];
			}
			else
			{
				state = STATE_SEARCHING;  // go back to searching
			}
		}

	//Run all interrupt routines for the slave node here
	#elif (NODE_TYPE==SLAVE_NODE)

		//Control code for states and receiving stuff
		if(state==STATE_SEARCHING) {
			runSearchingStateCodeISR();
		}
		else if(state==STATE_RECORDING){
			runRecordingStateCodeISR();
		}
		else if(state==STATE_CALCULATION){
			runCalculationStateCodeISR();
		}
		else if(state==STATE_TRANSMIT){
			// transmit initial sinc to master (beg for some precision)
			// transmit tick when virtual clock is at 0.5*VCLK_MAX
			// minus N is to center the peak of the sinc at the tick

			vir_clock_start = VCLK_MAX-N-(VCLK_MAX>>1);	// subject to change

			runResponseStateCodeISR();//this function will change state when it's done

		}
		else {
			//ERROR in STATE CODE LOGIC
		}

	#endif

	local_carrier_phase = ((char) vclock_counter) & 3;

	//if(clk_flag)
	//	runResponseClkSinc();

	//Write the output sample to the audio codec
	MCBSP_write(DSK6713_AIC23_DATAHANDLE, tempOutput.combo);

}

/**
	Sets up the transmit buffer for the sinc pulse modulated at quarter sampling frequency
*/
void SetupTransmitModulatedSincPulseBuffer(){

	for (i=-N;i<=N;i++){
		x = i*BW;//(i+0.5)*BW
		t = i*0.25;//(i+0.5)*0.25
		if (i!=0)
			y = cos(2*PI*t)*(sin(PI*x)/(PI*x)); // modulated sinc pulse at carrier freq = fs/4
		else
			y = 1;								//x = 0 case.
		tModulatedSincPulse[i+N] = y*32767;
	}
}

/**
	Sets up the transmit buffer for the sinc pulse modulated at quarter sampling frequency
	Delayed by half a sample.
*/
void SetupTransmitModulatedSincPulseBufferDelayed(){

	for (i=-N;i<=N;i++){
		x = ((double)i-0.5)*BW;
		t = ((double)i-0.5)*0.25;
		//if (i!=0)
			y = cos(2*PI*t)*(sin(PI*x)/(PI*x)); // modulated sinc pulse at carrier freq = fs/4
		//else
		//	y = 1;								//x = 0 case.
		tModulatedSincPulse_delayed[i+N] = y*32767;
	}
}

/**
 * Calculates a transmission waveform to be placed into a buffer.
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
			y = 1.0;
		else {
			cosine = cos(2*PI*t);
			sine = sin(PI*x);
			denom = (PI*x);
			y = (cosine * sine / denom);
		}

			//y = cos(2*PI*t)*(sin(PI*x)/(PI*x)); // modulated sinc pulse at carrier freq = fs/4
			//y = 1.0;
		tBuffer[idx + halfBufLen] = (short)(y*32767.0);
	}
}

/**
	Sets up the transmit buffer for the sinc pulse modulated at quarter sampling frequency
	Delayed by half a sample.
*/
void SetupTransmitModulatedSincPulseBufferDelayedFine(float fineDelay){

//	if(fineDelay == 0.0){
//		for (i=-N;i<=N;i++){
//			xf = ((float)i-fineDelay)*BW;
//			tf = ((float)i-fineDelay)*0.25;
//			if (i!=0)
//				yf = cos(2*PI*tf)*(sin(PI*xf)/(PI*xf)); // modulated sinc pulse at carrier freq = fs/4
//			else
//				yf = 1;								//x = 0 case.
//				ML[INDEX_WRAP(calc_head + i + N)] = yf*32767;
//		}
//	}else{
//		for (i=-N;i<=N;i++){
//			xf = ((float)i-fineDelay)*BW;
//			tf = ((float)i-fineDelay)*0.25;
//			//if (i!=0)
//				yf = cos(2*PI*tf)*(sin(PI*xf)/(PI*xf)); // modulated sinc pulse at carrier freq = fs/4
//			//else
//			//	y = 1;								//x = 0 case.
//				ML[INDEX_WRAP(calc_head + i + N)] = yf*32767;
//		}
//	}

#if (NODE_TYPE==MASTER_NODE)
	//copy the hardcoded sinc from SDRAM into the ML buffer in IRAM
	//figure out which of the hardcoded suncs is the best approximation to the fine_delay_estimate
	// if MAXDELAY = 100, then max resolution is 0.01, and we need to round ...

	float_history[0][age] = fineDelay;

	if(fineDelay < 0)
		fineDelay *= -1.0;//make positive

	if(((int)(fineDelay*1000)%10) >= 5.0)	// there must be a smarter way to roud
		fineDelay += 0.01;

	int index = ((int)(fineDelay*100))%100;	// might need to add an offset to the fine delay ...

	index_history[0][age] = index;

	age++;
	if(age==HISTORY)
		age=0;

	if(index > 99 || index < 0)
		index = 0;//this shouldn't happen, just for debug

	for (i=-N;i<=N;i++){
		ML[INDEX_WRAP(calc_head + i + N)] =  allMyDelayedWaveforms[index][i + N];
	}
#elif (NODE_TYPE==SLAVE_NODE)
	//copy the hardcoded sinc from SDRAM into the ML buffer in IRAM
	//figure out which of the hardcoded suncs is the best approximation to the fine_delay_estimate
	// if MAXDELAY = 100, then max resolution is 0.01, and we need to round ...

	float_history[0][age] = fineDelay;

	if(fineDelay < 0)//should never be negative
		fineDelay *= -1.0;//make positive

	if(((int)(fineDelay*1000)%10) >= 5.0)	// there must be a smarter way to roud
		fineDelay += 0.01;

	int index = ((int)(fineDelay*100))%100;	// might need to add an offset to the fine delay ...

	index_history[0][age] = index;

	age++;
	if(age==HISTORY)
		age=0;

	if(index > 99 || index < 0)
		index = 0;//this shouldn't happen, just for debug

	for (i=-N;i<=N;i++){
		SR[CLOCK_WRAP(i + N)] =  allMyDelayedWaveforms[index][i + N];
	}
#endif

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

*/
//void runMasterResponseSincPulseTimingControl(){
//	// --- Prepare for Response State ---
//	union {Uint32 combo; short channel[2];} temp;
//	temp.combo = 0; //Set to zero now for missed sets.
//
//	response_done = 0; //not done yet
//	response_buf_idx = 0; //index for output buffer
//
//#if (NODE_TYPE==MASTER_NODE)
//	//Wrap start timer around virtual clock origin.
//	vir_clock_start = CLOCK_WRAP(VCLK_MAX - CLOCK_WRAP(coarse_delay_estimate[cde_index]) - N2); //start time  //cde_index-1 -> take most recent estimate
//	//course delay estimate wraps with respect to VCLK_MAX, I dont think that's good?
//
//	if(CLOCK_WRAP(coarse_delay_estimate[cde_index])>vclock_counter){//i dont think this triggers ever
//		//temp.channel[0] = 25000;
//		//MCBSP_write(DSK6713_AIC23_DATAHANDLE, temp.combo);
//
//		if(vclock_counter==0){
//			v_clk[0]=666;
//			v_clk[0]=vclock_counter;
//			while(vclock_counter != (VCLK_MAX-1)) ;
//		}else
//			while(vclock_counter != 0) ;
//	}
//
//	if(CurTime > vir_clock_start){//i dont think this triggers ever
//		//temp.channel[0] = -15000;
//		//MCBSP_write(DSK6713_AIC23_DATAHANDLE, temp.combo);
//
//		if(vclock_counter==0){
//				v_clk[0]=666;
//			v_clk[1]=vclock_counter;
//			while(vclock_counter != (VCLK_MAX-1)) ;
//			while(vclock_counter != (VCLK_MAX-1)) ;
//
//
//		}
//		else{
//			while(vclock_counter != 0); //wait one additional tick because we've already passed previous starting point we need
//			while(vclock_counter != 0); //wait one additional tick because we've already passed previous starting point we need
//		}
//	}
//
//	while(vclock_counter != 0) ; //wait one additional tick because we've already passed previous starting point we need
//
//	state = STATE_TRANSMIT; //set to response for the ISR to pick the appropriate path
//	ToggleDebugGPIO(STATE_TRANSMIT);
//
//	while(state == STATE_TRANSMIT) ; //Loop and wait here until the responding output code works
//#elif (NODE_TYPE==SLAVE_NODE)
//
//	vir_clock_start = CLOCK_WRAP(CLOCK_WRAP(coarse_delay_estimate[cde_index])+ N); //start time  //cde_index-1 -> take most recent estimate
//
//	while(vclock_counter != vir_clock_start) ;
//
//	//temp.channel[TRANSMIT_CLOCK] = -15000;
//	//MCBSP_write(DSK6713_AIC23_DATAHANDLE, temp.combo);
//
//
//
//#endif
//}


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

	if ((corrSumIncoherent>T1)&&(local_carrier_phase==0)) {  // xxx should make sure this runs in real-time
		state = STATE_RECORDING; // enter "recording" state (takes effect in next interrupt), NO it takes effect in the same ISR (if instead of elseif)
		DSK6713_LED_off(STATE_SEARCHING);
		DSK6713_LED_on(STATE_RECORDING);
		ToggleDebugGPIO(STATE_RECORDING);
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
	recbuf[recbufindex] = (float) tempInput.channel[RECEIVE_SINC];  // right channel
	recbufindex++;
	if (recbufindex>=(2*N+2*M)) {
		CurTime = vclock_counter;
		state = STATE_CALCULATION;  // buffer is full (stop recording)
		DSK6713_LED_off(STATE_RECORDING);
		DSK6713_LED_on(STATE_CALCULATION);
		ToggleDebugGPIO(STATE_CALCULATION);
		recbufindex = 0; // shouldn't be necessary
	}
}

void playRecordingStateCodeISR(){
	// put sample in recording buffer
	tempOutput.channel[TRANSMIT_SINC] = ((short) recbuf[recbufindex])<<1; // right channel
	recbufindex--;

	if (recbufindex==0) {
		//CurTime = vclock_counter;
		state = STATE_SEARCHING;  // buffer is full (stop recording)
		DSK6713_LED_off(STATE_TRANSMIT);
		DSK6713_LED_on(STATE_SEARCHING);
		ToggleDebugGPIO(STATE_SEARCHING);
		//recbufindex = 0; // shouldn't be necessary
	}
}

void runCalculationStateCodeISR(){

	//dedicated_clk++;

}


void runResponseStateCodeISR(){
	if(vclock_counter==vir_clock_start){ //Okay, we've reached the appropriate wrap around point where we should start sending the dataers
		amSending = -1;
		//sinc_launch = -1;//center outgoing tick at virtual tick
						 // start at -1 since we dont want to count the first overflow (happens right away) since it is zero-th point
	}
	if(amSending){ //write the buffered output waveform to the output file, adn increment the index counter
		tempOutput.channel[TRANSMIT_SINC] = tModulatedSincPulse[response_buf_idx];
		response_buf_idx++;
	}
	if(response_buf_idx==response_buf_idx_max){
		amSending = 0;			//quits the sending part above
		response_buf_idx = 0;
		state=STATE_SEARCHING;
		DSK6713_LED_off(STATE_TRANSMIT);
		DSK6713_LED_on(STATE_SEARCHING);
		ToggleDebugGPIO(STATE_SEARCHING);
	}
}

void runResponseClkSinc(){

	//if(even)
	//	tempOutput.channel[TRANSMIT_CLOCK] = tModulatedSincPulse[response_buf_idx_clk];
	//else	//odd
		tempOutput.channel[TRANSMIT_CLOCK] = tModulatedSincPulse[response_buf_idx_clk];

	response_buf_idx_clk++;

	if(response_buf_idx_clk==response_buf_idx_max){
		clk_flag = 0;
		response_buf_idx_clk = 0;

	}
}

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
	coarse_delay_estimate[cde_index] = CLOCK_WRAP(recbuf_start_clock+corr_max_lag);

	// fine delay estimate
	y = (double) corr_max_s;
	x = (double) corr_max_c;
	phase_correction_factor = atan2(y,x)*2*INVPI; // phase
	if(phase_correction_factor != phase_correction_factor)// if NaN
		phase_correction_factor=0;
	if(phase_correction_factor == 0)
		phase_correction_factor=0;
	r = (recbuf_start_clock+corr_max_lag) & 3; // compute remainder
	float fine = 0;
	if (r==0){
		fine_delay_estimate[fde_index] = recbuf_start_clock+corr_max_lag+phase_correction_factor;
		fine = phase_correction_factor;
	}
	else if (r==1){
		fine_delay_estimate[fde_index] = recbuf_start_clock+corr_max_lag+phase_correction_factor-1;
		fine = phase_correction_factor-1;
	}
	else if (r==2) {
		if (phase_correction_factor>0){
			fine_delay_estimate[fde_index] = recbuf_start_clock+corr_max_lag+phase_correction_factor-2;
			fine = phase_correction_factor-2;
		}
		else{
			fine_delay_estimate[fde_index] = recbuf_start_clock+corr_max_lag+phase_correction_factor+2;
			fine = phase_correction_factor+2;
		}
	}
	else if (r==3){
		fine_delay_estimate[fde_index] = recbuf_start_clock+corr_max_lag+phase_correction_factor+1;
		fine = phase_correction_factor+1;
	}
	else
		printf("ERROR");

	//fine_delay_estimate[fde_index] -= recbuf_start_clock+corr_max_lag;	// we just want the fractional part

//	if(fine > 0)
//	{
//
//	}else if(fine < 0)
//	{
//		int whole = (int)fine;
//		float frac = 1+(fine-whole);
//		fine_delay_estimate[fde_index] += whole+frac;//
//	}else
//	{
//
//	}

	// --- Calculations Finished ---
}

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

}

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

/*
float sumFloatArray(float* array, short numElmts){
	float sum = 0.0;
	short idx = 0;
	for(idx=0; idx < numElmts; idx++){
		sum += array[idx];
	}
	return sum;
}
*/

/*
	Calculates the new virtual clock times based on incoming sinc pulse timing estimates.
*/
/*
void runSlaveSincPulseTimingUpdateCalcs(){
	slaveNewVClk = coarse_delay_estimate[cde_index] / 2; //This math is probably stupidly off because I need to compensate for different total clocks sometimes... maybe?
}
*/


/**
	@name isSincInSameWindowHuh
	@returns

*/
short isSincInSameWindowHuh(short curClock, short delayEstimate){
	return curClock > delayEstimate; //If its not greater, then it has already wrapped around the 0 tick, and we are currently in the next window.
}


void ToggleDebugGPIO(short IONum){
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
	else{
		//error
	}
}