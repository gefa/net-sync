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

short delayIndex = 0;
#define delayMax 6

float fineDelayIndex = 0.0;
#define fineDelayMax 8.0

//Output waveform buffers for clock and sync channels
short standardWaveformBuffer[N2];
short delayedWaveformBuffer[N2];

short allMyDelayedWaveforms[delayMax][N2];


float runningDelay = 0.0;

volatile int state = 0;

// Handles
DSK6713_AIC23_CodecHandle hCodec;							// Codec handle
DSK6713_AIC23_Config config = DSK6713_AIC23_DEFAULTCONFIG;  // Codec configuration with default settings


// ------------------------------------------
// end of variables
// ------------------------------------------


interrupt void serialPortRcvISR(void);

void main()
{

	setupTransmitBuffer(standardWaveformBuffer, N, BW, CBW, 0.0);
	setupTransmitBuffer(delayedWaveformBuffer, N, BW, CBW, 0.0);

	//For delayed buffer approach
	int idx;
	for(idx = 0; idx < delayMax; idx++){
		ToggleDebugGPIO(0);
		setupTransmitBuffer(&(allMyDelayedWaveforms[idx][0]), N, BW, CBW, ((float) idx) / delayMax);
		ToggleDebugGPIO(0);
	}

	setupTrigLookupTables(sin_lookup_table, trigLookupSize);



	// -------- DSK Hardware Setup --------
	DSK6713_init();		// Initialize the board support library, must be called first
	DSK6713_LED_init(); // initialize LEDs

	//Setup GPIO for possible debug
	gpioInit();

	//test
/*
	float del;

*/

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
		//Not working loop
		if(state == 1){
			ToggleDebugGPIO(0);
			fineDelayIndex += 0.2501;
			if (fineDelayIndex >= fineDelayMax)
				fineDelayIndex = 0.0;

			setupTransmitBufferTest(delayedWaveformBuffer, N, BW, CBW, fineDelayIndex);


			while(state == 1); //wait
			ToggleDebugGPIO(0);
		}
		/*
		//Working loop
		if(state == 1){ //in the section between end of buffer and next buffer start
			ToggleDebugGPIO(0);
			delayIndex += 1;
			if (delayIndex >= delayMax)
				delayIndex = 0;

			short* sourceBuffer = &(allMyDelayedWaveforms[delayIndex][0]); //INITIALIZE THIS!
			copyTransmitBuffer(sourceBuffer, delayedWaveformBuffer, N2);
			while(state == 1); //wait
			ToggleDebugGPIO(0);
		}
		*/
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
	union {uint32_t combo; short channel[2];} tempOutput;
	union {uint32_t combo; short channel[2];} tempInput;

	tempInput.combo = MCBSP_read(DSK6713_AIC23_DATAHANDLE);
	tempOutput.combo = 0; //Set to zero now for missed sets.
	// Note that right channel is in temp.channel[0]
	// Note that left channel is in temp.channel[1]

	//Clock counter wrap
	vclock_counter++; //Note! --- Not sure of the effects of moving the increment to the top
	//Code should be common to both
	if(vclock_counter >= VCLK_MAX){ //this will likely never happen
		vclock_counter = 0;
	}

	if (vclock_counter == 1025)
		state = 1; //update waveforms
	else
		state = 0;

	//Change from zero output to waveform
	if(vclock_counter <= N2-1){
		tempOutput.channel[CHANNEL_RIGHT] = standardWaveformBuffer[vclock_counter];
		tempOutput.channel[CHANNEL_LEFT] = delayedWaveformBuffer[vclock_counter];
	}


	//Write the output sample to the audio codec
	MCBSP_write(DSK6713_AIC23_DATAHANDLE, tempOutput.combo);

}








