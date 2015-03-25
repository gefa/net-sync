/**
 * @file 	time_stamper_master.h
 * @author 	Alex Ryan
 * @date	MAR 16, 2015
 * @brief 	header for the main file "time_stamper_master"
 *
 *	Allows extern declaration of variables for the math and debug functions if neccesary to view them
 *
 */

#ifndef TIME_STAMPER_MASTER_H_
#define TIME_STAMPER_MASTER_H_

//Because
#define CHIP_6713 1

//Board node definitions
#define MASTER_NODE 1
#define SLAVE_NODE 	2

//Node type - This changes whether setting
#define NODE_TYPE SLAVE_NODE

//If use floating point fixes
#define USE_FDE 0

//Define channel numbers for audio codec union variable
#define CHANNEL_LEFT 0
#define CHANNEL_RIGHT 1

//Define master/slave channels
#define TRANSMIT_SINC	CHANNEL_LEFT	//Channel along which the transmission sinc pulse is sent
#define RECEIVE_SINC	CHANNEL_LEFT	//Corresponding receive channel on the other end
#define TRANSMIT_CLOCK	CHANNEL_RIGHT	//Verification clock output that centers along the zero-point of the vclock

//Audio codec sample frequency
#define DSK_SAMPLE_FREQ DSK6713_AIC23_FREQ_8KHZ

// State definitions
#define STATE_SEARCHING 0
#define STATE_RECORDING 1
#define STATE_CALCULATION 2
#define STATE_TRANSMIT 3

#define STATE_SEARCHING_TIMEOUT_LIMIT 5 //Can look for 5 wrapped clock periods before we give up

// virtual clock counter maximum
#define LARGE_VCLK_MAX (1<<31) //about two billion (2e9)
#define SMALL_VCLK_MAX (1<<12) //4096 counts

#define LARGE_VCLK_WRAP(i) ((i)&(LARGE_VCLK_MAX-1))
#define SMALL_VCLK_WRAP(i) ((i)&(SMALL_VCLK_MAX-1))


#define MAX_STORED_DELAYS_COARSE 16
#define MAX_STORED_DELAYS_FINE 16

extern int coarse_delay_estimate[MAX_STORED_DELAYS_COARSE];
extern float fine_delay_estimate[MAX_STORED_DELAYS_FINE];
extern short cde_index;
extern short fde_index;

//state variable
extern volatile int state;

extern volatile int vclock_counter;
extern volatile int virClockTransmitCenterSinc;			//center for sinc pulse according to vclock_counter
extern volatile int virClockTransmitCenterVerify;		//center for the verification pulse on the second channel





//State functions run during ISR
void runSearchingStateCodeISR();
void runRecordingStateCodeISR();
void runCalculationStateCodeISR();
//Dual transmit State functions
void runSincPulseTransmitISR();
void runVerifyPulseTransmitISR();

//index control funcs
short GetVerifPulseIndex();
short GetSincPulseIndex();



#endif /* TIME_STAMPER_MASTER_H_ */
