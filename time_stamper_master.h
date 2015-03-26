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

#include "ProjectDefinitions.h"

//Because
#define CHIP_6713 1

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

#define STATE_SEARCHING_TIMEOUT_LIMIT 5 				//Can look for 5 wrapped clock periods before we give up

//state variable
extern volatile short state;

extern volatile int vclock_counter;
extern volatile int virClockTransmitCenterSinc;			//center for sinc pulse according to vclock_counter
extern volatile int virClockTransmitCenterVerify;		//center for the verification pulse on the second channel

extern short tClockSincPulse[N2];						//For transmitting on main channel to synchronization between the two nodes
extern short tVerifSincPulse[N2];						//For outputting on aux channel for verification
extern short tVerifSincPulsePhased[N2];					//Phased output on aux channel for coarse estimate mean offset removal

extern short tCoarseVerifSincePulseFlag;

extern short searchingStateTimeoutCounter;



//State functions run during ISR
void runSearchingStateCodeISR();
void runRecordingStateCodeISR();
void runCalculationStateCodeISR();
//Dual transmit State functions
void runSincPulseTransmitISR();
void runVerifyPulseTransmitISR();




#endif /* TIME_STAMPER_MASTER_H_ */
