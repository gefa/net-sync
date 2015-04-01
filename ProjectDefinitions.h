/*
 * ProjectDefinitions.h
 *
 *  Created on: Mar 25, 2015
 *      Author: spinlab
 */

#ifndef PROJECTDEFINITIONS_H_
#define PROJECTDEFINITIONS_H_



//Board node definitions
#define MASTER_NODE 1
#define SLAVE_NODE 	2

//Node type - This changes whether setting
#define NODE_TYPE SLAVE_NODE

//If use floating point fixes
#define USE_FDE 1

//Audio codec sample frequency
#define DSK_SAMPLE_FREQ DSK6713_AIC23_FREQ_8KHZ

#define VCLK_MAX 8000
#define CLOCK_WARP(i) ((i)&(VCLK_MAX-1))

#define MAX_STORED_DELAYS_COARSE 16
#define MAX_STORED_DELAYS_FINE 16

#define BW 0.0125 	//100Hz@8k Fs baseband sinc frequency
#define CBW 0.25 	//2kHz@8k Fs  carrier frequency


// 2*N+1 is the number of samples in the sinc function
#define N (1<<9) //512
#define N2 ((N*2)+1) //1025




#endif /* PROJECTDEFINITIONS_H_ */
