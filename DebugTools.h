/**
 * @file 	DebugTools.h
 * @author 	Alex Ryan
 * @date	MAR 16, 2015
 * @brief 	Header for debugging code
 */

#ifndef DEBUGTOOLS_H_
#define DEBUGTOOLS_H_

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


extern GPIO_Handle hGpio; /* GPIO handle */


void ToggleDebugGPIO(short IONum);
void gpioInit();
void gpioToggle();


#endif /* DEBUGTOOLS_H_ */
