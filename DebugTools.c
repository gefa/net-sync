/**
 * @file 	DebugTools.c
 * @author 	Alex Ryan
 * @date	MAR 16, 2015
 * @brief 	Contains the debugging capable code like GPIO toggling functions
 */

#include "DebugTools.h"




GPIO_Handle hGpio; /* GPIO handle */


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

/**
 *	Toggles GPIO pins infinitely -DEBUG ONLY DO NOT CALL
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




/**	\brief Quickly toggles a set pin number high and then low again, for timing view
 *
 * @param IONum Pin number to toggle (0-3 currently)
 */
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


