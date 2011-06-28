/**
 * @mainpage MARHES-TXT1
 *
 * @par Summary
 * The project is to create a set of robots using the Tamiya TXT-1 truck 
 * chassis. We are using the LPC2378 development board from Sparkfun for the 
 * lower level control and the Mac Mini for the higher level control. Version 
 * control for the lower level control is maintained on 
 * http://code.google.com/p/marhes-txt/. 
 * The CodeSourcery, and OpenOCD tools are used for the LPC2378. ROS on Ubuntu 
 * is used for the Mac Mini. The ROS TXT1 Drivers can be found on the 
 * marhes-ros-pkg repository.
 *
 * @par Description
 * This code uses the LPC2378, Codesourcery tools, and the OpenOCD JTAG 
 * debugger to create a lower level controller and ROS interface for the TXT1 
 * robot platform. The TXT1 is equipped with:
 *  - Wheel encoders
 *  - GPS
 *  - IMU
 *  - Camera
 *  - Laser scanner
 *  - Motor controller
 *  - Servos
 *  - Mac mini
 * The LPC2378 Board uses the following features:
 *  - 5 PWM Outputs
 *    - 1 Motor Controller
 *    - 2 Steering Servos
 *    - 2 Extra PWM Servo Outputs
 *  - 2 Quadrature Encoder Inputs
 *  - LCD Display for Debugging
 *    - Potentiometer to change display contrast
 *    - PWM Output to change display contrast
 *  - Battery monitoring ADC inputs
 *  - Left/Right Buttons to change display screen
 *  - Linear and Angular Velocity Controller
 *  - RS232 ROS Interface
 *  - RS232 Debug Interface
 *
 * @todo RS232 Debug Interface, USB and Ethernet Interfaces
 *
 * @author Titus Appel
 */ 

/**
 * @file main.c
 * 
 * @brief Starts up all the drivers and tasks so that the LPC2378 lower level
 * controller operates correctly.
 *
 * @par Hardware
 * This project uses FreeRTOS as the real time operating system. This file sets 
 * up all the hardware that is needed:
 *  - LED outputs
 *  - Oscillator
 *  - PWM
 *  - ADC
 *  - LCD
 *  - 2 Serial Ports
 *
 * @par Tasks
 * Then it starts the tasks of the RTOS, which are explained in their 
 * corresponing files:
 *  - LED Flash Task
 *  - Mode Task
 *  - Battery Task
 *  - Display Task
 *  - Button Task
 *  - Encoder Task
 *  - Controller Task
 *  - Serial Rx Task
 *
 * @author Titus Appel
 *
 * @version 1.0
 *
 * @date 2011/06/03
 *
 * Contact: titus.appel@gmail.com
 */

/*
    FreeRTOS V7.0.1 - Copyright (C) 2011 Real Time Engineers Ltd.
	

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

/* Scheduler includes. */
#include "FreeRTOS.h"

/* Demo app includes.  */
#include "flash.h"
#include "partest.h"

/* Project includes. */
#include "drivers/pwm.h"
#include "drivers/lcd.h"
#include "drivers/adc.h"
#include "serial.h"
#include "tasks/mode_task.h"
#include "tasks/battery_task.h"
#include "tasks/display_task.h"
#include "tasks/button_task.h"
#include "tasks/controller_task.h"
#include "tasks/encoder_task.h"
#include "tasks/rosiface_task.h"

/// Demo application definitions.
#define mainBASIC_WEB_STACK_SIZE        ( configMINIMAL_STACK_SIZE * 6 )

/// Task priorities.
#define mainFLASH_PRIORITY              ( tskIDLE_PRIORITY + 2 )

//@{
/// Constants to setup the PLL.
#define mainPLL_MUL			                ( ( unsigned portLONG ) ( 8 - 1 ) )
#define mainPLL_DIV			                ( ( unsigned portLONG ) 0x0000 )
#define mainCPU_CLK_DIV		              ( ( unsigned portLONG ) 0x0003 )
#define mainPLL_ENABLE		              ( ( unsigned portLONG ) 0x0001 )
#define mainPLL_CONNECT	   ( ( ( unsigned portLONG ) 0x0002 ) | mainPLL_ENABLE )
#define mainPLL_FEED_BYTE1              ( ( unsigned portLONG ) 0xaa )
#define mainPLL_FEED_BYTE2              ( ( unsigned portLONG ) 0x55 )
#define mainPLL_LOCK                    ( ( unsigned portLONG ) 0x4000000 )
#define mainPLL_CONNECTED               ( ( unsigned portLONG ) 0x2000000 )
#define mainOSC_ENABLE                  ( ( unsigned portLONG ) 0x20 )
#define mainOSC_STAT                    ( ( unsigned portLONG ) 0x40 )
#define mainOSC_SELECT                  ( ( unsigned portLONG ) 0x01 )
//@}

//@{
/// Constants to setup the MAM.
#define mainMAM_TIM_3                   ( ( unsigned portCHAR ) 0x03 )
#define mainMAM_TIM_4                   ( ( unsigned portCHAR ) 0x04 )
#define mainMAM_MODE_FULL               ( ( unsigned portCHAR ) 0x02 )
//@}

// The task that handles the uIP stack.  All TCP/IP processing is performed in
// this task.
//extern void vuIP_Task( void *pvParameters );

// Configure the hardware as required
static void prvSetupHardware( void );

/// ROS serial port handle
xComPortHandle rosPortHandle;
/// Debugging serial port handle
xComPortHandle debugPortHandle; 

/**
   @brief The main function.
   
   The entry point to the program. Sets up the hardware, starts the tasks, then 
   starts the scheduler and that runs the rest of the time. Only returns if 
   there is insufficient memory.
   @param None
   @return Returns only if insufficient memory is available to create idle task.
 */
int main( void )
{
  prvSetupHardware();                        // Setup the hardware peripherals
	
  // Create the uIP task.  This uses the lwIP RTOS abstraction layer.
  // xTaskCreate( vuIP_Task, ( signed portCHAR * ) "uIP", \
  //        mainBASIC_WEB_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 1, NULL );

  //vStartLEDFlashTasks( mainFLASH_PRIORITY ); // Flash the LEDs
  vModeTaskStart();                          // Keep track of Link/Encoder Modes
  vBatteryTaskStart();                       // Check battery status
  vDisplayTaskStart();                       // Update display
  vButtonTaskStart();                        // Read buttons to update display
  vEncoderTaskStart();                       // Get encoder readings
  vControllerTaskStart();                    // Run the velocity controller
  vRxTaskStart();                            // Receive data

  vTaskStartScheduler();                     // Start the scheduler

  // Will only get here if insufficient memory to create the idle task.
  return 0; 
}

/** 
 * @brief Sets up the hardware.
 * 
 * - Starts the clock at 48MHz with the PLL
 * - Starts the MAM (Memory accelerator module) for faster reads and writes
 * - Initialize the LED outputs
 * - Initialize the PWM outputs
 * - Initialize the ADC inputs
 * - Initialize the LCD outputs
 * - Start the ROS and Debug serial ports
 */
static void prvSetupHardware( void )
{
	#ifdef RUN_FROM_RAM
		/* Remap the interrupt vectors to RAM if we are are running from RAM. */
		SCB_MEMMAP = 2;
	#endif
	
	/* Disable the PLL. */
	PLLCON = 0;
	PLLFEED = mainPLL_FEED_BYTE1;
	PLLFEED = mainPLL_FEED_BYTE2;
	
	/* Configure clock source. */
	SCS |= mainOSC_ENABLE;
	while( !( SCS & mainOSC_STAT ) );
	CLKSRCSEL = mainOSC_SELECT; 
	
	/* Setup the PLL to multiply the XTAL input by 4. */
	PLLCFG = ( mainPLL_MUL | mainPLL_DIV );
	PLLFEED = mainPLL_FEED_BYTE1;
	PLLFEED = mainPLL_FEED_BYTE2;

	/* Turn on and wait for the PLL to lock... */
	PLLCON = mainPLL_ENABLE;
	PLLFEED = mainPLL_FEED_BYTE1;
	PLLFEED = mainPLL_FEED_BYTE2;
	CCLKCFG = mainCPU_CLK_DIV;	
	while( !( PLLSTAT & mainPLL_LOCK ) );
	
	/* Connecting the clock. */
	PLLCON = mainPLL_CONNECT;
	PLLFEED = mainPLL_FEED_BYTE1;
	PLLFEED = mainPLL_FEED_BYTE2;
	while( !( PLLSTAT & mainPLL_CONNECTED ) ); 
	
	/* 
	This code is commented out as the MAM does not work on the original revision
	LPC2368 chips.  If using Rev B chips then you can increase the speed though
	the use of the MAM.
	
	Setup and turn on the MAM.  Three cycle access is used due to the fast
	PLL used.  It is possible faster overall performance could be obtained by
	tuning the MAM and PLL settings.
	*/
	MAMCR = 0;
	MAMTIM = mainMAM_TIM_3;
	MAMCR = mainMAM_MODE_FULL;
	
	/* Setup the led's on the MCB2300 board */
	vParTestInitialise();
	PWMInit();
	ADCInit();
	LcdInit();
	rosPortHandle = xSerialPortInit( serCOM1, ser57600, serNO_PARITY, serBITS_8,
	                                  serSTOP_1, 250 );
	debugPortHandle = xSerialPortInit( serCOM2, ser57600, serNO_PARITY, serBITS_8,
	                                  serSTOP_1, 250 );
}
