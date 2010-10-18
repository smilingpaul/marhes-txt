/******************************************************************************
 *
 * main.c - Modified from Martin Thomas's code.
 * Date: Aug 23, 2010
 * Author: Titus Appel
 *
 * Summary
 * -------------
 * The main method to the TXT-1 Robot's lower level controller.
 * This lower level controller accepts linear and angular velocities from the
 * higher level controller.  The higher level controller is developed on the
 * ROS platform.  The lower level controller calculates the PID loop values to
 * acheive the desired velocities.  The inputs to the control system are the
 * encoder values, GPS, IMU, and VICON GPS. The GPS's are not used all the time
 * since the TXT-1 might not be in the lab or outside for GPS signal lock.  The
 * GPS's and the IMU are connected to the higher level controller and polled
 * through ROS.  The encoders are polled through the microcontroller board. The
 * source files are maintained through a mercurcial repository at:
 * https://marhes-txt.googlecode.com/hg/
 * For eclipse project configuration information visit:
 * http://sites.google.com/site/titusprojectpage/home/thesis/project-configuration
 *
 * More comments to come.
 *
 * Below are comments from the original file by Martin Thomas
 * ------------------------------------------------------------------------
 * base code from: Bill Knight, R O SoftWare <BillK@rosw.com>
 *
 * Extended and adapted to NXP LPC23xx/24xx by Martin Thomas
 * <eversmith(at)heizung-thomas(dot)de>
 * - Interrupt-Init and handling (UARTs and TIMER0)
 * - extended "systime" with callbacks
 * - adapted register-definitions to LPC23xx.h (included)
 * - clock-setups for LPC23xx/24xx (PLL, Core, PCLK etc.)
 * - removed GPIO init, use FIO
 *
 * ----------------------------------------------------------------------------
 *
 * Toolchain: Yagarto arm-none-eabi
 * Target   : NXP LPC2378 on Olimex LPC-2378-STK
 *
 * Copyright 2004, R O SoftWare
 * Copyright 2007, Martin Thomas
 * No guarantees, warrantees, or promises, implied or otherwise.
 * May be used for hobby or commercial purposes provided copyright
 * notice remains intact.
 *
 *****************************************************************************/

#define VERSION "0.8 mthomas 5/2007"
 
// Includes
#include "app_types.h"
#include "LPC23xx.h"
#include "armVIC.h"

#include "pwm.h"
#include "uart0.h"
#include "encoder.h"
#include "controller.h"
#include "lcd.h"
#include "display.h"
#include "button.h"
#include "ROSIFace.h"
#include "helperFuncs.h"

/*************************************************************************
 *             Definitions
 *************************************************************************/
// Defines to reset the interrupt vector addresses for SysInit
#define VECT_ADDR_INDEX	0x100
#define VECT_CNTL_INDEX 0x200
#define VECT_PRIO_INDEX 0x200

/*************************************************************************
 *             Function declarations
 *************************************************************************/
void InitMAM(void);
void InitClock(void);
void sysInit(void);

/******************************************************************************
 *
 * Function Name: main()
 *
 * Description:
 *    This function is the program entry point.  After initializing the
 *    system, it does some testing and then starts the low level PID controller.
 *
 * Calling Sequence:
 *    void
 *
 * Returns:
 *    void
 *
 *****************************************************************************/
int main(void)
{
	// Variable initialization
    unsigned char UpdateDisplay = 0;
    unsigned char buttonState;
	int i;

	// Wait a little
	for(i = 0; i < 100000; i++);

	// Initialize the system and turn on interrupts
	sysInit();
	enableIRQ();
	
	// Enter infinite while loop
	while(1)
	{    
	    ROSProcessPacket();
	    ControllerCalcPID();
	    
	    buttonState = ButtonGetChangedHigh();       
	    if(buttonState & BUT_CENTER_BIT)
	    {
	    	UpdateDisplay ^= 1;
	    	LcdBacklight(UpdateDisplay);
	    }
	    
	    if(UpdateDisplay)
	    {
	        if (buttonState & BUT_RIGHT_BIT)
                DisplaySetState(DisplayGetState() + 1);
            if (buttonState & BUT_LEFT_BIT)
                DisplaySetState(DisplayGetState() - 1);
            if (i > 50000)
            { 
                DisplayUpdate();
                i = 0;
            }
            else
            {
                i++;
            }
        }
	}
	return 0;
}

/******************************************************************************
 *
 * Function Name: sysInit()
 *
 * Description:
 *    This function is responsible for initializing the program
 *    specific hardware
 *
 * Calling Sequence:
 *    void
 *
 * Returns:
 *    void
 *
 *****************************************************************************/
void sysInit(void)
{
	// Initialize variables to clear interrupt vector addresses
	uint32_t i = 0;
	volatile uint32_t *vect_addr, *vect_prio;

	InitClock(); 						// Set clock to 72MHz
	InitMAM();							// Init the memory acceleration module
	MEMMAP = 1;			                // map interrupt vectors space into FLASH

	PCLKSEL0 = 0x55555555;	/* PCLK is the same as CCLK */
	PCLKSEL1 = 0x55555555;

	/* initialize VIC */
	VICIntEnClr  = 0xffffffff;
	VICVectAddr  = 0x00000000;
	VICIntSelect = 0x00000000; /* all IRQ */

	/* set all the vector and vector control register to 0 */
	for ( i = 0; i < 32; i++ ) {
		vect_addr = (uint32_t *)(VIC_BASE_ADDR + VECT_ADDR_INDEX + i*4);
		vect_prio = (uint32_t *)(VIC_BASE_ADDR + VECT_PRIO_INDEX + i*4);
		*vect_addr = 0x00000000;
		*vect_prio = 0x0000000F;
	}

	SCS |= (1UL<<0); 						// set GPIOM in SCS for fast IO
	FIO0DIR |= (1<<21); 					// LED-Pin as output for heartbeat

	// Call initialization functions of necessary peripherals
	PWMInit();
	Uart0Init();
	LcdInit();
	EncoderInit();
	ControllerInit();
	DisplayInit();
	ButtonInit();
}

/*************************************************************************
 * Function Name: InitMAM
 * Parameters: void
 * Return: void
 *
 * Description: Initialize the memory acceleration module to the correct
 *              settings for a clock of greater than 60 MHz.  Goto page
 *              103 of user manual for explanation.
 *
 *************************************************************************/
void InitMAM(void)
{
	MAMCR = 0;				// disable MAM
	MAMTIM = 4;    			// FCLK > 60 MHz
	MAMCR = 2;   			// MAM functions fully enabled
}

/*************************************************************************
 * Function Name: InitClock
 * Parameters: void
 * Return: void
 *
 * Description: Initialize PLL and clocks' dividers. Pclk - 288MHz,
 * Cclk- 72MHz, Usbclk - 48MHz
 *
 *************************************************************************/
void InitClock(void)
{
	// 1. If PLL is connected, disconnect it.
	if (PLLSTAT & (1<<25))
	{
		PLLCON = 1;		// Enable and disconnect PLL then write feed sequence
		PLLFEED = 0xAA;
		PLLFEED = 0x55;
	}

	// 2. Disable the PLL by clearing the PLL enable bit and then writing a
	//    feed sequence to load the bit
	PLLCON = 0;
	PLLFEED = 0xAA;
	PLLFEED = 0x55;

	// 3. Init OSC enabling the main oscillator
	SCS |= (1<<5);					// OSCEN = 1
	while (!(SCS & (1<<6)));		// Wait for OSC to be ready

	// 4. Select source clock for PLL.  Selects the main oscillator as the PLL
	//    clock source.
	CLKSRCSEL = 1;

	// 5. Set PLL settings 288 MHz, M=12, N=1
	//    M is bits 14:0, N is bits 23:16
	PLLCFG = ((12-1)<<0) | ((1-1)<<16);
	PLLFEED = 0xAA;
	PLLFEED = 0x55;

	// 6. Enable PLL
	PLLCON = 1;
	PLLFEED = 0xAA;
	PLLFEED = 0x55;

	// 7. Set clk divider settings - Fpll - 288 MHz
	CCLKCFG   = 4-1;        		// 1/4 Fpll - 72 MHz
	USBCLKCFG = 6-1;            	// 1/6 Fpll - 48 MHz

	// 8. Wait for the PLL to achieve lock
	while ((PLLSTAT & (1<<26)) == 0);

	// 9. Check if the readback of PLLSTAT is correct
	unsigned int readback = (PLLSTAT & 0x00FF7FFF);
	while (readback != (((12-1)<<0) | ((1-1)<<16)));

	// 10. Connect the PLL
	PLLCON = 3;						// Use equals because PLLCON isn't the real register
	PLLFEED = 0xAA;
	PLLFEED = 0x55;

	// 11. Wait for connect
	while ((PLLSTAT & (1<<1)) == 0);
}
