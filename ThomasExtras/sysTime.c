/******************************************************************************
 *
 * $RCSfile: $
 * $Revision: $
 *
 * This module provides the interface routines for initializing and
 * accessing the system timing functions.
 * Copyright 2004, R O SoftWare
 * No guarantees, warrantees, or promises, implied or otherwise.
 * May be used for hobby or commercial purposes provided copyright
 * notice remains intact.
 *
 *****************************************************************************/

// ISR-init and callback-handling added by Martin Thomas
// callback-handling as in the Atmel AVR Butterfly Demo-Application

#include <stdint.h>
#include <stdlib.h> /* for NULL */
#include "LPC23xx.h"
#include "sysTime.h"

#if defined(SYSTIME_INT_MODE)
#include "sysTimeISR.h"

volatile Timer_Callback_Function gCallbackFunctions[SYSTIME_MAX_CALLBACKS];
volatile uint32_t gCallbackRates[SYSTIME_MAX_CALLBACKS];
volatile uint32_t gCallbackCounters[SYSTIME_MAX_CALLBACKS];
#endif

static uint32_t sysTICs;
static uint32_t lastT0TC;

/******************************************************************************
 *
 * Function Name: initSysTime()
 *
 * Description:
 *    This function initializes the LPC's Timer 0 for use as the system timer.
 *
 * Calling Sequence: 
 *    void
 *
 * Returns:
 *    void
 *
 *****************************************************************************/
void initSysTime(void)
{
#if defined(SYSTIME_INT_MODE)
	uint8_t i;
#endif
	
	T0TCR = TxTCR_Counter_Reset;           // reset & disable timer 0
	// setup Timer 1 to count forever
	T0PR = T0_PCLK_DIV - 1;               // set the prescale divider
	T0CCR = 0;                            // disable compare registers
	T0EMR = 0;                            // disable external match register
#if defined(SYSTIME_INT_MODE)
	for ( i=0; i<SYSTIME_MAX_CALLBACKS; i++ ) {
	  gCallbackFunctions[i] = NULL;
	  gCallbackRates[i]     = 0;
	  gCallbackCounters[i]  = 0;
	}
	VICIntSelect &= ~(VIC_CHAN_TO_MASK(VIC_CHAN_NUM_Timer0));
	VICIntEnClr  = VIC_CHAN_TO_MASK(VIC_CHAN_NUM_Timer0);
	VICVectAddr4 = (uint32_t)sysTimeISR;
	VICVectPriority4 = 0x01;
	VICIntEnable = VIC_CHAN_TO_MASK(VIC_CHAN_NUM_Timer0);
	T0MR0 = T0TC + SYSTIME_INT_DT - 1;
	T0MCR = TxMCR_MR0I;        // interrupt on cmp-match0
	T0IR  = TxIR_MR0_Interrupt; // clear match0 interrupt
#else
	T0MCR = 0;                            // disable match registers
#endif
	T0TCR = TxTCR_Counter_Enable;         // enable timer 0
	sysTICs = 0;
}

/******************************************************************************
 *
 * Function Name: getSysTICs()
 *
 * Description:
 *    This function returns the current syetem time in TICs.
 *
 * Calling Sequence: 
 *    void
 *
 * Returns:
 *    The current time in TICs as represented by sysTICs
 *
 *****************************************************************************/
uint32_t getSysTICs(void)
{
  uint32_t now = T0TC;

  sysTICs += (uint32_t)(now - lastT0TC);
  lastT0TC = now;
  return sysTICs;
}


/******************************************************************************
 *
 * Function Name: getElapsedSysTICs()
 *
 * Description:
 *    This function then returns the difference in TICs between the
 *    given starting time and the current system time.
 *
 * Calling Sequence: 
 *    The starting time.
 *
 * Returns:
 *    The time difference.
 *
 *****************************************************************************/
uint32_t getElapsedSysTICs(uint32_t startTime)
{
  return getSysTICs() - startTime;
}


/******************************************************************************
 *
 * Function Name: pause()
 *
 * Description:
 *    This function does not return until the specified 'duration' in
 *    TICs has elapsed.
 *
 * Calling Sequence: 
 *    duration - length of time in TICs to wait before returning
 *
 * Returns:
 *    void
 *
 *****************************************************************************/
void pause(uint32_t duration)
{
  uint32_t startTime = getSysTICs();

  while (getElapsedSysTICs(startTime) < duration) {
    // WDOG();
  }
}

#if defined(SYSTIME_INT_MODE)
/******************************************************************************
 *
 * Function Name: systimeRegisterCallback
 *
 * Description:
 *    Registers a callback-functions which gets called by
 *    the systime ISR (see systimeISR.c)
 *
 * Parameter(s):
 *    function-pointer
 *    call-back rate (prescaler)
 *
 * Returns:
 *    2 if function has already been registered
 *    1 on firs successful registration
 *    0 if registration failed (all slots used)
 *
 * added by Martin Thomas
 *****************************************************************************/
uint8_t systimeRegisterCallback(Timer_Callback_Function pFunc, uint32_t rate )
{
    uint8_t i;
    
    for ( i=0; i<SYSTIME_MAX_CALLBACKS; i++ ) {
        if ( gCallbackFunctions[i] == pFunc ) {
            return 2; // already registered
        }
    }
    
    for ( i=0; i<SYSTIME_MAX_CALLBACKS; i++ ) {
        if ( gCallbackFunctions[i] == NULL ) {
			gCallbackRates[i]     = rate;
			gCallbackCounters[i]  = 0;
			gCallbackFunctions[i] = pFunc;
            return 1;
        }   
    }
    
    return 0;
}

/******************************************************************************
 *
 * Function Name: systimeRemoveCallback
 *
 * Description:
 *    Removes a callback-functions
 *
 * Parameter(s):
 *    function-pointer
 *
 * Returns:
 *    1 on success 
 *    0 if function has not been registered
 *
 * added by Martin Thomas
 *****************************************************************************/
uint8_t systimeRemoveCallback(Timer_Callback_Function pFunc)
{
    uint8_t i;
    
    for ( i=0; i<SYSTIME_MAX_CALLBACKS; i++ ) {
        if ( gCallbackFunctions[i] == pFunc ) {
            gCallbackFunctions[i] = NULL;
            return 1;
        }
    }
        
    return 0;
}
#endif // SYSTIME_INT_MODE

