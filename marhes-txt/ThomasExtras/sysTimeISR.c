/******************************************************************************
 *
 * $RCSfile: $
 * $Revision: $
 *
 * This module implements the ISR for the System-Timer (LPC Timer0).
 * Copyright 2007, Martin Thomas
 * No guarantees, warrantees, or promises, implied or otherwise.
 * May be used for hobby or commercial purposes provided copyright
 * notice remains intact.
 *
 *****************************************************************************/
#include <stdint.h>
#include <stdlib.h> /* NULL */
#include "LPC23xx.h"
#include "sysTime.h"
#include "sysTimeISR.h"
#include "armVIC.h"

#if defined(SYSTIME_INT_MODE)
/******************************************************************************
 *
 * Function Name: systimeISR()
 *
 * Description:
 *    This function implements the ISR for Timer0.
 *
 * Calling Sequence: 
 *    void
 *
 * Returns:
 *    void
 *
 *****************************************************************************/
void sysTimeISR(void)
{
	ISR_ENTRY();
	
	uint8_t i;
	uint32_t cnt;
	// perform proper ISR entry so thumb-interwork works properly
	
	for ( i=0; i<SYSTIME_MAX_CALLBACKS; i++ ) {
		if ( gCallbackFunctions[i] != NULL ) {
			cnt = gCallbackCounters[i];
			cnt++;
			if ( cnt == gCallbackRates[i] ) {
				gCallbackFunctions[i]();
				cnt = 0;
			}
			gCallbackCounters[i] = cnt;
		}
	}
	
	T0MR0 += SYSTIME_INT_DT - 1; // next match-value
	T0IR = TxIR_MR0_Interrupt;   // clear interrupt by writing an IR-bit
	
	VICVectAddr = 0x00000000;             // clear this interrupt from the VIC
	ISR_EXIT();                           // recover registers and return
}
#endif // defined(SYSTIME_INT_MODE)
