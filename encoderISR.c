/*
 * encoderISR.c
 *
 *  Created on: Aug 21, 2010
 *      Author: Titus
 */

#include "encoder.h"

extern uint32_t ticks[];
extern float vels[SIZE_ENCODER_VEL_ARR];

void EncoderISR(void)
{
	ISR_ENTRY();

	// Check for MR0 Interrupt
	if(T1IR | IR_MR0)
	{
		// Stop Counters
		T0TCR = 0;
		T3TCR = 0;

		// Store the right and left encoder counts and reset and enable the TC
		ticks[FRONT_RIGHT] = T0TC;
		ticks[FRONT_LEFT] = T3TC;

		//vels[0] = (ticks[FRONT_RIGHT] + ticks[FRONT_LEFT]) * 0.3 / (2 * 0.020);
		//vels[1] = (ticks[FRONT_RIGHT] - ticks[FRONT_LEFT]) * 0.3 / (275 * 0.020);

		// Reset and enable the TC
		T0TCR = TCR_CR;
		T3TCR = TCR_CR;
		T0TCR = TCR_CE;
		T3TCR = TCR_CE;
		T1TCR = TCR_CE;

		// Clear MR0 interrupt
		T1IR |= IR_MR0;
	}

	VICVectAddr = 0x00000000;
	ISR_EXIT();
}
