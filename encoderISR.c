/*
 * encoderISR.c
 *
 *  Created on: Aug 21, 2010
 *      Author: Titus
 */

#include "encoder.h"

extern int32_t ticks[];
extern int32_t vels[];
extern int32_t pos[];

void EncoderISR(void)
{
//	int32_t dx, dy, dt;
	static double st, ct;

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

		// Reset and enable the TC
		T0TCR = TCR_CR;
		T3TCR = TCR_CR;
		T0TCR = TCR_CE;
		T3TCR = TCR_CE;
		T1TCR = TCR_CE;

//		vels[0] = (ticks[FRONT_RIGHT] + ticks[FRONT_LEFT]) * 30 / (2 * 2);  	// mult by 100/100
//		vels[1] = (ticks[FRONT_RIGHT] - ticks[FRONT_LEFT]) * 30 / (275 * 2);
//
//		pos[0] = vels[0] * cos(pos[2]/1000) + pos[0];
//		pos[1] = vels[0] * sin(pos[2]/1000) + pos[1];
//		pos[2] = vels[1] + pos[2];

//		// Drive in circle
		//trads = vels[2]/1000;
		st = myFastSin(.1);
		ct = myFastCos(.1);
//		dx = (int32_t)((vels[0] * ct - vels[1] * st) * 0.050);
//		dy = (int32_t)((vels[0] * st + vels[1] * ct) * 0.050);
//		dt = (int32_t)(vels[2] * 0.050);
//
//		pos[0] += dx;
//		pos[1] += dy;
//		pos[2] += dt;

		// Send encoder message
		//ROSSendEncOdom(pos[0], pos[1], pos[2], vels[0], vels[1], vels[2]);

		FIO0PIN ^= (1<<21);

		// Clear MR0 interrupt
		T1IR |= IR_MR0;
	}

	VICVectAddr = 0x00000000;
	ISR_EXIT();
}
