/*! \file encoderISR.c
    \brief Encoder ISR.

    Created on: Aug 21, 2010\n
	Author: Titus Appel\n
	Institution: UNM-ECE Department

   	This file includes the Timer1 ISR and is compiled in ARM only mode.

    Pin definition:\n
    	P3.23 - CAP0.0 - Front Right Encoder Signal\n
     	P0.23 - CAP3.0 - Front Left Encoder Signal
 */

#include "encoder.h"

extern int32_t ticks[];
extern int32_t vels[];
extern int32_t pos[];

/*! \brief Samples the encoder counts and calculates velocities and positions.

	This is the ISR for Timer1.  It stops the counter, stores the encoder
	counts, and resets and enables the counters.  Then it calculates the linear
	and angular velocities and the position of the TXT.  It then sends the data
	to the ROS driver.
 */
void EncoderISR(void)
{
	ISR_ENTRY();

	// Check for MR0 Interrupt
	if(T1IR | IR_MR0)
	{
		// Stop Counters
		T0TCR = 0;
		T3TCR = 0;

		// Store the right and left encoder counts
		ticks[FRONT_RIGHT] = T0TC;
		ticks[FRONT_LEFT] = T3TC;

		// Reset and enable the TC
		T0TCR = TCR_CR;
		T3TCR = TCR_CR;
		T0TCR = TCR_CE;
		T3TCR = TCR_CE;
		T1TCR = TCR_CE;

//		vels[0] = (ticks[FRONT_RIGHT] + ticks[FRONT_LEFT]) * 30 / (2 * 2);
//		vels[1] = (ticks[FRONT_RIGHT] - ticks[FRONT_LEFT]) * 30 / (275 * 2);
//
//		pos[0] = vels[0] * cos(pos[2]/1000) + pos[0];
//		pos[1] = vels[0] * sin(pos[2]/1000) + pos[1];
//		pos[2] = vels[1] + pos[2];

		// Drive in circle
		vels[0] = 1000;		// Linear Velocity is 1000mm/s
		vels[1] = 100;		// Angular Velocity is 100mrad/s
		pos[0] += (int32_t)(vels[0] * 20 / 1000);    	// mm/s * 20ms / 1000
		pos[1] += (int32_t)(vels[0] * pos[2] * 20 / 1000000);// mm/s * 1/1000m * 20ms / 1000
		pos[2] += (int32_t)(vels[1] * 20 / 1000);		// mrad/s * 20ms / 1000

		// Send encoder message
		//ROSSendEncOdom(pos[0], pos[1], pos[2], vels[0], vels[1], vels[2]);
		//ROSSendEncOdom(1,1,1,1,1,1);

		FIO0PIN ^= (1<<21);

		// Clear MR0 interrupt
		T1IR |= IR_MR0;
	}

	VICVectAddr = 0x00000000;
	ISR_EXIT();
}
