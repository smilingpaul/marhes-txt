/*! \file encoderISR.c
    \brief Encoder ISR.

    \date Aug 21, 2010\n
	\author Titus Appel <titus.appel@gmail.com>\n
	\par Institution:
	UNM-ECE Department\n
	MARHES Lab

   	This file includes the Timer1 ISR and is compiled in ARM only mode.

    Pin definition:\n
    	P3.23 - CAP0.0 - Front Right Encoder Signal\n
     	P0.23 - CAP3.0 - Front Left Encoder Signal
 */

#include "encoder.h"

extern int32_t ticks[];
extern int32_t vels[];
extern int32_t pos[];

int32_t angtable[91]={0, 175, 349, 523, 698,872,1045,1219,1392,1564,1736,
		1908,2079,2250,2419,2588,2756,2924,3090,3256,3420,3584,3746,3907,4067,
		4226,4384,4540,4695,4848,5000,5150,5299,5446,5592,5736,5878,6018,6157,
		6293,6428,6561,6691,6820,6947,7071,7193,7314,7431,7547,7660,7771,7880,
		7986,8090,8192,8290,8387,8480,8572,8660,8746,8829,8910,8988,9063,9135,
		9205,9272,9336,9397,9455,9511,9563,9613,9659,9703,9744,9781,9816,9848,
		9877,9903,9925,9945,9962,9976,9986,9994,9998,10000};

/*! \brief Samples the encoder counts and calculates velocities and positions.

	This is the ISR for Timer1.  It stops the counter, stores the encoder
	counts, and resets and enables the counters.  Then it calculates the linear
	and angular velocities and the position of the TXT.  It then sends the data
	to the ROS driver.
 */
void EncoderISR(void)
{
//	ISR_ENTRY();
	int32_t dx, dy, dth;

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

		vels[0] = 100;
		vels[1] = 100;
		dx = vels[0] * anglookuptable(pos[2], 1) / 10000 * 20 / 1000;
		dy = vels[0] * anglookuptable(pos[2], 0) / 10000 * 20 / 1000;
		dth = vels[1] * 20 / 1000;
		pos[0] += dx;
		pos[1] += dy;
		pos[2] += dth;

		if (pos[2] > 6283)
			pos[2] -= 6283;

		if (pos[2] < 0)
			pos[2] += 6283;

		// Send encoder message
		ROSSendEncOdom(pos[0], pos[1], pos[2], vels[0], vels[1]);

		FIO0PIN ^= (1<<21);

		// Clear MR0 interrupt
		T1IR |= IR_MR0;
	}

	VICVectAddr = 0x00000000;
//	ISR_EXIT();
}

//trig lookup table, type 0 for cos, 1 for sin, degrees is from 0->360
int32_t anglookuptable(int32_t radians, int32_t type)

{
	int32_t c = 1;
	int32_t s = 1;

	int32_t i = radians * 180 / 3141; //includes 0 to 90 degrees

	while (i > 360)
		i -= 360;

	while (i < 0)
		i += 360;

	if (i > 90 && i <= 180) //between 91 and 180
	{
		i = 180 - i;
		c = -1;
	}

	if (i > 180 && i <= 270) //between 181 and 270
	{
		i = i - 180;
		c = -1;
		s = -1;
	}

	if (i > 270 && i <= 360) //between 271 and 360
	{
		i = 360 - i;
		s = -1;
	}

	if (type == 1) //cosine
	{
		c = c * angtable[90 - i];
		return c;
	}
	else //sine
	{
		s = s * angtable[i];
		return s;
	}
}
