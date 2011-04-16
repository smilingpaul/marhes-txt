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
extern float pos[];
extern int32_t intCount;

//int32_t angtable[91]={0, 175, 349, 523, 698,872,1045,1219,1392,1564,1736,
//		1908,2079,2250,2419,2588,2756,2924,3090,3256,3420,3584,3746,3907,4067,
//		4226,4384,4540,4695,4848,5000,5150,5299,5446,5592,5736,5878,6018,6157,
//		6293,6428,6561,6691,6820,6947,7071,7193,7314,7431,7547,7660,7771,7880,
//		7986,8090,8192,8290,8387,8480,8572,8660,8746,8829,8910,8988,9063,9135,
//		9205,9272,9336,9397,9455,9511,9563,9613,9659,9703,9744,9781,9816,9848,
//		9877,9903,9925,9945,9962,9976,9986,9994,9998,10000};

int32_t angtable[91]={0,18,35,52,70,87,105,122,139,156,174,191,208,225,242,259,
		276,292,309,326,342,358,375,391,407,423,438,454,470,485,500,515,530,545,
		559,574,588,602,616,629,643,656,669,682,695,707,719,731,743,755,766,777,
		788,799,809,819,829,839,848,857,866,875,883,891,899,906,914,921,927,934,
		940,946,951,956,961,966,970,974,978,982,985,988,990,993,995,996,998,999,
		999,1000,1000};

/*! \brief Samples the encoder counts and calculates velocities and positions.

	This is the ISR for Timer1.  It stops the counter, stores the encoder
	counts, and resets and enables the counters.  Then it calculates the linear
	and angular velocities and the position of the TXT.  It then sends the data
	to the ROS driver.
 */
void EncoderISR(void)
{
	float dx, dy, dth;
	int32_t vel1, vel2;
	// Check for MR0 Interrupt
	if(T1IR | IR_MR0)
	{
		// Stop Counters
		T0TCR = 0;
		T3TCR = 0;

		// Store the right and left encoder counts, get rid of noise
		if (T0TC > 2)
			ticks[FRONT_LEFT] = T0TC;
		else
			ticks[FRONT_LEFT] = 0;
		if (T3TC > 2)
			ticks[FRONT_RIGHT] = T3TC;
		else
			ticks[FRONT_RIGHT] = 0;

		// Reset and enable the TC
		T0TCR = TCR_CR;
		T3TCR = TCR_CR;
		T0TCR = TCR_CE;
		T3TCR = TCR_CE;
		T1TCR = TCR_CE;

//		vels[0] = (EncoderGetDirection(FRONT_RIGHT) * ticks[FRONT_RIGHT] + \
//						EncoderGetDirection(FRONT_LEFT) * ticks[FRONT_LEFT]) * 15 / 2;
//		vels[1] = (EncoderGetDirection(FRONT_RIGHT) * ticks[FRONT_RIGHT] - \
//				EncoderGetDirection(FRONT_LEFT) * ticks[FRONT_LEFT]) * 15000 / 285;
		vel1 = (EncoderGetDirection(FRONT_RIGHT) * ticks[FRONT_RIGHT] + \
						EncoderGetDirection(FRONT_LEFT) * ticks[FRONT_LEFT]) * 15 / 2;
		vel2 = (EncoderGetDirection(FRONT_RIGHT) * ticks[FRONT_RIGHT] - \
				EncoderGetDirection(FRONT_LEFT) * ticks[FRONT_LEFT]) * 15000 / 285;

//		int32_t temp = vels[0];
//		temp = vels[1];

		dx = (float)vels[0] * anglookuptable(pos[2], 1) * 20 / 1000000;
		dy = (float)vels[0] * anglookuptable(pos[2], 0) * 20 / 1000000;
		dth = (float)vels[1] * 20 / 1000;
		pos[0] += dx;
		pos[1] += dy;
		pos[2] += dth;
		PWMSetDuty(MOTOR_CHANNEL, DUTY_1_5 + 10000);
		if (pos[2] > 6283)
			pos[2] -= 6283;

		if (pos[2] < 0)
			pos[2] += 6283;

		// Send encoder message
		if(intCount > 5)
		{
			ROSSendOdomEnc((int32_t)pos[0], (int32_t)pos[1], (int32_t)pos[2], ticks[0], ticks[1]);//vel1, vel2);//vels[0], vels[1]);
			intCount = 0;
		}

		intCount++;

//		FIO0PIN ^= (1<<21);

		// Clear MR0 interrupt
		T1IR |= IR_MR0;
	}

	if(T1IR | IR_MR1)
	{
		ControllerPIDLoop();
//		FIO0PIN ^= (1<<21);
		T1IR |= IR_MR1;
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
