/*
 * timer2ISR.c
 *
 *  Created on: Feb 27, 2011
 *      Author: titus
 */

#include "timer.h"
#include "timer2.h"

extern uint16_t OdomCombRxCount, CmdVelRxCount, cell1, cell2, cell3, status;
extern boolean UseOdomComb, StopLostConn;

uint8_t twoSecCount = 0, tenSecCount = 0;

void Timer2ISR(void)
{
	if (T2IR | IR_MR0) 							// ODOM_COMB (1s)
	{
		if (OdomCombRxCount > ODOM_COMB_LIMIT)
			UseOdomComb = true;
		else
			UseOdomComb = false;

		OdomCombRxCount = 0;

		if (twoSecCount > 0)
		{
			if (CmdVelRxCount > CMD_VEL_LIMIT)
				StopLostConn = false;
			else
				StopLostConn = true;

			CmdVelRxCount = 0;
//			FIO0PIN ^= (1<<21);
			twoSecCount = 0;
		}
		else
		{
			twoSecCount++;
		}

		if (tenSecCount > 4)
		{
			BatteryUpdateVoltages();
			BatteryUpdateStatus();
			ROSSendBattery((uint32_t)cell1, (uint32_t)cell2, (uint32_t)cell3);
//			FIO0PIN ^= (1<<21);
			tenSecCount = 0;
		}
		else
		{
			tenSecCount++;
		}

		T2IR |= IR_MR0;
	}
//	else if (T2IR | IR_MR1)						// CMD_VEL (2s)
//	{
//		if (CmdVelRxCount > CMD_VEL_LIMIT)
//			StopLostConn = false;
//		else
//			StopLostConn = true;
//
//		count_MR1++;
//		if (count_MR1 > 5)
//		{
//			count_MR1 = 1;
//		}
//		T2MR1 = MCR_2S * count_MR1;
//
//		CmdVelRxCount = 0;
//		T2IR |= IR_MR1;
////		FIO0PIN ^= (1<<21);
//	}
//	else if (T2IR | IR_MR2)						// BATTERY MSG (10s)
//	{
//		int16_t val = ADCGetChannel(5);
//		ROSSendBattery((uint32_t)val, 3700, 3400);
//		T2IR |= IR_MR2;
//		T2TCR = 1;
////		FIO0PIN ^= (1<<21);
//	}
//	else if (T2IR | IR_MR3)						// PWM
//	{
//		T2IR |= IR_MR3;
//	}
	else										// NONE
	{
		T2IR |= IR_MR0 | IR_MR1 | IR_MR2 | IR_MR3;
	}

	T2TCR = TCR_CE;
	VICVectAddr = 0x00000000;
}

