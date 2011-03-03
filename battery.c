/*
 * battery.c
 *
 *  Created on: Mar 1, 2011
 *      Author: titus
 */

#include "battery.h"

uint16_t cell1, cell2, cell3, status;

void BatteryUpdateVoltages(void)
{
	cell1 = ADCGetChannel(BATTERY_CELL1);
	cell2 = ADCGetChannel(BATTERY_CELL2);
	cell3 = ADCGetChannel(BATTERY_CELL3);
}

void BatteryUpdateStatus(void)
{
	if((cell1 > BATTERY_VWARN) && (cell2 > BATTERY_VWARN) && (cell3 > BATTERY_VWARN))
		status = BATTERY_GOOD;
	else if ((cell1 > BATTERY_VBAD) && (cell2 > BATTERY_VBAD) && (cell3 > BATTERY_VBAD))
		status = BATTERY_WARN;
	else
		status = BATTERY_BAD;
}
