/*
 * battery.h
 *
 *  Created on: Mar 1, 2011
 *      Author: titus
 */

#ifndef BATTERY_H_
#define BATTERY_H_

#include "app_types.h"
#include "LPC23xx.h"
#include "adc.h"

#define BATTERY_CELL1		1
#define BATTERY_CELL2		3
#define BATTERY_CELL3		6

#define BATTERY_GOOD		0
#define BATTERY_WARN		1
#define BATTERY_BAD			2

#define BATTERY_VWARN		680
#define BATTERY_VBAD		340

void BatteryUpdateVoltages(void);
void BatteryUpdateStatus(void);

#endif /* BATTERY_H_ */
