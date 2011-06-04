#ifndef BATTERY_TASK_H_
#define BATTERY_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "serial.h"
#include "drivers/adc.h"
#include "tasks/rosiface_task.h"

#define BATTERY_1		1
#define BATTERY_2		3

#define BATTERY_GOOD		0
#define BATTERY_WARN		1
#define BATTERY_BAD			2

#define BATTERY_VWARN		900
#define BATTERY_VBAD		700

void vBatteryTaskStart(void);
void BatteryUpdateVoltages(void);
void BatteryUpdateStatus(void);
unsigned portBASE_TYPE BatteryVoltage(unsigned portBASE_TYPE channel);
unsigned portBASE_TYPE BatteryStatus(void);

#endif
