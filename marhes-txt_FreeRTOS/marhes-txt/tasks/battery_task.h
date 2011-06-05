/**
 @file battery_task.h
  
 @brief Provides the constants and function declarations for the battery task.
 
 @author Titus Appel

 @version 1.0

 @date 2011/06/03

 Contact: titus.appel@gmail.com
*/

#ifndef BATTERY_TASK_H_
#define BATTERY_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "serial.h"
#include "drivers/adc.h"
#include "tasks/rosiface_task.h"

#define BATTERY_1		1               ///< Battery 1 ADC input channel
#define BATTERY_2		3               ///< Battery 1 ADC input channel

#define BATTERY_GOOD		0           ///< Battery status good
#define BATTERY_WARN		1           ///< Battery status warning
#define BATTERY_BAD			2           ///< Battery status bad

#define BATTERY_VWARN		900         ///< Battery status warning voltage
#define BATTERY_VBAD		700         ///< Battery status bad voltage

void vBatteryTaskStart(void);
void BatteryUpdateVoltages(void);
void BatteryUpdateStatus(void);
unsigned portBASE_TYPE BatteryVoltage(unsigned portBASE_TYPE channel);
unsigned portBASE_TYPE BatteryStatus(void);

#endif
