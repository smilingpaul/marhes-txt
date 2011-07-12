/**
 @file battery_task.c
  
 @brief This task gets the battery voltages and then updates the status
        indication.
        
 The batteries are connected to channels 1 and 3 of the ADC. The battery task
 checks the voltage and status of the batteries and then sends the ROS 
 computer the data every 1s.
 
 @author Titus Appel

 @version 1.0

 @date 2011/06/03
 
 @todo Switch off the motors when the batteries too low.

 Contact: titus.appel@gmail.com
*/

#include "battery_task.h"

extern xComPortHandle debugPortHandle, rosPortHandle;

static unsigned portBASE_TYPE batt1;      ///< The voltage of battery 1. 
static unsigned portBASE_TYPE batt2;      ///< The voltage of battery 2.
static unsigned portBASE_TYPE status;     ///< The status of the batteries.
static msg_u data;                        ///< The battery message to transmit

/**
 @brief Checks and reports the state of the batteries to the ROS computer every
        1s.
 @param[in] pvParameters The parameters from the task create call
 @todo Allow the delay to be variable
*/
static void vBatteryTask( void *pvParameters )
{
  for( ;; )
  {
    BatteryUpdateVoltages();
    BatteryUpdateStatus();
//    ROSSendBattery(&data, batt1, batt2);
  
    vTaskDelay( 1000 / portTICK_RATE_MS );
  }
}

/**
 @brief Starts the battery task.
 @todo Should make the Priority and Stack Size reconfigurable.
*/
void vBatteryTaskStart(void)
{
  xTaskCreate( vBatteryTask, "BatteryTask", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
}

/**
 @brief Updates the battery voltages.
*/
void BatteryUpdateVoltages(void)
{
	batt1 = ADCGetChannel(BATTERY_1);
	batt2 = ADCGetChannel(BATTERY_2);
}

/**
 @brief Updates the status of the batteries from the voltages.
*/
void BatteryUpdateStatus(void)
{
	if((batt1 > BATTERY_VWARN) && (batt2 > BATTERY_VWARN))
	{
		status = BATTERY_GOOD;
//		vSerialPutString( debugPortHandle, "BATTERY_GOOD\r\n", 14 );
	}
	else if ((batt1 > BATTERY_VBAD) && (batt2 > BATTERY_VBAD))
	{
		status = BATTERY_WARN;
//		vSerialPutString( debugPortHandle, "BATTERY_WARN\r\n", 14 );
	}
	else
	{
		status = BATTERY_BAD;
//		vSerialPutString( debugPortHandle, "BATTERY_BAD\r\n", 13 );
  }
}

/**
 @brief Get the battery voltage.
 @param[in] channel The battery number's voltage to get.
 @return            The voltage of the battery
*/
unsigned portBASE_TYPE BatteryVoltage(unsigned portBASE_TYPE channel)
{
  if (channel == 1)
    return batt1;
  else
    return batt2;
}

/**
 @brief Get the battery status
 @return The battery status (good, warn, bad)
*/
unsigned portBASE_TYPE BatteryStatus(void)
{
  return status;
}

