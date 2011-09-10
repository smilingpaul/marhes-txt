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
static unsigned portBASE_TYPE cntSpkr;    ///< Speaker timing count
static msg_u data;                        ///< The battery message to transmit

/**
 @brief Checks and reports the state of the batteries to the ROS computer every
        1s.
 @param[in] pvParameters The parameters from the task create call
 @todo Allow the delay to be variable
*/
static void vBatteryTask( void *pvParameters )
{
  cntSpkr = 0;

  for( ;; )
  {
    BatteryUpdateVoltages();
    BatteryUpdateStatus();
    portENTER_CRITICAL();
    ROSSendBattery(&data, batt1, batt2);
    portEXIT_CRITICAL();  
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
        double alpha = 0.9;
	batt1 = batt1 * alpha + (ADCGetChannel(BATTERY_1) * 13.978) * (1 - alpha);
	batt2 = batt2 * alpha + (ADCGetChannel(BATTERY_2) * 13.978) * (1 - alpha);
}

/**
 @brief Updates the status of the batteries from the voltages.
*/
void BatteryUpdateStatus(void)
{
  if(FIO3PIN & (1<<4))
  {
    status = BATTERY_EXT;
  }
	else if((batt1 > BATTERY_VWARN) || (batt2 > BATTERY_VWARN))
	{
		status = BATTERY_GOOD;
		FIO3PIN &= ~(1<<5);
//		vSerialPutString( debugPortHandle, "BATTERY_GOOD\r\n", 14 );
	}
	else if ((batt1 > BATTERY_VBAD) || (batt2 > BATTERY_VBAD))
	{
		status = BATTERY_WARN;
		cntSpkr++;
		if (cntSpkr < 3)
		  FIO3PIN |= (1<<5);
		else if (cntSpkr == 4)
		  cntSpkr = 0;
		else
		  FIO3PIN &= ~(1<<5);
//		vSerialPutString( debugPortHandle, "BATTERY_WARN\r\n", 14 );
	}
	else
	{
		status = BATTERY_BAD;
		FIO3PIN ^= (1<<5);
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

