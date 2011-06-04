#include "battery_task.h"

extern xComPortHandle debugPortHandle, rosPortHandle;

static unsigned portBASE_TYPE batt1, batt2, status;
static msg_u data;

static void vBatteryTask( void *pvParameters )
{
  for( ;; )
  {
    BatteryUpdateVoltages();
    BatteryUpdateStatus();
    ROSSendBattery(&data, batt1, batt2);
  
    vTaskDelay( 1000 / portTICK_RATE_MS );
  }
}

void vBatteryTaskStart(void)
{
  xTaskCreate( vBatteryTask, "BatteryTask", configMINIMAL_STACK_SIZE, NULL, 2, NULL );
}

void BatteryUpdateVoltages(void)
{
	batt1 = ADCGetChannel(BATTERY_1);
	batt2 = ADCGetChannel(BATTERY_2);
}

void BatteryUpdateStatus(void)
{
	if((batt1 > BATTERY_VWARN) && (batt2 > BATTERY_VWARN))
	{
		status = BATTERY_GOOD;
		vSerialPutString( debugPortHandle, "BATTERY_GOOD\r\n", 14 );
	}
	else if ((batt1 > BATTERY_VBAD) && (batt2 > BATTERY_VBAD))
	{
		status = BATTERY_WARN;
		vSerialPutString( debugPortHandle, "BATTERY_WARN\r\n", 14 );
	}
	else
	{
		status = BATTERY_BAD;
		vSerialPutString( debugPortHandle, "BATTERY_BAD\r\n", 13 );
  }
}

unsigned portBASE_TYPE BatteryVoltage(unsigned portBASE_TYPE channel)
{
  if (channel == 1)
    return batt1;
  else
    return batt2;
}

unsigned portBASE_TYPE BatteryStatus(void)
{
  return status;
}

