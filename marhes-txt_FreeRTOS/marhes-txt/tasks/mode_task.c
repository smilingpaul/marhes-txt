#include "mode_task.h"

#define ODOM_COMB_LIMIT				2
#define CMD_VEL_LIMIT				  1

extern xComPortHandle debugPortHandle;

static unsigned portBASE_TYPE OdomCombRxCount = 0, CmdVelRxCount = 0;
static unsigned portBASE_TYPE UseOdomComb = pdFALSE;
static unsigned portBASE_TYPE StopLostConn = pdTRUE;

static void vModeTask( void *pvParameters )
{
  /* As per most tasks, this task is implemented in an infinite loop. */
  for( ;; )
  {
    if (OdomCombRxCount > ODOM_COMB_LIMIT)
    {
			UseOdomComb = pdTRUE;
			vSerialPutString( debugPortHandle, "ODOM_COMB\r\n", 11 );
		}
		else
		{
			UseOdomComb = pdFALSE;
			vSerialPutString( debugPortHandle, "ODOM_ENCR\r\n", 11 );
    }
    
		OdomCombRxCount = 0;

  	if (CmdVelRxCount > CMD_VEL_LIMIT)
		{
			StopLostConn = pdFALSE;
			vSerialPutString( debugPortHandle, "LINK_ACTIVE\r\n", 13 );
	  }
		else
		{
			StopLostConn = pdTRUE;
			vSerialPutString( debugPortHandle, "LINK_ERROR\r\n", 12 );
		}

		CmdVelRxCount = 0;
 
    vTaskDelay( 1000 / portTICK_RATE_MS );
  }
}

void vModeTaskStart(void)
{
  xTaskCreate( vModeTask, "ModeTask", configMINIMAL_STACK_SIZE, NULL, 2, NULL );
}

void vModeOdomCombRx(void)
{
  OdomCombRxCount++;
}

void vModeCmdVelRx(void)
{
  CmdVelRxCount++;
}

unsigned portBASE_TYPE ModeUseOdomComb(void)
{
  return UseOdomComb;
}

unsigned portBASE_TYPE ModeStopLostConn(void)
{
  return StopLostConn;
}

