/**
 @file mode_task.c
  
 @brief This task decides whether the ROS communication link is active and
        whether the controller should use the encoder or combined odometry.
        
 The mode task runs at 1Hz and checks if sufficient numbers of CMD_VEL and 
 CMD_ODOM_COMB messages have arrived in the second. If not enough CMD_VEL
 messages are received then the motors are turned off because the ROS link is 
 not active. If not enough CMD_ODOM_COMB messages are received then the 
 controller uses the encoder odometry and not the combined odometry.
 
 @author Titus Appel

 @version 1.0

 @date 2011/06/03

 Contact: titus.appel@gmail.com
*/

#include "mode_task.h"

#define ODOM_COMB_LIMIT				2   ///< Number of ODOM_COMB messages needed to receive within period
#define CMD_VEL_LIMIT				  1   ///< Number of CMD_VEL messages needed to receive within period

extern xComPortHandle debugPortHandle;

/// The running count of combined odometry messages received
static unsigned portBASE_TYPE OdomCombRxCount = 0;
/// The running count of command velocity messages received
static unsigned portBASE_TYPE CmdVelRxCount = 0;

/// If combined odometry from ROS is to be used by controller
static unsigned portBASE_TYPE UseOdomComb = pdFALSE;

/// If the motors should stop due to lost connection
static unsigned portBASE_TYPE StopLostConn = pdTRUE;

/**
 @brief Task that sets the boolean variables to their correct mode based on if
        the minimum amount of messages is received in the time increment.
        
 This also outputs what state we are in to the debug port to debug if the task
 is working or not.  After they are checked the counters are reset to 0.
 @param[in] pvParameters The input parameters from the task creation call.
 @todo Allow the delay to be variable
*/
static void vModeTask( void *pvParameters )
{
  static unsigned portBASE_TYPE cnt = 0;
  
  /* As per most tasks, this task is implemented in an infinite loop. */
  for( ;; )
  {
    if (OdomCombRxCount > ODOM_COMB_LIMIT)
    {
			UseOdomComb = pdTRUE;
//			vSerialPutString( debugPortHandle, "ODOM_COMB\r\n", 11 );
		}
		else
		{
			UseOdomComb = pdFALSE;
//			vSerialPutString( debugPortHandle, "ODOM_ENCR\r\n", 11 );
    }
    
		OdomCombRxCount = 0;

  	if (CmdVelRxCount > CMD_VEL_LIMIT)
		{
			StopLostConn = pdFALSE;
			if (BatteryStatus() == BATTERY_GOOD || BatteryStatus() == BATTERY_EXT)
			  FIO3PIN &= ~(1<<5);
//			vSerialPutString( debugPortHandle, "LINK_ACTIVE\r\n", 13 );
	  }
		else
		{
			StopLostConn = pdTRUE;
			if (BatteryStatus() == BATTERY_GOOD || BatteryStatus() == BATTERY_EXT)
			{
			  cnt++;
    		if (cnt < 2)
	    	  FIO3PIN |= (1<<5);
		    else if (cnt > 3)
		      cnt = 0;
		    else
		      FIO3PIN &= ~(1<<5);
		  }
//			vSerialPutString( debugPortHandle, "LINK_ERROR\r\n", 12 );
		}

		CmdVelRxCount = 0;
 
    vTaskDelay( 1000 / portTICK_RATE_MS );
  }
}

/**
 @brief Starts the mode task.
 @todo Should make the Priority and Stack Size reconfigurable.
*/
void vModeTaskStart(void)
{
  xTaskCreate( vModeTask, "ModeTask", configMINIMAL_STACK_SIZE, NULL, 2, NULL );
}

/**
 @brief Used by rosiface_task.c to increment the counter when this message is
        received.
*/
void vModeOdomCombRx(void)
{
  OdomCombRxCount++;
}

/**
 @brief Used by rosiface_task.c to increment the counter when this message is
        received.
*/
void vModeCmdVelRx(void)
{
  CmdVelRxCount++;
}

/**
 @brief Gets the mode of the odometry to be used.
*/
unsigned portBASE_TYPE ModeUseOdomComb(void)
{
  return UseOdomComb;
}

/**
 @brief Gets the mode of the motors from the status of the connection.
*/
unsigned portBASE_TYPE ModeStopLostConn(void)
{
  return StopLostConn;
}
