/**
 @file display_task.c
  
 @brief This task updates the display and sets the brightness of the backlight.
 
 The task updates the display and brightness every 100ms.  The display is
 preconfigured to a few types:
 - Display Status
 - Display 2
 - Display 3
 The data in the display is updated at every step. If the display is switched,
 then the labels of the data are updated once. The brightness uses PWM6 and AD5.
 
 @author Titus Appel

 @version 1.0

 @date 2011/06/03

 Contact: titus.appel@gmail.com
*/

#include "display_task.h"

extern int32_t ticks[SIZE_ENCODER_TICKS_ARR];
extern int32_t vels[SIZE_ENCODER_VEL_ARR];

static int8_t DisplayState;              ///< The display type to show
static int8_t SwitchedState;             ///< If the display has switch states
static uint8_t SelectedPIDDisplayIndex;  ///< The selected PID display index

/**
 @brief The display task first initializes the display, then updates it and the
        brightness of the backlight.
 @param[in] pvParameters The parameteDisplayPidSelectedrs from the task create call
 @todo Allow the delay to be variable 
*/
static void vDisplayTask( void *pvParameters )
{
  DisplayInit();
  
  for ( ;; )
  {
    DisplayUpdate();
    DisplayBrightUpdate();
    
    vTaskDelay( 100 / portTICK_RATE_MS );
  }
}

/**
 @brief Start the display task.
 @todo Should make the Priority and Stack Size reconfigurable.
*/
void vDisplayTaskStart(void)
{
  xTaskCreate( vDisplayTask, "DisplayTask", configMINIMAL_STACK_SIZE * 2, NULL, 2, NULL );
}

/**
 @brief Initialize the display to the status screen and start by clearing the 
        screen.
*/
void DisplayInit(void)
{
    DisplayState = DISPLAY_STATUS;
    SwitchedState = 1;   
    SelectedPIDDisplayIndex = 0; 
    LcdClearScreen(BCOLOR);
}

/**
 @brief Decrease the screen state type.
*/
void DisplayDecreaseState(void)
{
  DisplayState--;
  if (DisplayState < DISPLAY_MIN)
    DisplayState = DISPLAY_MAX;
  SwitchedState = 1;  
}

/**
 @brief Increase the screen state type.
*/
void DisplayIncreaseState(void)
{
  DisplayState++;
  if (DisplayState > DISPLAY_MAX)
    DisplayState = DISPLAY_MIN;
  SwitchedState = 1;  
}

/**
 @brief Update the display to the correct type.
*/
void DisplayUpdate(void)
{
  switch(DisplayState)
  {
    case DISPLAY_STATUS:
    	DisplayStatus();
	    break;
    case DISPLAY_PWM:
    	DisplayPWM();
	    break;
    case DISPLAY_ENCODER:
    	DisplayEncoders();
	    break;
	  case DISPLAY_GAINS:
	    DisplayGains();
	    break;		    		    
    default:
	    // Don't do anything
	    break;
  }
}

/**
 @brief Read the potentiometer and then set the duty cycle of the backlight.
*/
void DisplayBrightUpdate(void)
{
  int16_t val = ADCGetChannel(5);
  uint32_t temp = AD0DR5;
  PWMSetDuty(6, (val * 1407));
}

void DisplayPidSelected(int8_t direction)
{
  if (direction == -1 && SelectedPIDDisplayIndex > 0)
    SelectedPIDDisplayIndex--;
  if (direction == 1 && SelectedPIDDisplayIndex < ANG_PID_CNT_MAX / 4 - 6)
    SelectedPIDDisplayIndex++;
}

/**
 @brief Update the display to the status screen.
*/
void DisplayStatus(void)
{
  unsigned portBASE_TYPE temp;

	if (SwitchedState)
	{
		LcdClearScreen(BCOLOR);
		LcdPutStr("TXT1 STATUS", 0, 0, LARGE, FCOLOR, BCOLOR);
		LcdPutStr("LINK ATV: ", 20, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("ODOM SEL: ", 32, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("BAT STAT: ", 44, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("BATT1 V:  ", 56, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("BATT2 V:  ", 68, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("CMD_LV:   ", 80, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("CMD_AV:   ", 92, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("K_LV:     ", 104, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("K_AV:     ", 116, 0, SMALL, FCOLOR, BCOLOR);
		LcdSetLine(18, 0, 18, 131, FCOLOR);
		SwitchedState = 0;
	}

	if (ModeStopLostConn())
		LcdPutStr("FALSE", 20, 60, SMALL, RED, BCOLOR);
	else
		LcdPutStr("TRUE ", 20, 60, SMALL, GREEN, BCOLOR);

	if (ModeUseOdomComb())
		LcdPutStr("COMBINED", 32, 60, SMALL, FCOLOR, BCOLOR);
	else
		LcdPutStr("ENCODER ", 32, 60, SMALL, FCOLOR, BCOLOR);

  temp = BatteryStatus();
	if (temp == BATTERY_GOOD)
		LcdPutStr("GOOD          ", 44, 60, SMALL, GREEN, BCOLOR);
	else if (temp == BATTERY_WARN)
		LcdPutStr("WARNING       ", 44, 60, SMALL, YELLOW, BCOLOR);
	else if (temp == BATTERY_BAD)
		LcdPutStr("BAD           ", 44, 60, SMALL, RED, BCOLOR);
  else
    LcdPutStr("EXTERNAL POWER", 44, 60, SMALL, GREEN, BCOLOR);

  temp = BatteryVoltage(1);
	LcdSetRect(56, 60, 64, SCREEN_MAX, FILL, BCOLOR);
	if (temp > BATTERY_VWARN)
		LcdPutStr(itoa(temp), 56, 60, SMALL, GREEN, BCOLOR);
	else if (temp > BATTERY_VBAD)
		LcdPutStr(itoa(temp), 56, 60, SMALL, YELLOW, BCOLOR);
	else
		LcdPutStr(itoa(temp), 56, 60, SMALL, RED, BCOLOR);

  temp = BatteryVoltage(2);
	LcdSetRect(68, 60, 76, SCREEN_MAX, FILL, BCOLOR);
	if (temp > BATTERY_VWARN)
		LcdPutStr(itoa(temp), 68, 60, SMALL, GREEN, BCOLOR);
	else if (temp > BATTERY_VBAD)
		LcdPutStr(itoa(temp), 68, 60, SMALL, YELLOW, BCOLOR);
	else
		LcdPutStr(itoa(temp), 68, 60, SMALL, RED, BCOLOR);

	LcdSetRect(80, 60, 88, SCREEN_MAX, FILL, BCOLOR);
	LcdPutStr(itoa(ControllerGetLinearVelocity()), 80, 60, SMALL, FCOLOR, BCOLOR);
	LcdSetRect(92, 60, 100, SCREEN_MAX, FILL, BCOLOR);
	LcdPutStr(itoa(ControllerGetAngularVelocity()), 92, 60, SMALL, FCOLOR, BCOLOR);

	LcdSetRect(104, 60, 112, SCREEN_MAX, FILL, BCOLOR);
	LcdPutStr(itoa((int32_t)ControllerGetLinPid(0)), 104, 60, SMALL, FCOLOR, BCOLOR);
	LcdPutStr(itoa((int32_t)ControllerGetLinPid(1)), 104, 85, SMALL, FCOLOR, BCOLOR);
	LcdPutStr(itoa((int32_t)ControllerGetLinPid(2)), 104, 110, SMALL, FCOLOR, BCOLOR);
	LcdSetRect(116, 60, 124, SCREEN_MAX, FILL, BCOLOR);
	LcdPutStr(itoa((int32_t)ControllerGetAngPid(0)), 116, 60, SMALL, FCOLOR, BCOLOR);
	LcdPutStr(itoa((int32_t)ControllerGetAngPid(1)), 116, 85, SMALL, FCOLOR, BCOLOR);
	LcdPutStr(itoa((int32_t)ControllerGetAngPid(2)), 116, 110, SMALL, FCOLOR, BCOLOR);
}

/**
 @brief Update the display to the 2nd screen.
*/
void DisplayPWM(void)
{
	if (SwitchedState)
	{
    LcdClearScreen(BCOLOR);
		LcdPutStr("TXT1 PWM", 0, 0, LARGE, FCOLOR, BCOLOR);
		LcdPutStr("PWM1:     ", 20, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("PWM2:     ", 32, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("PWM3:     ", 44, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("PWM4:     ", 56, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("PWM5:     ", 68, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("PWM6:     ", 80, 0, SMALL, FCOLOR, BCOLOR);
    LcdPutStr("Mode:     ", 92, 0, SMALL, FCOLOR, BCOLOR);
	  LcdSetLine(18, 0, 18, 131, FCOLOR);
		SwitchedState = 0;
  }
  
  LcdSetRect(20, 60, 28, SCREEN_MAX, FILL, BCOLOR);
	LcdPutStr(itoa(PWMGetDuty(1)), 20, 60, SMALL, FCOLOR, BCOLOR);
	LcdSetRect(32, 60, 40, SCREEN_MAX, FILL, BCOLOR);
	LcdPutStr(itoa(PWMGetDuty(2)), 32, 60, SMALL, FCOLOR, BCOLOR);
	LcdSetRect(44, 60, 52, SCREEN_MAX, FILL, BCOLOR);
	LcdPutStr(itoa(PWMGetDuty(3)), 44, 60, SMALL, FCOLOR, BCOLOR);
	LcdSetRect(56, 60, 64, SCREEN_MAX, FILL, BCOLOR);
	LcdPutStr(itoa(PWMGetDuty(4)), 56, 60, SMALL, FCOLOR, BCOLOR);
	LcdSetRect(68, 60, 76, SCREEN_MAX, FILL, BCOLOR);
	LcdPutStr(itoa(PWMGetDuty(5)), 68, 60, SMALL, FCOLOR, BCOLOR);
	LcdSetRect(80, 60, 88, SCREEN_MAX, FILL, BCOLOR);
	LcdPutStr(itoa(PWMGetDuty(6)), 80, 60, SMALL, FCOLOR, BCOLOR);
	LcdSetRect(92, 60, 100, SCREEN_MAX, FILL, BCOLOR);
	if (ControllerGetMode())
	  LcdPutStr("Controller", 92, 60, SMALL, FCOLOR, BCOLOR);
	else
	  LcdPutStr("Manual", 92, 60, SMALL, FCOLOR, BCOLOR);
		
}

/**
 @brief Update the display to the 3rd screen.
*/
void DisplayEncoders(void)
{
	if (SwitchedState)
	{
    LcdClearScreen(BCOLOR);
		LcdPutStr("TXT1 ENCODERS", 0, 0, LARGE, FCOLOR, BCOLOR);
    LcdPutStr("TICKS FR:   ", 20, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("TICKS FL:   ", 32, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("VEL LIN:    ", 44, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("VEL ANG:    ", 56, 0, SMALL, FCOLOR, BCOLOR);
	  LcdSetLine(18, 0, 18, 131, FCOLOR);
		SwitchedState = 0;
  }
  
  LcdSetRect(20, 60, 28, SCREEN_MAX, FILL, BCOLOR);
	LcdPutStr(itoa(ticks[TICKS_FR]), 20, 60, SMALL, FCOLOR, BCOLOR);
	LcdSetRect(32, 60, 40, SCREEN_MAX, FILL, BCOLOR);
	LcdPutStr(itoa(ticks[TICKS_FL]), 32, 60, SMALL, FCOLOR, BCOLOR);
	LcdSetRect(44, 60, 52, SCREEN_MAX, FILL, BCOLOR);
	LcdPutStr(itoa(vels[VELS_LINEAR]), 44, 60, SMALL, FCOLOR, BCOLOR);
	LcdSetRect(56, 60, 64, SCREEN_MAX, FILL, BCOLOR);
	LcdPutStr(itoa(vels[VELS_ANGULAR]), 56, 60, SMALL, FCOLOR, BCOLOR);
}

/**
 @brief Update the 4th display screen with the stored PID gains
 */
void DisplayGains(void)
{
  int i;
  
	if (SwitchedState)
	{
    LcdClearScreen(BCOLOR);
		LcdPutStr("TXT1 PID GAINS", 0, 0, LARGE, FCOLOR, BCOLOR);
    LcdPutStr("LIN GAINS:  ", 20, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("ANG GAINS:  ", 32, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("VEL  KP   KI   KD", 44, 0, SMALL, FCOLOR, BCOLOR);
	  LcdSetLine(18, 0, 18, 131, FCOLOR);
		SwitchedState = 0;
  }
  
  LcdSetRect(20, 60, 28, SCREEN_MAX, FILL, BCOLOR);
	LcdPutStr(itoa((int32_t)ControllerGetLinPid(0)), 20, 60, SMALL, FCOLOR, BCOLOR);
	LcdPutStr(itoa((int32_t)ControllerGetLinPid(1)), 20, 85, SMALL, FCOLOR, BCOLOR);
	LcdPutStr(itoa((int32_t)ControllerGetLinPid(2)), 20, 110, SMALL, FCOLOR, BCOLOR);
	
	for (i = 0; i < 6; i++)
	{
	  LcdSetRect(56 + i * 12, 0, 64 + i * 12, SCREEN_MAX, FILL, BCOLOR);
	  LcdPutStr(itoa((int32_t)ControllerGetAngPidAddr(SelectedPIDDisplayIndex + i,
	                 -1)), 56 + i * 12, 0, SMALL, FCOLOR, BCOLOR);
	  LcdPutStr(itoa((int32_t)ControllerGetAngPidAddr(SelectedPIDDisplayIndex + i,
	                  0)), 56 + i * 12, 30, SMALL, FCOLOR, BCOLOR);
	  LcdPutStr(itoa((int32_t)ControllerGetAngPidAddr(SelectedPIDDisplayIndex + i,
	                  1)), 56 + i * 12, 60, SMALL, FCOLOR, BCOLOR);
    LcdPutStr(itoa((int32_t)ControllerGetAngPidAddr(SelectedPIDDisplayIndex + i, 
                    2)), 56 + i * 12, 90, SMALL, FCOLOR, BCOLOR);
  }
}
