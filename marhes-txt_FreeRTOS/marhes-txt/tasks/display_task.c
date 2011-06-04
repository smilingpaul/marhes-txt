#include "display_task.h"

static int8_t DisplayState;
static int8_t SwitchedState;

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

void vDisplayTaskStart(void)
{
  xTaskCreate( vDisplayTask, "DisplayTask", configMINIMAL_STACK_SIZE, NULL, 2, NULL );
}

void DisplayInit(void)
{
    DisplayState = DISPLAY_STATUS;
    SwitchedState = 1;    
    LcdClearScreen(BCOLOR);
}

void DisplayDecreaseState(void)
{
  DisplayState--;
  if (DisplayState < DISPLAY_MIN)
    DisplayState = DISPLAY_MAX;
  SwitchedState = 1;  
}

void DisplayIncreaseState(void)
{
  DisplayState++;
  if (DisplayState > DISPLAY_MAX)
    DisplayState = DISPLAY_MIN;
  SwitchedState = 1;  
}

void DisplayUpdate(void)
{
  switch(DisplayState)
  {
    case DISPLAY_STATUS:
    	DisplayStatus();
	    break;
    case DISPLAY_VEL:
    	Display2();
	    break;
    case DISPLAY_ENCODER:
    	Display3();
	    break;		    		    
    default:
	    // Don't do anything
	    break;
  }
}

void DisplayBrightUpdate(void)
{
  int16_t val = ADCGetChannel(5);
  PWMSetDuty(6, (val * 1407));
}

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
		LcdPutStr("GOOD   ", 44, 60, SMALL, GREEN, BCOLOR);
	else if (temp == BATTERY_WARN)
		LcdPutStr("WARNING", 44, 60, SMALL, YELLOW, BCOLOR);
	else
		LcdPutStr("BAD    ", 44, 60, SMALL, RED, BCOLOR);

  temp = BatteryVoltage(BATTERY_1);
	LcdSetRect(56, 60, 64, SCREEN_MAX, FILL, BCOLOR);
	if (temp > BATTERY_VWARN)
		LcdPutStr(itoa(temp), 56, 60, SMALL, GREEN, BCOLOR);
	else if (temp > BATTERY_VBAD)
		LcdPutStr(itoa(temp), 56, 60, SMALL, YELLOW, BCOLOR);
	else
		LcdPutStr(itoa(temp), 56, 60, SMALL, RED, BCOLOR);

  temp = BatteryVoltage(BATTERY_2);
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
	LcdPutStr(itoa(ControllerGetPid(0)), 104, 60, SMALL, FCOLOR, BCOLOR);
	LcdPutStr(itoa(ControllerGetPid(1)), 104, 85, SMALL, FCOLOR, BCOLOR);
	LcdPutStr(itoa(ControllerGetPid(2)), 104, 110, SMALL, FCOLOR, BCOLOR);
	LcdSetRect(116, 60, 124, SCREEN_MAX, FILL, BCOLOR);
	LcdPutStr(itoa(ControllerGetPid(3)), 116, 60, SMALL, FCOLOR, BCOLOR);
	LcdPutStr(itoa(ControllerGetPid(4)), 116, 85, SMALL, FCOLOR, BCOLOR);
	LcdPutStr(itoa(ControllerGetPid(5)), 116, 110, SMALL, FCOLOR, BCOLOR);
}

void Display2(void)
{
	if (SwitchedState)
	{
    LcdClearScreen(BCOLOR);
		LcdPutStr("TXT1 2", 0, 0, LARGE, FCOLOR, BCOLOR);
	  LcdSetLine(18, 0, 18, 131, FCOLOR);
		SwitchedState = 0;
  }
}

void Display3(void)
{
	if (SwitchedState)
	{
    LcdClearScreen(BCOLOR);
		LcdPutStr("TXT1 3", 0, 0, LARGE, FCOLOR, BCOLOR);
	  LcdSetLine(18, 0, 18, 131, FCOLOR);
		SwitchedState = 0;
  }
}
