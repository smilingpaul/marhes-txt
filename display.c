#include "display.h"

extern boolean UseOdomComb, StopLostConn;
extern int32_t odomCombined[];
extern uint16_t cell1, cell2, cell3, status;
static uint8_t DisplayState;
static int8_t Switched;
//static int16_t currentVC[SIZE_VEL_ARR];
//static float currentImuData[SIZE_IMU_ARR];
//static int16_t currentGpsStat[SIZE_GPS_STAT_ARR];
//static float currentGpsData[SIZE_GPS_DATA_ARR];
//static uint32_t currentEncoderCounts[SIZE_ENCODER_ARR];
//static float currentEncoderVels[SIZE_ENCODER_VEL_ARR];

void DisplayInit(void)
{
    DisplayState = DISPLAY_STATUS;
    Switched = 1;    
    LcdClearScreen(BCOLOR);
}

void DisplaySetState(int8_t dispNum)
{
    char temp = dispNum;

    if (temp < DISPLAY_MIN)
        temp = DISPLAY_MAX;

    if (temp > DISPLAY_MAX)
        temp = DISPLAY_MIN;

    DisplayState = temp;
    Switched = 1;
}

uint8_t DisplayGetState(void)
{
    return DisplayState;
}

void DisplayUpdate(void)
{
    switch(DisplayState)
    {
	    case DISPLAY_STATUS:
	    	DisplayStatus();
		    break;
//	    case DISPLAY_VEL:
//		    DisplayVelocity();
//		    break;
//	    case DISPLAY_ENCODER:
//		    DisplayEncoder();
//		    break;
	    default:
		    // Don't do anything
		    break;
    }
}

//int16_t DisplayChangeValueS(int16_t prevValue, int16_t currentValue, uint8_t xLoc, uint8_t yLoc)
//{
//    char* strTemp;
//    if(prevValue != currentValue)
//    {
//        strTemp = itoa((int)currentValue);
//        LcdSetRect(xLoc, yLoc, xLoc + 8, SCREEN_MAX, FILL, BCOLOR);
//        LcdPutStr(strTemp, xLoc, yLoc, SMALL, FCOLOR, BCOLOR);
//    }
//    return currentValue;
//}
//
//float DisplayChangeValueF(float prevValue, float currentValue, char* str, uint8_t xLoc, uint8_t yLoc)
//{
//    if(prevValue != currentValue)
//    {
//        LcdSetRect(xLoc, yLoc, xLoc + 8, SCREEN_MAX, FILL, BCOLOR);
//        LcdPutStr(str, xLoc, yLoc, SMALL, FCOLOR, BCOLOR);
//    }
//    return currentValue;
//
//}

void DisplayStatus(void)
{
	if (Switched)
	{
		LcdClearScreen(BCOLOR);
		LcdPutStr("TXT1 STATUS", 0, 0, LARGE, FCOLOR, BCOLOR);
		LcdPutStr("LINK ATV: ", 20, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("ODOM SEL: ", 32, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("BAT STAT: ", 44, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("CELL1:    ", 56, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("CELL2:    ", 68, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("CELL3:    ", 80, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("CMD_LV:   ", 92, 0, SMALL, FCOLOR, BCOLOR);
//		LcdPutStr("CMD_AV:   ", 104, 0, SMALL, FCOLOR, BCOLOR);
		LcdPutStr("RX_LV:    ", 116, 0, SMALL, FCOLOR, BCOLOR);
//		LcdPutStr("RX_AV:    ", 128, 0, SMALL, FCOLOR, BCOLOR);
		LcdSetLine(18, 0, 18, 131, FCOLOR);
//		LcdSetLine(30, 0, 30, 131, FCOLOR);
//		LcdSetLine(42, 0, 42, 131, FCOLOR);
//		LcdSetLine(54, 0, 54, 131, FCOLOR);
//		LcdSetLine(66, 0, 66, 131, FCOLOR);
//		LcdSetLine(78, 0, 78, 131, FCOLOR);
//		LcdSetLine(90, 0, 90, 131, FCOLOR);
//		LcdSetLine(18, 50, 90, 50, FCOLOR);
		Switched = 0;
	}

	if (StopLostConn)
		LcdPutStr("FALSE", 20, 60, SMALL, RED, BCOLOR);
	else
		LcdPutStr("TRUE ", 20, 60, SMALL, GREEN, BCOLOR);

	if (UseOdomComb)
		LcdPutStr("COMBINED", 32, 60, SMALL, FCOLOR, BCOLOR);
	else
		LcdPutStr("ENCODER ", 32, 60, SMALL, FCOLOR, BCOLOR);

	if (status == BATTERY_GOOD)
		LcdPutStr("GOOD   ", 44, 60, SMALL, GREEN, BCOLOR);
	else if (status == BATTERY_WARN)
		LcdPutStr("WARNING", 44, 60, SMALL, YELLOW, BCOLOR);
	else
		LcdPutStr("BAD    ", 44, 60, SMALL, RED, BCOLOR);

	LcdSetRect(56, 60, 64, SCREEN_MAX, FILL, BCOLOR);
	if (cell1 > BATTERY_VWARN)
		LcdPutStr(itoa(cell1), 56, 60, SMALL, GREEN, BCOLOR);
	else if (cell1 > BATTERY_VBAD)
		LcdPutStr(itoa(cell1), 56, 60, SMALL, YELLOW, BCOLOR);
	else
		LcdPutStr(itoa(cell1), 56, 60, SMALL, RED, BCOLOR);

	LcdSetRect(68, 60, 76, SCREEN_MAX, FILL, BCOLOR);
	if (cell2 > BATTERY_VWARN)
		LcdPutStr(itoa(cell2), 68, 60, SMALL, GREEN, BCOLOR);
	else if (cell2 > BATTERY_VBAD)
		LcdPutStr(itoa(cell2), 68, 60, SMALL, YELLOW, BCOLOR);
	else
		LcdPutStr(itoa(cell2), 68, 60, SMALL, RED, BCOLOR);

	LcdSetRect(80, 60, 88, SCREEN_MAX, FILL, BCOLOR);
	if (cell3 > BATTERY_VWARN)
		LcdPutStr(itoa(cell3), 80, 60, SMALL, GREEN, BCOLOR);
	else if (cell3 > BATTERY_VBAD)
		LcdPutStr(itoa(cell3), 80, 60, SMALL, YELLOW, BCOLOR);
	else
		LcdPutStr(itoa(cell3), 80, 60, SMALL, RED, BCOLOR);

	LcdSetRect(92, 60, 100, SCREEN_MAX, FILL, BCOLOR);
	LcdPutStr(itoa(ControllerGetLinearVelocity()), 92, 60, SMALL, FCOLOR, BCOLOR);
//	LcdSetRect(104, 60, 112, SCREEN_MAX, FILL, BCOLOR);
//	LcdPutStr(itoa(ControllerGetAngularVelocity()), 104, 60, SMALL, FCOLOR, BCOLOR);
	LcdSetRect(116, 60, 124, SCREEN_MAX, FILL, BCOLOR);
	LcdPutStr(itoa(odomCombined[3]), 116, 60, SMALL, FCOLOR, BCOLOR);
//	LcdSetRect(128, 60, 136, SCREEN_MAX, FILL, BCOLOR);
//	LcdPutStr(itoa(1), 128, 60, SMALL, FCOLOR, BCOLOR);
}

//void DisplayVelocity(void)
//{
//    if(Switched)
//    {
//        LcdClearScreen(BCOLOR);
//        LcdPutStr("VELOCITY CMD", 0, 0, LARGE, FCOLOR, BCOLOR);
//        LcdSetLine(18, 0, 18, 131, FCOLOR);
//        LcdPutStr("LIN VEL:", 24, 0, SMALL, FCOLOR, BCOLOR);
//        LcdPutStr("ANG VEL:", 36, 0, SMALL, FCOLOR, BCOLOR);
//        Switched = 0;
//    }
//
//    currentVC[0] = DisplayChangeValueS(currentVC[0], ROSGetVelocityCmd(0), 24, 52);
//    currentVC[1] = DisplayChangeValueS(currentVC[1], ROSGetVelocityCmd(1), 36, 52);
//}
//
//void DisplayEncoder(void)
//{
//    uint8_t i = 0;
//
//    if(Switched)
//    {
//        LcdClearScreen(BCOLOR);
//        LcdPutStr("ENCODER DATA", 0, 0, LARGE, FCOLOR, BCOLOR);
//	    LcdSetLine(18, 0, 18, 131, FCOLOR);
//	    LcdPutStr("FR  :", 24, 0, SMALL, FCOLOR, BCOLOR);
//	    LcdPutStr("FL  :", 36, 0, SMALL, FCOLOR, BCOLOR);
//	    LcdPutStr("LINV:", 48, 0, SMALL, FCOLOR, BCOLOR);
//	    LcdPutStr("ANGV:", 60, 0, SMALL, FCOLOR, BCOLOR);
//        Switched = 0;
//    }
//
//    for(i = 0; i < SIZE_ENCODER_ARR; i++)
//        currentEncoderCounts[i] = (uint32_t)DisplayChangeValueS((int16_t)currentEncoderCounts[i], \
//        		(int16_t)EncoderCount(i), 24 + 12 * i, 36);
//
//    for(i = 0; i < SIZE_ENCODER_VEL_ARR; i++)
//    	currentEncoderVels[i] = DisplayChangeValueS((int16_t)currentEncoderVels[i], \
//    			(int16_t)EncoderVel(i), 48 + 12 * i, 36);
//}
