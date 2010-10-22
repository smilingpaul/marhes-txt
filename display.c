#include "display.h"

static unsigned char DisplayState;
static char Switched;
static int16_t currentVC[SIZE_VEL_ARR];
static float currentImuData[SIZE_IMU_ARR];
static int16_t currentGpsStat[SIZE_GPS_STAT_ARR];
static float currentGpsData[SIZE_GPS_DATA_ARR];
static uint32_t currentEncoderCounts[SIZE_ENCODER_ARR];

void DisplayInit(void)
{
    DisplayState = DISPLAY_VEL;
    Switched = 1;    
    LcdClearScreen(BCOLOR);
}

void DisplaySetState(signed char dispNum)
{
    char temp = dispNum;
    
    if (temp < DISPLAY_MIN)
        temp = DISPLAY_MAX;
        
    if (temp > DISPLAY_MAX)
        temp = DISPLAY_MIN;
    
    DisplayState = temp;
    Switched = 1;
}

unsigned char DisplayGetState(void)
{
    return DisplayState;
}

void DisplayUpdate(void)
{  
    switch(DisplayState)
    {
	    case DISPLAY_IMAGE:
            
		    break;
	    case DISPLAY_VEL:
		    DisplayVelocity();
		    break;
	    case DISPLAY_IMU:
		    DisplayIMU();
		    break;
	    case DISPLAY_GPS:
		    DisplayGPS();
		    break;
	    case DISPLAY_ENCODER:
		    DisplayEncoder();
		    break;
	    default:
		    // Don't do anything
		    break;
    }
}

int16_t DisplayChangeValueS(int16_t prevValue, int16_t currentValue, uint8_t xLoc, uint8_t yLoc)
{
    char* strTemp;
    if(prevValue != currentValue)
    {
        strTemp = itoa((int)currentValue);
        LcdSetRect(xLoc, yLoc, xLoc + 8, SCREEN_MAX, FILL, BCOLOR);
        LcdPutStr(strTemp, xLoc, yLoc, SMALL, FCOLOR, BCOLOR);
    }
    return currentValue;
}

float DisplayChangeValueF(float prevValue, float currentValue, char* str, \
    uint8_t xLoc, uint8_t yLoc)
{
    if(prevValue != currentValue)
    {
        LcdSetRect(xLoc, yLoc, xLoc + 8, SCREEN_MAX, FILL, BCOLOR);
        LcdPutStr(str, xLoc, yLoc, SMALL, FCOLOR, BCOLOR);
    }
    return currentValue; 

}

void DisplayVelocity(void)
{
    if(Switched)
    {
        LcdClearScreen(BCOLOR);
        LcdPutStr("VELOCITY CMD", 0, 0, LARGE, FCOLOR, BCOLOR);
        LcdSetLine(18, 0, 18, 131, FCOLOR);
        LcdPutStr("LIN VEL:", 24, 0, SMALL, FCOLOR, BCOLOR);
        LcdPutStr("ANG VEL:", 36, 0, SMALL, FCOLOR, BCOLOR);
        Switched = 0;
    }
    
    currentVC[0] = DisplayChangeValueS(currentVC[0], ROSGetVelocityCmd(0), 24, 52);
    currentVC[1] = DisplayChangeValueS(currentVC[1], ROSGetVelocityCmd(1), 36, 52);
}

void DisplayIMU(void)
{
    uint8_t i = 0;

    if(Switched)
    {
        LcdClearScreen(BCOLOR);
        LcdPutStr("IMU DATA", 0, 0, LARGE, FCOLOR, BCOLOR);
        LcdSetLine(18, 0, 18, 131, FCOLOR);
        LcdPutStr("ROTX:", 24, 0, SMALL, FCOLOR, BCOLOR);
        LcdPutStr("ROTY:", 36, 0, SMALL, FCOLOR, BCOLOR);
        LcdPutStr("ROTZ:", 48, 0, SMALL, FCOLOR, BCOLOR);
        LcdPutStr("ROTW:", 60, 0, SMALL, FCOLOR, BCOLOR);
        LcdPutStr("LINX:", 72, 0, SMALL, FCOLOR, BCOLOR);
        LcdPutStr("LINY:", 84, 0, SMALL, FCOLOR, BCOLOR);
        LcdPutStr("LINZ:", 96, 0, SMALL, FCOLOR, BCOLOR);
        Switched = 0;
    }

    for(i = 0; i < SIZE_IMU_ARR; i++)
        currentImuData[i] = DisplayChangeValueF(currentImuData[i], \
            ROSGetImuData(i), ROSGetImuDataString(i), 24 + 12 * i, 36);
}

void DisplayGPS(void)
{
    uint8_t i = 0;
    
    if(Switched)
    {
        LcdClearScreen(BCOLOR);
	    LcdPutStr("GPS DATA", 0, 0, LARGE, FCOLOR, BCOLOR);
	    LcdSetLine(18, 0, 18, 131, FCOLOR);
	    LcdPutStr("LAT :", 24, 0, SMALL, FCOLOR, BCOLOR);
	    LcdPutStr("LONG:", 36, 0, SMALL, FCOLOR, BCOLOR);
        LcdPutStr("ALT :", 48, 0, SMALL, FCOLOR, BCOLOR);
	    LcdPutStr("HEAD:", 60, 0, SMALL, FCOLOR, BCOLOR);
	    LcdPutStr("STAT:", 72, 0, SMALL, FCOLOR, BCOLOR);
	    LcdPutStr("VIS :", 84, 0, SMALL, FCOLOR, BCOLOR);
        LcdPutStr("USED:", 96, 0, SMALL, FCOLOR, BCOLOR);    
        Switched = 0;
    }
    
    for(i = 0; i < SIZE_GPS_DATA_ARR; i++)
        currentGpsData[i] = DisplayChangeValueF(currentGpsData[i], \
            ROSGetGpsData(i), ROSGetGpsDataString(i), 24 + 12 * i, 36);
            
    for(i = 0; i < SIZE_GPS_STAT_ARR; i++)
        currentGpsStat[i] = DisplayChangeValueS(currentGpsStat[i], \
            ROSGetGpsStatus(i), 72 + 12 * i, 36);
}

void DisplayEncoder(void)
{
    uint8_t i = 0;

    if(Switched)
    {    
        LcdClearScreen(BCOLOR);
        LcdPutStr("ENCODER DATA", 0, 0, LARGE, FCOLOR, BCOLOR);
	    LcdSetLine(18, 0, 18, 131, FCOLOR);
	    LcdPutStr("FR  :", 24, 0, SMALL, FCOLOR, BCOLOR);
	    LcdPutStr("FL  :", 36, 0, SMALL, FCOLOR, BCOLOR);
	    LcdPutStr("LINV:", 48, 0, SMALL, FCOLOR, BCOLOR);
	    LcdPutStr("ANGV:", 60, 0, SMALL, FCOLOR, BCOLOR);
        Switched = 0;
    }

    for(i = 0; i < SIZE_ENCODER_ARR; i++)
        currentEncoderCounts[i] = (uint32_t)DisplayChangeValueS((int16_t)currentEncoderCounts[i], \
        		(int16_t)EncoderCount(i), 24 + 12 * i, 36);
}
