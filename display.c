#include "display.h"

static unsigned char DisplayState;
static char Switched;
static uint16_t currentVC[SIZE_VEL_ARR];
static float currentImuData[SIZE_IMU_ARR];
static uint16_t currentGpsStat[SIZE_GPS_STAT_ARR];
static float currentGpsData[SIZE_GPS_DATA_ARR];

void DisplayInit(void)
{
    DisplayState = DISPLAY_VEL;
    Switched = 1;
    
    ROSGetVelocityCmd(currentVC);
    ROSGetImuData(currentImuData);
    ROSGetGpsStatus(currentGpsStat);
    ROSGetGpsData(currentGpsData);
    
    LcdClearScreen(BCOLOR);
}

void DisplaySetState(signed char dispNum)
{
    signed char temp = dispNum;
    
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

short DisplayChangeValueS(short prevValue, short currentValue, uint8_t xLoc, uint8_t yLoc)
{
    static char strTemp[15];
    if(prevValue != currentValue)
    {
        snprintf(strTemp, 15, "%u", currentValue);
        LcdSetRect(xLoc, yLoc, xLoc + 8, SCREEN_MAX, FILL, BCOLOR);
        LcdPutStr(strTemp, xLoc, yLoc, SMALL, FCOLOR, BCOLOR);
    }
    return currentValue;
}

float DisplayChangeValueF(float prevValue, float currentValue, uint8_t xLoc, uint8_t yLoc)
{
    static char strTemp[15];
    if(prevValue != currentValue)
    {
        ftostr(strTemp, currentValue, 6);
        LcdSetRect(xLoc, yLoc, xLoc + 8, SCREEN_MAX, FILL, BCOLOR);
        LcdPutStr(strTemp, xLoc, yLoc, SMALL, FCOLOR, BCOLOR);
    }
    return currentValue; 

}

char* ftostr( char* buffer, float value, int places )
{
    int whole ;
    int fraction ;
    char sign[2] = "" ;

    if( value < 0 )
    {
        value = -value ;
        sign[0] = '-' ;
        sign[1] = '\0' ;
    }

    whole = (int)value ;
    fraction = (int)((value - whole) * powf(10.0f,places) + 0.5f) ;
    sprintf( buffer, "%s%d.%*.*d", sign, whole, places, places, fraction);

    return buffer;
}

void DisplayVelocity(void)
{
    uint16_t vc[SIZE_VEL_ARR];
    ROSGetVelocityCmd(vc);    

    if(Switched)
    {
        LcdClearScreen(BCOLOR);
        LcdPutStr("VELOCITY CMD", 0, 0, LARGE, FCOLOR, BCOLOR);
        LcdSetLine(18, 0, 18, 131, FCOLOR);
        LcdPutStr("LIN VEL:", 24, 0, SMALL, FCOLOR, BCOLOR);
        LcdPutStr("ANG VEL:", 36, 0, SMALL, FCOLOR, BCOLOR);
        Switched = 0;
    }
    
    currentVC[0] = DisplayChangeValueS(currentVC[0], vc[0], 24, 52);
    currentVC[1] = DisplayChangeValueS(currentVC[1], vc[1], 36, 52);
}

void DisplayIMU(void)
{
    uint8_t i = 0;
    float imuD[SIZE_IMU_ARR];
    ROSGetImuData(imuD);

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
        currentImuData[i] = DisplayChangeValueF(currentImuData[i], imuD[i], \
            24 + 12 * i, 36);
}

void DisplayGPS(void)
{
    uint8_t i = 0;
    float gpsD[SIZE_GPS_DATA_ARR];
    ROSGetGpsData(gpsD);
    uint8_t gpsS[SIZE_GPS_STAT_ARR];
    ROSGetGpsStatus(gpsS);
    
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
    
/*    for(i = 0; i < SIZE_GPS_DATA_ARR; i++)*/
/*        currentGpsData[i] = DisplayChangeValueF(currentGpsData[i], gpsD[i], \*/
/*            24 + 12 * i, 36);*/
/*            */
/*    for(i = 0; i < SIZE_GPS_STAT_ARR; i++)*/
/*        currentGpsData[i] = DisplayChangeValueS(currentGpsStat[i], gpsS[i], \*/
/*            72 + 12 * i, 36);*/
}

void DisplayEncoder(void)
{
    if(Switched)
    {    
        LcdClearScreen(BCOLOR);
        LcdPutStr("ENCODER DATA", 0, 0, LARGE, FCOLOR, BCOLOR);
	    LcdSetLine(18, 0, 18, 131, FCOLOR);
	    LcdPutStr("FR  :", 24, 0, SMALL, FCOLOR, BCOLOR);
	    LcdPutStr("FL  :", 36, 0, SMALL, FCOLOR, BCOLOR);
        LcdPutStr("RR  :", 48, 0, SMALL, FCOLOR, BCOLOR);
	    LcdPutStr("RL  :", 60, 0, SMALL, FCOLOR, BCOLOR);
	    LcdPutStr("LINV:", 72, 0, SMALL, FCOLOR, BCOLOR);
	    LcdPutStr("ANGV:", 84, 0, SMALL, FCOLOR, BCOLOR);
        Switched = 0;
    }
}
