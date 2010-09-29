#include "display.h"

static unsigned char DisplayState = DISPLAY_IMAGE;

void DisplayInit(void)
{
    LcdClearScreen(BCOLOR);
	// First start displaying the TXT picture
	//LcdImage();
	DisplayIMU();
}

void DisplaySwitch(signed char dispNum)
{
    signed char temp = dispNum;
    
    if (temp < DISPLAY_MIN)
        temp = DISPLAY_MAX;
        
    if (temp > DISPLAY_MAX)
        temp = DISPLAY_MIN;
    
    LcdClearScreen(BCOLOR);
    
    switch(temp)
    {
	    case DISPLAY_IMAGE:
            DisplayState = DISPLAY_IMAGE;
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

unsigned char DisplayGetState(void)
{
    return DisplayState;
}

void DisplayVelocity(void)
{
    DisplayState = DISPLAY_VEL;
	LcdPutStr("VELOCITY CMD", 0, 0, LARGE, FCOLOR, BCOLOR);
	LcdSetLine(18, 0, 18, 131, FCOLOR);
	LcdPutStr("LIN VEL:", 24, 0, SMALL, FCOLOR, BCOLOR);
	LcdPutStr("ANG VEL:", 36, 0, SMALL, FCOLOR, BCOLOR);
/*	LcdSetRect(42, 0, 42, 131, FILL, BCOLOR);*/
}

void DisplayIMU(void)
{
    DisplayState = DISPLAY_IMU;
	LcdPutStr("IMU DATA", 0, 0, LARGE, FCOLOR, BCOLOR);
	LcdSetLine(18, 0, 18, 131, FCOLOR);
	LcdPutStr("ROTX:", 24, 0, SMALL, FCOLOR, BCOLOR);
	LcdPutStr("ROTY:", 36, 0, SMALL, FCOLOR, BCOLOR);
    LcdPutStr("ROTZ:", 48, 0, SMALL, FCOLOR, BCOLOR);
	LcdPutStr("ROTW:", 60, 0, SMALL, FCOLOR, BCOLOR);
	LcdPutStr("LINX:", 72, 0, SMALL, FCOLOR, BCOLOR);
	LcdPutStr("LINY:", 84, 0, SMALL, FCOLOR, BCOLOR);
    LcdPutStr("LINZ:", 96, 0, SMALL, FCOLOR, BCOLOR);    
/*	LcdSetRect(104, 0, 104, 131, FILL, BCOLOR);*/
}

void DisplayGPS(void)
{
    DisplayState = DISPLAY_GPS;
	LcdPutStr("GPS DATA", 0, 0, LARGE, FCOLOR, BCOLOR);
	LcdSetLine(18, 0, 18, 131, FCOLOR);
	LcdPutStr("LAT :", 24, 0, SMALL, FCOLOR, BCOLOR);
	LcdPutStr("LONG:", 36, 0, SMALL, FCOLOR, BCOLOR);
    LcdPutStr("ALT :", 48, 0, SMALL, FCOLOR, BCOLOR);
	LcdPutStr("HEAD:", 60, 0, SMALL, FCOLOR, BCOLOR);
	LcdPutStr("STAT:", 72, 0, SMALL, FCOLOR, BCOLOR);
	LcdPutStr("VIS :", 84, 0, SMALL, FCOLOR, BCOLOR);
    LcdPutStr("USED:", 96, 0, SMALL, FCOLOR, BCOLOR);    
/*	LcdSetRect(104, 0, 104, 131, FILL, BCOLOR);*/
}

void DisplayEncoder(void)
{
    DisplayState = DISPLAY_ENCODER;
	LcdPutStr("ENCODER DATA", 0, 0, LARGE, FCOLOR, BCOLOR);
	LcdSetLine(18, 0, 18, 131, FCOLOR);
	LcdPutStr("FR  :", 24, 0, SMALL, FCOLOR, BCOLOR);
	LcdPutStr("FL  :", 36, 0, SMALL, FCOLOR, BCOLOR);
    LcdPutStr("RR  :", 48, 0, SMALL, FCOLOR, BCOLOR);
	LcdPutStr("RL  :", 60, 0, SMALL, FCOLOR, BCOLOR);
	LcdPutStr("LINV:", 72, 0, SMALL, FCOLOR, BCOLOR);
	LcdPutStr("ANGV:", 84, 0, SMALL, FCOLOR, BCOLOR);
/*    LcdSetRect(92, 0, 92, 131, FILL, BCOLOR);*/
}
