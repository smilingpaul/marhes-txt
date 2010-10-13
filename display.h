#ifndef DISPLAY_H_
#define DISPLAY_H_

#include "app_types.h"
#include "armVIC.h"
#include "LPC23xx.h"
#include "lcd.h"
#include "ROSIFace.h"
#include "helperFuncs.h"

/*************************************************************************
 *             Definitions
 *************************************************************************/

#define DISPLAY_IMAGE		0
#define DISPLAY_VEL		    1
#define DISPLAY_IMU		    2
#define DISPLAY_GPS		    3
#define DISPLAY_ENCODER		4

#define FCOLOR			    YELLOW
#define BCOLOR			    BLACK

#define SCREEN_MAX          131

#define DISPLAY_MIN         0
#define DISPLAY_MAX         4

/*************************************************************************
 *             Function declarations
 *************************************************************************/

void DisplayInit(void);
void DisplaySetState(signed char dispNum);
unsigned char DisplayGetState(void);
uint16_t DisplayChangeValueS(uint16_t prevValue, uint16_t currentValue, \
    uint8_t xLoc, uint8_t yLoc);
float DisplayChangeValueF(float prevValue, float currentValue, char* str, \
    uint8_t xLoc, uint8_t yLoc);
void DisplayUpdate(void);
void DisplayVelocity(void);
void DisplayIMU(void);
void DisplayGPS(void);
void DisplayEncoder(void);

#endif
