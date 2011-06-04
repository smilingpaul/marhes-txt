#ifndef DISPLAY_H_
#define DISPLAY_H_

#include "app_types.h"
#include "LPC23xx.h"
#include "lcd.h"
#include "ROSIFace.h"
#include "encoder.h"
#include "battery.h"
#include "helperFuncs.h"
#include "controller.h"

/*************************************************************************
 *             Definitions
 *************************************************************************/

#define DISPLAY_STATUS		0
#define DISPLAY_VEL		    1
#define DISPLAY_ENCODER		2

#define DISPLAY_MIN         0
#define DISPLAY_MAX         2

#define FCOLOR			    YELLOW
#define BCOLOR			    BLACK

#define SCREEN_MAX          131



/*************************************************************************
 *             Function declarations
 *************************************************************************/

void DisplayInit(void);
void DisplaySetState(int8_t dispNum);
uint8_t DisplayGetState(void);
//int16_t DisplayChangeValueS(int16_t prevValue, int16_t currentValue, uint8_t xLoc, uint8_t yLoc);
//float DisplayChangeValueF(float prevValue, float currentValue, char* str, uint8_t xLoc, uint8_t yLoc);
void DisplayUpdate(void);
void DisplayStatus(void);
//void DisplayVelocity(void);
//void DisplayIMU(void);
//void DisplayGPS(void);
//void DisplayEncoder(void);

#endif
