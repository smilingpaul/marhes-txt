#ifndef DISPLAY_H_
#define DISPLAY_H_

#include "app_types.h"
#include "armVIC.h"
#include "LPC23xx.h"
#include "lcd.h"

/*************************************************************************
 *             Definitions
 *************************************************************************/

#define DISPLAY_IMAGE		0
#define DISPLAY_VEL		    1
#define DISPLAY_IMU		    2
#define DISPLAY_GPS		    3
#define DISPLAY_ENCODER		4

#define FCOLOR			    WHITE
#define BCOLOR			    BLACK

#define DISPLAY_MIN         0
#define DISPLAY_MAX         4

/*************************************************************************
 *             Function declarations
 *************************************************************************/

void DisplayInit(void);
void DisplaySwitch(signed char dispNum);
unsigned char DisplayGetState(void);
void DisplayVelocity(void);
void DisplayIMU(void);
void DisplayGPS(void);
void DisplayEncoder(void);

#endif
