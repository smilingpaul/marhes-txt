#ifndef DISPLAY_TASK_H_
#define DISPLAY_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "drivers/lcd.h"
#include "tasks/mode_task.h"
#include "tasks/battery_task.h"
#include "tasks/controller_task.h"
#include "helperFuncs.h"

#define DISPLAY_STATUS		  0
#define DISPLAY_VEL		      1
#define DISPLAY_ENCODER		  2

#define DISPLAY_MIN         0
#define DISPLAY_MAX         2

#define FCOLOR			        YELLOW
#define BCOLOR			        BLACK

#define SCREEN_MAX          131

void vDisplayTaskStart(void);
void DisplayInit(void);
void DisplayDecreaseState(void);
void DisplayIncreaseState(void);
void DisplayUpdate(void);
void DisplayBrightUpdate(void);
void DisplayStatus(void);
void Display2(void);
void Display3(void);

#endif
