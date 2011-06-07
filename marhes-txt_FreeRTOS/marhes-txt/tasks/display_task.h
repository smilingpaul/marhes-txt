/**
 @file display_task.h
  
 @brief Includes function declarations for the display task.
 
 @author Titus Appel

 @version 1.0

 @date 2011/06/03

 Contact: titus.appel@gmail.com
*/

#ifndef DISPLAY_TASK_H_
#define DISPLAY_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "drivers/lcd.h"
#include "tasks/mode_task.h"
#include "tasks/battery_task.h"
#include "tasks/controller_task.h"
#include "helperFuncs.h"

#define DISPLAY_STATUS		  0             ///< Status display type
#define DISPLAY_PWM		      1             ///< PWM display type
#define DISPLAY_ENCODER		  2             ///< Encoder display type

#define DISPLAY_MIN         0             ///< Min display type
#define DISPLAY_MAX         2             ///< Max display type

#define FCOLOR			        YELLOW        ///< Foreground color
#define BCOLOR			        BLACK         ///< Background color

#define SCREEN_MAX          131           ///< Maximum screen address

void vDisplayTaskStart(void);
void DisplayInit(void);
void DisplayDecreaseState(void);
void DisplayIncreaseState(void);
void DisplayUpdate(void);
void DisplayBrightUpdate(void);
void DisplayStatus(void);
void DisplayPWM(void);
void Display3(void);

#endif
