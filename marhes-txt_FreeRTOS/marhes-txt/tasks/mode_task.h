/**
 @file mode_task.h
  
 @brief Provides function declarations to update the modes of operation.
 
 @author Titus Appel

 @version 1.0

 @date 2011/06/03

 Contact: titus.appel@gmail.com
*/

#ifndef MODE_TASK_H_
#define MODE_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "serial.h"
#include "tasks/battery_task.h"

void vModeTaskStart(void);
void vOdomCombRx(void);
void vModeCmdVelRx(void);
unsigned portBASE_TYPE ModeUseOdomComb(void);
unsigned portBASE_TYPE ModeStopLostConn(void);

#endif
