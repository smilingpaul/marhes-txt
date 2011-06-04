#ifndef MODE_TASK_H_
#define MODE_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "serial.h"

void vModeTaskStart(void);
void vOdomCombRx(void);
void vModeCmdVelRx(void);
unsigned portBASE_TYPE ModeUseOdomComb(void);
unsigned portBASE_TYPE ModeStopLostConn(void);

#endif
