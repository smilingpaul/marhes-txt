#ifndef BUTTON_TASK_H_
#define BUTTON_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "app_types.h"
#include "serial.h"
#include "tasks/display_task.h"

#define BUT_BUT1        (1<<29)
#define BUT_BUT2        (1<<18)
#define BUT_CENTER      (1<<25)
#define BUT_UP          (1<<18)
#define BUT_DOWN        (1<<19)
#define BUT_RIGHT       (1<<22)
#define BUT_LEFT        (1<<27)

#define BUT_BUT1_SHFT   (29-0)
#define BUT_BUT2_SHFT   (18-1)
#define BUT_CENTER_SHFT (25-2)
#define BUT_UP_SHFT     (18-3)
#define BUT_DOWN_SHFT   (19-4)
#define BUT_RIGHT_SHFT  (22-5)
#define BUT_LEFT_SHFT   (27-6)

#define BUT_BUT1_BIT    (1<<0)  
#define BUT_BUT2_BIT    (1<<1)   
#define BUT_CENTER_BIT  (1<<2)
#define BUT_UP_BIT      (1<<3)   
#define BUT_DOWN_BIT    (1<<4)    
#define BUT_RIGHT_BIT   (1<<5)  
#define BUT_LEFT_BIT    (1<<6)  

void vButtonTaskStart(void);
void ButtonInit(void);
uint8_t ButtonGetChangedHigh(void);

#endif
