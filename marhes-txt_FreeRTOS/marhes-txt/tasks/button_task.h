/**
 @file button_task.h
  
 @brief Includes the bit definitions that tell whether a button is pressed and 
        function declarations.
 
 @author Titus Appel

 @version 1.0

 @date 2011/06/03

 Contact: titus.appel@gmail.com
*/

#ifndef BUTTON_TASK_H_
#define BUTTON_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "app_types.h"
#include "serial.h"
#include "tasks/display_task.h"

#define BUT_BUT1        (1<<29) ///< The bit in the PORT1 register of the button
#define BUT_BUT2        (1<<18) ///< The bit in the PORT1 register of the button
#define BUT_CENTER      (1<<25) ///< The bit in the PORT1 register of the button
#define BUT_UP          (1<<18) ///< The bit in the PORT1 register of the button
#define BUT_DOWN        (1<<19) ///< The bit in the PORT1 register of the button
#define BUT_RIGHT       (1<<22) ///< The bit in the PORT1 register of the button
#define BUT_LEFT        (1<<27) ///< The bit in the PORT1 register of the button

#define BUT_BUT1_SHFT   (29-0)  ///< The shifted bit in button_changed
#define BUT_BUT2_SHFT   (18-1)  ///< The shifted bit in button_changed
#define BUT_CENTER_SHFT (25-2)  ///< The shifted bit in button_changed
#define BUT_UP_SHFT     (18-3)  ///< The shifted bit in button_changed
#define BUT_DOWN_SHFT   (19-4)  ///< The shifted bit in button_changed
#define BUT_RIGHT_SHFT  (22-5)  ///< The shifted bit in button_changed
#define BUT_LEFT_SHFT   (27-6)  ///< The shifted bit in button_changed

#define BUT_BUT1_BIT    (1<<0)  ///< The button's bit in button_changed
#define BUT_BUT2_BIT    (1<<1)  ///< The button's bit in button_changed 
#define BUT_CENTER_BIT  (1<<2)  ///< The button's bit in button_changed
#define BUT_UP_BIT      (1<<3)  ///< The button's bit in button_changed 
#define BUT_DOWN_BIT    (1<<4)  ///< The button's bit in button_changed  
#define BUT_RIGHT_BIT   (1<<5)  ///< The button's bit in button_changed
#define BUT_LEFT_BIT    (1<<6)  ///< The button's bit in button_changed

void vButtonTaskStart(void);
void ButtonInit(void);
uint8_t ButtonGetChangedHigh(void);

#endif
