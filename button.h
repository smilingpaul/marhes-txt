#ifndef BUTTON_H_
#define BUTTON_H_

#include "app_types.h"
#include "armVIC.h"
#include "LPC23xx.h"

/*************************************************************************
 *             Definitions
 *************************************************************************/

#define BUT_BUT1        (1<<29)
#define BUT_BUT2        (1<<18)
#define BUT_CENTER      (1<<25)
#define BUT_UP          (1<<18)
#define BUT_DOWN        (1<<19)
#define BUT_RIGHT       (1<<22)
#define BUT_LEFT        (1<<27)

#define BUT_BUT1_BIT    0
#define BUT_BUT2_BIT    1
#define BUT_CENTER_BIT  2
#define BUT_UP_BIT      3
#define BUT_DOWN_BIT    4
#define BUT_RIGHT_BIT   5
#define BUT_LEFT_BIT    6

/*************************************************************************
 *             Function declarations
 *************************************************************************/

void ButtonInit(void);
void ButtonUpdate(void);
unsigned char ButtonGetMask(void);

#endif
