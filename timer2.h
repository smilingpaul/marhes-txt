/*
 * timer2.h
 *
 *  Created on: Feb 27, 2011
 *      Author: titus
 */

#ifndef TIMER2_H_
#define TIMER2_H_

#include "app_types.h"
#include "LPC23xx.h"
#include "ROSIFace.h"
#include "adc.h"
#include "battery.h"

#define ODOM_COMB_LIMIT				2
#define CMD_VEL_LIMIT				1

void Timer2Init(void);
void Timer2ISR(void);

#endif /* TIMER2_H_ */
