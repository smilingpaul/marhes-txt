/**
 @file pwm.h
  
 @brief Includes PWM bit defines and functions declarations.
 
 @author Titus Appel

 @version 1.0

 @date 2010/05/29

 Contact: titus.appel@gmail.com
*/

#ifndef PWM_H_
#define PWM_H_

//#include "app_types.h"
#include "FreeRTOS.h"
#include "lpc23xx.h"

/*************************************************************************
 *             Definitions
 *************************************************************************/

#define PCPWM1 				0x0040		// PWM Power Control Bit
#define PCLKSEL0_PWM_DIV1 	(1<<12)		// PWM Peripheral Clock Sel-CLK/1
#define PINSEL4_PWM_EN		0x0155		// PWM Pin Enable First 5 Outputs
#define PINSEL3_PWM6		(2<<20)		// PWM Pin Enable #6 Output
#define TCR_CR				(1<<1)		// Reset PWM Counters
#define MR0_FREQ			(configCPU_CLOCK_HZ / 50)	// PWM Output Freq - 50Hz - 20ms
#define MCR_MR0R			(1<<1)		// Reset counters on MR0
#define PCR_OUT_EN			0x7E00		// Enable PWM outputs on P2.0-5
#define TCR_C_EN			(1<<0)		// Enable PWM Counter
#define TCR_PWM_EN			(1<<3)		// Enable PWM
#define DUTY_1_5			(configCPU_CLOCK_HZ * 0.0015)		// On time of 1.5 ms
#define LER_ALL				0x7F		// Latch all MRx registers

#define PWM_MAX       (configCPU_CLOCK_HZ * 0.002)
#define PWM_MIN       (configCPU_CLOCK_HZ * 0.001)

/*************************************************************************
 *             Function declarations
 *************************************************************************/

void PWMInit(void);
void PWMSetDuty(char channel, unsigned long duty);
unsigned long PWMGetDuty(char channel);

#endif /* PWM_H_ */
