/*
 * timer.h
 *
 *  Created on: Feb 27, 2011
 *      Author: titus
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "FreeRTOS.h"
#include "app_types.h"

#define PCONP_PCTIM0			(1<<1)		//!< Power bit for timer0
#define PCONP_PCTIM1			(1<<2)		//!< Power bit for timer1
#define PCONP_PCTIM2			(1<<22)		//!< Power bit for timer2
#define PCONP_PCTIM3			(1<<23)		//!< Power bit for timer3
#define PCLKSEL0_TIM0_DIV1		(1<<2)		//!< Peripheral Clk is CCLK/1
#define PCLKSEL0_TIM1_DIV1		(1<<4)		//!< Peripheral Clk is CCLK/1
#define PCLKSEL1_TIM2_DIV1		(1<<12)		//!< Peripheral Clk is CCLK/1
#define PCLKSEL1_TIM3_DIV1		(1<<14)		//!< Peripheral Clk is CCLK/1
#define PINSEL7_CAP00			(2<<14)		//!< P3.23 is CAP0.0
#define PINSEL7_CAP01			(2<<16)		//!< P3.24 is CAP0.1
#define PINSEL1_CAP30			(3<<14)		//!< P0.23 is CAP3.0
#define PINSEL1_CAP31			(3<<16)		//!< P0.24 is CAP3.1

// Timer control register bits
#define TCR_CE					(1<<0)		//!< Counter Enable
#define TCR_CR					(1<<1)		//!< Counter Reset

// Capture control register bits
#define CCR_CAP0RE				(1<<0)		//!< Capture CAPn.0 on Rising Edge
#define CCR_CAP0FE				(1<<1)		//!< Capture CAPn.0 on Falling Edge
#define CCR_CAP0I				(1<<2)		//!< Enable CAPn.0 Interrupt
#define CCR_CAP1RE				(1<<3)		//!< Capture CAPn.1 on Rising Edge
#define CCR_CAP1FE				(1<<4)		//!< Capture CAPn.1 on Falling Edge
#define CCR_CAP1I				(1<<5)		//!< Enable CAPn.1 Interrupt

// Count control register bits
#define CTCR_TM					(0<<0)		//!< Select Timer Mode
#define CTCR_CM_R				(1<<0)		//!< Select Counter mode on Rising edge
#define CTCR_CM_F				(2<<0)		//!< Select Counter mode on Falling edge
#define CTCR_CM_RF				(3<<0)		//!< Select Counter mode on Both edges
#define CTCR_CAP_SEL_0			(0<<2)		//!< Select 0 channel for counter mode
#define CTCR_CAP_SEL_1			(1<<2)		//!< Select 1 channel for counter mode

// Match control register bits
#define MCR_MR0I				(1<<0)		//!< Enable match 0 interrupt
#define MCR_MR0R				(1<<1)		//!< Enable reset on match 0
#define MCR_MR0S				(1<<2)		//!< Enable stop on match 0
#define MCR_MR1I				(1<<3)		//!< Enable match 1 interrupt
#define MCR_MR1R				(1<<4)		//!< Enable reset on match 1
#define MCR_MR1S				(1<<5)		//!< Enable stop on match 1
#define MCR_MR2I				(1<<6)		//!< Enable match 2 interrupt
#define MCR_MR2R				(1<<7)		//!< Enable reset on match 2
#define MCR_MR2S				(1<<8)		//!< Enable stop on match 2
#define MCR_MR3I				(1<<9)		//!< Enable match 3 interrupt
#define MCR_MR3R				(1<<10)		//!< Enable reset on match 3
#define MCR_MR3S				(1<<11)		//!< Enable stop on match 3

// Interrupt register bits
#define IR_MR0					(1<<0)		//!< MR0 Interrupt Bit
#define IR_MR1					(1<<1)		//!< MR1 Interrupt Bit
#define IR_MR2					(1<<2)		//!< MR2 Interrupt Bit
#define IR_MR3					(1<<3)		//!< MR3 Interrupt Bit
#define IR_CAP0					(1<<4)		//!< CAP0 Interrupt Bit
#define IR_CAP1					(1<<5)		//!< CAP1 Interrupt Bit

// PR Times
#define PR_1MS					0x11705		//!< 0x11705 / 72MHz ~= 1ms

// MCR Times
#define MCR_10MS				720000		//!< 720kHz / 72MHz = 10ms
#define MCR_20MS				1440000		//!< 1.44MHz / 72MHz = 20ms
#define MCR_1S					1000		//!< 1ms * 1000 = 1s
#define MCR_2S					2000		//!< 1ms * 2000 = 2s
#define MCR_10S					10000		//!< 1ms * 10000 = 10s

#endif /* TIMER_H_ */
