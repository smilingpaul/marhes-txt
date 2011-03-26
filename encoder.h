/*! \file encoder.h
    \brief Encoder timer defines.

    \date Jun 19, 2010\n
	\author Titus Appel <titus.appel@gmail.com>\n
	\par Institution:
	UNM-ECE Department\n
	MARHES Lab

   	This file includes the bit values for the timers and other constants.
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "app_types.h"
//#include "armVIC.h"
#include "LPC23xx.h"
#include "ROSIFace.h"
#include "timer.h"
#include "controller.h"

/*************************************************************************
 *             Definitions
 *************************************************************************/

// Encoder channels
#define FRONT_RIGHT				0			//!< Front right encoder channel
#define FRONT_LEFT				1			//!< Front left encoder channel
#define REAR_RIGHT				2			//!< Rear right encoder channel
#define REAR_LEFT				3			//!< Rear left encoder channel

// Encoder Direction Inputs
#define LEFT_IN					(1<<0)
#define RIGHT_IN				(1<<1)

// Array sizes
#define SIZE_ENCODER_TICKS_ARR	2			//!< Ticks array size
#define SIZE_ENCODER_VEL_ARR	2			//!< Velocity array size
#define SIZE_ENCODER_POS_ARR	3			//!< Velocity array size

/*************************************************************************
 *             Function declarations
 *************************************************************************/

void EncoderInit(void);
int32_t EncoderCount(uint8_t channel);
int32_t EncoderVel(uint8_t channel);
int8_t EncoderGetDirection(uint8_t channel);
void EncoderISR(void);
int32_t anglookuptable(int32_t degrees, int32_t type);

#endif /* ENCODER_H_ */
