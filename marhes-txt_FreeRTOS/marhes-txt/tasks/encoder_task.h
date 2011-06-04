#ifndef ENCODER_TASK_H_
#define ENCODER_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "app_types.h"
#include "drivers/timer.h"
#include "tasks/rosiface_task.h"

// Encoder channels
#define TICKS_FR                0   //!< Front right encoder channel
#define TICKS_FL                1   //!< Front left encoder channel
#define TICKS_RR                2   //!< Rear right encoder channel
#define TICKS_RL                3   //!< Rear left encoder channel

// Encoder Direction Inputs
#define DIR_LEFT                (1<<0)
#define DIR_RIGHT               (1<<1)

#define POS_X                   0
#define POS_Y                   1
#define POS_T                   2

// Vel indexes
#define VELS_LINEAR             0
#define VELS_ANGULAR            1

// Array sizes
#define SIZE_ENCODER_TICKS_ARR	2	  //!< Ticks array size
#define SIZE_ENCODER_VEL_ARR	  2   //!< Velocity array size
#define SIZE_ENCODER_POS_ARR	  3   //!< Velocity array size

void vEncoderTaskStart(void);
void EncoderInit(void);
int8_t EncoderGetDirection(uint8_t channel);
int32_t EncoderLinVel(void);
int32_t EncoderAngVel(void);

#endif
