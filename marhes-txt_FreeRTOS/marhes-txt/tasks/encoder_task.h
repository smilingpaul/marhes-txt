/**
 @file encoder_task.h
  
 @brief Defines the encoder constants and declares functions
 
 @author Titus Appel

 @version 1.0

 @date 2011/06/03

 Contact: titus.appel@gmail.com
*/

#ifndef ENCODER_TASK_H_
#define ENCODER_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "app_types.h"
#include "drivers/timer.h"
#include "tasks/rosiface_task.h"
#include "tasks/mode_task.h"

// Encoder channels
#define TICKS_FR                0      //!< Front right encoder channel
#define TICKS_FL                1      //!< Front left encoder channel
#define TICKS_RR                2      //!< Rear right encoder channel
#define TICKS_RL                3      //!< Rear left encoder channel

// Encoder Direction Inputs
#define DIR_LEFT                (1<<0) //!< Left Encoder Direction Input
#define DIR_RIGHT               (1<<1) //!< Right Encoder Direction Input

// Pos Matrix indices
#define POS_X                   0      //!< X Position Matrix Index
#define POS_Y                   1      //!< Y Position Matrix Index
#define POS_T                   2      //!< Theta Position Matrix Index

// Vel indices
#define VELS_LINEAR             0      //!< Linear Velocity Matrix Index
#define VELS_ANGULAR            1      //!< Angular Velocity Matrix Index

// Array sizes
#define SIZE_ENCODER_TICKS_ARR	2	     //!< Ticks array size
#define SIZE_ENCODER_VEL_ARR	  2      //!< Velocity array size
#define SIZE_ENCODER_POS_ARR	  3      //!< Velocity array size

void vEncoderTaskStart(void);
void EncoderInit(void);
int8_t EncoderGetDirection(uint8_t channel);
int32_t EncoderLinVel(void);
int32_t EncoderAngVel(void);

#endif
