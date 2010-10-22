/*
 * controller.h
 *
 *  Created on: Aug 12, 2010
 *      Author: Titus
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "app_types.h"
#include "LPC23xx.h"
#include "pwm.h"
#include "ROSIFace.h"

/*************************************************************************
 *             Definitions
 *************************************************************************/

// The PWM channels of the different motors
#define MOTOR_CHANNEL				1
#define FRONT_SERVO_CHANNEL			2
#define REAR_SERVO_CHANNEL			3

// The maximum and minimum velocities accepted from ROS
#define VELOCITY_MAX 				1600
#define VELOCITY_MIN 				-1600
#define THETA_MAX 					1600
#define THETA_MIN  					-1600

// The maximum and minimum PWM values for the servos/motor controller
#define VELOCITY_PWM_MAX			144000	// PWM - Freq = 0x15F900 (1440000)
#define VELOCITY_PWM_MIN			72000
#define THETA_PWM_MAX				144000	// PWM - Freq = 0x15F900 (1440000)
#define THETA_PWM_MIN				72000

/*************************************************************************
 *             Function declarations
 *************************************************************************/

void ControllerInit(void);
void ControllerCalcPID(void);
uint32_t ControllerCalcPWM(uint16_t channel);
void ControllerSetTheta(int16_t value);
int16_t ControllerGetTheta(void);
void ControllerSetVelocity(int16_t value);
int16_t ControllerGetVelocity(void);

#endif /* CONTROLLER_H_ */
