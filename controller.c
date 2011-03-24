/*
 * controller.c
 *
 *  Created on: Aug 12, 2010
 *      Author: Titus
 */

#include "controller.h"

extern int32_t odomCombined[];
extern int32_t vels[];
extern float pos[];

boolean UseOdomComb = false, StopLostConn = true;

//static int16_t theta = 0;
//static int16_t velocity = 0;
//static float velocitySlope, velocityInt, thetaSlope, thetaInt;

static int16_t linVelocity = 0, angVelocity = 0;

void ControllerInit(void)
{
//	velocitySlope = ((VELOCITY_PWM_MIN - VELOCITY_PWM_MAX) / (VELOCITY_MIN - VELOCITY_MAX));
//	velocityInt =  VELOCITY_PWM_MAX - velocitySlope * VELOCITY_MAX;
//	thetaSlope = (THETA_PWM_MIN - THETA_PWM_MAX) / (THETA_MIN - THETA_MAX);
//	thetaInt = THETA_PWM_MAX - thetaSlope * THETA_MAX;

}

void ControllerPIDLoop(void)
{

}

void ControllerSetLinearVelocity(int16_t value)
{
	if (value > LIN_VEL_MAX)
		linVelocity = LIN_VEL_MAX;
	else if (value < LIN_VEL_MIN)
		linVelocity = LIN_VEL_MIN;
	else
		linVelocity = value;
}

int16_t ControllerGetLinearVelocity(void)
{
	return linVelocity;
}

void ControllerSetAngularVelocity(int16_t value)
{
	if (value > ANG_VEL_MAX)
		angVelocity = ANG_VEL_MAX;
	else if (value < ANG_VEL_MIN)
		angVelocity = ANG_VEL_MIN;
	else
		angVelocity = value;
}

int16_t ControllerGetAngularVelocity(void)
{
	return angVelocity;
}

//void ControllerCalcPID(void)
//{
//	// Just test code
//	ControllerSetVelocity(ROSGetVelocityCmd(ROS_LINEAR_VEL));
//	ControllerSetTheta(ROSGetVelocityCmd(ROS_ANGULAR_VEL));
//
//	PWMSetDuty(MOTOR_CHANNEL, ControllerCalcPWM(MOTOR_CHANNEL));
//	PWMSetDuty(FRONT_SERVO_CHANNEL, ControllerCalcPWM(FRONT_SERVO_CHANNEL));
//	PWMSetDuty(REAR_SERVO_CHANNEL, ControllerCalcPWM(REAR_SERVO_CHANNEL));
//}
//
//uint32_t ControllerCalcPWM(uint16_t channel)
//{
//	uint32_t temp;
//
//	if (StopLostConn)
//		channel = DEFAULT_CHANNEL;
//
//	switch(channel)
//	{
//		case MOTOR_CHANNEL:
//			temp = (uint32_t)(velocitySlope * velocity + velocityInt);
//			break;
//		case FRONT_SERVO_CHANNEL:
//			temp = (uint32_t)(-thetaSlope * theta + thetaInt);
//			break;
//		case REAR_SERVO_CHANNEL:
//			temp = (uint32_t)(thetaSlope * theta + thetaInt);
//			break;
//		default:
//			temp = DUTY_1_5;
//			break;
//	}
//
//	return temp;
//}
//
//void ControllerSetTheta(int16_t value)
//{
//	if (value > THETA_MAX)
//		theta = THETA_MAX;
//	else if (value < THETA_MIN)
//		theta = THETA_MIN;
//	else
//		theta = value;
//}
//
//int16_t ControllerGetTheta(void)
//{
//	return theta;
//}
//
//void ControllerSetVelocity(int16_t value)
//{
//	if (value > VELOCITY_MAX)
//		velocity = VELOCITY_MAX;
//	else if (value < VELOCITY_MIN)
//		velocity = VELOCITY_MIN;
//	else
//		velocity = value;
//}
//
//int16_t ControllerGetVelocity(void)
//{
//	return velocity;
//}
