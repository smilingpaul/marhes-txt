/*
 * controller.c
 *
 *  Created on: Aug 12, 2010
 *      Author: Titus
 */

#include "controller.h"

float theta = 0;
float velocity = 0;
float velocitySlope, velocityInt, thetaSlope, thetaInt;

void ControllerInit(void)
{
	velocitySlope = ((VELOCITY_PWM_MIN - VELOCITY_PWM_MAX) / (VELOCITY_MIN - VELOCITY_MAX));
	velocityInt =  VELOCITY_PWM_MAX - velocitySlope * VELOCITY_MAX;
	thetaSlope = (THETA_PWM_MIN - THETA_PWM_MAX) / (THETA_MIN - THETA_MAX);
	thetaInt = THETA_PWM_MAX - thetaSlope * THETA_MAX;
}

void ControllerCalcPID(void)
{
	// Just test code
	PWMSetDuty(MOTOR_CHANNEL, ControllerCalcPWM(MOTOR_CHANNEL));
	PWMSetDuty(FRONT_SERVO_CHANNEL, ControllerCalcPWM(FRONT_SERVO_CHANNEL));
	//PWMSetDuty(REAR_SERVO_CHANNEL, ControllerCalcPWM(REAR_SERVO_CHANNEL));
}

uint32_t ControllerCalcPWM(uint16_t channel)
{
	uint32_t temp;

	switch(channel)
	{
		case MOTOR_CHANNEL:
			temp = velocitySlope * velocity + velocityInt;
			break;
		case FRONT_SERVO_CHANNEL:
			temp = thetaSlope * theta + thetaInt;
			break;
		case REAR_SERVO_CHANNEL:
			temp = thetaSlope * theta + thetaInt;
			break;
		default:
			temp = DUTY_1_5;
			break;
	}

	return temp;
}

void ControllerSetTheta(float value)
{
	if (value > THETA_MAX)
		theta = THETA_MAX;
	else if (value < THETA_MIN)
		theta = THETA_MIN;
	else
		theta = value;
}

float ControllerGetTheta(void)
{
	return theta;
}

void ControllerSetVelocity(float value)
{
	if (value > VELOCITY_MAX)
		velocity = VELOCITY_MAX;
	else if (value < VELOCITY_MIN)
		velocity = VELOCITY_MIN;
	else
		velocity = value;
}

float ControllerGetVelocity(void)
{
	return velocity;
}

void ControllerTestMotors(void)
{
	unsigned long counter;
	unsigned long duty = THETA_PWM_MIN;
	while(duty <= THETA_PWM_MAX)
	{
		//PWMSetDuty(MOTOR_CHANNEL, duty);
		PWMSetDuty(FRONT_SERVO_CHANNEL, duty);
		duty += 50;
		for ( counter=0; counter<0x00005000; counter++ );
	}
	PWMSetDuty(MOTOR_CHANNEL, DUTY_1_5);
	PWMSetDuty(FRONT_SERVO_CHANNEL, DUTY_1_5);
}
