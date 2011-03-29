/*
 * controller.c
 *
 *  Created on: Aug 12, 2010
 *      Author: Titus
 */

#include "controller.h"

boolean UseOdomComb = false, StopLostConn = true;

//static int16_t theta = 0;
//static int16_t velocity = 0;
//static float velocitySlope, velocityInt, thetaSlope, thetaInt;

int16_t linVelocity = 0, angVelocity = 0;
int32_t e_lv_last = 0, e_av_last = 0, e_lv_sum = 0, e_av_sum = 0;
float kp_lv = 0.5, ki_lv = 0.5, kd_lv = 0.5;
float kp_av = 0.5, ki_av = 0.5, kd_av = 0.5;

void ControllerInit(void)
{
	// Most things done in encoder.c
	// Setup timer1 for a sample period of 20msec
	//T1TCR = TCR_CR;							// Reset timer1 counter
	//T1CTCR = CTCR_TM;						// Timer 1 is in timer mode
	T1MR1 = MCR_10MS;						// Match at 20ms
	T1MCR |= MCR_MR1I;						// On match interrupt

	// Setup T1 Interrupt
//	VICIntSelect &= ~VIC_CHAN_TO_MASK(VIC_CHAN_NUM_Timer1);	// Change to IRQ
//	VICIntEnClr |= VIC_CHAN_TO_MASK(VIC_CHAN_NUM_Timer1); 	// Disable interrupt
//	VICVectAddr5 = (uint32_t)(void *)EncoderISR;			// Assign the ISR
//	VICVectPriority5 = 0xE;									// Set the priority
//	VICIntEnable |= VIC_CHAN_TO_MASK(VIC_CHAN_NUM_Timer1);	// Enable the INT

//	// 6. Enable the Timer counters
//	T0TCR = TCR_CE;
//	T3TCR = TCR_CE;
//	T1TCR = TCR_CE;

//	velocitySlope = ((VELOCITY_PWM_MIN - VELOCITY_PWM_MAX) / (VELOCITY_MIN - VELOCITY_MAX));
//	velocityInt =  VELOCITY_PWM_MAX - velocitySlope * VELOCITY_MAX;
//	thetaSlope = (THETA_PWM_MIN - THETA_PWM_MAX) / (THETA_MIN - THETA_MAX);
//	thetaInt = THETA_PWM_MAX - thetaSlope * THETA_MAX;
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
