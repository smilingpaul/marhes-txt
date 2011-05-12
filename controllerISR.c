/*
 * controllerISR.c
 *
 *  Created on: Mar 29, 2011
 *      Author: Titus
 */

#include "controller.h"

//extern int32_t odomCombined[];
extern int32_t vels[];
extern boolean UseOdomComb, StopLostConn;
extern int16_t linVelocity, angVelocity;
extern int32_t combLinVelocity, combAngVelocity;

int32_t e_lv_last = 0, e_av_last = 0, e_lv_sum = 0, e_av_sum = 0;
int32_t u_lv = 0, u_av = 0;
float kp_lv = 500, ki_lv = 0, kd_lv = 0;
float kp_av = 0.5, ki_av = 0.5, kd_av = 0.5;

void ControllerPIDLoop(void)
{
	int32_t e_lv, e_av;

	if (StopLostConn == false)
	{
		// Get the error signals
		if (UseOdomComb)
		{
			e_lv = linVelocity - combLinVelocity;
			e_av = angVelocity - combAngVelocity;
		}
		else
		{
			e_lv = linVelocity - vels[0];
			e_av = angVelocity - vels[1];
		}

		e_lv_sum += e_lv;
		e_av_sum += e_av;

		// Calculate the PID linear velocity control signal
		u_lv = (int32_t)(kp_lv * e_lv + ki_lv * e_lv_sum + kd_lv * (e_lv - e_lv_last));
		u_av = (int32_t)(kp_av * e_av + ki_av * e_av_sum + kd_av * (e_av - e_av_last));

		// Set the PWM duty cycles for the motor and the steering servos
		PWMSetDuty(MOTOR_CHANNEL, DUTY_1_5 + u_lv);
	//	PWMSetDuty(FRONT_SERVO_CHANNEL, ControllerCalcPWM(FRONT_SERVO_CHANNEL));
	//	PWMSetDuty(REAR_SERVO_CHANNEL, ControllerCalcPWM(REAR_SERVO_CHANNEL));

		// Store last velocity errors
		e_lv_last = e_lv;
		e_av_last = e_av;
	}
	else
	{
		PWMSetDuty(MOTOR_CHANNEL, DUTY_1_5);
		e_lv_last = 0;
		e_av_last = 0;
		e_lv_sum = 0;
		e_av_sum = 0;
		u_lv = 0;
		u_av = 0;
	}
}
