/*
 * controllerISR.c
 *
 *  Created on: Mar 29, 2011
 *      Author: Titus
 */

#include "controller.h"

extern int32_t odomCombined[];
extern int32_t vels[];
extern boolean UseOdomComb, StopLostConn;
extern int16_t linVelocity, angVelocity;
extern int32_t e_lv_last, e_av_last, e_lv_sum, e_av_sum;
extern float kp_lv, ki_lv, kd_lv;
extern float kp_av, ki_av, kd_av;

void ControllerPIDLoop(void)
{
	int32_t e_lv, e_av, u_lv, u_av;

	// Get the error signals
	if (UseOdomComb)
	{
		e_lv = linVelocity - odomCombined[3];
		e_av = angVelocity - odomCombined[4];
	}
	else
	{
		e_lv = linVelocity - vels[0];
		e_av = angVelocity - vels[1];
	}

	e_lv_sum += e_lv;
	e_av_sum += e_av;

	// Calculate the PID linear velocity control signal
	u_lv = (uint32_t)(kp_lv * e_lv + ki_lv * e_lv_sum + kd_lv * (e_lv - e_lv_last));
	u_av = (uint32_t)(kp_av * e_av + ki_av * e_av_sum + kd_av * (e_av - e_av_last));

	// Set the PWM duty cycles for the motor and the steering servos
	PWMSetDuty(MOTOR_CHANNEL, u_lv);
//	PWMSetDuty(FRONT_SERVO_CHANNEL, ControllerCalcPWM(FRONT_SERVO_CHANNEL));
//	PWMSetDuty(REAR_SERVO_CHANNEL, ControllerCalcPWM(REAR_SERVO_CHANNEL));

	// Store last velocity errors
	e_lv_last = e_lv;
	e_av_last = e_av;
}
