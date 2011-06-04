#ifndef CONTROLLER_TASK_H_
#define CONTROLLER_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "app_types.h"
#include "serial.h"
#include "drivers/pwm.h"
#include "tasks/mode_task.h"
#include "tasks/encoder_task.h"

// The PWM channels of the different motors
#define DEFAULT_CHANNEL 			0
#define MOTOR_CHANNEL				1
#define FRONT_SERVO_CHANNEL			2
#define REAR_SERVO_CHANNEL			3

// The maximum linear and angular velocities
#define LIN_VEL_MAX					3000
#define LIN_VEL_MIN					-3000
#define ANG_VEL_MAX					5000
#define ANG_VEL_MIN					-5000

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

// Gain Defines
enum GAINS{ KP_LV, KI_LV, KD_LV, KP_AV, KI_AV, KD_AV };

void vControllerTaskStart(void);
void ControllerSetLinearVelocity(int16_t value);
int16_t ControllerGetLinearVelocity(void);
void ControllerSetAngularVelocity(int16_t value);
int16_t ControllerGetAngularVelocity(void);
void ControllerSetOdomCombined(int32_t linVel, int32_t angVel);
void ControllerSetPid(int32_t lp, int32_t li, int32_t ld, \
					  int32_t ap, int32_t ai, int32_t ad);
int32_t ControllerGetPid(uint8_t gain);

#endif

