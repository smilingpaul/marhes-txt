/**
 @file controller_task.h
  
 @brief Contains the PWM channels, min and max values, and function declarations
 
 @author Titus Appel

 @version 1.0

 @date 2011/06/03

 Contact: titus.appel@gmail.com
*/

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
#define DEFAULT_CHANNEL 		    0
#define MOTOR_CHANNEL				    1
#define FRONT_SERVO_CHANNEL			2
#define REAR_SERVO_CHANNEL			3
#define PWM4                    4
#define PWM5                    5

#define DELTA_T               50

// The maximum linear and angular velocities
#define LIN_VEL_MAX					1500
#define LIN_VEL_MIN					-1500

#define RADIUS_MIN          555  // In mm

// The maximum and minimum PWM values for the servos/motor controller
#define VELOCITY_PWM_MAX			(PWM_MAX - DUTY_1_5)
#define VELOCITY_PWM_MIN			(PWM_MIN - DUTY_1_5)
#define ANGULAR_PWM_MAX				18000
#define ANGULAR_PWM_MIN				-18000

#define ANG_PID_CNT_MAX       24

/**
 @brief Array indices for the pid gain matrix
*/
enum GAINS{ KP_LV,                  ///< Linear Velocity Proportional Gain 
            KI_LV,                  ///< Linear Velocity Integral Gain
            KD_LV,                  ///< Linear Velocity Derivative Gain
            KP_AV,                  ///< Angular Velocity Proportional Gain
            KI_AV,                  ///< Angular Velocity Integral Gain
            KD_AV                   ///< Angular Velocity Derivative Gain
          };
void vControllerTaskStart(void);
void ControllerSetVelocity(int16_t lv, int16_t av);
int16_t ControllerGetLinearVelocity(void);
int16_t ControllerGetAngularVelocity(void);
void ControllerSetOdomCombined(int32_t linVel, int32_t angVel);
void ControllerSetPid(int32_t lp, int32_t li, int32_t ld, \
					  int32_t ap, int32_t ai, int32_t ad);
float ControllerGetPid(uint8_t gain);
float ControllerGetLinPid(uint8_t gain);
float ControllerGetAngPid(uint8_t gain);
float ControllerGetAngPidAddr(uint8_t velIndex, int8_t gain);
void ControllerSetLinPid(int32_t * linPids);
void ControllerSetAngPid(int32_t * angPids, uint8_t count);
void ControllerToggleMode(void);
int8_t ControllerGetMode(void);

#endif

