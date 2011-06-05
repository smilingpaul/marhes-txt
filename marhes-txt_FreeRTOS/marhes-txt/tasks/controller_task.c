/**
 @file controller_task.c
  
 @brief This task controls the linear and angular velocities.
 
 Need to work on more.
  
 @author Titus Appel

 @version 1.0

 @date 2011/06/03

 Contact: titus.appel@gmail.com
 
 @todo Linear velocity controller
 @todo Angular velocity controller
*/

#include "tasks/controller_task.h"

extern xComPortHandle debugPortHandle;

static int16_t linVelocity, angVelocity;
static int32_t combLinVelocity, combAngVelocity;
static int32_t kp_lv, ki_lv, kd_lv;
static int32_t kp_av, ki_av, kd_av;

static int32_t e_lv_last = 0, e_av_last = 0, e_lv_sum = 0, e_av_sum = 0;
static int32_t u_lv = 0, u_av = 0;
static int32_t count = 0;

static void vControllerTask( void *pvParameters )
{
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  for( ;; )
  {
	  int32_t e_lv, e_av;

	  if (ModeStopLostConn() == pdFALSE)
	  {
		  // Get the error signals
		  if (ModeUseOdomComb() == pdTRUE)
		  {
			  e_lv = linVelocity - combLinVelocity;
			  e_av = angVelocity - combAngVelocity;
		  }
		  else
		  {
			  e_lv = linVelocity - EncoderLinVel();
			  e_av = angVelocity - EncoderAngVel();
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
		  count = count + 100;
		  if (count > 2147483647)
			  count = 0;
		  PWMSetDuty(MOTOR_CHANNEL, count);//DUTY_1_5);
		  e_lv_last = 0;
		  e_av_last = 0;
		  e_lv_sum = 0;
		  e_av_sum = 0;
		  u_lv = 0;
		  u_av = 0;
	  }
	  //FIO0PIN ^= (1<<21);

    vTaskDelayUntil( &xLastWakeTime, ( 20 / portTICK_RATE_MS ) );
  }
}

void vControllerTaskStart(void)
{
  xTaskCreate( vControllerTask, "ControllerTask", configMINIMAL_STACK_SIZE * 2, NULL, 4, NULL );
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

void ControllerSetOdomCombined(int32_t linVel, int32_t angVel)
{
	combLinVelocity = linVel;
	combAngVelocity = angVel;
}

void ControllerSetPid(int32_t lp, int32_t li, int32_t ld, \
					  int32_t ap, int32_t ai, int32_t ad)
{
  kp_lv = lp;
  ki_lv = li;
  kd_lv = ld;
  kp_av = ap;
  ki_av = ai;
  kd_av = ad;
}

int32_t ControllerGetPid(uint8_t gain)
{
  switch(gain)
  {
    case KP_LV:
      return kp_lv;
      break;
    case KI_LV:
      return ki_lv;
      break;
    case KD_LV:
      return kd_lv;
      break;
    case KP_AV:
      return kp_av;
      break;
    case KI_AV:
      return ki_av;
      break;
    case KD_AV:
      return kd_av;
      break;
    default:
      break;
  }
}
