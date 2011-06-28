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
static int8_t controllerMode = 1;
static float kp_lv, ki_lv, kd_lv;
static float kp_av, ki_av, kd_av;

static int32_t e_lv_last = 0, e_av_last = 0, e_lv_sum = 0, e_av_sum = 0;
static int32_t lv = 0, av = 0, lv_last = 0, av_last = 0, e_lv_diff = 0, e_av_diff = 0;
static int32_t e_lv_last2 = 0, e_av_last2 = 0;
static int32_t lv_last2 = 0, av_last2 = 0;
static float pterm, iterm, dterm;
static int32_t u_lv = 0, u_av = 0;
static int32_t count = 0;
static float dt;
static uint32_t ticks, lastTicks = 0;

static msg_u data;

static void vControllerTask( void *pvParameters )
{
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  for( ;; )
  {
	  int32_t e_lv, e_av;

    if (controllerMode == 1)
    {
	    if (ModeStopLostConn() == pdFALSE)// && ModeUseOdomComb() == pdTRUE)
	    {
		    // Get the error signals and process signals
		    if (ModeUseOdomComb() == pdTRUE)
		    {
		      lv = combLinVelocity;
		      av = combAngVelocity;
			    e_lv = linVelocity - lv;
			    e_av = angVelocity - av;
		    }
		    else
		    {
		      lv = EncoderLinVel();
		      av = EncoderAngVel();
			    e_lv = linVelocity - lv;
			    e_av = angVelocity - av;
		    }

        ticks = xTaskGetTickCount();
        if (ticks > lastTicks)
          dt = (float)(ticks - lastTicks) / configTICK_RATE_HZ;
        else
          dt = (float)(0xFFFFFFFF - lastTicks + ticks + 1) / configTICK_RATE_HZ;
        lastTicks = ticks;
		    
		    // VELOCITY PID CONTROLLER
		    // du = kp * (e(t) - e(t-1)) + ki * e(t) * dt - kd * (meas(t) - 2meas(t-1) + meas(t-2)) / dt
		    pterm = (float)(e_lv - e_lv_last);
		    iterm = (float)e_lv * dt;//0.02;//DELTA_T / 1000;
		    //dterm = (float)(lv - 2 * lv_last + lv_last2) / dt;//0.02;//1000 / DELTA_T;
		    dterm = (float)(e_lv - 2 * e_lv_last + e_lv_last2) / dt;
		    
		    u_lv += (int32_t)(kp_lv * pterm + ki_lv * iterm + kd_lv * dterm);
		    
		    

/*        // POSITIONAL PID CONTROLLER
        // ui = kc / Ti * sum(((e - e_last) / 2) * dt)
		    e_lv_sum += ((e_lv - e_lv_last) >> 2) * DELTA_T / 1000;
		    e_av_sum += ((e_av - e_av_last) >> 2) * DELTA_T / 1000;
		    
		    e_lv_diff = lv - lv_last;
 		    e_av_diff = av - av_last;
		    		    
		    // Calculate the PID linear velocity control signal
		    u_lv += (int32_t)(kp_lv * e_lv + ki_lv * e_lv_sum + kd_lv * e_lv_diff);//(e_lv - e_lv_last));
		    u_av += (int32_t)(kp_av * e_av + ki_av * e_av_sum + kd_av * e_av_diff);//(e_av - e_av_last));
*/

		    if (u_lv > VELOCITY_PWM_MAX)
		      u_lv = VELOCITY_PWM_MAX;
		      		    
		    if (u_lv < VELOCITY_PWM_MIN)
		      u_lv = VELOCITY_PWM_MIN;

		    // Set the PWM duty cycles for the motor and the steering servos
		    PWMSetDuty(MOTOR_CHANNEL, DUTY_1_5 + u_lv);
	    //	PWMSetDuty(FRONT_SERVO_CHANNEL, ControllerCalcPWM(FRONT_SERVO_CHANNEL));
	    //	PWMSetDuty(REAR_SERVO_CHANNEL, ControllerCalcPWM(REAR_SERVO_CHANNEL));
	    
	      count++;
	      if (count > 5)
	      {
	        ROSSendPidTerms(&data, (int32_t)pterm, (int32_t)iterm, (int32_t)dterm, u_lv);
	        count = 0;
	      }

		    // Store last velocity errors
		    e_lv_last2 = e_lv_last;
 		    e_av_last2 = e_av_last;
		    e_lv_last = e_lv;
		    e_av_last = e_av;
		    
		    lv_last2 = lv_last;
		    av_last2 = av_last;
		    lv_last = lv;
		    av_last = lv;
	    }
	    else
	    {
//		    count = count + 100;
//		    if (count > 2147483647)
//			    count = 0;
		    PWMSetDuty(MOTOR_CHANNEL, DUTY_1_5);
		    e_lv_last = 0;
		    e_av_last = 0;
		    e_lv_sum = 0;
		    e_av_sum = 0;
		    u_lv = 0;
		    u_av = 0;
	    }
	  }
	  //FIO0PIN ^= (1<<21);

    vTaskDelayUntil( &xLastWakeTime, ( DELTA_T / portTICK_RATE_MS ) );
  }
}

void vControllerTaskStart(void)
{
  controllerMode = 1;
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
  kp_lv = (float)lp / 1000;
  ki_lv = (float)li / 1000;
  kd_lv = (float)ld / 1000;
  kp_av = (float)ap / 1000;
  ki_av = (float)ai / 1000;
  kd_av = (float)ad / 1000;
}

float ControllerGetPid(uint8_t gain)
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

void ControllerToggleMode(void)
{
  controllerMode ^= 1;
}

int8_t ControllerGetMode(void)
{
  return controllerMode;
}
