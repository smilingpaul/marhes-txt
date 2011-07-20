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

static msg_u data;

static int16_t absValue(int16_t value);
static int16_t signValue(int16_t value);

static void vControllerTask( void *pvParameters )
{
  static int32_t lv = 0, av = 0;
  static int32_t e_lv = 0, e_av = 0;
  static int32_t e_lv_last = 0, e_av_last = 0;
  static int32_t e_lv_last2 = 0, e_av_last2 = 0;
  static float lpterm, literm, ldterm;
  static float apterm, aiterm, adterm;
  static int32_t u_lv = 0, u_av = 0;
  static int32_t count = 0;
  static float dt;
  static uint32_t ticks, lastTicks = 0;
  //static int32_t y, y_prev = 0, lv_last = 0;;
  
  static int64_t e_sum = 0;

  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  for( ;; )
  {
    portENTER_CRITICAL();
    if (controllerMode == 1)
    {
	    if (ModeStopLostConn() == pdFALSE)// && ModeUseOdomComb() == pdTRUE)
	    {
		    if (ModeUseOdomComb() == pdTRUE)
		    {
		      //lv = ki_av * combLinVelocity + (1-ki_av) * lv_last;
		      lv = combLinVelocity;
		      av = combAngVelocity;
		    }
		    else
		    {
		      lv = EncoderLinVel();
		      av = EncoderAngVel();
		    }        
        
        // Get the error signals and process signals
		    e_lv = linVelocity - lv;
		    e_av = angVelocity - av;
        
        ticks = xTaskGetTickCount();
        if (ticks > lastTicks)
          dt = (float)(ticks - lastTicks) / configTICK_RATE_HZ;
        else
          dt = (float)(0xFFFFFFFF - lastTicks + ticks + 1) / configTICK_RATE_HZ;
        lastTicks = ticks;
		    
		    // VELOCITY PID CONTROLLER
		    lpterm = (float)kp_lv * (e_lv - e_lv_last);
		    literm = (float)ki_lv * (e_lv + e_lv_last) / 2 * dt;
		    ldterm = (float)kd_lv * (e_lv - 2 * e_lv_last + e_lv_last2) / dt;
		    
		    u_lv += (int32_t)(lpterm + literm + ldterm);
		    
		    if (u_lv > VELOCITY_PWM_MAX)
		      u_lv = VELOCITY_PWM_MAX;
		      		    
		    if (u_lv < VELOCITY_PWM_MIN)
		      u_lv = VELOCITY_PWM_MIN;
		      
		    apterm = (float)kp_av * (e_av - e_av_last);
		    aiterm = (float)ki_av * (e_av + e_av_last) / 2 * dt;
		    adterm = (float)kd_av * (e_av - 2 * e_av_last + e_av_last2) / dt;
		    
		    u_av += (int32_t)(apterm + aiterm + adterm);
		    
		    if (u_av > ANGULAR_PWM_MAX)
		      u_av = ANGULAR_PWM_MAX;
		      		    
		    if (u_av < ANGULAR_PWM_MIN)
		      u_av = ANGULAR_PWM_MIN;

		    // Set the PWM duty cycles for the motor and the steering servos
		    PWMSetDuty(MOTOR_CHANNEL, DUTY_1_5 + u_lv);
	      PWMSetDuty(FRONT_SERVO_CHANNEL, DUTY_1_5 - u_av);
	      PWMSetDuty(REAR_SERVO_CHANNEL, DUTY_1_5 + u_av);
	    
	      count++;
	      if (count > 5)
	      {
	        //ROSSendPidTerms(&data, (int32_t)lpterm, (int32_t)literm, (int32_t)ldterm, u_lv);
	        count = 0;
	      }

		    // Store last velocity errors
		    e_lv_last2 = e_lv_last;
 		    e_av_last2 = e_av_last;
		    e_lv_last = e_lv;
		    e_av_last = e_av;
	    }
	    else
	    {
		    PWMSetDuty(MOTOR_CHANNEL, DUTY_1_5);
		    PWMSetDuty(FRONT_SERVO_CHANNEL, DUTY_1_5);
		    PWMSetDuty(REAR_SERVO_CHANNEL, DUTY_1_5);
		    e_lv_last = 0;
		    e_av_last = 0;
		    e_lv_last2 = 0;
		    e_av_last2 = 0;
		    u_lv = 0;
		    u_av = 0;
	    }
	  }
	  //FIO0PIN ^= (1<<21);
	  portEXIT_CRITICAL();

    vTaskDelayUntil( &xLastWakeTime, ( DELTA_T / portTICK_RATE_MS ) );
  }
}

void vControllerTaskStart(void)
{
  controllerMode = 1;
  xTaskCreate( vControllerTask, "ControllerTask", configMINIMAL_STACK_SIZE * 4, NULL, 4, NULL );
}

void ControllerSetVelocity(int16_t lv, int16_t av) 
{
  float maxAV;
  portENTER_CRITICAL();
  
  if (lv > LIN_VEL_MAX)
    linVelocity = LIN_VEL_MAX;
  else if (lv < LIN_VEL_MIN)
    linVelocity = LIN_VEL_MIN;
  else
    linVelocity = lv;
    
  maxAV = 1000 * absValue(linVelocity) / RADIUS_MIN;
  
  if (absValue(av) > maxAV)
  {
    angVelocity = signValue(av) * maxAV;
  }
  else
  {
    angVelocity = av;
  }
		
  portEXIT_CRITICAL();
}

int16_t ControllerGetLinearVelocity(void) 
{
	return linVelocity;
}

int16_t ControllerGetAngularVelocity(void) 
{
	return angVelocity;
}

void ControllerSetOdomCombined(int32_t linVel, int32_t angVel)
{
  portENTER_CRITICAL();
	combLinVelocity = linVel;
	combAngVelocity = angVel;
	portEXIT_CRITICAL();
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
  portENTER_CRITICAL();
  controllerMode ^= 1;
  portEXIT_CRITICAL();
}

int8_t ControllerGetMode(void)
{
  return controllerMode;
}

static int16_t absValue(int16_t value)
{
  int16_t retVal = value;
  if (value < 0)
    retVal = -value;
  
  return retVal;
}

static int16_t signValue(int16_t value)
{
  int16_t retValue = 1;
  if (value < 0)
    retValue = -1;
  return retValue;
}
