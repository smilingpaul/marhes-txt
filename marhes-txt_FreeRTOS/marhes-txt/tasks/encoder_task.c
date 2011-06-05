/**
 @file encoder_task.c
  
 @brief The encoder task measures the amount of ticks on the encoders in certain
        time intervals.
        
 The encoder task uses timer 0 and 3 to count rising and falling ticks from the
 exclusive or'ed A and B quadrature signals from the encoders.  The task runs
 at 20ms or 50Hz. The task then calculates the linear and angular velocities and 
 the position and orientation from the encoder values. The encoder send task
 sends the encoder packet to the ROS computer every 100 ms. The direction is
 read using a flip flop with the A and B encoder signals. The direction is then 
 a high or low signal.
 
 @author Titus Appel

 @version 1.0

 @date 2011/06/03

 Contact: titus.appel@gmail.com
*/

#include "tasks/encoder_task.h"

extern xComPortHandle debugPortHandle, rosPortHandle;

int32_t ticks[SIZE_ENCODER_TICKS_ARR] = {0};	/*!< The number of ticks with\n
                                                 {0} = front right\n
                                                 {1} = front left
											 	                       */
int32_t vels[SIZE_ENCODER_VEL_ARR] = {0};		  /*!< The velocity with\n
                                                 {0} = linear velocity\n
                                                 {1} = angular velocity.
                                               */
float pos[SIZE_ENCODER_POS_ARR] = {0};			  /*!< The position with\n
                                                 {0} = xpos\n
                                                 {1} = ypos\n
                                                 {2} = theta.
                                               */

/// The lookup table for getting the cos and sin more efficiently.                                               
int32_t angtable[91]={0,18,35,52,70,87,105,122,139,156,174,191,208,225,242,259,
		276,292,309,326,342,358,375,391,407,423,438,454,470,485,500,515,530,545,
		559,574,588,602,616,629,643,656,669,682,695,707,719,731,743,755,766,777,
		788,799,809,819,829,839,848,857,866,875,883,891,899,906,914,921,927,934,
		940,946,951,956,961,966,970,974,978,982,985,988,990,993,995,996,998,999,
		999,1000,1000};  

/// Storage for sending the encoder message to the ROS computer 
static msg_u data;

int32_t anglookuptable(int32_t radians, int32_t type);

/**
 @brief The encoder task counts the ticks and calculates the position and 
        velocities.
        
 The basic outline if as follows.
 - Stop the counters.
 - Filter out the noise of small rotations when stationary and store the tick 
   counts.
 - Reset and enable the counters.
 - Calculate the velocities, change in position and orientation, and then add to 
   position and orientation totals.
 - Fix orientation to 0 - 2*Pi
 - Delay until next reading
 @param[in] pvParameters The parameters from the task create call
 @todo Make the encoder sample rate variable
*/
static void vEncoderTask( void *pvParameters )
{
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  EncoderInit();

  for( ;; )
  {
    float dx, dy, dth;
	  int32_t vel1, vel2;
	  
    // Stop Counters
	  T0TCR = 0;
	  T3TCR = 0;

	  // Store the right and left encoder counts, get rid of noise
	  if (T0TC > 2)
		  ticks[TICKS_FL] = T0TC;
	  else
		  ticks[TICKS_FL] = 0;
	  if (T3TC > 2)
		  ticks[TICKS_FR] = T3TC;
	  else
		  ticks[TICKS_FR] = 0;

	  // Reset and enable the TC
	  T0TCR = TCR_CR;
	  T3TCR = TCR_CR;
	  T0TCR = TCR_CE;
	  T3TCR = TCR_CE;

		vels[VELS_LINEAR] = (EncoderGetDirection(TICKS_FR) * \
		      ticks[TICKS_FR] + EncoderGetDirection(TICKS_FL) * \
		      ticks[TICKS_FL]) * 15 / 2;
		vels[VELS_ANGULAR] = (EncoderGetDirection(TICKS_FR) * \
		      ticks[TICKS_FR] - EncoderGetDirection(TICKS_FL) * \
		      ticks[TICKS_FL]) * 15000 / 285;

	  dx = (float)vels[VELS_LINEAR] * anglookuptable(pos[2], 1) * 20 / 1000000;
	  dy = (float)vels[VELS_LINEAR] * anglookuptable(pos[2], 0) * 20 / 1000000;
	  dth = (float)vels[VELS_ANGULAR] * 20 / 1000;
	  pos[POS_X] += dx;
	  pos[POS_Y] += dy;
	  pos[POS_T] += dth;
	  
	  if (pos[POS_T] > 6283)
		  pos[POS_T] -= 6283;

	  if (pos[POS_T] < 0)
		  pos[POS_T] += 6283;

		//FIO0PIN ^= (1<<21);

    vTaskDelayUntil( &xLastWakeTime, ( 20 / portTICK_RATE_MS ) );
  }
}

/**
 @brief The encoder send task sends the current odometry information at a rate.
 @param[in] pvParameters The parameters from the task create call
 @todo Make the encoder send rate variable 
*/
static void vEncoderSendTask( void *pvParameters )
{
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  for ( ;; )
  {
    // Send encoder message
	  ROSSendOdomEnc(&data, (int32_t)pos[0], (int32_t)pos[1], (int32_t)pos[2], vels[0], vels[1]);
	  vTaskDelayUntil( &xLastWakeTime, ( 100 / portTICK_RATE_MS ) );	
	}
}

/**
 @brief Start both the encoder and the encoder send tasks.
 @todo Should make the Priority and Stack Size reconfigurable. 
*/
void vEncoderTaskStart(void)
{
  xTaskCreate( vEncoderTask, "EncoderTask", configMINIMAL_STACK_SIZE * 2, NULL, 4, NULL );
  xTaskCreate( vEncoderSendTask, "EncoderSendTask", configMINIMAL_STACK_SIZE, NULL, 2, NULL );
}

/*! \brief Initializes the encoder counting.

    Initializes the Timer 0 and 3 peripherals to count rising and falling
    edges of the encoder signals.  The A and B encoder signals are XORed on the
    connector board to get twice the resolution.  Also sets up the direction 
    inputs.
 */
void EncoderInit(void)
{
  portENTER_CRITICAL();
  {
	// 1. Power up timers 0, 1, and 3
	PCONP |= PCONP_PCTIM0 | PCONP_PCTIM3;

	// 2. Make the peripheral clocks 72 MHz = divided by one
	PCLKSEL0 |= PCLKSEL0_TIM0_DIV1;
	PCLKSEL1 |= PCLKSEL1_TIM3_DIV1;

	// 3. Select pin functions.  Make P3.23 be CAP0 and P0.23 be CAP3.0.
	PINSEL7 |= (PINSEL7_CAP00);
	PINSEL1 |= (PINSEL1_CAP30);

	// 4. Setup timer counter
	T0CTCR = CTCR_CM_RF | CTCR_CAP_SEL_0;  // Timer is counter mode on rise/fall
										                     // Start the counting on channel 0
	T0TCR = TCR_CR;						             // Reset timer0 counter
	T0PR = 0;							                 // Increment TC after every rise/fall

	T3CTCR = CTCR_CM_RF | CTCR_CAP_SEL_0;  // Timer is counter mode on rise/fall
										                     // Start the counting on channel 0
	T3TCR = TCR_CR;						             // Reset timer3 counter
	T3PR = 0;							                 // Increment TC after every rise/fall

	// 5. Enable the Timer counters
	T0TCR = TCR_CE;
	T3TCR = TCR_CE;

	// 6. Setup direction inputs (input is 0)
	FIO3DIR &= ~DIR_LEFT & ~DIR_RIGHT;
	}
	portEXIT_CRITICAL();
}

/**
 @brief Gets the direction of the encoder.
 @param[in] channel The encoder channel to get the direction
 @return The direction of the encoder channel
*/
int8_t EncoderGetDirection(uint8_t channel)
{
	uint8_t val = 0;
	if (channel == TICKS_FL)
		val = FIO3PIN & DIR_LEFT;

	if (channel == TICKS_FR)
		val = FIO3PIN & DIR_RIGHT;

	if (val > 0)
		return -1;
	else
		return 1;
}

/**
 @brief Get the linear velocity from the encoders
 @return The linear velocity
*/
int32_t EncoderLinVel(void)
{
  return vels[VELS_LINEAR];
}

/**
 @brief Get the angular velocity from the encoders
 @return The angular velocity
*/
int32_t EncoderAngVel(void)
{
  return vels[VELS_ANGULAR];
}

/**
 @brief Calculates the cos or sin with radians from a lookup table
 @param[in] radians The radians from 0-2*Pi
 @param[in] type    The operation to do -> 0 - cos, 1 - sin
*/
//trig lookup table, type 0 for cos, 1 for sin, degrees is from 0->360
int32_t anglookuptable(int32_t radians, int32_t type)
{
	int32_t c = 1;
	int32_t s = 1;

	int32_t i = radians * 180 / 3141; //includes 0 to 90 degrees

	while (i > 360)
		i -= 360;

	while (i < 0)
		i += 360;

	if (i > 90 && i <= 180) //between 91 and 180
	{
		i = 180 - i;
		c = -1;
	}

	if (i > 180 && i <= 270) //between 181 and 270
	{
		i = i - 180;
		c = -1;
		s = -1;
	}

	if (i > 270 && i <= 360) //between 271 and 360
	{
		i = 360 - i;
		s = -1;
	}

	if (type == 1) //cosine
	{
		c = c * angtable[90 - i];
		return c;
	}
	else //sine
	{
		s = s * angtable[i];
		return s;
	}
}
