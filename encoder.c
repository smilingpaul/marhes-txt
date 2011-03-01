/*! \file encoder.c
    \brief Code for the encoders.

    \date Jun 19, 2010\n
	\author Titus Appel <titus.appel@gmail.com>\n
	\par Institution:
	UNM-ECE Department\n
	MARHES Lab

   	These routines initialize Timers 0 and 3 for capture mode to
   	count the encoder ticks to measure the velocity of the TXT-1.
   	The encoders being used are the US Digital E7MS optical encoders
   	which is currently discontinued.  The E7MS has 400 CPR (counts
   	per revolution).  These are quadrature encoders so you can measure
   	the direction also.  They have an A and a B signal.  We will use
   	the A signal to measure the speed with the capture pins.  We will Count the
   	number of ticks for a specific time interval.  This will give ticks/ms.
   	Timer 1 is used for the sample rate.  The interrupt is setup in the
   	initialization code.

    Pin definition:\n
    	P3.23 - CAP0.0 - Front Right Encoder Signal\n
     	P0.23 - CAP3.0 - Front Left Encoder Signal
 */

#include "encoder.h"

int32_t ticks[SIZE_ENCODER_TICKS_ARR] = {0};	/*!< The number of ticks with\n
                                                 {0} = front right\n
                                                 {1} = front left
											 	 */
int32_t vels[SIZE_ENCODER_VEL_ARR] = {0};		/*!< The velocity with\n
                                                 {0} = linear velocity\n
                                                 {1} = angular velocity.*/
float pos[SIZE_ENCODER_POS_ARR] = {0};			/*!< The position with\n
                                                 {0} = xpos\n
                                                 {1} = ypos\n
                                                 {2} = theta.*/
int32_t intCount = 0;

/*! \brief Initializes the encoder counting.

    Initializes the Timer 0 and 3 peripherals to count rising and falling
    edges of the encoder signals.  The A and B encoder signals are XORed on the
    connector board to get twice the resolution.  The Timer 1 interrupt is then
    used to sample the counted ticks and store them.  Timer 1 samples at 50Hz.
 */
void EncoderInit(void)
{
	// 1. Power up timers 0, 1, and 3
	PCONP |= PCONP_PCTIM0 | PCONP_PCTIM1 | PCONP_PCTIM3;

	// 2. Make the peripheral clocks 72 MHz = divided by one
	PCLKSEL0 |= PCLKSEL0_TIM0_DIV1 | PCLKSEL0_TIM1_DIV1;
	PCLKSEL1 |= PCLKSEL1_TIM3_DIV1;

	// 3. Select pin functions.  Make P3.23 be CAP0 and P0.23 be CAP3.0.
	PINSEL7 |= (PINSEL7_CAP00);
	PINSEL1 |= (PINSEL1_CAP30);

	// 4. Setup timer counter
	T0CTCR = CTCR_CM_RF | CTCR_CAP_SEL_0;  // Timer is counter mode on rise/fall
										   // Start the counting on channel 0
	T0TCR = TCR_CR;						   // Reset timer0 counter
	T0PR = 0;							   // Increment TC after every rise/fall

	T3CTCR = CTCR_CM_RF | CTCR_CAP_SEL_0;  // Timer is counter mode on rise/fall
										   // Start the counting on channel 0
	T3TCR = TCR_CR;						   // Reset timer3 counter
	T3PR = 0;							   // Increment TC after every rise/fall

	// 5. Setup timer1 for a sample period of 20msec for counting
	T1TCR = TCR_CR;							// Reset timer1 counter
	T1CTCR = CTCR_TM;						// Timer 1 is in timer mode
	T1MR0 = MCR_20MS;						// Match at 20ms
	T1MCR = MCR_MR0I | MCR_MR0S | MCR_MR0R;	// On match, interrupt,reset,stop TC

	// Setup T1 Interrupt
	VICIntSelect &= ~VIC_CHAN_TO_MASK(VIC_CHAN_NUM_Timer1);	// Change to IRQ
	VICIntEnClr |= VIC_CHAN_TO_MASK(VIC_CHAN_NUM_Timer1); 	// Disable interrupt
	VICVectAddr5 = (uint32_t)(void *)EncoderISR;			// Assign the ISR
	VICVectPriority5 = 0xE;									// Set the priority
	VICIntEnable |= VIC_CHAN_TO_MASK(VIC_CHAN_NUM_Timer1);	// Enable the INT

	// 6. Enable the Timer counters
	T0TCR = TCR_CE;
	T3TCR = TCR_CE;
	T1TCR = TCR_CE;

	// 7. Setup direction inputs (input is 0)
	FIO3DIR &= ~LEFT_IN & ~RIGHT_IN;
}

/*! \brief Gets the ticks counted for the specified channel.

    Returns the ticks counted in the previous sample for the specified channel.

    \param channel The wheel to get the ticks.
    \return The number of ticks in the last sample period.
 */
int32_t EncoderCount(uint8_t channel)
{
	int32_t count;
	count = ticks[channel];
	return count;
}

/*! \brief Gets the velocity for the specified direction (linear, angular).

    Returns the specified velocity (linear, angular) during the previous sample.

    \param channel The velocity direction to get the velocity.
    \return The velocity from the last sample.
 */
int32_t EncoderVel(uint8_t channel)
{
	int32_t vel;
	vel = vels[channel];
	return vel;
}

int8_t EncoderGetDirection(uint8_t channel)
{
	uint8_t val = 0;
	if (channel == FRONT_LEFT)
		val = FIO3PIN & LEFT_IN;

	if (channel == FRONT_RIGHT)
		val = FIO3PIN & RIGHT_IN;

	if (val > 0)
		return -1;
	else
		return 1;
}
