/**
 @file pwm.c
  
 @brief Sets up the PWM peripheral for the motor controller and servos.
 
 @par Description
 pwm.c allows the setup of the PWM peripheral for control of the
 motor controller and servos.  Both motors take a PWM signal
 at 50Hz.  1msec on time is full one way, 1.5msec on time is in
 the middle, and 2msec on time is full the other way.  All six
 PWM outputs are set up for 50Hz and 1.5msec on time. A function
 is provided to set the PWM on times.  Another function get the
 current on time.
 
 @par Output Pins 
 - P2.0 - PWM1.1 - EXT2-37
 - P2.1 - PWM1.2 - EXT2-36
 - P2.2 - PWM1.3 - EXT2-35
 - P2.3 - PWM1.4 - EXT2-34
 - P2.4 - PWM1.5 - EXT2-33
 - P1.26- PWM1.6 - EXT2-20 
 
 @note PWM1.6 is used for the backlight of the LCD
  
 @author Titus Appel

 @version 1.0

 @date 2010/05/29

 Contact: titus.appel@gmail.com
*/

#include "pwm.h"

 /**
 @brief Sets up PWM 1-6 on pins P2.0-P2.5, initial frequency is 50Hz
        and initial duty cycle is 1.5ms on time.
 @note  PWM6 is for LCD Brightness
*/
void PWMInit(void)
{
	// 1. First turn on PWM peripheral
	PCONP |= PCPWM1;

	// 2. Select the divider of the main clock to use, PCLK_PWM1-13:12
	//    Use 01-CCLK / 1 = 72MHz / 1 = 72MHz
	//    00 => CCLK / 4
	//    01 => CCLK / 1
	//    10 => CCLK / 2
	//    11 => CCLK / 8
	PCLKSEL0 |= PCLKSEL0_PWM_DIV1;	// Problem: doesn't write properly 0x00000000

	// 3. Choose the pins the PWM output is attached to
	PINSEL4 |= PINSEL4_PWM_EN;
	PINSEL3 |= PINSEL3_PWM6;

	// 4. Set PWM mode options
	PWM1TCR = TCR_CR;	// Reset the counter so the counting stops
	PWM1PR = 0;			// Set prescale register to increment TC every cycle

	// 5. Set the PWM timing interval options
	//    PCLK = 72MHz, We want 50Hz, 0.02 / (1.38 * 10^-8) = 1440000 = 0x15F900
	PWM1MR0 = MR0_FREQ;

	// 6. More PWM Options
	PWM1MCR = MCR_MR0R;		// Reset the timer counter on MR0 reach
	PWM1PCR = PCR_OUT_EN;	// Output PWM on all outputs
	PWM1TCR = TCR_PWM_EN | TCR_C_EN; 	// Enable PWM and Counter

	// 7. Set the duty cycle of PWM outputs
	PWM1MR1 = DUTY_1_5;	// Set the pulse width to 1.5ms, which is 1.5ms * 72MHz
	PWM1MR2 = DUTY_1_5;	// or 108000 -> 0x1A5E0
	PWM1MR3 = DUTY_1_5;
	PWM1MR4 = DUTY_1_5;
	PWM1MR5 = DUTY_1_5;
	PWM1MR6 = DUTY_1_5;

	// 8. Load the match registers by setting the latch enable bits
	PWM1LER = LER_ALL;
}

/**
 @brief Sets the match register of the specified channel to change
        the duty cycle.  The output goes low on match.
 @param[in] channel The pwm channel, valid values are 1-6
 @param[in] duty    The value to load in the channels match register
*/
void PWMSetDuty(char channel, unsigned long duty)
{
	// Load the correct match register with duty
	switch(channel)
	{
		case 1:
			PWM1MR1 = duty;
			break;
		case 2:
			PWM1MR2 = duty;
			break;
		case 3:
			PWM1MR3 = duty;
			break;
		case 4:
			PWM1MR4 = duty;
			break;
		case 5:
			PWM1MR5 = duty;
			break;
		case 6:
			PWM1MR6 = duty;
			break;
		default:
			break;
	}

	// Load the value in the match register as the new duty
	// Or the latch if other latches are pending
	if (channel >= 1 && channel <= 6)
		PWM1LER |= (1<<channel);
}

/**
 @brief Gets the match register of the specified channel.
 @param[in] channel The pwm channel, valid values are 1-6
 @return            The value to load in the channels match register
*/
unsigned long PWMGetDuty(char channel)
{
	unsigned long duty = 0;

	// Get the correct match register's duty
	switch(channel)
	{
		case 1:
			duty = PWM1MR1;
			break;
		case 2:
			duty = PWM1MR2;
			break;
		case 3:
			duty = PWM1MR3;
			break;
		case 4:
			duty = PWM1MR4;
			break;
		case 5:
			duty = PWM1MR5;
			break;
		case 6:
			duty = PWM1MR6;
			break;
		default:
			duty = 0;
			break;
	}
	return duty;
}
