/**
 @file adc.c
  
 @brief Includes functions to initialize the ADC on AD0.1, 3, 5, and 6 and get
 the reading from specific channels

 The ADC is set up to continuously sample the specified channels without
 interrupting the processor. The clock is set up and after converting one one
 channel it goes to the next and then repeats forever. Each channel has its own
 register so the most up-to-date reading is in its channel's register. When the 
 application needs a value it just reads the appropriate register.
 
 @author Titus Appel

 @version 1.0

 @date 2011/06/03

 Contact: titus.appel@gmail.com
*/

#include "adc.h"

/**
 @brief Initialize the ADC.
 
 Setup the ADC to sample continuously on channels 1, 3, 5, and 6. 
*/

void ADCInit(void)
{
	PCONP |= PCADC;
	PCLKSEL0 |= PCLKSEL0_ADC_DIV8;		// Make PCLK for ADC 72MHz / 8 = 9MHz
	// Enable A/D function on AD0.1,3,5,6
	PINSEL0 |= PINSEL0_AD06;
	PINSEL1 |= PINSEL1_AD01 | PINSEL1_AD03;
	PINSEL3 |= PINSEL3_AD05;
	AD0INTEN &= ~AD0INTEN_ADGINTEN;		// Turn off global A/D interrupt

	// Sample AD0.1,3,5,6, Have A/D clk be 1MHz = 9MHz / 9, use burst mode w/ 11 clks
	AD0CR = AD0CR_SEL1 | AD0CR_SEL3 | AD0CR_SEL5 | AD0CR_SEL6 |
			AD0CR_CLKDIV9 | AD0CR_BURST | AD0CR_CLKS11;
	AD0CR |= AD0CR_PDN;
}

/**
 @brief Get the most up-to-date reading from the specified channel.
 @param[in] channel The channel number to be read. Make sure its initialized
                    first.
 @return The value of the specified channel's reading  
 @note Unused channels are commented out.
*/
unsigned portBASE_TYPE ADCGetChannel(unsigned portBASE_TYPE channel)
{
	switch(channel)
	{
//		case 0:
//			return (AD0DR0 >> 6) & BITMASK_10;
//			break;
		case 1:
			return (AD0DR1 >> 6) & BITMASK_10;
			break;
//		case 2:
//			return (AD0DR2 >> 6) & BITMASK_10;
//			break;
		case 3:
			return (AD0DR3 >> 6) & BITMASK_10;
			break;
//		case 4:
//			return (AD0DR4 >> 6) & BITMASK_10;
//			break;
		case 5:
			return (AD0DR5 >> 6) & BITMASK_10;
			break;
		case 6:
			return (AD0DR6 >> 6) & BITMASK_10;
			break;
//		case 7:
//			return (AD0DR7 >> 6) & BITMASK_10;
//			break;
		default:
			break;
	}
}
