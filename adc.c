/*
 * adc.c
 *
 *  Created on: Feb 23, 2011
 *      Author: titus
 */

#include "adc.h"

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

int16_t ADCGetChannel(uint8_t channel)
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
			return -1;
			break;
	}
}
