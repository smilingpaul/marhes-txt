/**
 * @file adc.h
 * 
 * @brief Defines most of the bit options of the ADC Module of the LPC2378.  
 * Contains the function declarations of adc.c
 *
 * @author Titus Appel
 *
 * @version 1.0
 *
 * @date 2011/06/03
 *
 * Contact: titus.appel@gmail.com
 */

#ifndef ADC_H_
#define ADC_H_

#include "lpc23xx.h"
#include "FreeRTOS.h"

// General ADC Bit defines
#define PCADC					      (1<<12)
#define PCLKSEL0_ADC_DIV8   (3<<24)
#define PINSEL0_AD06        (3<<24)
#define PINSEL1_AD01        (1<<16)
#define PINSEL1_AD03        (1<<20)
#define PINSEL3_AD05        (3<<30)

#define AD0CR_SEL0          (1<<0)
#define AD0CR_SEL1          (1<<1)
#define AD0CR_SEL2          (1<<2)
#define AD0CR_SEL3          (1<<3)
#define AD0CR_SEL4          (1<<4)
#define AD0CR_SEL5          (1<<5)
#define AD0CR_SEL6          (1<<6)
#define AD0CR_SEL7          (1<<7)
#define AD0CR_CLKDIV9       (8<<8)
#define AD0CR_CLKDIV60      (59<<8)
#define AD0CR_BURST         (1<<16)
#define AD0CR_CLKS11        (0<<17)
#define AD0CR_CLKS10        (1<<17)
#define AD0CR_CLKS9         (2<<17)
#define AD0CR_CLKS8         (3<<17)
#define AD0CR_CLKS7         (4<<17)
#define AD0CR_CLKS6         (5<<17)
#define AD0CR_CLKS5         (6<<17)
#define AD0CR_CLKS4         (7<<17)
#define AD0CR_PDN           (1<<21)
#define AD0CR_START_NONE    (0<<24)
#define AD0CR_START_NOW     (1<<24)
#define AD0CR_START_EINT0   (2<<24)
#define AD0CR_START_CAP01   (3<<24)
#define AD0CR_START_MAT01   (4<<24)
#define AD0CR_START_MAT03   (5<<24)
#define AD0CR_START_MAT10   (6<<24)
#define AD0CR_START_MAT11   (7<<24)
#define AD0CR_EDGE_FALL     (1<<27)

#define AD0INTEN_ADGINTEN   (1<<8)

#define BITMASK_10          0x3FF

void ADCInit(void);
unsigned portBASE_TYPE ADCGetChannel(unsigned portBASE_TYPE channel);

#endif
