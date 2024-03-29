// ***********************************************************************
//
//  LPC2378 header file
//
// ***********************************************************************

#ifndef __LPC237x_H
#define __LPC237x_H

#define	SCS			   (*((volatile unsigned long *) 0xE01FC1A0))

/* Vectored Interrupt Controller (VIC) */
#define VICIRQStatus   (*((volatile unsigned long *) 0xFFFFF000))
#define VICFIQStatus   (*((volatile unsigned long *) 0xFFFFF004))
#define VICRawIntr     (*((volatile unsigned long *) 0xFFFFF008))
#define VICIntSelect   (*((volatile unsigned long *) 0xFFFFF00C))
#define VICIntEnable   (*((volatile unsigned long *) 0xFFFFF010))
#define VICIntEnClr    (*((volatile unsigned long *) 0xFFFFF014))
#define VICSoftInt     (*((volatile unsigned long *) 0xFFFFF018))
#define VICSoftIntClr  (*((volatile unsigned long *) 0xFFFFF01C))
#define VICProtection  (*((volatile unsigned long *) 0xFFFFF020))
#define VICSWPriorityMask (*((volatile unsigned long *) 0xFFFFF024))

#define VICVectAddr0   (*((volatile unsigned long *) 0xFFFFF100))
#define VICVectAddr1   (*((volatile unsigned long *) 0xFFFFF104))
#define VICVectAddr2   (*((volatile unsigned long *) 0xFFFFF108))
#define VICVectAddr3   (*((volatile unsigned long *) 0xFFFFF10C))
#define VICVectAddr4   (*((volatile unsigned long *) 0xFFFFF110))
#define VICVectAddr5   (*((volatile unsigned long *) 0xFFFFF114))
#define VICVectAddr6   (*((volatile unsigned long *) 0xFFFFF118))
#define VICVectAddr7   (*((volatile unsigned long *) 0xFFFFF11C))
#define VICVectAddr8   (*((volatile unsigned long *) 0xFFFFF120))
#define VICVectAddr9   (*((volatile unsigned long *) 0xFFFFF124))
#define VICVectAddr10  (*((volatile unsigned long *) 0xFFFFF128))
#define VICVectAddr11  (*((volatile unsigned long *) 0xFFFFF12C))
#define VICVectAddr12  (*((volatile unsigned long *) 0xFFFFF130))
#define VICVectAddr13  (*((volatile unsigned long *) 0xFFFFF134))
#define VICVectAddr14  (*((volatile unsigned long *) 0xFFFFF138))
#define VICVectAddr15  (*((volatile unsigned long *) 0xFFFFF13C))
#define VICVectAddr16  (*((volatile unsigned long *) 0xFFFFF140))
#define VICVectAddr17  (*((volatile unsigned long *) 0xFFFFF144))
#define VICVectAddr18  (*((volatile unsigned long *) 0xFFFFF148))
#define VICVectAddr19  (*((volatile unsigned long *) 0xFFFFF14C))
#define VICVectAddr20  (*((volatile unsigned long *) 0xFFFFF150))
#define VICVectAddr21  (*((volatile unsigned long *) 0xFFFFF154))
#define VICVectAddr22  (*((volatile unsigned long *) 0xFFFFF158))
#define VICVectAddr23  (*((volatile unsigned long *) 0xFFFFF15C))
#define VICVectAddr24  (*((volatile unsigned long *) 0xFFFFF160))
#define VICVectAddr25  (*((volatile unsigned long *) 0xFFFFF164))
#define VICVectAddr26  (*((volatile unsigned long *) 0xFFFFF168))
#define VICVectAddr27  (*((volatile unsigned long *) 0xFFFFF16C))
#define VICVectAddr28  (*((volatile unsigned long *) 0xFFFFF170))
#define VICVectAddr29  (*((volatile unsigned long *) 0xFFFFF174))
#define VICVectAddr30  (*((volatile unsigned long *) 0xFFFFF178))
#define VICVectAddr31  (*((volatile unsigned long *) 0xFFFFF17C))

#define VICVectPriority0   (*((volatile unsigned long *) 0xFFFFF200))
#define VICVectPriority1   (*((volatile unsigned long *) 0xFFFFF204))
#define VICVectPriority2   (*((volatile unsigned long *) 0xFFFFF208))
#define VICVectPriority3   (*((volatile unsigned long *) 0xFFFFF20C))
#define VICVectPriority4   (*((volatile unsigned long *) 0xFFFFF210))
#define VICVectPriority5   (*((volatile unsigned long *) 0xFFFFF214))
#define VICVectPriority6   (*((volatile unsigned long *) 0xFFFFF218))
#define VICVectPriority7   (*((volatile unsigned long *) 0xFFFFF21C))
#define VICVectPriority8   (*((volatile unsigned long *) 0xFFFFF220))
#define VICVectPriority9   (*((volatile unsigned long *) 0xFFFFF224))
#define VICVectPriority10  (*((volatile unsigned long *) 0xFFFFF228))
#define VICVectPriority11  (*((volatile unsigned long *) 0xFFFFF22C))
#define VICVectPriority12  (*((volatile unsigned long *) 0xFFFFF230))
#define VICVectPriority13  (*((volatile unsigned long *) 0xFFFFF234))
#define VICVectPriority14  (*((volatile unsigned long *) 0xFFFFF238))
#define VICVectPriority15  (*((volatile unsigned long *) 0xFFFFF23C))
#define VICVectPriority16  (*((volatile unsigned long *) 0xFFFFF240))
#define VICVectPriority17  (*((volatile unsigned long *) 0xFFFFF244))
#define VICVectPriority18  (*((volatile unsigned long *) 0xFFFFF248))
#define VICVectPriority19  (*((volatile unsigned long *) 0xFFFFF24C))
#define VICVectPriority20  (*((volatile unsigned long *) 0xFFFFF250))
#define VICVectPriority21  (*((volatile unsigned long *) 0xFFFFF254))
#define VICVectPriority22  (*((volatile unsigned long *) 0xFFFFF258))
#define VICVectPriority23  (*((volatile unsigned long *) 0xFFFFF25C))
#define VICVectPriority24  (*((volatile unsigned long *) 0xFFFFF260))
#define VICVectPriority25  (*((volatile unsigned long *) 0xFFFFF264))
#define VICVectPriority26  (*((volatile unsigned long *) 0xFFFFF268))
#define VICVectPriority27  (*((volatile unsigned long *) 0xFFFFF26C))
#define VICVectPriority28  (*((volatile unsigned long *) 0xFFFFF270))
#define VICVectPriority29  (*((volatile unsigned long *) 0xFFFFF274))
#define VICVectPriority30  (*((volatile unsigned long *) 0xFFFFF278))
#define VICVectPriority31  (*((volatile unsigned long *) 0xFFFFF27C))

/* VIC */
#define VIC_CHAN_NUM_WDT      				(1<<0)
#define VIC_CHAN_NUM_UNUSED   				(1<<1)
#define VIC_CHAN_NUM_ARM_Core_DgbCommRX 	(1<<2)
#define VIC_CHAN_NUM_ARM_Core_DbgCommTX 	(1<<3)
#define VIC_CHAN_NUM_Timer0   				(1<<4)
#define VIC_CHAN_NUM_Timer1   				(1<<5)
#define VIC_CHAN_NUM_UART0    				(1<<6)
#define VIC_CHAN_NUM_UART1    				(1<<7)
#define VIC_CHAN_NUM_PWM1     				(1<<8)
#define VIC_CHAN_NUM_I2C0     				(1<<9)
#define VIC_CHAN_NUM_SPI     				(1<<10)
#define VIC_CHAN_NUM_SSP0    				(1<<10)
#define VIC_CHAN_NUM_SSP1    				(1<<11)
#define VIC_CHAN_NUM_PLL     				(1<<12)
#define VIC_CHAN_NUM_RTC     				(1<<13)
#define VIC_CHAN_NUM_EINT0   				(1<<14)
#define VIC_CHAN_NUM_EINT1   				(1<<15)
#define VIC_CHAN_NUM_EINT2   				(1<<16)
#define VIC_CHAN_NUM_EINT3   				(1<<17)
#define VIC_CHAN_NUM_ADC0    				(1<<18)
#define VIC_CHAN_NUM_I2C1    				(1<<19)
#define VIC_CHAN_NUM_BOD     				(1<<20)
#define VIC_CHAN_NUM_Ethernet 				(1<<21)
#define VIC_CHAN_NUM_USB     				(1<<22)
#define VIC_CHAN_NUM_CAN     				(1<<23)
#define VIC_CHAN_NUM_SD_MMC  				(1<<24)
#define VIC_CHAN_NUM_GP_DMA  				(1<<25)
#define VIC_CHAN_NUM_Timer2  				(1<<26)
#define VIC_CHAN_NUM_Timer3  				(1<<27)
#define VIC_CHAN_NUM_UART2   				(1<<28)
#define VIC_CHAN_NUM_UART3   				(1<<29)
#define VIC_CHAN_NUM_I2C2    				(1<<30)
#define VIC_CHAN_NUM_I2S     				(1<<31)

#define VICAddress		(*((volatile unsigned long *) 0xFFFFFF00))

/* Pin Connect Block */
#define PINSEL0        (*((volatile unsigned long *) 0xE002C000))
#define PINSEL1        (*((volatile unsigned long *) 0xE002C004))
#define PINSEL2        (*((volatile unsigned long *) 0xE002C008))
#define PINSEL3        (*((volatile unsigned long *) 0xE002C00C))
#define PINSEL4        (*((volatile unsigned long *) 0xE002C010))
#define PINSEL5        (*((volatile unsigned long *) 0xE002C014))
#define PINSEL6        (*((volatile unsigned long *) 0xE002C018))
#define PINSEL7        (*((volatile unsigned long *) 0xE002C01C))
#define PINSEL8        (*((volatile unsigned long *) 0xE002C020))
#define PINSEL9        (*((volatile unsigned long *) 0xE002C024))
#define PINSEL10       (*((volatile unsigned long *) 0xE002C028))

#define	PINMODE0			 (*((volatile unsigned long *) 0xE002C040))
#define	PINMODE1			 (*((volatile unsigned long *) 0xE002C044))
#define	PINMODE2			 (*((volatile unsigned long *) 0xE002C048))
#define	PINMODE3			 (*((volatile unsigned long *) 0xE002C04C))
#define	PINMODE4			 (*((volatile unsigned long *) 0xE002C050))
#define	PINMODE5			 (*((volatile unsigned long *) 0xE002C054))
#define	PINMODE6			 (*((volatile unsigned long *) 0xE002C058))
#define	PINMODE7			 (*((volatile unsigned long *) 0xE002C05C))
#define	PINMODE8			 (*((volatile unsigned long *) 0xE002C060))
#define	PINMODE9			 (*((volatile unsigned long *) 0xE002C064))

/* Legacy GPIO */
#define	IO0PIN					(*((volatile unsigned long *) 0xE0028000))
#define	IO1PIN					(*((volatile unsigned long *) 0xE0028010))
#define	IO0SET					(*((volatile unsigned long *) 0xE0028004))
#define	IO1SET					(*((volatile unsigned long *) 0xE0028014))
#define	IO0DIR					(*((volatile unsigned long *) 0xE0028008))
#define	IO1DIR					(*((volatile unsigned long *) 0xE0028018))
#define	IO0CLR					(*((volatile unsigned long *) 0xE002800C))
#define	IO1CLR					(*((volatile unsigned long *) 0xE002801C))

/* Fast GPIO */
/* Port 0 */
#define	FIO0DIR					(*((volatile unsigned long *) 0x3FFFC000))
#define	FIO0DIR0				(*((volatile unsigned long *) 0x3FFFC000))
#define	FIO0DIR1				(*((volatile unsigned long *) 0x3FFFC001))
#define	FIO0DIR2				(*((volatile unsigned long *) 0x3FFFC002))
#define	FIO0DIR3				(*((volatile unsigned long *) 0x3FFFC003))
#define	FIO0DIRL				(*((volatile unsigned long *) 0x3FFFC000))
#define	FIO0DIRU				(*((volatile unsigned long *) 0x3FFFC002))

/* Port 1 */
#define	FIO1DIR					(*((volatile unsigned long *) 0x3FFFC020))
#define	FIO1DIR0				(*((volatile unsigned long *) 0x3FFFC020))
#define	FIO1DIR1				(*((volatile unsigned long *) 0x3FFFC021))
#define	FIO1DIR2				(*((volatile unsigned long *) 0x3FFFC022))
#define	FIO1DIR3				(*((volatile unsigned long *) 0x3FFFC023))
#define	FIO1DIRL				(*((volatile unsigned long *) 0x3FFFC020))
#define	FIO1DIRU				(*((volatile unsigned long *) 0x3FFFC022))

/* Port 2 */
#define	FIO2DIR					(*((volatile unsigned long *) 0x3FFFC040))
#define	FIO2DIR0				(*((volatile unsigned long *) 0x3FFFC040))
#define	FIO2DIR1				(*((volatile unsigned long *) 0x3FFFC041))
#define	FIO2DIR2				(*((volatile unsigned long *) 0x3FFFC042))
#define	FIO2DIR3				(*((volatile unsigned long *) 0x3FFFC043))
#define	FIO2DIRL				(*((volatile unsigned long *) 0x3FFFC040))
#define	FIO2DIRU				(*((volatile unsigned long *) 0x3FFFC042))

/* Port 3 */
#define	FIO3DIR					(*((volatile unsigned long *) 0x3FFFC060))
#define	FIO3DIR0				(*((volatile unsigned long *) 0x3FFFC060))
#define	FIO3DIR1				(*((volatile unsigned long *) 0x3FFFC061))
#define	FIO3DIR2				(*((volatile unsigned long *) 0x3FFFC062))
#define	FIO3DIR3				(*((volatile unsigned long *) 0x3FFFC063))
#define	FIO3DIRL				(*((volatile unsigned long *) 0x3FFFC060))
#define	FIO3DIRU				(*((volatile unsigned long *) 0x3FFFC062))

/* Port 4*/
#define	FIO4DIR					(*((volatile unsigned long *) 0x3FFFC080))
#define	FIO4DIR0				(*((volatile unsigned long *) 0x3FFFC080))
#define	FIO4DIR1				(*((volatile unsigned long *) 0x3FFFC081))
#define	FIO4DIR2				(*((volatile unsigned long *) 0x3FFFC082))
#define	FIO4DIR3				(*((volatile unsigned long *) 0x3FFFC083))
#define	FIO4DIRL				(*((volatile unsigned long *) 0x3FFFC080))
#define	FIO4DIRU				(*((volatile unsigned long *) 0x3FFFC082))

/* Fast IOSET Port 0*/
#define	FIO0SET					(*((volatile unsigned long *) 0x3FFFC018))
#define	FIO0SET0				(*((volatile unsigned long *) 0x3FFFC018))
#define	FIO0SET1				(*((volatile unsigned long *) 0x3FFFC019))
#define	FIO0SET2				(*((volatile unsigned long *) 0x3FFFC01A))
#define	FIO0SET3				(*((volatile unsigned long *) 0x3FFFC01B))
#define	FIO0SETL				(*((volatile unsigned long *) 0x3FFFC018))
#define	FIO0SETU				(*((volatile unsigned long *) 0x3FFFC01A))

/* Fast IOSET Port 1*/
#define	FIO1SET					(*((volatile unsigned long *) 0x3FFFC038))
#define	FIO1SET0				(*((volatile unsigned long *) 0x3FFFC038))
#define	FIO1SET1				(*((volatile unsigned long *) 0x3FFFC039))
#define	FIO1SET2				(*((volatile unsigned long *) 0x3FFFC03A))
#define	FIO1SET3				(*((volatile unsigned long *) 0x3FFFC03B))
#define	FIO1SETL				(*((volatile unsigned long *) 0x3FFFC038))
#define	FIO1SETU				(*((volatile unsigned long *) 0x3FFFC03A))

/* Fast IOSET Port 2*/
#define	FIO2SET					(*((volatile unsigned long *) 0x3FFFC058))
#define	FIO2SET0				(*((volatile unsigned long *) 0x3FFFC058))
#define	FIO2SET1				(*((volatile unsigned long *) 0x3FFFC059))
#define	FIO2SET2				(*((volatile unsigned long *) 0x3FFFC05A))
#define	FIO2SET3				(*((volatile unsigned long *) 0x3FFFC05B))
#define	FIO2SETL				(*((volatile unsigned long *) 0x3FFFC058))
#define	FIO2SETU				(*((volatile unsigned long *) 0x3FFFC05A))

/* Fast IOSET Port 3*/
#define	FIO3SET					(*((volatile unsigned long *) 0x3FFFC078))
#define	FIO3SET0				(*((volatile unsigned long *) 0x3FFFC078))
#define	FIO3SET1				(*((volatile unsigned long *) 0x3FFFC079))
#define	FIO3SET2				(*((volatile unsigned long *) 0x3FFFC07A))
#define	FIO3SET3				(*((volatile unsigned long *) 0x3FFFC07B))
#define	FIO3SETL				(*((volatile unsigned long *) 0x3FFFC078))
#define	FIO3SETU				(*((volatile unsigned long *) 0x3FFFC07A))

/* Fast IOSET Port 4*/
#define	FIO4SET					(*((volatile unsigned long *) 0x3FFFC098))
#define	FIO4SET0				(*((volatile unsigned long *) 0x3FFFC098))
#define	FIO4SET1				(*((volatile unsigned long *) 0x3FFFC099))
#define	FIO4SET2				(*((volatile unsigned long *) 0x3FFFC09A))
#define	FIO4SET3				(*((volatile unsigned long *) 0x3FFFC09B))
#define	FIO4SETL				(*((volatile unsigned long *) 0x3FFFC098))
#define	FIO4SETU				(*((volatile unsigned long *) 0x3FFFC09A))

/* Fast IOCLR Port 0*/
#define	FIO0CLR					(*((volatile unsigned long *) 0x3FFFC01C))
#define	FIO0CLR0				(*((volatile unsigned long *) 0x3FFFC01C))
#define	FIO0CLR1				(*((volatile unsigned long *) 0x3FFFC01D))
#define	FIO0CLR2				(*((volatile unsigned long *) 0x3FFFC01E))
#define	FIO0CLR3				(*((volatile unsigned long *) 0x3FFFC01F))
#define	FIO0CLRL				(*((volatile unsigned long *) 0x3FFFC01C))
#define	FIO0CLRU				(*((volatile unsigned long *) 0x3FFFC01E))

/* Fast IOCLR Port 1*/
#define	FIO1CLR					(*((volatile unsigned long *) 0x3FFFC03C))
#define	FIO1CLR0				(*((volatile unsigned long *) 0x3FFFC03C))
#define	FIO1CLR1				(*((volatile unsigned long *) 0x3FFFC03D))
#define	FIO1CLR2				(*((volatile unsigned long *) 0x3FFFC03E))
#define	FIO1CLR3				(*((volatile unsigned long *) 0x3FFFC03F))
#define	FIO1CLRL				(*((volatile unsigned long *) 0x3FFFC03C))
#define	FIO1CLRU				(*((volatile unsigned long *) 0x3FFFC03E))

/* Fast IOCLR Port 2*/
#define	FIO2CLR					(*((volatile unsigned long *) 0x3FFFC05C))
#define	FIO2CLR0				(*((volatile unsigned long *) 0x3FFFC05C))
#define	FIO2CLR1				(*((volatile unsigned long *) 0x3FFFC05D))
#define	FIO2CLR2				(*((volatile unsigned long *) 0x3FFFC05E))
#define	FIO2CLR3				(*((volatile unsigned long *) 0x3FFFC05F))
#define	FIO2CLRL				(*((volatile unsigned long *) 0x3FFFC05C))
#define	FIO2CLRU				(*((volatile unsigned long *) 0x3FFFC05E))

/* Fast IOCLR Port 3*/
#define	FIO3CLR					(*((volatile unsigned long *) 0x3FFFC07C))
#define	FIO3CLR0				(*((volatile unsigned long *) 0x3FFFC07C))
#define	FIO3CLR1				(*((volatile unsigned long *) 0x3FFFC07D))
#define	FIO3CLR2				(*((volatile unsigned long *) 0x3FFFC07E))
#define	FIO3CLR3				(*((volatile unsigned long *) 0x3FFFC07F))
#define	FIO3CLRL				(*((volatile unsigned long *) 0x3FFFC07C))
#define	FIO3CLRU				(*((volatile unsigned long *) 0x3FFFC07E))

/* Fast IOCLR Port 4*/
#define	FIO4CLR					(*((volatile unsigned long *) 0x3FFFC09C))
#define	FIO4CLR0				(*((volatile unsigned long *) 0x3FFFC09C))
#define	FIO4CLR1				(*((volatile unsigned long *) 0x3FFFC09D))
#define	FIO4CLR2				(*((volatile unsigned long *) 0x3FFFC09E))
#define	FIO4CLR3				(*((volatile unsigned long *) 0x3FFFC09F))
#define	FIO4CLRL				(*((volatile unsigned long *) 0x3FFFC09C))
#define	FIO4CLRU				(*((volatile unsigned long *) 0x3FFFC09E))

/* Fast IOPIN Port 0*/
#define	FIO0PIN					(*((volatile unsigned long *) 0x3FFFC014))
#define	FIO0PIN0				(*((volatile unsigned long *) 0x3FFFC014))
#define	FIO0PIN1				(*((volatile unsigned long *) 0x3FFFC015))
#define	FIO0PIN2				(*((volatile unsigned long *) 0x3FFFC016))
#define	FIO0PIN3				(*((volatile unsigned long *) 0x3FFFC017))
#define	FIO0PINL				(*((volatile unsigned long *) 0x3FFFC014))
#define	FIO0PINU				(*((volatile unsigned long *) 0x3FFFC016))

/* Fast IOPIN Port 1*/
#define	FIO1PIN					(*((volatile unsigned long *) 0x3FFFC034))
#define	FIO1PIN0				(*((volatile unsigned long *) 0x3FFFC034))
#define	FIO1PIN1				(*((volatile unsigned long *) 0x3FFFC035))
#define	FIO1PIN2				(*((volatile unsigned long *) 0x3FFFC036))
#define	FIO1PIN3				(*((volatile unsigned long *) 0x3FFFC037))
#define	FIO1PINL				(*((volatile unsigned long *) 0x3FFFC034))
#define	FIO1PINU				(*((volatile unsigned long *) 0x3FFFC036))

/* Fast IOPIN Port 2*/
#define	FIO2PIN					(*((volatile unsigned long *) 0x3FFFC054))
#define	FIO2PIN0				(*((volatile unsigned long *) 0x3FFFC054))
#define	FIO2PIN1				(*((volatile unsigned long *) 0x3FFFC055))
#define	FIO2PIN2				(*((volatile unsigned long *) 0x3FFFC056))
#define	FIO2PIN3				(*((volatile unsigned long *) 0x3FFFC057))
#define	FIO2PINL				(*((volatile unsigned long *) 0x3FFFC054))
#define	FIO2PINU				(*((volatile unsigned long *) 0x3FFFC056))

/* Fast IOPIN Port 3*/
#define	FIO3PIN					(*((volatile unsigned long *) 0x3FFFC074))
#define	FIO3PIN0				(*((volatile unsigned long *) 0x3FFFC074))
#define	FIO3PIN1				(*((volatile unsigned long *) 0x3FFFC075))
#define	FIO3PIN2				(*((volatile unsigned long *) 0x3FFFC076))
#define	FIO3PIN3				(*((volatile unsigned long *) 0x3FFFC077))
#define	FIO3PINL				(*((volatile unsigned long *) 0x3FFFC074))
#define	FIO3PINU				(*((volatile unsigned long *) 0x3FFFC076))

/* Fast IOPIN Port 4*/
#define	FIO4PIN					(*((volatile unsigned long *) 0x3FFFC094))
#define	FIO4PIN0				(*((volatile unsigned long *) 0x3FFFC094))
#define	FIO4PIN1				(*((volatile unsigned long *) 0x3FFFC095))
#define	FIO4PIN2				(*((volatile unsigned long *) 0x3FFFC096))
#define	FIO4PIN3				(*((volatile unsigned long *) 0x3FFFC097))
#define	FIO4PINL				(*((volatile unsigned long *) 0x3FFFC094))
#define	FIO4PINU				(*((volatile unsigned long *) 0x3FFFC096))

/* Fast IOmask Port 0*/
#define	FIO0MASK				(*((volatile unsigned long *) 0x3FFFC010))
#define	FIO0MASK0				(*((volatile unsigned long *) 0x3FFFC010))
#define	FIO0MASK1				(*((volatile unsigned long *) 0x3FFFC011))
#define	FIO0MASK2				(*((volatile unsigned long *) 0x3FFFC012))
#define	FIO0MASK3				(*((volatile unsigned long *) 0x3FFFC013))
#define	FIO0MASKL				(*((volatile unsigned long *) 0x3FFFC010))
#define	FIO0MASKU				(*((volatile unsigned long *) 0x3FFFC012))

/* Fast IOmask Port 1*/
#define	FIO1MASK				(*((volatile unsigned long *) 0x3FFFC030))
#define	FIO1MASK0				(*((volatile unsigned long *) 0x3FFFC030))
#define	FIO1MASK1				(*((volatile unsigned long *) 0x3FFFC031))
#define	FIO1MASK2				(*((volatile unsigned long *) 0x3FFFC032))
#define	FIO1MASK3				(*((volatile unsigned long *) 0x3FFFC033))
#define	FIO1MASKL				(*((volatile unsigned long *) 0x3FFFC030))
#define	FIO1MASKU				(*((volatile unsigned long *) 0x3FFFC032))

/* Fast IOmask Port 2*/
#define	FIO2MASK				(*((volatile unsigned long *) 0x3FFFC050))
#define	FIO2MASK0				(*((volatile unsigned long *) 0x3FFFC050))
#define	FIO2MASK1				(*((volatile unsigned long *) 0x3FFFC051))
#define	FIO2MASK2				(*((volatile unsigned long *) 0x3FFFC052))
#define	FIO2MASK3				(*((volatile unsigned long *) 0x3FFFC053))
#define	FIO2MASKL				(*((volatile unsigned long *) 0x3FFFC050))
#define	FIO2MASKU				(*((volatile unsigned long *) 0x3FFFC052))

/* Fast IOmask Port 3*/
#define	FIO3MASK				(*((volatile unsigned long *) 0x3FFFC070))
#define	FIO3MASK0				(*((volatile unsigned long *) 0x3FFFC070))
#define	FIO3MASK1				(*((volatile unsigned long *) 0x3FFFC071))
#define	FIO3MASK2				(*((volatile unsigned long *) 0x3FFFC072))
#define	FIO3MASK3				(*((volatile unsigned long *) 0x3FFFC073))
#define	FIO3MASKL				(*((volatile unsigned long *) 0x3FFFC070))
#define	FIO3MASKU				(*((volatile unsigned long *) 0x3FFFC072))

/* Fast IOmask Port 4*/
#define	FIO4MASK				(*((volatile unsigned long *) 0x3FFFC090))
#define	FIO4MASK0				(*((volatile unsigned long *) 0x3FFFC090))
#define	FIO4MASK1				(*((volatile unsigned long *) 0x3FFFC091))
#define	FIO4MASK2				(*((volatile unsigned long *) 0x3FFFC092))
#define	FIO4MASK3				(*((volatile unsigned long *) 0x3FFFC093))
#define	FIO4MASKL				(*((volatile unsigned long *) 0x3FFFC090))
#define	FIO4MASKU				(*((volatile unsigned long *) 0x3FFFC092))

/* Memory Accelerator Module (MAM) */
#define MAMCR          (*((volatile unsigned char *) 0xE01FC000))
#define MAMTIM         (*((volatile unsigned char *) 0xE01FC004))

/* Phase Locked Loop (PLL)  clocks and power control */
#define PLLCON         	(*((volatile unsigned char *) 0xE01FC080))
#define PLLCFG         	(*((volatile unsigned long *) 0xE01FC084))
#define PLLSTAT        	(*((volatile unsigned long*) 0xE01FC088))
#define PLLFEED        	(*((volatile unsigned char *) 0xE01FC08C))

#define	CCLKCFG					(*((volatile unsigned char *) 0xE01FC104))
#define	USBCLKCFG				(*((volatile unsigned char *) 0xE01FC108))
#define	CLKSRCSEL				(*((volatile unsigned char *) 0xE01FC10C))

#define	IRCTRIM					(*((volatile unsigned short *) 0xE01FC1A4))
#define	PCLKSEL0				(*((volatile unsigned long *) 0xE01FC1A8))
#define	PCLKSEL1				(*((volatile unsigned long *) 0xE01FC1AC))

#define	PCON						(*((volatile unsigned char *) 0xE01FC0C0))
#define	PCONP						(*((volatile unsigned long *) 0xE01FC0C4))
#define	INTWAKE					(*((volatile unsigned short *) 0xE01FC144))

/* External Interrupts */
#define EXTINT         (*((volatile unsigned char *) 0xE01FC140))
#define INTWAKE        (*((volatile unsigned short *) 0xE01FC144))

/* Timer 0 */
#define TIMER0_IR      (*((volatile unsigned char *) 0xE0004000))
#define TIMER0_TCR     (*((volatile unsigned char *) 0xE0004004))
#define TIMER0_TC      (*((volatile unsigned long *) 0xE0004008))
#define TIMER0_PR      (*((volatile unsigned long *) 0xE000400C))
#define TIMER0_PC      (*((volatile unsigned long *) 0xE0004010))
#define TIMER0_MCR     (*((volatile unsigned short *) 0xE0004014))
#define TIMER0_MR0     (*((volatile unsigned long *) 0xE0004018))
#define TIMER0_MR1     (*((volatile unsigned long *) 0xE000401C))
#define TIMER0_MR2     (*((volatile unsigned long *) 0xE0004020))
#define TIMER0_MR3     (*((volatile unsigned long *) 0xE0004024))
#define TIMER0_CCR     (*((volatile unsigned short *) 0xE0004028))
#define TIMER0_CR0     (*((volatile unsigned long *) 0xE000402C))
#define TIMER0_CR1     (*((volatile unsigned long *) 0xE0004030))
#define TIMER0_EMR     (*((volatile unsigned short *) 0xE000403C))
#define TIMER0_CTCR	   (*((volatile unsigned char *) 0xE0004070))

/* Timer 1 */
#define TIMER1_IR      (*((volatile unsigned char *) 0xE0008000))
#define TIMER1_TCR     (*((volatile unsigned char *) 0xE0008004))
#define TIMER1_TC      (*((volatile unsigned long *) 0xE0008008))
#define TIMER1_PR      (*((volatile unsigned long *) 0xE000800C))
#define TIMER1_PC      (*((volatile unsigned long *) 0xE0008010))
#define TIMER1_MCR     (*((volatile unsigned short *) 0xE0008014))
#define TIMER1_MR0     (*((volatile unsigned long *) 0xE0008018))
#define TIMER1_MR1     (*((volatile unsigned long *) 0xE000801C))
#define TIMER1_MR2     (*((volatile unsigned long *) 0xE0008020))
#define TIMER1_MR3     (*((volatile unsigned long *) 0xE0008024))
#define TIMER1_CCR     (*((volatile unsigned short *) 0xE0008028))
#define TIMER1_CR0     (*((volatile unsigned long *) 0xE000802C))
#define TIMER1_CR1     (*((volatile unsigned long *) 0xE0008030))
#define TIMER1_EMR     (*((volatile unsigned short *) 0xE000803C))
#define TIMER1_CTCR	   (*((volatile unsigned char *) 0xE0008070))

/* Timer 2 */
#define TIMER2_IR      (*((volatile unsigned char *) 0xE0070000))
#define TIMER2_TCR     (*((volatile unsigned char *) 0xE0070004))
#define TIMER2_TC      (*((volatile unsigned long *) 0xE0070008))
#define TIMER2_PR      (*((volatile unsigned long *) 0xE007000C))
#define TIMER2_PC      (*((volatile unsigned long *) 0xE0070010))
#define TIMER2_MCR     (*((volatile unsigned short *) 0xE0070014))
#define TIMER2_MR0     (*((volatile unsigned long *) 0xE0070018))
#define TIMER2_MR1     (*((volatile unsigned long *) 0xE007001C))
#define TIMER2_MR2     (*((volatile unsigned long *) 0xE0070020))
#define TIMER2_MR3     (*((volatile unsigned long *) 0xE0070024))
#define TIMER2_CCR     (*((volatile unsigned short *) 0xE0070028))
#define TIMER2_CR0     (*((volatile unsigned long *) 0xE007002C))
#define TIMER2_CR1     (*((volatile unsigned long *) 0xE0070030))
#define TIMER2_EMR     (*((volatile unsigned short *) 0xE007003C))
#define TIMER2_CTCR	   (*((volatile unsigned char *) 0xE0070070))

/* Timer 3 */
#define TIMER3_IR      (*((volatile unsigned char *) 0xE0074000))
#define TIMER3_TCR     (*((volatile unsigned char *) 0xE0074004))
#define TIMER3_TC      (*((volatile unsigned long *) 0xE0074008))
#define TIMER3_PR      (*((volatile unsigned long *) 0xE007400C))
#define TIMER3_PC      (*((volatile unsigned long *) 0xE0074010))
#define TIMER3_MCR     (*((volatile unsigned short *) 0xE0074014))
#define TIMER3_MR0     (*((volatile unsigned long *) 0xE0074018))
#define TIMER3_MR1     (*((volatile unsigned long *) 0xE007401C))
#define TIMER3_MR2     (*((volatile unsigned long *) 0xE0074020))
#define TIMER3_MR3     (*((volatile unsigned long *) 0xE0074024))
#define TIMER3_CCR     (*((volatile unsigned short *) 0xE0074028))
#define TIMER3_CR0     (*((volatile unsigned long *) 0xE007402C))
#define TIMER3_CR1     (*((volatile unsigned long *) 0xE0074030))
#define TIMER3_EMR     (*((volatile unsigned short *) 0xE007403C))
#define TIMER3_CTCR	   (*((volatile unsigned char *) 0xE0074070))

/* Pulse Width Modulator (PWM) */
#define PWM_IR         (*((volatile unsigned short *) 0xE0018000))
#define PWM_TCR        (*((volatile unsigned char *) 0xE0018004))
#define PWM_TC         (*((volatile unsigned long *) 0xE0018008))
#define PWM_PR         (*((volatile unsigned long *) 0xE001800C))
#define PWM_PC         (*((volatile unsigned long *) 0xE0018010))
#define PWM_MCR        (*((volatile unsigned long *) 0xE0018014))
#define PWM_MR0        (*((volatile unsigned long *) 0xE0018018))
#define PWM_MR1        (*((volatile unsigned long *) 0xE001801C))
#define PWM_MR2        (*((volatile unsigned long *) 0xE0018020))
#define PWM_MR3        (*((volatile unsigned long *) 0xE0018024))
#define PWM_MR4        (*((volatile unsigned long *) 0xE0018040))
#define PWM_MR5        (*((volatile unsigned long *) 0xE0018044))
#define PWM_MR6        (*((volatile unsigned long *) 0xE0018048))
#define PWM_CCR        (*((volatile unsigned long *) 0xE0018028))
#define PWM_CR0        (*((volatile unsigned long *) 0xE001802C))
#define PWM_CR1        (*((volatile unsigned long *) 0xE0018030))
#define PWM_CR2        (*((volatile unsigned long *) 0xE0018034))
#define PWM_CR3        (*((volatile unsigned long *) 0xE0018038))
#define PWM_PCR        (*((volatile unsigned long *) 0xE001804C))
#define PWM_LER        (*((volatile unsigned char *) 0xE0018050))
#define PWM_CTCR       (*((volatile unsigned char *) 0xE0018070))

/* Universal Asynchronous Receiver Transmitter 0 (UART0) */
#define UART0_RBR      (*((volatile unsigned char *) 0xE000C000))
#define UART0_THR      (*((volatile unsigned char *) 0xE000C000))
#define UART0_DLL      (*((volatile unsigned char *) 0xE000C000))
#define UART0_DLM      (*((volatile unsigned char *) 0xE000C004))
#define UART0_IER      (*((volatile unsigned long *) 0xE000C004))
#define UART0_IIR      (*((volatile unsigned long *) 0xE000C008))
#define UART0_FCR      (*((volatile unsigned char *) 0xE000C008))
#define UART0_LCR      (*((volatile unsigned char *) 0xE000C00C))
#define UART0_LSR      (*((volatile unsigned char *) 0xE000C014))
#define UART0_SCR      (*((volatile unsigned char *) 0xE000C01C))
#define UART0_ACR      (*((volatile unsigned long *) 0xE000C020))
#define UART0_FDR      (*((volatile unsigned long *) 0xE000C028))
#define UART0_TER      (*((volatile unsigned char *) 0xE000C030))

/* Universal Asynchronous Receiver Transmitter 2 (UART2) */
#define UART2_RBR      (*((volatile unsigned char *) 0xE0078000))
#define UART2_THR      (*((volatile unsigned char *) 0xE0078000))
#define UART2_DLL      (*((volatile unsigned char *) 0xE0078000))
#define UART2_DLM      (*((volatile unsigned char *) 0xE0078004))
#define UART2_IER      (*((volatile unsigned long *) 0xE0078004))
#define UART2_IIR      (*((volatile unsigned long *) 0xE0078008))
#define UART2_FCR      (*((volatile unsigned char *) 0xE0078008))
#define UART2_LCR      (*((volatile unsigned char *) 0xE007800C))
#define UART2_LSR      (*((volatile unsigned char *) 0xE0078014))
#define UART2_SCR      (*((volatile unsigned char *) 0xE007801C))
#define UART2_ACR      (*((volatile unsigned long *) 0xE0078020))
#define UART2_FDR      (*((volatile unsigned long *) 0xE0078028))
#define UART2_TER      (*((volatile unsigned char *) 0xE0078030))

/* Universal Asynchronous Receiver Transmitter 3 (UART3) */
#define UART3_RBR      (*((volatile unsigned char *) 0xE007C000))
#define UART3_THR      (*((volatile unsigned char *) 0xE007C000))
#define UART3_DLL      (*((volatile unsigned char *) 0xE007C000))
#define UART3_DLM      (*((volatile unsigned char *) 0xE007C004))
#define UART3_IER      (*((volatile unsigned long *) 0xE007C004))
#define UART3_IIR      (*((volatile unsigned long *) 0xE007C008))
#define UART3_FCR      (*((volatile unsigned char *) 0xE007C008))
#define UART3_LCR      (*((volatile unsigned char *) 0xE007C00C))
#define UART3_LSR      (*((volatile unsigned char *) 0xE007C014))
#define UART3_SCR      (*((volatile unsigned char *) 0xE007C01C))
#define UART3_ACR      (*((volatile unsigned long *) 0xE007C020))
#define UART3_ICR      (*((volatile unsigned long *) 0xE007C024))
#define UART3_FDR      (*((volatile unsigned long *) 0xE007C028))
#define UART3_TER      (*((volatile unsigned char *) 0xE007C030))

/* Universal Asynchronous Receiver Transmitter 1 (UART1) */
#define UART1_RBR      (*((volatile unsigned char *) 0xE0010000))
#define UART1_THR      (*((volatile unsigned char *) 0xE0010000))
#define UART1_DLL      (*((volatile unsigned char *) 0xE0010000))
#define UART1_DLM      (*((volatile unsigned char *) 0xE0010004))
#define UART1_IER      (*((volatile unsigned long *) 0xE0010004))
#define UART1_IIR      (*((volatile unsigned long *) 0xE0010008))
#define UART1_FCR      (*((volatile unsigned char *) 0xE0010008))
#define UART1_LCR      (*((volatile unsigned char *) 0xE001000C))
#define UART1_MCR      (*((volatile unsigned char *) 0xE0010010))
#define UART1_LSR      (*((volatile unsigned char *) 0xE0010014))
#define UART1_MSR      (*((volatile unsigned char *) 0xE0010018))
#define UART1_SCR      (*((volatile unsigned char *) 0xE001001C))
#define UART1_ACR      (*((volatile unsigned long *) 0xE0010020))
#define UART1_FDR      (*((volatile unsigned long *) 0xE0010028))
#define UART1_TER      (*((volatile unsigned char *) 0xE0010030))

/* I2C 0 Interface */
#define I2C0CONSET   (*((volatile unsigned char *) 0xE001C000))
#define I2C0STAT     (*((volatile unsigned char *) 0xE001C004))
#define I2C0DAT      (*((volatile unsigned char *) 0xE001C008))
#define I2C0ADR      (*((volatile unsigned char *) 0xE001C00C))
#define I2C0SCLH     (*((volatile unsigned short*) 0xE001C010))
#define I2C0SCLL     (*((volatile unsigned short*) 0xE001C014))
#define I2C0CONCLR   (*((volatile unsigned char *) 0xE001C018))

/* I2C 1 Interface */
#define I2C1CONSET   (*((volatile unsigned char *) 0xE005C000))
#define I2C1STAT     (*((volatile unsigned char *) 0xE005C004))
#define I2C1DAT      (*((volatile unsigned char *) 0xE005C008))
#define I2C1ADR      (*((volatile unsigned char *) 0xE005C00C))
#define I2C1SCLH     (*((volatile unsigned short*) 0xE005C010))
#define I2C1SCLL     (*((volatile unsigned short*) 0xE005C014))
#define I2C1CONCLR   (*((volatile unsigned char *) 0xE005C018))

/* I2C 2 Interface */
#define I2C2CONSET   (*((volatile unsigned char *) 0xE0080000))
#define I2C2STAT     (*((volatile unsigned char *) 0xE0080004))
#define I2C2DAT      (*((volatile unsigned char *) 0xE0080008))
#define I2C2ADR      (*((volatile unsigned char *) 0xE008000C))
#define I2C2SCLH     (*((volatile unsigned short*) 0xE0080010))
#define I2C2SCLL     (*((volatile unsigned short*) 0xE0080014))
#define I2C2CONCLR   (*((volatile unsigned char *) 0xE0080018))

/* SPI (Serial Peripheral Interface) */
#define SPI_SPCR       (*((volatile unsigned short *) 0xE0020000))
#define SPI_SPSR       (*((volatile unsigned char *) 0xE0020004))
#define SPI_SPDR       (*((volatile unsigned short *) 0xE0020008))
#define SPI_SPCCR      (*((volatile unsigned char *) 0xE002000C))
#define SPI_SPTCR      (*((volatile unsigned char *) 0xE0020010))
#define SPI_SPTSR      (*((volatile unsigned char *) 0xE0020014))
#define SPI_SPINT      (*((volatile unsigned char *) 0xE002001C))

/* Real Time Clock */
#define RTC_ILR        (*((volatile unsigned char *) 0xE0024000))
#define RTC_CTC        (*((volatile unsigned short*) 0xE0024004))
#define RTC_CCR        (*((volatile unsigned char *) 0xE0024008))
#define RTC_CIIR       (*((volatile unsigned char *) 0xE002400C))
#define RTC_CISS       (*((volatile unsigned char *) 0xE0024040))
#define RTC_AMR        (*((volatile unsigned char *) 0xE0024010))
#define RTC_CTIME0     (*((volatile unsigned long *) 0xE0024014))
#define RTC_CTIME1     (*((volatile unsigned long *) 0xE0024018))
#define RTC_CTIME2     (*((volatile unsigned long *) 0xE002401C))
#define RTC_SEC        (*((volatile unsigned char *) 0xE0024020))
#define RTC_MIN        (*((volatile unsigned char *) 0xE0024024))
#define RTC_HOUR       (*((volatile unsigned char *) 0xE0024028))
#define RTC_DOM        (*((volatile unsigned char *) 0xE002402C))
#define RTC_DOW        (*((volatile unsigned char *) 0xE0024030))
#define RTC_DOY        (*((volatile unsigned short*) 0xE0024034))
#define RTC_MONTH      (*((volatile unsigned char *) 0xE0024038))
#define RTC_YEAR       (*((volatile unsigned short*) 0xE002403C))
#define RTC_ALSEC      (*((volatile unsigned char *) 0xE0024060))
#define RTC_ALMIN      (*((volatile unsigned char *) 0xE0024064))
#define RTC_ALHOUR     (*((volatile unsigned char *) 0xE0024068))
#define RTC_ALDOM      (*((volatile unsigned char *) 0xE002406C))
#define RTC_ALDOW      (*((volatile unsigned char *) 0xE0024070))
#define RTC_ALDOY      (*((volatile unsigned short*) 0xE0024074))
#define RTC_ALMON      (*((volatile unsigned char *) 0xE0024078))
#define RTC_ALYEAR     (*((volatile unsigned short*) 0xE002407C))
#define RTC_PREINT     (*((volatile unsigned short*) 0xE0024080))
#define RTC_PREFRAC    (*((volatile unsigned short*) 0xE0024084))

/* Watchdog */
#define WDMOD          (*((volatile unsigned char *) 0xE0000000))
#define WDTC           (*((volatile unsigned long *) 0xE0000004))
#define WDFEED         (*((volatile unsigned char *) 0xE0000008))
#define WDTV           (*((volatile unsigned long *) 0xE000000C))
#define WDCLKSEL       (*((volatile unsigned long *) 0xE0000010))

#endif  // __LPC210x_H
