/*
 * lcd.h
 *
 *  Created on: Jul 25, 2010
 *      Author: Titus
 */

#ifndef LCD_H_
#define LCD_H_

#include "app_types.h"
#include "LPC23xx.h"

/*************************************************************************
 *             Definitions
 *************************************************************************/
#define PCSSP0			(1<<21)
#define PCLK_SSP0_1		(1<<10)
#define PINSEL3_SCK0	(3<<8)
#define PINSEL3_SSEL0	(3<<10)
#define PINSEL3_MISO0	(3<<14)
#define PINSEL3_MOSI0	(3<<16)
#define LCD_CS_MASK		(1<<21)

#define SSP0CR0_DSS_4	(3)
#define SSP0CR0_DSS_5	(4)
#define SSP0CR0_DSS_6	(5)
#define SSP0CR0_DSS_7	(6)
#define SSP0CR0_DSS_8	(7)
#define SSP0CR0_DSS_9	(8)
#define SSP0CR0_DSS_10	(9)
#define SSP0CR0_DSS_11	(10)
#define SSP0CR0_DSS_12	(11)
#define SSP0CR0_DSS_13	(12)
#define SSP0CR0_DSS_14	(13)
#define SSP0CR0_DSS_15	(14)
#define SSP0CR0_DSS_16	(15)
#define SSP0CR0_FRF_SPI	(0<<4)
#define SSP0CR0_FRF_TI	(1<<4)
#define SSP0CR0_FRF_MW	(2<<4)
#define SSP0CR0_CPOL	(1<<6)
#define SSP0CR0_CPHA	(1<<7)
#define SSP0CR0_SCR		(35<<8)

#define SSP0CR1_LBM		(1<<0)
#define SSP0CR1_SSE		(1<<1)
#define SSP0CR1_MS		(1<<2)
#define SSP0CR1_SOD		(1<<3)

#define SSP0SR_TFE		(1<<0)
#define SSP0SR_TNF		(1<<1)
#define SSP0SR_RNE		(1<<2)
#define SSP0SR_RFF		(1<<3)
#define SSP0SR_BSY		(1<<4)

#define SSP0CPSR_CPSDVSR (72)

#define LCD_BACKLIGHT	(1<<26)
#define LCD_RESET		(1<<25)

// Epson S1D15G00 Controller
#define DISON     0xAF      // Display on
#define DISOFF    0xAE      // Display off
#define DISNOR    0xA6      // Normal display
#define DISINV    0xA7      // Inverse display
#define COMSCN    0xBB      // Common scan direction
#define DISCTL    0xCA      // Display control
#define SLPIN     0x95      // Sleep in
#define SLPOUT    0x94      // Sleep out
#define PASET     0x75      // Page address set
#define CASET     0x15      // Column address set
#define DATCTL    0xBC      // Data scan direction, etc.
#define RGBSET8   0xCE      // 256-color position set
#define RAMWR     0x5C      // Writing to memory
#define RAMRD     0x5D      // Reading from memory
#define PTLIN     0xA8      // Partial display in
#define PTLOUT    0xA9      // Partial display out
#define RMWIN     0xE0      // Read and modify write
#define RMWOUT    0xEE      // End
#define ASCSET    0xAA      // Area scroll set
#define SCSTART   0xAB      // Scroll start set
#define OSCON     0xD1      // Internal oscillation on
#define OSCOFF    0xD2      // Internal oscillation off
#define PWRCTR    0x20      // Power control
#define VOLCTR    0x81      // Electronic volume control
#define VOLUP     0xD6      // Increment electronic control by 1
#define VOLDOWN   0xD7      // Decrement electronic control by 1
#define TMPGRD    0x82      // Temperature gradient set
#define EPCTIN    0xCD      // Control EEPROM
#define EPCOUT    0xCC      // Cancel EEPROM control
#define EPMWR     0xFC      // Write into EEPROM
#define EPMRD     0xFD      // Read from EEPROM
#define EPSRRD1   0x7C      // Read register 1
#define EPSRRD2   0x7D      // Read register 2
#define NOP       0x25      // NOP instruction
// Booleans
#define NOFILL 			0
#define FILL 			1
// 12-bit color definitions
#define WHITE 			0xFFF
#define BLACK 			0x000
#define RED 			0xF00
#define GREEN 			0x0F0
#define BLUE 			0x00F
#define CYAN 			0x0FF
#define MAGENTA 		0xF0F
#define YELLOW 			0xFF0
#define BROWN 			0xB22
#define ORANGE 			0xFA0
#define PINK 			0xF6A
// Font sizes
#define SMALL 			0
#define MEDIUM 			1
#define LARGE 			2

/*************************************************************************
 *             Function declarations
 *************************************************************************/
void LcdInit(void);
void LcdChipSelect(char select);
void LcdSendCommand(unsigned char command);
void LcdSendData(unsigned char data);
void LcdBacklight(unsigned char state);
void LcdReset(unsigned char state);
void LcdClearScreen(uint16_t color);
void LcdSetXY(int x, int y);
void LcdSetPixel(int  x, int  y, int  color);
void LcdSetLine(int x1, int y1, int x2, int y2, int color);
void LcdSetRect(int x0, int y0, int x1, int y1, unsigned char fill, int color);
void LcdSetCircle(int x0, int y0, int radius, int color);
void LcdPutChar(char c, int  x, int  y, int size, int fcolor, int bcolor);
void LcdPutStr(char *pString, int  x, int  y, int Size, int fColor, int bColor);
//void Delay (unsigned long a);

#endif /* LCD_H_ */
