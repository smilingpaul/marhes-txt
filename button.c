#include "button.h"

void ButtonInit(void)
{
    // Set direction for buttons to inputs (clear the bit)
    FIO0DIR &= ~(BUT_BUT1 | BUT_BUT2);
    FIO1DIR &= ~(BUT_CENTER | BUT_UP | BUT_DOWN | BUT_RIGHT | BUT_LEFT);
}

unsigned char ButtonGetMask(void)
{
	unsigned char buttonState;

    buttonState = (~FIO0PIN>>BUT_BUT1_SHFT) & 0x01;
    buttonState |= (~FIO0PIN>>BUT_BUT2_SHFT) & 0x02;
    buttonState |= (~FIO1PIN>>BUT_CENTER_SHFT) & 0x04;
    buttonState |= (~FIO1PIN>>BUT_UP_SHFT) & 0x08;
    buttonState |= (~FIO1PIN>>BUT_DOWN_SHFT) & 0x10;
    buttonState |= (~FIO1PIN>>BUT_RIGHT_SHFT) & 0x20;
    buttonState |= (~FIO1PIN>>BUT_LEFT_SHFT) & 0x40;

	return buttonState;
}
