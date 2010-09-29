#include "button.h"

static unsigned char ButtonState = 0;

void ButtonInit(void)
{
    // Set direction for buttons to inputs (clear the bit)
    FIO0DIR &= ~(BUT_BUT1 | BUT_BUT2);
    FIO1DIR &= ~(BUT_CENTER | BUT_UP | BUT_DOWN | BUT_RIGHT | BUT_LEFT);
}

void ButtonUpdate(void)
{
    ButtonState = ((~FIO0PIN & BUT_BUT1) << BUT_BUT1_BIT);
    ButtonState |= ((~FIO0PIN & BUT_BUT2) << BUT_BUT2_BIT);
    ButtonState |= ((~FIO1PIN & BUT_CENTER) << BUT_CENTER_BIT);
    ButtonState |= ((~FIO1PIN & BUT_UP) << BUT_UP_BIT);
    ButtonState |= ((~FIO1PIN & BUT_DOWN) << BUT_DOWN_BIT);
    ButtonState |= ((~FIO1PIN & BUT_RIGHT) << BUT_RIGHT_BIT);
    ButtonState |= ((~FIO1PIN & BUT_LEFT) << BUT_LEFT_BIT);
}

unsigned char ButtonGetMask(void)
{
    return ButtonState;
}
