#include "button.h"

static uint8_t prevButtonState = 0;

void ButtonInit(void)
{
    // Set direction for buttons to inputs (clear the bit)
    FIO0DIR &= ~(BUT_BUT1 | BUT_BUT2);
    FIO1DIR &= ~(BUT_CENTER | BUT_UP | BUT_DOWN | BUT_RIGHT | BUT_LEFT);
}

uint8_t ButtonGetChangedHigh(void)
{
    uint8_t buttonState;
    uint8_t temp;
    
    // Get the state of the buttons first
    buttonState = (~FIO0PIN>>BUT_BUT1_SHFT) & 0x01;
    buttonState |= (~FIO0PIN>>BUT_BUT2_SHFT) & 0x02;
    buttonState |= (~FIO1PIN>>BUT_CENTER_SHFT) & 0x04;
    buttonState |= (~FIO1PIN>>BUT_UP_SHFT) & 0x08;
    buttonState |= (~FIO1PIN>>BUT_DOWN_SHFT) & 0x10;
    buttonState |= (~FIO1PIN>>BUT_RIGHT_SHFT) & 0x20;
    buttonState |= (~FIO1PIN>>BUT_LEFT_SHFT) & 0x40;

    // XOR previous state with current state to get changes, then and with 
    // current state to get buttons that went higher
    temp = (prevButtonState ^ buttonState) & buttonState;
    
    // Store the current button state
    prevButtonState = buttonState;
    
    return temp;
}
