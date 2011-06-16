/**
 @file button_task.c
  
 @brief The task that reads the button inputs and detectes pressed transitions.
 
 @author Titus Appel

 @version 1.0

 @date 2011/06/03

 Contact: titus.appel@gmail.com
*/

#include "tasks/button_task.h"

extern xComPortHandle debugPortHandle;

/**
 @brief The button task checks the state of the button inputs every 1/10 second.
 
 After it checks the state, it adjusts the Display if the Left of Right buttons
 are pressed. It also prints to the debug serial port if anything changed.
 
 @param[in] pvParameters The parameters from the task create call
*/
static void vButtonTask( void *pvParameters )
{
  uint8_t buttonChanged;

	ButtonInit();

  for( ;; )
  {
    buttonChanged = ButtonGetChangedHigh();
    if (buttonChanged == BUT_RIGHT_BIT)
    {
      DisplayIncreaseState();
      vSerialPutString( debugPortHandle, "RIGHT\r\n", 7 );
    }
    else if (buttonChanged == BUT_LEFT_BIT)
    {
      DisplayDecreaseState();
      vSerialPutString( debugPortHandle, "LEFT\r\n", 6 );
    }
    
    if (buttonChanged == BUT_CENTER_BIT)
    {
      ControllerToggleMode();
      vSerialPutString( debugPortHandle, "CENTER\r\n", 8 );
    }
    
    vTaskDelay( 100 / portTICK_RATE_MS );
  }
}

/**
 @brief Start the button task
 @todo Should make the Priority and Stack Size reconfigurable. 
*/
void vButtonTaskStart(void)
{
  xTaskCreate( vButtonTask, "ButtonTask", configMINIMAL_STACK_SIZE, NULL, 2, NULL );
}

/**
 @brief Initialize the button inputs
*/
void ButtonInit(void)
{
    // Set direction for buttons to inputs (clear the bit)
    FIO0DIR &= ~(BUT_BUT1 | BUT_BUT2);
    FIO1DIR &= ~(BUT_CENTER | BUT_UP | BUT_DOWN | BUT_RIGHT | BUT_LEFT);
}

/**
 @brief Get the buttons that have changed from unpressed to pressed.
 
 This method gets the inputs that have been pressed since the previous call to 
 this method. Since this method depends on the previous call, it should be
 called regularly to get changes in the button presses.
 
 @return The buttonState where 1's are buttons that have been pressed from the 
         previous timestep.
*/
uint8_t ButtonGetChangedHigh(void)
{
    static uint8_t prevButtonState = 0;
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

