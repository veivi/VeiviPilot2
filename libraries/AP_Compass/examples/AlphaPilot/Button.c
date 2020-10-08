#include <math.h>
#include "Time.h"
#include "Button.h"
#include "Console.h"

//
// Buttons
//

#ifdef CH_BUTTON
Button_t rateButton = BUTTON(-1.0), levelButton = BUTTON(0.34), gearButton = BUTTON(-0.3), trimButton = BUTTON(1);
#else
Button_t levelButton = BUTTON(1.0), trimButton = BUTTON(1.0), gearButton = BUTTON(1.0);
#endif

bool buttonInit(Button_t *button, float aValue)
{
  button->activeValue = aValue;
  buttonReset(button);
  return true;
}

void buttonFinalize(Button_t *button)
{
}

void buttonReset(Button_t *button)
{
  button->count = 0;
  button->pulseSingle = button->pulseDouble = button->buttonPress = button->pulseArmed = false;
  button->transition = 0;
}
    
void buttonInput(Button_t *button, float inputValue)
{
  if(fabsf(inputValue - button->activeValue)/fabsf(button->activeValue) > 0.2) {
    button->inertiaCount = 0;
    button->inputState = false;
  } else if(button->inertiaCount < INERTIA)
    button->inertiaCount++;
  else
    button->inputState = true;

  if(button->inputState != button->statePrev) {
    button->transition = vpTimeMicros();

    if(button->inputState)
      button->pulseArmed = true;
      
    else {
      button->stateLazy = false;
      
      if(button->pulseArmed) {
	if(button->count > 0) {
	  button->pulseDouble = true;
	  button->pulseSingle = false;
	  button->count = 0;
	} else
	  button->count = 1;
      
	button->pulseArmed = false;
      }
    }
    
    button->statePrev = button->inputState;
  } else if(vpTimeMicros() - button->transition > 0.5e6) {

    if(button->stateLazy != button->inputState)
      button->buttonPress = button->inputState;
    
    button->stateLazy = button->inputState;
    
    if(!button->inputState && button->count > 0)
      button->pulseSingle = true;

    button->count = 0;
    button->pulseArmed = false;
  }
}

bool buttonSinglePulse(Button_t *button)
{
  bool value = button->pulseSingle;
  button->pulseSingle = false;
  return value;
}

bool buttonDoublePulse(Button_t *button)
{
  bool value = button->pulseDouble;
  button->pulseDouble = false;
  return value;
}

bool buttonDepressed(Button_t *button)
{
  bool value = button->buttonPress;
  button->buttonPress = false;
  return value;
}

bool buttonState(Button_t *button)
{
  return button->inputState;
}

bool buttonLazy(Button_t *button)
{
  return button->stateLazy;
}

