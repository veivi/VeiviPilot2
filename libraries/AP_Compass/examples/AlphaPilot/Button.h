#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h>
#include <stdbool.h>
#include "Time.h"

#define INERTIA 3

typedef struct Button {
  float activeValue;
  uint8_t inertiaCount;
  bool inputState;
  bool statePrev, stateLazy, pulseArmed;
  VP_TIME_MICROS_T transition;
  uint8_t count;
  bool pulseDouble, pulseSingle, buttonPress;
} Button_t;

#define BUTTON(avalue) ((Button_t) { avalue })

bool buttonInit(Button_t*, float);
void buttonFinalize(Button_t*);
void buttonReset(Button_t*);
void buttonInput(Button_t*, float);
bool buttonSinglePulse(Button_t*);
bool buttonDoublePulse(Button_t*);
bool buttonDepressed(Button_t*);
bool buttonState(Button_t*);
bool buttonLazy(Button_t*);

//
// Buttons
//

extern Button_t levelButton, gearButton, trimButton, rateButton;

#define LEVELBUTTON levelButton
#define TRIMBUTTON trimButton
#define GEARBUTTON gearButton

#ifdef CH_BUTTON
#define RATEBUTTON rateButton
#endif

#endif
