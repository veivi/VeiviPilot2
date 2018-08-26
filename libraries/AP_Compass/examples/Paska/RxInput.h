#ifndef RXINPUT_H
#define RXINPUT_H

#include <math.h>
#include "AlphaPilot.h"
#include "InputOutput.h"
#include "Math.h"
#include "Button.h"

extern struct RxInputRecord aileInput, elevInput, throttleInput,
  buttonInput, tuningKnobInput, flightModeInput, rudderInput, flapInput;

extern struct RxInputRecord *ppmInputs[];

//
// Mode selector inputs
//

extern struct SwitchRecord flightModeSelector, flapSelector;
extern int8_t flightModeSelectorValue, flapSelectorValue;

//
// Buttons
//

extern Button rightDownButton, rightUpButton, leftDownButton, leftUpButton;

#define LEVELBUTTON rightUpButton
#define RATEBUTTON rightDownButton
#define TRIMBUTTON leftUpButton
#define GEARBUTTON leftDownButton

struct RxInputRecord {
  struct PinDescriptor pin;
  bool freqOnly, alive;
  int32_t pulseMin, pulseMax, pulseCenter;
  int32_t pulseStart;
  int32_t pulseCount;
  int32_t pulseWidthAcc;
  int32_t pulseWidthLast;
};
  
struct SwitchRecord {
  struct RxInputRecord *input;
  int8_t state;
  float prevValue;
};

void rxInputInit(struct RxInputRecord *record);
bool inputValid(struct RxInputRecord *record);
float inputValue(struct RxInputRecord *record);
int8_t readSwitch(struct SwitchRecord *record);

float applyNullZone(float value, float nz, bool *pilotInput);
float applyNullZone(float value, float nz);
float applyExpo(float value);
float applyExpoTrim(float value, float trim);

extern bool pciWarn;

// pin change int callback

extern "C" void rxInterrupt_callback(uint8_t num);

#endif

