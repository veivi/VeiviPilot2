#include "RxInput.h"
#include "Interrupt.h"
#include "Console.h"
#include "Objects.h"
#include "NVState.h"
#include "Math.h"
#include <avr/io.h>

//
// RC input
//

struct RxInputRecord aileInput, elevInput, throttleInput,
  buttonInput, tuningKnobInput, flightModeInput, rudderInput, stabModeInput;

#if RX_CHANNELS < 8
struct RxInputRecord *ppmInputs[] = 
  { &aileInput, &elevInput, &throttleInput, &buttonInput, &tuningKnobInput, &flightModeInput, NULL };
#else
struct RxInputRecord *ppmInputs[] = 
  { &aileInput, &elevInput, &throttleInput, &rudderInput, &buttonInput, &tuningKnobInput, &flightModeInput, &stabModeInput, NULL };
#endif

//
// Mode selector inputs
//

struct SwitchRecord flightModeSelector = { &flightModeInput };
struct SwitchRecord stabModeSelector = { &stabModeInput };

int8_t flightModeSelectorValue, stabModeSelectorValue;

//
// Buttons
//

Button rightDownButton(-1.0), rightUpButton(0.33),
  leftDownButton(-0.3), leftUpButton(1);

#define LEVELBUTTON rightUpButton
#define FLAPBUTTON rightDownButton
#define TRIMBUTTON leftUpButton
#define GEARBUTTON leftDownButton

struct RxInputRecord *rxInputIndex0[8], *rxInputIndex1[8], *rxInputIndex2[8];
struct RxInputRecord **rxInputIndexList[] = { rxInputIndex0, rxInputIndex1, rxInputIndex2 };
uint8_t log2Table[1<<8];
bool_t pciWarn;

bool_t inputValid(struct RxInputRecord *record)
{
  return record->pulseCount > 0;
}

float inputValue(struct RxInputRecord *record)
{
  FORBID;
  
  int32_t value = record->pulseWidthLast;
  
  PERMIT;
  
  if(value < record->pulseCenter) {
    if(record->pulseCenter-record->pulseMin < 100)
      return 0;
    else
      return (float) (value - record->pulseCenter)/(record->pulseCenter-record->pulseMin);
  } else {
    if(record->pulseMax-record->pulseCenter < 100)
      return 0;
    else
      return (float) (value - record->pulseCenter)/(record->pulseMax-record->pulseCenter);
    }
}

void rxInputInit(struct RxInputRecord *record)
{
  static bool_t initialized = false;

  if(!initialized) {
    for(int i = 1; i < (1<<8); i++) {
      int j = 7;
      while(((1<<j) & i) == 0 && j > 0)
	j--;
      log2Table[i] = j;
    }

    PCMSK0 = PCMSK1 = PCMSK2 = 0;

    initialized = true;
  }
  
  const struct PortDescriptor *port = &portTable[record->pin.port];

  if(port->mask) {
    FORBID;
    rxInputIndexList[port->pci][record->pin.index] = record;
    *port->mask |= 1<<record->pin.index;
    PCICR |= pcIntMask[port->pci];
    PERMIT;
  } else
      consoleNoteLn("PASKA PCI-PORTTI");
}

extern "C" void rxInterrupt_callback(uint8_t num)
{
  const struct PortDescriptor *port = &portTable[pcIntPort[num]];

  if(!port || !port->mask) {
    consoleNoteLn("PASKA PCI.");
    return;
  }

  static uint8_t prevState[3];
  uint8_t state = *port->pin, event = (state ^ prevState[num]) & *port->mask;

  prevState[num] = state;
  
  uint32_t current = hal.scheduler->micros();
  
  while(event) {
    uint8_t i = log2Table[event];
    uint8_t mask = 1U<<i;
  
    if(!rxInputIndexList[num][i]) {
      pciWarn = true;
    } else if(rxInputIndexList[num][i]->freqOnly) {
      rxInputIndexList[num][i]->pulseCount += (state & mask) ? 1 : 0;
    } else if(state & mask) {
      rxInputIndexList[num][i]->pulseStart = current;
    } else if(rxInputIndexList[num][i]->pulseStart > 0) {
      uint32_t width = current - rxInputIndexList[num][i]->pulseStart;
      rxInputIndexList[num][i]->pulseWidthAcc += width;
      rxInputIndexList[num][i]->pulseWidthLast = width;
      rxInputIndexList[num][i]->pulseCount++;      
      rxInputIndexList[num][i]->alive = true;
    }
    
    event &= ~mask;
  }
}

int8_t readSwitch(struct SwitchRecord *record)
{
  if(inputValid(record->input)) {
    const float value = inputValue(record->input),
      diff = fabsf(value - record->prevValue);
    
    record->prevValue = value;
  
    if(diff < 0.05) {
      if(fabs(value) < 1.0/3)
	record->state = 0;
      else
	record->state = value < 0.0 ? -1 : 1;
    }
  }

  return record->state;
}

float applyNullZone(float value, float nz, bool_t *pilotInput)
{
  const float zone = 1.0 - nz;
  
  if(pilotInput)
    *pilotInput = fabsf(value) > nz;
  
  if(value < -nz)
    return (value + nz) / zone;
  else if(value > nz)
    return (value - nz) / zone;
    
  return 0.0;
}

float applyNullZone(float value, float nz)
{
  return applyNullZone(value, nz, NULL);
}

#define EXPO 0.3
#define HALF_RATE 0.75

float applyExpo(float value)
{
  const float rate = vpMode.halfRate ? HALF_RATE : 1;
  
  return rate*sign(value)*powf(fabsf(value),
			  EXPO + sqrt(effIAS()/vpDerived.minimumIAS));
}

float applyExpoTrim(float value, float trim)
{
  const float boost = sign(value) == sign(trim) ? 0 : fabs(value * trim);

  return clamp(applyExpo(value) * (1 + boost) + trim, -1, 1);
}
