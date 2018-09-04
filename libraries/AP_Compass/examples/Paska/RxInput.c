#include <string.h>
#include "RxInput.h"
#include "Objects.h"
#include "Console.h"
#include "NVState.h"
#include "DSP.h"
#include "Math.h"
#include "Time.h"
#include "StaP.h"

//
// RC input
//

struct RxInputRecord aileInput, elevInput, throttleInput,
  btnInput, tuningKnobInput, flightModeInput, rudderInput, flapInput;

#if RX_CHANNELS < 8
struct RxInputRecord *ppmInputs[] = 
  { &aileInput, &elevInput, &throttleInput, &btnInput, &tuningKnobInput, &flightModeInput, NULL };
#else
struct RxInputRecord *ppmInputs[] = 
  { &aileInput, &elevInput, &throttleInput, &rudderInput, &btnInput, &tuningKnobInput, &flightModeInput, &flapInput, NULL };
#endif

//
// Mode selector inputs
//

struct SwitchRecord flightModeSelector = { &flightModeInput };
struct SwitchRecord flapSelector = { &flapInput };

int8_t flightModeSelectorValue, flapSelectorValue;

struct RxInputRecord *rxInputIndex0[8], *rxInputIndex1[8], *rxInputIndex2[8];
struct RxInputRecord **rxInputIndexList[] = { rxInputIndex0, rxInputIndex1, rxInputIndex2 };
uint8_t log2Table[1<<8];
bool pciWarn;

bool inputValid(struct RxInputRecord *record)
{
  return record->pulseCount > 0;
}

float inputValue(struct RxInputRecord *record)
{
  STAP_FORBID;
  
  int32_t value = record->pulseWidthLast;
  
  STAP_PERMIT;
  
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

float applyNullZone(float value, float nz, bool *pilotInput)
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

float applyNullZoneBlind(float value, float nz)
{
  return applyNullZone(value, nz, NULL);
}

#define EXPO 0.2
#define HALF_RATE 0.7

float applyExpo(float value)
{
  const float rate = vpMode.halfRate ? HALF_RATE : 1;
  
  return rate*sign(value)*powf(fabsf(value),
			  EXPO + 0.7*sqrt(effIAS()/vpDerived.minimumIAS));
}

float applyExpoTrim(float value, float trim)
{
  const float boost = sign(value) == sign(trim) ? 0 : fabs(value * trim);

  return clamp(applyExpo(value) * (1 + boost) + trim, -1, 1);
}
