#include <string.h>
#include "AlphaPilot.h"
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

struct RxInputRecord rxInput[MAX_CH];

//
// Mode selector inputs
//

struct SwitchRecord flightModeSelector = { CH_MODE };
#ifdef CH_FLAP
struct SwitchRecord flapSelector = { CH_FLAP };
#endif

static uint16_t ppmFrames;
static bool calibrating, ppmWarnShort, ppmWarnSlow;

void inputCalibStart()
{
  int i = 0;
  
  calibrating = true;
  
  for(i = 0; i < MAX_CH; i++)
    nvState.rxCenter[i] = nvState.rxMin[i] = nvState.rxMax[i]
      = rxInput[i].pulseWidth;
}
  
void inputCalibStop(void)
{
    calibrating = false;
}
  
bool inputSourceGood(void)
{
  bool status = !ppmWarnSlow && !ppmWarnShort;
  ppmWarnSlow = ppmWarnShort = false;
  return status;
}

void inputSource(const uint16_t *pulse, int numCh)
{
  static VP_TIME_MICROS_T prev;
  VP_TIME_MICROS_T current = vpTimeMicros(), cycle = current - prev;

  ppmFrameReceived = true;
  ppmFrames++;

  if(prev > 0 && cycle > 30000)
    ppmWarnSlow = true;

  prev = current;
  
  if(numCh < RX_CHANNELS)
    ppmWarnShort = true;
  else {
    int i = 0;
    
    for(i = 0; i < RX_CHANNELS; i++) {
      rxInput[i].alive = true;
      rxInput[i].pulseWidth = pulse[i];              

      if(calibrating) {
	nvState.rxMin[i] = MIN(rxInput[i].pulseWidth, nvState.rxMin[i]);
	nvState.rxMax[i] = MAX(rxInput[i].pulseWidth, nvState.rxMax[i]);
      }
    }
  }
}

bool inputValid(uint8_t ch)
{
  return rxInput[ch].alive;
}

float inputValue(uint8_t ch)
{
  STAP_FORBID;
  
  int32_t value = rxInput[ch].pulseWidth;
  
  STAP_PERMIT;
  
  if(value < nvState.rxCenter[ch]) {
    if(nvState.rxCenter[ch]-nvState.rxMin[ch] < 100)
      return 0;
    else
      return (float) (value - nvState.rxCenter[ch])/(nvState.rxCenter[ch]-nvState.rxMin[ch]);
  } else {
    if(nvState.rxMax[ch]-nvState.rxCenter[ch] < 100)
      return 0;
    else
      return (float) (value - nvState.rxCenter[ch])/(nvState.rxMax[ch]-nvState.rxCenter[ch]);
    }
}

int8_t readSwitch(struct SwitchRecord *record)
{
  if(inputValid(record->ch)) {
    const float value = inputValue(record->ch),
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
  if(fabsf(value) > nz) {
    const float zone = 1.0 - nz;
  
    if(pilotInput)
      *pilotInput = true;
  
    if(value > 0)
      return (value - nz) / zone;
    else
      return (value + nz) / zone;
  }
  
  if(pilotInput)
    *pilotInput = false;
  
  return 0.0;
}

float applyNullZoneBlind(float value, float nz)
{
  return applyNullZone(value, nz, NULL);
}

#define EXPO 0.2f
#define HALF_RATE 0.65f

float applyExpo(float value)
{
  const float index = 1.0 - vpDerived.minimumDynP/vpFlight.effDynP,
    rate_c = vpMode.halfRate ? mixValue(index, 1, HALF_RATE) : 1,
    expo_c = 1.1 + EXPO * index;

  return rate_c*sign(value)*powf(fabsf(value), expo_c);
}

float applyExpoTrim(float value, float trim)
{
  const float valueExp = applyExpo(value),
    boost = sign(value) == sign(trim) ? 0 : fabs(valueExp * trim);

  return clamp(valueExp * (1 + boost) + trim, -1, 1);
}

float inputSourceRate()
{
  static VP_TIME_MICROS_T prevMeasurement;
  
  STAP_FORBID;
  uint16_t count = ppmFrames;
  ppmFrames = 0;
  STAP_PERMIT;
  
  float result = 1.0e6 * count / (vpTimeMicros() - prevMeasurement);
  
  prevMeasurement = vpTimeMicros();

  if(result < 30)
    ppmWarnSlow = true;
    
  return result;
}
