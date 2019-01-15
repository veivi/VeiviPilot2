#include <string.h>
#include "RxInput.h"
#include "Objects.h"
#include "Console.h"
#include "NVState.h"
#include "DSP.h"
#include "Math.h"
#include "StaP.h"

//
// RC input
//

struct RxInputRecord rxInput[RX_CHANNELS];

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
  static uint32_t prev;
  uint32_t current = stap_timeMicros(), cycle = current - prev;
  int i = 0;

  if(prev > 0 && cycle > 30000)
    ppmWarnSlow = true;

  if(numCh < RX_CHANNELS)
    ppmWarnShort = true;

  prev = current;
  
  ppmFrames++;

  for(i = 0; i < MIN(RX_CHANNELS, numCh); i++) {
    rxInput[i].alive = true;
    rxInput[i].pulseWidth = pulse[i];              

    if(calibrating) {
      nvState.rxMin[i] = MIN(rxInput[i].pulseWidth, nvState.rxMin[i]);
      nvState.rxMax[i] = MAX(rxInput[i].pulseWidth, nvState.rxMax[i]);
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

#define EXPO 0.35
#define HALF_RATE 0.5

float applyExpo(float value)
{
  const float p = 1.0 - powf(vpDerived.minimumIAS / effIAS(), 1.5),
    rate = vpMode.halfRate ? mixValue(p, 1, HALF_RATE) : 1;
  
  return rate*sign(value)*powf(fabsf(value), 1 + EXPO*p);
}

float applyExpoTrim(float value, float trim)
{
  const float valueExp = applyExpo(value),
    boost = sign(value) == sign(trim) ? 0 : fabs(valueExp * trim);

  return clamp(valueExp * (1 + boost) + trim, -1, 1);
}

float inputSourceRate()
{
  static uint32_t prevMeasurement;
  
  STAP_FORBID;
  uint16_t count = ppmFrames;
  ppmFrames = 0;
  STAP_PERMIT;
  
  float result = 1.0e6 * count / (stap_timeMicros() - prevMeasurement);
  
  prevMeasurement = stap_timeMicros();

  if(result < 30)
    ppmWarnSlow = true;
    
  return result;
}
