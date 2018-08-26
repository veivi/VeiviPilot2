#include "PPM.h"
#include "Interrupt.h"
#include "Math.h"
#include "NVState.h"
#include <avr/interrupt.h>

extern "C" {
#include "Time.h"
}
struct PinDescriptor ppmInputPin = { PortL, 1 }; 
  
#define AVR_RC_INPUT_MAX_CHANNELS 10
#define AVR_RC_INPUT_MIN_CHANNELS 6

/*
  mininum pulse width in microseconds to signal end of a PPM-SUM
  frame. This value is chosen to be smaller than the default 3000 sync
  pulse width for OpenLRSng. Note that this is the total pulse with
  (typically 300us low followed by a long high pulse)
 */

#define AVR_RC_INPUT_MIN_SYNC_PULSE_WIDTH 2700

static uint16_t _pulse_capt[AVR_RC_INPUT_MAX_CHANNELS];

uint8_t ppmNumChannels;
bool ppmWarnShort, ppmWarnSlow;

static struct RxInputRecord **inputRecords;
static int numInputs;
static uint32_t ppmFrames;
static bool calibrating;

void calibStart()
{
  calibrating = true;
  
  for(int i = 0; i < numInputs; i++)
    if(inputRecords[i]) {
      inputRecords[i]->pulseCenter
	= inputRecords[i]->pulseMin
	= inputRecords[i]->pulseMax = inputRecords[i]->pulseWidthLast;
    }
}
  
void calibStop(int32_t *min, int32_t *center, int32_t *max)
{
  if(calibrating) {
    for(uint8_t i = 0; i < numInputs; i++) {
      min[i] = inputRecords[i]->pulseMin;
      center[i] = inputRecords[i]->pulseCenter;
      max[i] = inputRecords[i]->pulseMax;
    }

    calibrating = false;
  }
}
  
static void handlePPMInput(const uint16_t *pulse, int numCh)
{
  static uint32_t prev;
  uint32_t current = currentMicros(), cycle = current - prev;

  if(prev > 0 && cycle > 30000)
    ppmWarnSlow = true;

  prev = current;
  
  ppmNumChannels = numCh;
  ppmFrames++;

  for(int i = 0; i < numCh; i++) {
    if(inputRecords[i]) {
       inputRecords[i]->alive = true;
       inputRecords[i]->pulseCount = 1;
       inputRecords[i]->pulseWidthAcc = inputRecords[i]->pulseWidthLast
	 = pulse[i]/2;              

       if(calibrating) {
	 inputRecords[i]->pulseMin = MIN(inputRecords[i]->pulseWidthLast, inputRecords[i]->pulseMin);
	 inputRecords[i]->pulseMax = MAX(inputRecords[i]->pulseWidthLast, inputRecords[i]->pulseMax);
       }
    }
  }
}

extern "C" ISR(TIMER5_CAPT_vect)
{
  static uint16_t icr5_prev;
  static uint8_t  channel_ctr;

  const uint16_t icr5_current = ICR5;
  uint16_t pulse_width;
    
  if (icr5_current < icr5_prev) {
    pulse_width =  OCR5A + 1 + icr5_current - icr5_prev;
  } else {
    pulse_width = icr5_current - icr5_prev;
  }

  if (pulse_width > AVR_RC_INPUT_MIN_SYNC_PULSE_WIDTH*2) {
    // sync pulse
	
    if( channel_ctr < AVR_RC_INPUT_MIN_CHANNELS )
      ppmWarnShort = true;
    else
      handlePPMInput(_pulse_capt, channel_ctr);

    channel_ctr = 0;
  } else if (channel_ctr < numInputs)
    _pulse_capt[channel_ctr++] = pulse_width;

  icr5_prev = icr5_current;
}

void ppmInputInitPrim(struct RxInputRecord *inputs[], const int32_t *min, const int32_t *center, const int32_t *max)
{
  inputRecords = inputs;
  
  numInputs = 0;

  while(inputs[numInputs] && numInputs < AVR_RC_INPUT_MAX_CHANNELS)
    numInputs++;
  
  for(uint8_t i = 0; i < numInputs; i++) {
    inputs[i]->pulseMin = min[i];
    inputs[i]->pulseCenter = center[i];
    inputs[i]->pulseMax = max[i];
  }
  
  FORBID;
    
    TCCR5A = _BV(WGM50) | _BV(WGM51);
    TCCR5B |= _BV(WGM53) | _BV(WGM52) | _BV(CS51) | _BV(ICES5);
    OCR5A  = 40000 - 1; // -1 to correct for wrap

    /* OCR5B and OCR5C will be used by RCOutput_APM2. Init to 0xFFFF to prevent premature PWM output */
    OCR5B  = 0xFFFF;
    OCR5C  = 0xFFFF;

    /* Enable input capture interrupt */
    TIMSK5 |= _BV(ICIE5);
    /*
  TCCR5B |= (1<<ICES5) | (1<<CS51);
  TIMSK5 |= 1<<ICIE5;
  OCR5A  = 40000 - 1; // -1 to correct for wrap
    */
    
  PERMIT;
}

void ppmInputInit(const int32_t *min, const int32_t *center, const int32_t *max)
{
  configureInput(&ppmInputPin, true);
  ppmInputInitPrim(ppmInputs, min, center, max);
}

float ppmFrameRate()
{
  static uint32_t prevMeasurement;
  
  FORBID;
  float result = 1.0e6 * ppmFrames / (currentMicros() - prevMeasurement);
  ppmFrames = 0;
  PERMIT;

  prevMeasurement = currentMicros();

  return result;
}
