#include <stdlib.h>
#include "MS4525.h"
#include "NVState.h"
#include "Console.h"
#include "BaseI2C.h"

#define LOG2_CALIB_WINDOW 9
#define LOG2_CALIB_DECAY 4

static BaseI2CTarget_t target = { "pitot" } ;

bool MS4525DO_isOnline(void)
{
  return basei2cIsOnline(&target);
}

static bool MS4525DO_readGeneric(uint8_t *storage, uint8_t bytes) 
{
  return basei2cInvoke(&target, basei2cRead(MS4525DO_DEVICE, storage, bytes));
}

bool MS4525DO_read(uint16_t *result)
{
  uint8_t buf[sizeof(uint16_t)];
  uint8_t status = MS4525DO_readGeneric(buf, sizeof(buf));
  
  if(status && result)
    *result = ((((uint16_t) (buf[0] & 0x3F)) << 8) + buf[1])<<2;

  return status && !(buf[0] & (1<<6));
}

static uint32_t calibAcc = 0;
static int calibCount = 0;
static uint32_t ms4525_refAcc = 0;
static uint16_t ms4525_ref = 0;

bool MS4525DO_pressure(int16_t *result) 
{
  uint16_t raw = 0;

  if(!MS4525DO_read(&raw))
    return false;

  basei2cEntropySample(&target, raw);
  
  if(vpMode.takeOff || vpStatus.positiveIAS || vpStatus.aloft) {
    if(calibCount > 0)
      consoleNoteLn_P(CS_STRING("Airspeed calibration STOPPED"));
     
    calibAcc = 0;
    calibCount = 0;
  } else if(calibCount < 1<<LOG2_CALIB_WINDOW) {
    calibAcc += raw;
    calibCount++;
  } else {
    // Use the previous value as the new reference to be safe (the next
    // value might already be contaminated by a takeoff that didn't yet
    // trigger status.positiveIAS)
    
    ms4525_ref = ms4525_refAcc>>LOG2_CALIB_DECAY;

    // Update the reference accumulator
    
    if(ms4525_refAcc == 0)
      ms4525_refAcc = calibAcc>>(LOG2_CALIB_WINDOW - LOG2_CALIB_DECAY);
    else
      ms4525_refAcc =
	(((1<<LOG2_CALIB_DECAY)-1)*ms4525_refAcc>>LOG2_CALIB_DECAY)
	+ (calibAcc>>LOG2_CALIB_WINDOW);
      
    calibAcc = 0;
    calibCount = 0;
    
    consoleNote_P(CS_STRING("Airspeed accumulator = "));
    consolePrintLnUL(ms4525_refAcc);
  }

  if(ms4525_ref != 0)
    *result = raw - ms4525_ref;
  
  return true;
}

float MS4525DO_entropy(void)
{
  return basei2cEntropy(&target);
}

