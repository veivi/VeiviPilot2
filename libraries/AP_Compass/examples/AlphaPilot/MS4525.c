#include <stdlib.h>
#include "MS4525.h"
#include "NVState.h"
#include "Console.h"
#include "BaseI2C.h"

static BaseI2CTarget_t target = { "pitot" } ;

bool MS4525DO_isOnline(void)
{
  return basei2cIsOnline(&target);
}

bool MS4525DO_readGeneric(uint8_t *storage, uint8_t bytes) 
{
  return basei2cInvoke(&target, basei2cRead(MS4525DO_DEVICE, storage, bytes));
}

bool MS4525DO_read(uint16_t *result)
{
  uint8_t buf[sizeof(uint16_t)];
  uint8_t status = MS4525DO_readGeneric(buf, sizeof(buf));
  
  if(status && result)
    *result = (((uint16_t) (buf[0] & 0x3F)) << 8) + buf[1];

  return status && !(buf[0] & (1<<6));
}

static uint32_t acc = 0;
static uint16_t ms4525_ref;
static int accCount = 0;
static bool calibrating = true;

void MS4525DO_calibrate(void)
{
  calibrating = true;
  acc = 0;
  accCount = 0;
}

bool MS4525DO_pressure(int16_t *result) 
{
  const int log2CalibWindow = 9;
  uint16_t raw = 0;

  if(!MS4525DO_read(&raw))
    return false;

  basei2cEntropySample(&target, raw);
  
  if(calibrating) {
    if(accCount < 1<<log2CalibWindow) {
      acc += raw;
      accCount++;
    } else {
      ms4525_ref = acc>>(log2CalibWindow - 2);
      calibrating = false;
      consoleNote_P(CS_STRING("Airspeed calibration DONE, ref = "));
      consolePrintLnUI(ms4525_ref);
    }
  } else if(result)
    *result = (raw<<2) - ms4525_ref;
  
  return true;
}

float MS4525DO_entropy(void)
{
  return basei2cEntropy(&target);
}

