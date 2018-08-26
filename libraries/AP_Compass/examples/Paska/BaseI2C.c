#include <stdlib.h>
#include <stdbool.h>
#include "BaseI2C.h"
#include "Console.h"
#include "Time.h"

#define BACKOFF (0.5e3)

bool basei2cIsOnline(BaseI2CTarget_t *target)
{
  return !target->failed || currentMillis() > target->failedAt+target->backoff;
}

bool basei2cWarning(BaseI2CTarget_t *target)
{
  return target->warn || target->failed;
}

bool basei2cInvoke(BaseI2CTarget_t *target, uint8_t status)
{
  if(status) {
    target->warn = true;
    
    consoleNote_P(PSTR("Bad "));
    consolePrintLn(target->name);

    if(target->failed)
      target->backoff += target->backoff/2;
    else if(++target->failCount > 3) {
      consoleNote("");
      consolePrint(target->name);
      consolePrintLn_P(PSTR(" failed"));
      target->failed = true;
    }
    
    target->failedAt = currentMillis();
  } else {    
    if(target->failCount > 0) {
      consoleNote("");
      consolePrint(target->name);
      consolePrintLn_P(PSTR(" recovered"));
      target->failCount = 0;
      target->failed = target->warn = false;
      target->backoff = BACKOFF;
    }
  }
  
  return status == 0;
}

