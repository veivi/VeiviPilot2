#include <AP_HAL/AP_HAL.h>
#include "Time.h"

extern const AP_HAL::HAL& hal;

uint32_t currentMicros()
{
  return hal.scheduler->micros();
}
    
uint32_t currentMillis()
{
  return hal.scheduler->millis();
}
    
void delayMicros(uint32_t x)
{
  uint32_t current = currentMicros();
  while(currentMicros() < current+x);
}
