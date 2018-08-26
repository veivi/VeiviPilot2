#include "Objects.h"

extern "C" {
#include "Time.h"
}
  
uint32_t currentTime;
  
extern "C" uint32_t currentMicros()
{
  currentTime = hal.scheduler->micros();
  return currentTime;
}
    
extern "C" uint32_t currentMillis()
{
  return hal.scheduler->millis();
}
    
extern "C" void delayMicros(uint32_t x)
{
  uint32_t current = currentMicros();
  while(currentMicros() < current+x);
}
