#include "Objects.h"
#include "Time.h"

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
