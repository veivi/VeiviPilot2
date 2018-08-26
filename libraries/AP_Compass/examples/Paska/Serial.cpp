#include <AP_HAL/AP_HAL.h>

extern "C" {
#include "Serial.h"
}
  
extern const AP_HAL::HAL& hal;

extern "C" void serialOut(uint8_t c)
{
  hal.uartA->write(c);
}

extern "C" void serialFlush()
{
  // hal.uartA->flush();
}

