#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>

extern "C" {
#include "MainLoop.h"
}

extern const AP_HAL::HAL& hal;

void setup()
{
  hal.init(0, NULL);
  
  mainLoopSetup();
}

void loop() 
{
  mainLoop();
}

AP_HAL_MAIN();
