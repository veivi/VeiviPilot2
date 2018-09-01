#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>

extern "C" {
#include "MainLoop.h"
}

void setup()
{
  mainLoopSetup();
}

void loop() 
{
  mainLoop();
}

extern const AP_HAL::HAL& hal;

AP_HAL_MAIN();
