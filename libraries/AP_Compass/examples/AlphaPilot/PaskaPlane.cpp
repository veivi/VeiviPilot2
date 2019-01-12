#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>

extern "C" {
#include "StaP.h"
#include "MainLoop.h"
#include "Objects.h"
}

extern const AP_HAL::HAL& hal;

void setup()
{
  hal.init(0, NULL);
  configureOutput(&led);

  mainLoopSetup();
}

void loop() 
{
  mainLoop();
}

AP_HAL_MAIN();
