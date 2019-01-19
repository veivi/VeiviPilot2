#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>

extern "C" {
#include "StaP.h"
#include "MainLoop.h"
#include "Objects.h"
}

extern const AP_HAL::HAL& hal;
extern AP_InertialSensor ins;
//AP_GPS gps;
extern AP_AHRS_DCM ahrs;// {ins,  barometer, gps};

void setup()
{
  hal.init(0, NULL);

  /*ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_50HZ);
  ahrs.init();
  */
  configureOutput(&led);
  
  mainLoopSetup();
}

void loop() 
{
  mainLoop();
}

AP_HAL_MAIN();
