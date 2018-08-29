#ifndef OBJECTS_H
#define OBJECTS_H

#include <stdint.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>

extern "C" {
#include "DSP.h"
#include "CoreObjects.h"
#include "Controller.h"
}

extern PIDCtrl_t elevCtrl, pushCtrl, throttleCtrl, aileCtrl;
extern Sampler_t pressureBuffer;
extern SWAvg_t alphaFilter, liftFilter;
extern SlopeLimiter_t aileActuator,  rollAccelLimiter, trimRateLimiter;
extern const AP_HAL::HAL& hal;
extern AP_HAL::BetterStream* cliSerial;

extern AP_Baro barometer;
extern AP_InertialSensor ins;
extern AP_GPS gps;
extern AP_AHRS_DCM ahrs;
#ifdef USE_COMPASS
extern Compass compass;
#endif

struct GPSFix {
  float altitude;
  float track;
  float lat;
  float lon;
  float speed;
};

// struct GPSFix gpsFix;

extern "C" {
#include "Datagram.h"
  
extern struct SimLinkSensor sensorData;
extern uint16_t simFrames;
extern int linkDownCount, heartBeatCount;
}

#endif
