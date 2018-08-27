#ifndef OBJECTS_H
#define OBJECTS_H

#include <stdint.h>
#include "Controller.h"
#include "Filter.h"
#include "NewI2C.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>

extern "C" {
#include "DSP.h"
#include "CoreObjects.h"
}

extern float controlCycle;
extern float outer_P, rudderMix, throttleMix;
extern uint8_t gearSel, flapSel;
extern const float sampleRate;
extern float idleAvg, logBandWidth, ppmFreq, simInputFreq;
extern uint32_t simTimeStamp, idleMicros;
extern const int maxParams;
extern uint8_t gaugeCount, gaugeVariable[];
extern bool paramsModified;
extern uint32_t lastPPMWarn;
extern float fieldStrength;

extern Controller elevCtrl, pushCtrl, throttleCtrl;
extern UnbiasedController aileCtrl;
extern Damper_t iasFilter, iasFilterSlow, ball, accAvg, iasEntropy, alphaEntropy;
extern AlphaBuffer pressureBuffer;
extern RunningAvgFilter alphaFilter;
extern RateLimiter aileActuator,  rollAccelLimiter, flapActuator, trimRateLimiter;

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
