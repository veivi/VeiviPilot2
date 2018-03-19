#ifndef OBJECTS_H
#define OBJECTS_H

#include <stdint.h>
#include "Status.h"
#include "Controller.h"
#include "Filter.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>

struct ModeRecord {
  bool test;
  bool alphaFailSafe;
  bool sensorFailSafe;
  bool radioFailSafe;
  bool bankLimiter;
  bool takeOff;
  bool wingLeveler;
  bool slowFlight;
  bool progressiveFlight;
  bool gusty;
  bool autoThrottle;
  bool loggingSuppressed;
};

struct FeatureRecord {
  bool keepLevel;
  bool stabilizeBank;
  bool stabilizePitch;
  bool alphaHold;
  bool pusher;
  bool aileFeedforward;
  bool ailePID;
};

struct FlightState {
  float IAS, dynP, alpha;
  float accX, accY, accZ, acc, alt, bank, pitch, rollR, pitchR, yawR;
  uint16_t heading;
  float accDir, relWind, slope;
};

struct InputState {
  float aile, elev, elevExpo, throttle, rudder, tuningKnob;
  bool ailePilotInput, elevPilotInput, rudderPilotInput;
};

struct ControlState {
  float testGain;
  float elevTrim, targetAlpha, targetPressure, targetPitchR, minThrottle;
  float elevPredict, ailePredict, aileNeutral, pusher;
};

struct OutputState {
  float elev, aile, brake, rudder, steer, thrustVert, thrustHoriz;
};

extern struct ModeRecord vpMode;
extern struct FeatureRecord vpFeature;
extern struct StatusRecord vpStatus;
extern struct FlightState vpFlight;
extern struct InputState vpInput;
extern struct ControlState vpControl;
extern struct OutputState vpOutput;

extern float controlCycle;
extern float outer_P, rudderMix, throttleMix;
extern uint8_t flapSel, gearSel;
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
extern Damper ball, iasFilterSlow, iasFilter, accAvg, iasEntropyAcc, alphaEntropyAcc;
extern AlphaBuffer pressureBuffer;
extern RunningAvgFilter alphaFilter;
extern RateLimiter aileRateLimiter, flapActuator, trimRateLimiter;

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

extern NewI2C I2c;
extern I2CDevice alphaDevice, pitotDevice, eepromDevice, displayDevice;

extern "C" {
#include "CRC16.h"
#include "System.h"
#include "Datagram.h"
extern struct SimLinkSensor sensorData;
extern uint16_t simFrames;
extern int linkDownCount, heartBeatCount;
}

#endif
