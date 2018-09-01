#ifndef COREOBJECTS_H
#define COREOBJECTS_H

#include <stdint.h>
#include <stdbool.h>
#include "DSP.h"
#include "Controller.h"
#include "Datagram.h"

struct StatusRecord {
  bool armed;
  bool consoleLink;
  bool simulatorLink;
  bool silent;
  bool positiveIAS;
  bool fullStop;
  bool pitotFailed;
  bool pitotBlocked;
  bool stall;
  bool weightOnWheels;
  bool aloft;
  int fault;
  bool alphaFailed;
  bool alphaUnreliable;
  bool upright;
  bool belowFloor;
};

struct ModeRecord {
  bool test;
  bool alphaFailSafe;
  bool sensorFailSafe;
  bool radioFailSafe;
  bool bankLimiter;
  bool takeOff;
  bool wingLeveler;
  bool slowFlight;
  bool halfRate;
  bool autoThrottle;
  bool loggingSuppressed;
};

struct FeatureRecord {
  bool keepLevel;
  bool stabilizeBank;
  bool stabilizePitch;
  bool alphaHold;
  bool pusher;
};

struct FlightState {
  float IAS, dynP, alpha;
  float accX, accY, accZ, acc, alt, bank, pitch, rollR, pitchR, yawR;
  uint16_t heading;
  float accDir, relWind, slope;
};

struct InputState {
  float aile, elev, aileExpo, elevExpo, throttle, rudder, tuningKnob;
  bool ailePilotInput, elevPilotInput, rudderPilotInput;
};

struct ControlState {
  float testGain;
  float elevTrim, targetAlpha, targetPressure, targetPitchR, minThrottle;
  float elevPredict, ailePredict, aileNeutral, pusher;
  uint8_t gearSel, flapSel;
};

struct OutputState {
  float elev, aile, brake, rudder, steer, thrustVert, thrustHoriz, flap;
};

struct GPSFix {
  float altitude;
  float track;
  float lat;
  float lon;
  float speed;
};

//
// Control and signal processing
//

extern PIDCtrl_t aileCtrl, elevCtrl, pushCtrl, throttleCtrl;
extern Sampler_t iasSampler;
extern SWAvg_t alphaFilter, liftFilter;
extern SlopeLimiter_t aileActuator, flapActuator, rollAccelLimiter, trimRateLimiter;
extern Damper_t iasFilter, iasFilterSlow, ball, accAvg;

// struct GPSFix gpsFix;

extern struct SimLinkSensor sensorData;
extern uint16_t simFrames;
extern int linkDownCount, heartBeatCount;

extern struct ModeRecord vpMode;
extern struct FeatureRecord vpFeature;
extern struct StatusRecord vpStatus;
extern struct FlightState vpFlight;
extern struct InputState vpInput;
extern struct ControlState vpControl;
extern struct OutputState vpOutput;

extern float controlCycle;
extern float outer_P, rudderMix, throttleMix;
extern float idleAvg, logBandWidth, ppmFreq, simInputFreq;
extern uint32_t simTimeStamp, idleMicros;
extern const int maxParams;
extern uint8_t gaugeCount, gaugeVariable[];
extern bool paramsModified;
extern uint32_t lastPPMWarn;
extern float fieldStrength;

#endif
