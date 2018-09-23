#ifndef OBJECTS_H
#define OBJECTS_H

#include <stdint.h>
#include <stdbool.h>
#include "DSP.h"
#include "Controller.h"
#include "Datagram.h"

struct StatusRecord {
  bool armed;
  bool consoleLink;
  bool simulatorLink;
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
  bool flare;
  float load;
};

struct ModeRecord {
  bool test;
  bool silent;
  bool alphaFailSafe;
  bool sensorFailSafe;
  bool radioFailSafe;
  bool bankLimiter;
  bool takeOff;
  bool wingLeveler;
  bool slowFlight;
  bool halfRate;
  bool autoThrottle;
  bool dontLog;
  bool gearSelected;
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
  float stickForce;
};

struct ControlState {
  float testGain;
  float o_P, r_Mix, yd_P;
  float s_Ku_ref, i_Ku_ref, yd_P_ref;
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
extern Sampler_t alphaSampler, iasSampler;
extern SWAvg_t liftFilter;
extern SlopeLimiter_t aileActuator, flapActuator, rollAccelLimiter, trimRateLimiter;
extern Damper_t iasFilter, iasFilterSlow, ball, accAvg;
extern Washout_t yawDamper;

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
extern float logBandWidth, ppmFreq, simInputFreq;
extern uint32_t simTimeStamp, idleMicros;
extern const int maxParams;
extern uint8_t gaugeCount, gaugeVariable[];
extern bool paramsModified;
extern uint32_t lastPPMWarn;
extern float fieldStrength;

#endif
