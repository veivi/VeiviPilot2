#ifndef OBJECTS_H
#define OBJECTS_H

#include <stdint.h>
#include "Status.h"

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
  float accX, accY, accZ, acc, alt, bank, pitch, rollRate, pitchRate, yawRate, slope;
  uint16_t heading;
  float accDir, relWind;
};

struct InputState {
  float aile, elev, elevExpo, throttle, rudder, tuningKnob;
  bool ailePilotInput, elevPilotInput, rudderPilotInput;
};

struct ControlState {
  float testGain;
  float elevTrim, targetAlpha, targetPressure, targetPitchRate, minThrottle;
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

struct GPSFix {
  float altitude;
  float track;
  float lat;
  float lon;
  float speed;
};

// struct GPSFix gpsFix;

#endif
