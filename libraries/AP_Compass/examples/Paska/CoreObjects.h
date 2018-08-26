#ifndef COREOBJECTS_H
#define COREOBJECTS_H

#include <stdint.h>
#include <stdbool.h>

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
  bool progressiveFlight;
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
  bool aileFeedforward;
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
};

struct OutputState {
  float elev, aile, brake, rudder, steer, thrustVert, thrustHoriz, flap;
};

extern struct ModeRecord vpMode;
extern struct FeatureRecord vpFeature;
extern struct StatusRecord vpStatus;
extern struct FlightState vpFlight;
extern struct InputState vpInput;
extern struct ControlState vpControl;
extern struct OutputState vpOutput;

#endif
