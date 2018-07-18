#ifndef COREOBJECTS_H
#define COREOBJECTS_H

#include <stdint.h>

typedef uint8_t bool_t;

struct StatusRecord {
  bool_t armed;
  bool_t consoleLink;
  bool_t simulatorLink;
  bool_t silent;
  bool_t positiveIAS;
  bool_t fullStop;
  bool_t pitotFailed;
  bool_t pitotBlocked;
  bool_t stall;
  bool_t weightOnWheels;
  bool_t aloft;
  int fault;
  bool_t alphaFailed;
  bool_t alphaUnreliable;
  bool_t upright;
  bool_t belowFloor;
};

struct ModeRecord {
  bool_t test;
  bool_t alphaFailSafe;
  bool_t sensorFailSafe;
  bool_t radioFailSafe;
  bool_t bankLimiter;
  bool_t takeOff;
  bool_t wingLeveler;
  bool_t slowFlight;
  bool_t progressiveFlight;
  bool_t halfRate;
  bool_t autoThrottle;
  bool_t loggingSuppressed;
};

struct FeatureRecord {
  bool_t keepLevel;
  bool_t stabilizeBank;
  bool_t stabilizePitch;
  bool_t alphaHold;
  bool_t pusher;
  bool_t aileFeedforward;
  bool_t flareAllowed;
};

struct FlightState {
  float IAS, dynP, alpha;
  float accX, accY, accZ, acc, alt, bank, pitch, rollR, pitchR, yawR;
  uint16_t heading;
  float accDir, relWind, slope;
};

struct InputState {
  float aile, elev, aileExpo, elevExpo, throttle, rudder, tuningKnob;
  bool_t ailePilotInput, elevPilotInput, rudderPilotInput;
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
