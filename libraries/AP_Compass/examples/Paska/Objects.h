#ifndef OBJECTS_H
#define OBJECTS_H

#include <stdint.h>
#include "Status.h"

struct FlightState {
  float iAS, dynP, alpha;
  float accX, accY, accZ, acc, alt, bank, pitch, rollRate, pitchRate, targetPitchRate, yawRate, slope;
  uint16_t heading;
  float accDir, relWind;
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

struct GPSFix {
  float altitude;
  float track;
  float lat;
  float lon;
  float speed;
};

extern struct ModeRecord vpMode;
extern struct FeatureRecord vpFeature;
extern struct StatusRecord vpStatus;
extern struct FlightState vpFlight;
// struct GPSFix gpsFix;

extern uint32_t lastPPMWarn;
extern float controlCycle;
extern uint32_t idleMicros;
extern float idleAvg, logBandWidth, ppmFreq, simInputFreq;
extern float testGain;
extern float aileStick, elevStick, elevStickExpo, throttleStick, rudderStick, tuningKnob;
extern bool ailePilotInput, elevPilotInput, rudderPilotInput;

extern float elevTrim, targetAlpha, targetPressure, targetPitchRate, minThrottle;
extern float outer_P, rudderMix, throttleMix;
extern uint8_t flapSel, gearSel;
extern float elevOutput, elevOutputFeedForward, aileOutput, aileOutputFeedForward, brakeOutput, rudderOutput, steerOutput, vertOutput, horizOutput, aileNeutral, pusherOutput;
extern float rollBias, aileBias, alphaBias, pitchBias;
extern float sampleRate, fieldStrength;

#endif
