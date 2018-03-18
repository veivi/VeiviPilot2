#include "Objects.h"

struct ModeRecord vpMode;
struct FeatureRecord vpFeature;
struct StatusRecord vpStatus;
struct FlightState vpFlight;
// struct GPSFix gpsFix;

uint32_t lastPPMWarn;
float controlCycle;
uint32_t idleMicros;
float idleAvg, logBandWidth, ppmFreq, simInputFreq;
float testGain;
float aileStick, elevStick, elevStickExpo, throttleStick, rudderStick, tuningKnob;
bool ailePilotInput, elevPilotInput, rudderPilotInput;

float elevTrim, targetAlpha, targetPressure, targetPitchRate, minThrottle;
float outer_P, rudderMix, throttleMix;
uint8_t flapSel, gearSel;
float elevOutput, elevOutputFeedForward, aileOutput, aileOutputFeedForward, brakeOutput, rudderOutput, steerOutput, vertOutput, horizOutput, aileNeutral, pusherOutput;
float rollBias, aileBias, alphaBias, pitchBias;
float fieldStrength;
