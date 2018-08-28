#include "CoreObjects.h"
#include "NVState.h"

struct ModeRecord vpMode;
struct FeatureRecord vpFeature;
struct StatusRecord vpStatus;
struct FlightState vpFlight;
struct InputState vpInput;
struct ControlState vpControl;
struct OutputState vpOutput;
// struct GPSFix gpsFix;

float controlCycle;
float outer_P, rudderMix, throttleMix;
uint8_t gearSel, flapSel;
float idleAvg, logBandWidth, ppmFreq, simInputFreq;
uint32_t simTimeStamp, idleMicros;
uint8_t gaugeCount, gaugeVariable[MAX_PARAMS];
bool paramsModified = false;
uint32_t lastPPMWarn;
float fieldStrength;

Damper_t ball, iasFilter, iasFilterSlow, accAvg, iasEntropy, alphaEntropy;
SlopeLimiter_t flapActuator;

