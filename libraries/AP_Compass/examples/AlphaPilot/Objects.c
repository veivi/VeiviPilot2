#include "Objects.h"
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
float logBandWidth, ppmFreq, simInputFreq;
uint32_t idleMicros;
uint8_t gaugeCount, gaugeVariable[MAX_PARAMS];
uint32_t lastPPMWarn;
float fieldStrength;

PIDCtrl_t elevCtrl, pushCtrl, rudderCtrl, aileCtrl;
Sampler_t alphaSampler, iasSampler;
SlopeLimiter_t aileActuator, trimRateLimiter, flapActuator;
SWAvg_t liftFilter, primaryIASDataFilter;
Damper_t dynPFilter, accAvg, iasEntropy, alphaEntropy;
Washout_t yawDamper;
Turbine_t engine;

struct SimLinkSensor sensorData;
uint16_t simFrames;
int linkDownCount, heartBeatCount;
uint32_t simTimeStamp;
