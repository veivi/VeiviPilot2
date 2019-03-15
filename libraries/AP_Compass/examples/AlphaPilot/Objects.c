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
bool paramsModified = false;
uint32_t lastPPMWarn;
float fieldStrength;

PIDCtrl_t elevCtrl, pushCtrl, rudderCtrl, throttleCtrl, aileCtrl;
Sampler_t alphaSampler, iasSampler;
MedianFilter_t elevFilter;
SlopeLimiter_t aileActuator, rollAccelLimiter, trimRateLimiter, flapActuator;
SWAvg_t liftFilter;
Damper_t iasFilter, iasFilterSlow, accAvg, iasEntropy, alphaEntropy;
Washout_t yawDamper;

struct SimLinkSensor sensorData;
uint16_t simFrames;
int linkDownCount, heartBeatCount;
uint32_t simTimeStamp;
