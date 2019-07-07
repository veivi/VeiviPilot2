#include "Objects.h"
#include "NVState.h"
#include "AlphaPilot.h"

struct ModeRecord vpMode;
struct FeatureRecord vpFeature;
struct StatusRecord vpStatus;
struct FlightState vpFlight;
struct InputState vpInput;
struct ControlState vpControl;
struct OutputState vpOutput;

float controlCycle;
float logBandWidth, ppmFreq;
uint32_t idleMicros;
uint8_t gaugeCount, gaugeVariable[MAX_PARAMS];
uint32_t lastPPMWarn;
float fieldStrength;

struct SimLinkSensor sensorData;
uint16_t simFrames;
float simInputFreq;
int linkDownCount, heartBeatCount;
uint32_t simTimeStamp;

PIDCtrl_t elevCtrl = PIDCTRL_CONS(1);
PIDCtrl_t pushCtrl = PIDCTRL_CONS(1);
PIDCtrl_t rudderCtrl = PIDCTRL_U_CONS(RATIO(1/3));
PIDCtrl_t aileCtrl = PIDCTRL_U_CONS(RATIO(2/3));
Damper_t avgDynP = DAMPER_CONS(8*CONFIG_HZ, 0);
Damper_t accAvg = DAMPER_CONS(4*CONFIG_HZ, G);
Sampler_t alphaSampler = SAMPLER_CONS;
Sampler_t iasSampler = SAMPLER_CONS;
SlopeLimiter_t aileActuator = SLOPE_CONS(0);
SlopeLimiter_t trimRateLimiter = SLOPE_CONS(3/RADIAN);
SlopeLimiter_t flapActuator = SLOPE_CONS(0.5);
Washout_t yawDamper = WASHOUT_CONS(0.5*CONTROL_HZ, 0);
Turbine_t engine;
SWAvg_t liftFilter = SWAVG_CONS(CONFIG_HZ/4);
SWAvg_t primaryIASDataFilter = SWAVG_CONS(CONTROL_HZ/8);

