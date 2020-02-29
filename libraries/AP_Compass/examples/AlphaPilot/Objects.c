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
bool ppmFrameReceived;
VP_TIME_MILLIS_T simTimeStamp;
VP_TIME_MICROS_T idleMicros;
uint8_t gaugeCount, gaugeVariable[MAX_PARAMS];
uint32_t lastPPMWarn;
float fieldStrength;

struct SimLinkSensor sensorData;
uint16_t simFrames;
float simInputFreq;
uint8_t linkDownCount[2], heartBeatCount[2];
uint16_t uptimeMinutes = 0;

PIDCtrl_t elevCtrl = PIDCTRL_CONS(1);
PIDCtrl_t canardCtrl = PIDCTRL_CONS(PI_F/2);
PIDCtrl_t pushCtrl = PIDCTRL_CONS(1);
#if AUTO_RUDDER
PIDCtrl_t rudderCtrl = PIDCTRL_U_CONS(RATIO(1/3));
#endif
PIDCtrl_t aileCtrl = PIDCTRL_U_CONS(RATIO(2/3));
Damper_t avgDynP = DAMPER_CONS(8*CONFIG_HZ, 0);
Damper_t accAvg = DAMPER_CONS(4*CONFIG_HZ, G);
Sampler_t alphaSampler = SAMPLER_CONS;
Sampler_t iasSampler = SAMPLER_CONS;
SlopeLimiter_t aileActuator = SLOPE_CONS(0);
SlopeLimiter_t trimRateLimiter = SLOPE_CONS(3/RADIAN);
SlopeLimiter_t flapActuator = SLOPE_CONS(0.5);
#if YAW_DAMPER
Washout_t yawDamper = WASHOUT_CONS(0.5*CONTROL_HZ, 0);
#endif
Turbine_t engine;
SWAvg_t liftFilter = SWAVG_CONS(CONFIG_HZ/4);
SWAvg_t primaryIASDataFilter = SWAVG_CONS(CONTROL_HZ/8);

