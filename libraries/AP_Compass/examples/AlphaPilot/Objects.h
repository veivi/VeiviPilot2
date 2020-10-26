
#ifndef OBJECTS_H
#define OBJECTS_H

#include <stdint.h>
#include <stdbool.h>
#include "DSP.h"
#include "Controller.h"
#include "Datagram.h"

struct StatusRecord {
  bool consoleLink;
  bool telemetryLink;
  bool simulatorLink;
  bool positiveIAS;
  bool fullStop;
  bool pitotFailed;
  bool pitotBlocked;
  bool stall;
  bool weightOnWheels;
  bool airborne;
  bool goAround;
  bool canopyClosed;
  int fault;
  bool alphaFailed;
  bool alphaUnreliable;
  bool trimLimited;
  bool upright;
  bool flare;
  float load;
  float fuel;
  float mass;
};

struct ModeRecord {
  bool armed;
  bool test;
  uint8_t testCount;
  bool silent;
  bool alphaFailSafe;
  bool sensorFailSafe;
  bool radioFailSafe;
  bool bankLimiter;
  bool takeOff;
  bool wingLeveler;
  bool slowFlight;
  bool halfRate;
  bool autoThrottle;
  bool dontLog;
  bool gearSelected;
  bool passive;
};

struct FeatureRecord {
  bool keepLevel;
  bool stabilizeBank;
  bool stabilizePitch;
  bool alphaHold;
  bool pusher;
};

struct FlightState {
  float IAS, dynP, alpha;
  float effIAS, effDynP;
  float relativeIAS, relativeEffIAS;
  float accX, accY, accZ, acc, alt, bank, pitch, rollR, pitchR, yawR, ball;
  uint16_t heading;
  float accDir, relWind;
};

struct InputState {
  float aile, elev, aileExpo, elevExpo, throttle, rudder, tuningKnob;
  bool ailePilotInput, elevPilotInput, rudderPilotInput;
  int8_t modeSel, flapSel;
  float stickForce;
};

typedef enum { gs_down, gs_goingup_open, gs_goingup, gs_goingup_close, gs_up, gs_goingdown_open, gs_goingdown, gs_goingdown_close } gearState_t;

typedef enum { it_init, it_read_nv, it_done } InitTaskState_t;

struct ControlState {
  InitTaskState_t initState;
  float testGain;
  float o_P, r_Mix, yd_P, t_Mix;
  float s_Ku_ref, i_Ku_ref, yd_P_ref, r_Ku_ref;
  float elevTrim, targetAlpha, targetPitchR;
  float elevPredict, ailePredict, aileNeutral, pusher;
  uint8_t gearSel, flapSel;
  bool parking, flaring;
  gearState_t gearState;
  uint8_t pwmCount;
};

struct OutputState {
  float elev, aile, brake, rudder, steer, thrustVert, thrustHoriz, flap, canard;
};

struct GPSFix {
  float altitude;
  float track;
  float lat;
  float lon;
  float speed;
};

//
// Control and signal processing
//

extern PIDCtrl_t aileCtrl, elevCtrl, rudderCtrl, pushCtrl, canardCtrl;
extern Sampler_t alphaSampler, iasSampler;
extern SWAvg_t liftFilter;
extern SWAvg_t primaryIASDataFilter;
extern SlopeLimiter_t aileActuator, flapActuator, trimRateLimiter;
extern Washout_t yawDamper;
extern Turbine_t engine;
extern Damper_t avgDynP, accAvg;

// struct GPSFix gpsFix;

extern struct SimLinkSensor sensorData;
extern uint16_t simFrames;
extern uint8_t linkDownCount[], heartBeatCount[];
extern uint16_t uptimeMinutes;

extern struct ModeRecord vpMode;
extern struct FeatureRecord vpFeature;
extern struct StatusRecord vpStatus;
extern struct FlightState vpFlight;
extern struct InputState vpInput;
extern struct ControlState vpControl;
extern struct OutputState vpOutput;

extern float controlCycle;
extern int16_t controlFreq;
extern float logBandWidth, ppmFreq, simInputFreq;
extern bool ppmFrameReceived;
extern VP_TIME_MILLIS_T simTimeStamp;
extern VP_TIME_MICROS_T idleMicros;
extern const int maxParams;
extern uint8_t gaugeCount, gaugeVariable[];
extern uint16_t ppmGoodSeconds;
extern VP_TIME_MICROS_T ppmInputTimeStamp, controlInputTimeStamp;
extern VP_TIME_MICROS_T controlLatencyTotal;
extern float controlLatencyAvg;
extern uint16_t controlLatencyCount;
extern float fieldStrength;

#endif
