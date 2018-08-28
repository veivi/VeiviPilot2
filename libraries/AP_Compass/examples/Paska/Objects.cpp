#include "Objects.h"
#include "NVState.h"
#include "AlphaPilot.h"

extern "C" {
#include "DSP.h"
}

/*
struct ModeRecord vpMode;
struct FeatureRecord vpFeature;
struct StatusRecord vpStatus;
struct FlightState vpFlight;
struct InputState vpInput;
struct ControlState vpControl;
struct OutputState vpOutput;
// struct GPSFix gpsFix;
*/

Controller elevCtrl, pushCtrl, throttleCtrl;
UnbiasedController aileCtrl;
AlphaBuffer pressureBuffer;
RunningAvgFilter alphaFilter(ALPHAWINDOW*ALPHA_HZ);
RateLimiter aileActuator, rollAccelLimiter, trimRateLimiter;

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
AP_HAL::BetterStream* cliSerial;

AP_Baro barometer;
AP_InertialSensor ins;
AP_GPS gps;
AP_AHRS_DCM ahrs {ins,  barometer, gps};

#ifdef USE_COMPASS
static Compass compass;
#endif

NewI2C I2c = NewI2C();

extern "C" {
struct SimLinkSensor sensorData;
uint16_t simFrames;
int linkDownCount, heartBeatCount;
}

