#include "Objects.h"
#include "AlphaPilot.h"
#include "NewI2C.h"

extern "C" {
#include "DSP.h"
#include "NVState.h"
}

Controller elevCtrl, pushCtrl, throttleCtrl;
UnbiasedController aileCtrl;
Sampler_t pressureBuffer;
SlopeLimiter_t aileActuator, rollAccelLimiter, trimRateLimiter;
SWAvg_t alphaFilter, liftFilter;

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

