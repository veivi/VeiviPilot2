#include "Objects.h"
#include "NVState.h"
#include "AlphaPilot.h"

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

float controlCycle;
float outer_P, rudderMix, throttleMix;
uint8_t gearSel, flapSel;
float idleAvg, logBandWidth, ppmFreq, simInputFreq;
uint32_t simTimeStamp, idleMicros;
const int maxParams = MAX_SERVO;
uint8_t gaugeCount, gaugeVariable[maxParams];
bool paramsModified = false;
uint32_t lastPPMWarn;
float fieldStrength;

Controller elevCtrl, pushCtrl, throttleCtrl;
UnbiasedController aileCtrl;
Damper ball(1.5*CONTROL_HZ), iasFilterSlow(3*CONTROL_HZ), iasFilter(2), accAvg(2*CONTROL_HZ), iasEntropyAcc(CONFIG_HZ), alphaEntropyAcc(CONFIG_HZ);
AlphaBuffer pressureBuffer;
RunningAvgFilter alphaFilter(ALPHAWINDOW*ALPHA_HZ);
RateLimiter aileActuator, rollAccelLimiter, flapActuator, trimRateLimiter;

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
I2CDevice alphaDevice("alpha"), pitotDevice("pitot"), eepromDevice("EEPROM"), displayDevice("display");

const float sampleRate = LOG_HZ_SLOW;

extern "C" {
struct SimLinkSensor sensorData;
uint16_t simFrames;
int linkDownCount, heartBeatCount;
}

