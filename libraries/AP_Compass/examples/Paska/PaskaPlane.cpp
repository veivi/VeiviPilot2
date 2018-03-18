#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "Status.h"
#include "Filter.h"
#include "Math.h"
#include "Console.h"
#include "Controller.h"
#include "NewI2C.h"
#include "Storage.h"
#include "Interrupt.h"
#include "RxInput.h"
#include "Logging.h"
#include "NVState.h"
#include "PWMOutput.h"
#include "PPM.h"
#include "Command.h"
#include "Button.h"
#include "Time.h"
#include "AS5048B.h"
#include "MS4525.h"
#include "SSD1306.h"
#include "Objects.h"
#include <AP_Progmem/AP_Progmem.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>

extern "C" {
#include "CRC16.h"
#include "System.h"
}

//
// Configuration
//

#define RX_CHANNELS          6
#define THROTTLE_SIGN        1
// #define HARD_PUSHER 1     // Uncomment to select "hard" pusher
// #define USE_COMPASS  1

//
// Constants
//

const float alphaWindow_c = RATIO(1/25);
const int flapSteps_c = 2;

//
// Ok let's do some magic stuff!
//

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
AP_HAL::BetterStream* cliSerial;

AP_Baro barometer;
AP_InertialSensor ins;
AP_GPS gps;
AP_AHRS_DCM ahrs {ins,  barometer, gps};
#ifdef USE_COMPASS
static Compass compass;
#endif

//
// LED output
//

const struct PinDescriptor led[] = {{ PortA, 3 }, { PortA, 4 }, { PortA, 5 }};

#define GREEN_LED led[0]
#define BLUE_LED led[1]
#define RED_LED led[2]

//
// RC input
//

struct PinDescriptor ppmInputPin = { PortL, 1 }; 
struct RxInputRecord aileInput, elevInput, throttleInput,
  buttonInput, tuningKnobInput, flightModeInput;

#if RX_CHANNELS < 8
struct RxInputRecord *ppmInputs[] = 
  { &aileInput, &elevInput, &throttleInput, &buttonInput, &tuningKnobInput, &flightModeInput };
#else
struct RxInputRecord rudderInput, stabModeInput;

struct RxInputRecord *ppmInputs[] = 
  { &aileInput, &elevInput, &throttleInput, &rudderInput, &buttonInput, &tuningKnobInput, &flightModeInput, &stabModeInput };
#endif

//
// Mode selector inputs
//

struct SwitchRecord flightModeSelector = { &flightModeInput };
#if RX_CHANNELS >= 8
struct SwitchRecord stabModeSelector = { &stabModeInput };
#endif

int8_t flightModeSelectorValue, stabModeSelectorValue;

//
// Buttons
//

Button rightDownButton(-1.0), rightUpButton(0.33),
  leftDownButton(-0.3), leftUpButton(1);

#define LEVELBUTTON rightUpButton
#define FLAPBUTTON rightDownButton
#define TRIMBUTTON leftUpButton
#define GEARBUTTON leftDownButton

//
// Periodic task stuff
//

#define CONTROL_HZ 50
#define CONFIG_HZ (CONTROL_HZ/4.0)
#define ALPHA_HZ (CONTROL_HZ*10)
#define AIRSPEED_HZ (CONTROL_HZ*5)
#define TRIM_HZ CONFIG_HZ
#define LED_HZ 3
#define LED_TICK 100
#define LOG_HZ_FAST CONTROL_HZ
#define LOG_HZ_SLOW (CONTROL_HZ/4.0)
#define LOG_HZ_COMMIT 3
#define LOG_HZ_FLUSH 5
#define HEARTBEAT_HZ 1
  
struct Task {
  void (*code)(void);
  uint32_t period, lastExecuted;
};

#define HZ_TO_PERIOD(f) ((uint32_t) (1.0e6/(f)))

const float sampleRate = LOG_HZ_SLOW;

NewI2C I2c = NewI2C();
Controller elevCtrl, pushCtrl, throttleCtrl;
UnbiasedController aileCtrl;
Damper ball(1.5*CONTROL_HZ), iasFilterSlow(3*CONTROL_HZ), iasFilter(2), accAvg(2*CONTROL_HZ), iasEntropyAcc(CONFIG_HZ), alphaEntropyAcc(CONFIG_HZ);
AlphaBuffer pressureBuffer;
RunningAvgFilter alphaFilter(alphaWindow_c*ALPHA_HZ);
RateLimiter aileRateLimiter, flapActuator, trimRateLimiter;
I2CDevice alphaDevice("alpha"), pitotDevice("pitot"), eepromDevice("EEPROM"), displayDevice("display");

//
// Misc local variables
//

static uint32_t simTimeStamp;
static uint16_t iasEntropy, alphaEntropy, sensorHash = 0xFFFF;
const int maxParams = MAX_SERVO;
static uint8_t gaugeCount, gaugeVariable[maxParams];
static bool paramsModified = false;
static uint32_t idleMicros;
static uint32_t lastPPMWarn;
static float fieldStrength;

//
// Datagram protocol integration
//

#include "Serial.h"

#define MAX_DG_SIZE  (1<<7)

extern "C" {

#include "Datagram.h"

int maxDatagramSize = MAX_DG_SIZE;
uint8_t datagramRxStore[MAX_DG_SIZE];

void executeCommand(char *buf);

struct SimLinkSensor sensorData;
uint16_t simFrames;
int linkDownCount = 0, heartBeatCount = 0;
  
void datagramRxError(const char *error)
{
  consoleNote_P(PSTR("DG "));
  consolePrintLn(error);
}
  
void datagramInterpreter(uint8_t t, uint8_t *data, int size)
{
  switch(t) {
  case DG_HEARTBEAT:
    if(!vpStatus.consoleLink) {
      consoleNoteLn_P(PSTR("Console CONNECTED"));
      vpStatus.consoleLink = true;
    }
    heartBeatCount++;
    linkDownCount = 0;
    break;
    
  case DG_CONSOLE:
    executeCommand((char*) data);
    break;

  case DG_SIMLINK:
    if(vpStatus.consoleLink && size == sizeof(sensorData)) {
      if(!vpStatus.simulatorLink) {
	consoleNoteLn_P(PSTR("Simulator CONNECTED"));
	vpStatus.simulatorLink = vpMode.loggingSuppressed = true;
      }

      memcpy(&sensorData, data, sizeof(sensorData));
      simTimeStamp = currentMicros();
      simFrames++;    
    }
    break;

  case DG_PING:
    break;
    
  default:
    consoleNote_P(PSTR("FUNNY DATAGRAM TYPE "));
    consolePrintLn(t);
  }
}
  
void datagramSerialOut(uint8_t c)
{
  serialOut(c);
}

void datagramSerialFlush()
{
  serialFlush();
}
}

//
// Log interface
//

void logAlpha(void)
{
  logGeneric(lc_alpha, vpFlight.alpha*RADIAN);
}

void logConfig(void)
{
  bool mode[] = { vpMode.slowFlight,
		  vpMode.bankLimiter,
		  vpMode.wingLeveler,
		  vpMode.takeOff,
		  vpMode.autoThrottle,
		  vpMode.radioFailSafe,
		  vpMode.sensorFailSafe,
		  vpMode.alphaFailSafe };

  float sum = 0;
  
  for(uint16_t i = 0; i < sizeof(mode)/sizeof(bool); i++)
    if(mode[i])
      sum += 1.0/(2<<i);
  
  logGeneric(lc_mode, sum);
  
  bool status[] = { vpStatus.weightOnWheels,
		    vpStatus.positiveIAS,
		    gearSel == 1,
		    vpStatus.stall,
		    vpStatus.alphaFailed,
		    vpStatus.alphaUnreliable,
		    vpStatus.pitotFailed,
		    vpStatus.pitotBlocked };
  
  sum = 0;
  
  for(uint16_t i = 0; i < sizeof(status)/sizeof(bool); i++)
    if(status[i])
      sum += 1.0/(2<<i);
  
  logGeneric(lc_status, sum);
  
  logGeneric(lc_target, vpControl.targetAlpha*RADIAN);
  logGeneric(lc_target_pr, vpControl.targetPitchR*RADIAN);
  logGeneric(lc_trim, vpControl.elevTrim*100);

  if(vpMode.test) {
    logGeneric(lc_gain, vpControl.testGain);
    logGeneric(lc_test, nvState.testNum);
  } else {
    logGeneric(lc_gain, 0);
    logGeneric(lc_test, 0);
  }
}

void logPosition(void)
{
  logGeneric(lc_alt, vpFlight.alt);
}
  
void logInput(void)
{
  logGeneric(lc_ailestick, vpInput.aile);
  logGeneric(lc_elevstick, vpInput.elevExpo);
  logGeneric(lc_thrstick, throttleCtrl.output());
  logGeneric(lc_rudstick, vpInput.rudder);
}

void logActuator(void)
{
  logGeneric(lc_aileron, vpOutput.aile);
  logGeneric(lc_aileron_ff, vpControl.ailePredict);
  logGeneric(lc_elevator, vpOutput.elev);
  logGeneric(lc_elevator_ff, vpControl.elevPredict);
  logGeneric(lc_rudder, vpOutput.rudder);
  logGeneric(lc_flap, flapActuator.output());
}

void logFlight(void)
{
  logGeneric(lc_dynpressure, vpFlight.dynP);
  logGeneric(lc_accx, vpFlight.accX);
  logGeneric(lc_accy, vpFlight.accY);
  logGeneric(lc_accz, vpFlight.accZ);
  logGeneric(lc_roll, vpFlight.bank*RADIAN);
  logGeneric(lc_rollrate, vpFlight.rollR*RADIAN);
  logGeneric(lc_pitch, vpFlight.pitch*RADIAN);
  logGeneric(lc_pitchrate, vpFlight.pitchR*RADIAN);
  logGeneric(lc_heading, vpFlight.heading);
  logGeneric(lc_yawrate, vpFlight.yawR*RADIAN);
}

//
// Takeoff configuration test
//

typedef enum {
  toc_alpha,
  toc_pitot,
  toc_link,
  toc_lstick,
  toc_rstick,
  toc_tuning,
  toc_button,
  toc_attitude,
  toc_gyro,
  toc_mode,
  toc_fdr,
  toc_ram,
  toc_load,
} testCode_t;

#define TOC_TEST_NAME_MAX 16

struct TakeoffTest {
  char description[TOC_TEST_NAME_MAX];
  bool (*function)(bool);
};

const float toc_margin_c = RATIO(3/100);

bool toc_test_mode(bool reset)
{
  return !vpMode.test
    && vpMode.wingLeveler && vpMode.bankLimiter && vpMode.takeOff;
}

bool toc_test_link(bool reset)
{
  return currentTime - lastPPMWarn > 10e6 &&
    (!vpParam.virtualOnly || vpStatus.simulatorLink);
}

bool toc_test_ram(bool reset)
{
  return hal.util->available_memory() > (1<<9);
}

bool toc_test_load(bool reset)
{
  return idleAvg > 0.15;
}

bool toc_test_fdr(bool reset)
{
  return !eepromDevice.warning() && logReady(false);
}

bool toc_test_alpha_sensor(bool reset)
{
  return !vpStatus.alphaFailed && alphaEntropyAcc.output() > 50
    && fieldStrength > 0.15 && fieldStrength < 0.80;
}

bool toc_test_alpha_range(bool reset)
{
  static bool bigAlpha, zeroAlpha;
  static uint32_t lastNonZeroAlpha, lastSmallAlpha;

  if(reset) {
    zeroAlpha = bigAlpha = false;
    lastNonZeroAlpha = lastSmallAlpha = currentTime;
    
  } else if(!zeroAlpha) {
    if(fabs(vpFlight.alpha) > 1.5/RADIAN) {
      lastNonZeroAlpha = currentTime;
    } else if(currentTime > lastNonZeroAlpha + 1.0e6) {
      consoleNoteLn_P(PSTR("Stable ZERO ALPHA"));
      zeroAlpha = true;
    }
  } else if(!bigAlpha) {
    if(fabsf(vpFlight.alpha - 90/RADIAN) > 30/RADIAN) {
      lastSmallAlpha = currentTime;
    } else if(currentTime > lastSmallAlpha + 1.0e6) {
      consoleNoteLn_P(PSTR("Stable BIG ALPHA"));
      bigAlpha = true;
    }
  }
  
  return (bigAlpha && zeroAlpha);
}

bool toc_test_alpha(bool reset)
{
  return (toc_test_alpha_sensor(reset) && toc_test_alpha_range(reset))
     || vpStatus.simulatorLink;
}

bool toc_test_pitot(bool reset)
{
  static bool positiveIAS;
  
  if(reset)
    positiveIAS = false;
  else if(vpStatus.positiveIAS)
    positiveIAS = true;
  
  return (!vpStatus.pitotFailed && iasEntropyAcc.output() > 50
	  && !vpStatus.pitotBlocked && positiveIAS && vpFlight.IAS < 5)
    || vpStatus.simulatorLink;
}

bool toc_test_attitude(bool reset)
{
  return fabsf(vpFlight.pitch) < 10/RADIAN && fabsf(vpFlight.bank) < 5/RADIAN;
}

bool toc_test_gyro(bool reset)
{
  return (fabsf(vpFlight.pitchR) < 1.0/RADIAN
	  && fabsf(vpFlight.rollR) < 1.0/RADIAN
	  && fabsf(vpFlight.yawR) < 1.0/RADIAN);
}

struct TOCRangeTestState {
  float valueMin, valueMax;
};

bool toc_test_range_generic(struct TOCRangeTestState *state, bool reset, struct RxInputRecord *input, float expectedMin, float expectedMax)
{
  const float value = inputValue(input);
  
  if(reset) {
    state->valueMin = state->valueMax = value;
    return true;
  } else {
    state->valueMin = fminf(state->valueMin, value);
    state->valueMax = fmaxf(state->valueMax, value);
  }
  
  return (fabsf(state->valueMin - expectedMin) < toc_margin_c
	  && fabsf(state->valueMax - expectedMax) < toc_margin_c);
}

bool toc_test_rstick_range(bool reset)
{
  static struct TOCRangeTestState stateElev, stateAile;
  return toc_test_range_generic(&stateElev, reset, &elevInput, -1, 1)
    && toc_test_range_generic(&stateAile, reset, &aileInput, -1, 1);
}

bool toc_test_rstick_neutral(bool reset)
{
  return ( fabsf(inputValue(&aileInput)) < toc_margin_c )
    && ( fabsf(inputValue(&elevInput)) < toc_margin_c );
}

bool toc_test_rstick(bool reset)
{
  return toc_test_rstick_range(reset) && toc_test_rstick_neutral(reset);
}

bool toc_test_lstick_range(bool reset)
{
  static struct TOCRangeTestState stateThr;
  bool status = toc_test_range_generic(&stateThr, reset, &throttleInput, 0, 1);

#if RX_CHANNELS >= 8
  static struct TOCRangeTestState stateRudder;
  bool status2 = toc_test_range_generic(&stateRudder, reset, &rudderInput, -1, 1);
  status = status && status2;
#endif

  return status;
}

bool toc_test_lstick_neutral(bool reset)
{
  bool status = fabsf(inputValue(&throttleInput)) < toc_margin_c;
    
#if RX_CHANNELS >= 8
  status = status && fabsf(inputValue(&rudderInput)) < toc_margin_c;
#endif
  
  return status;
}

bool toc_test_lstick(bool reset)
{
  return toc_test_lstick_range(reset) && toc_test_lstick_neutral(reset);
}

bool toc_test_tuning_range(bool reset)
{
  static struct TOCRangeTestState state;
  return toc_test_range_generic(&state, reset, &tuningKnobInput, 0, 1);
}

bool toc_test_tuning_zero(bool reset)
{
  return fabsf(inputValue(&tuningKnobInput)) < toc_margin_c;
}

bool toc_test_tuning(bool reset)
{
  return toc_test_tuning_range(reset) && toc_test_tuning_zero(reset);
}

bool toc_test_button_range(bool reset)
{
  static struct TOCRangeTestState state;
  return toc_test_range_generic(&state, reset, &buttonInput, -1, 1);
}

bool toc_test_button_neutral(bool reset)
{
  return  !leftUpButton.state() && !leftDownButton.state()
    && !rightUpButton.state() && !rightDownButton.state();
}

bool toc_test_button(bool reset)
{
  return toc_test_button_range(reset) && toc_test_button_neutral(reset);
}

const struct TakeoffTest tocTest[] PROGMEM =
  {
    [toc_alpha] = { "ALPHA", toc_test_alpha },
    [toc_pitot] = { "PITOT", toc_test_pitot },
    [toc_link] = { "LINK", toc_test_link },
    [toc_lstick] = { "LSTIK", toc_test_lstick },
    [toc_rstick] = { "RSTIK", toc_test_rstick },
    [toc_tuning] = { "TUNE", toc_test_tuning },
    [toc_button] = { "BUTTN", toc_test_button },
    [toc_attitude] = { "ATTI", toc_test_attitude },
    [toc_gyro] = { "GYRO", toc_test_gyro },
    [toc_mode] = { "MODE", toc_test_mode },
    [toc_fdr] = { "FDR", toc_test_fdr },
    [toc_ram] = { "RAM", toc_test_ram },
    [toc_load] = { "LOAD", toc_test_load }
  };

const int tocNumOfTests = sizeof(tocTest)/sizeof(struct TakeoffTest);

bool tocTestInvoke(bool reset, bool challenge, void (*report)(bool, int, const char *))
{
  bool fail = false;
  struct TakeoffTest cache;
    
  for(int i = 0; i < tocNumOfTests; i++) {
    memcpy_P(&cache, &tocTest[i], sizeof(cache));

    bool result = (*cache.function)(reset);
    
    if(challenge) {
      (*report)(result, i, cache.description);

      if(!result)
	fail = true;
    }
  }

  return !fail;
}

bool tocTestReset()
{
  return tocTestInvoke(true, false, NULL);
}

void tocTestUpdate()
{
  tocTestInvoke(false, false, NULL);
}

bool tocStatusFailed;

void tocReportConsole(bool result, int i, const char *s)
{
  if(result)
    return;
  
  if(!tocStatusFailed) {
    consoleNote_P(PSTR("T/O/C FAIL :"));
    tocStatusFailed = true;
  }

  consolePrint(" ");
  consolePrint(s);
}

bool tocTestStatus(void (*reportFn)(bool, int, const char*))
{
  tocStatusFailed = false;
  return tocTestInvoke(false, true, reportFn);
}

//
// Command interpreter
//

const prog_char_t *applyParamUpdate()
{
  /*
  vpParam.i_Ku_C /= 2*PI;
  vpParam.s_Ku_C /= 2*PI;
  vpParam.p_Ku_C /= 2*PI;
  vpParam.o_P /= 2*PI;
  return PSTR("i_Ku, s_Ku, p_Ku, o_P scaled by 1/2/PI");
  */
  /*  
  vpParam.ff_B *= RADIAN;
  return PSTR("ff_B scaled by RADIAN");
  */
  /*
  vpParam.cL_A *= vpDerived.totalMass;
  vpParam.cL_B *= vpDerived.totalMass;
  vpParam.cL_max *= vpDerived.totalMass;
  
  return PSTR("CoL scaled by mass (now indicates F instead of a)");
  */
  
  return NULL;
}

char *parse(char *ptr)
{
  while(*ptr && !isblank(*ptr))
    ptr++;

  if(*ptr) {
    *ptr++ = '\0';
    while(*ptr && isblank(*ptr))
      ptr++;
  }

  return ptr;
}

void printCoeffElement(float y0, float y1, float x, float v)
{
    consoleNote("");
    consolePrint(x);
    consoleTab(10);

    const int col1 = 78, col0 = 10+(col1-10)*-y0/(y1-y0);
    int y = col0 + (col1-col0) * v / y1;

    if(y < col0) {
      consoleTab(y);
      consolePrint("*");
      consoleTab(col0);
      consolePrint("|");
      consoleTab(col1);
      consolePrintLn("|");
    } else if(y > col0) {
      consoleTab(col0);
      consolePrint("|");
      consoleTab(y);
      if(y < col1) {
	consolePrint("*");
	consoleTab(col1);
	consolePrintLn("|");
      } else
	consolePrintLn("*");
    } else {
      consoleTab(col0);
      consolePrint("*");
      consoleTab(col1);
      consolePrintLn("|");
    }
}

void executeCommand(char *buf)
{
  while(*buf && isblank(*buf))
    buf++;
  
  consolePrint_P(PSTR("// % "));
  consolePrintLn(buf);

  if(!*buf || buf[0] == '/') {
    gaugeCount = 0;
    calibStop(nvState.rxMin, nvState.rxCenter, nvState.rxMax);
    return;
  } else if(atoi(buf) > 0) {
    gaugeCount = 1;
    gaugeVariable[0] = atoi(buf);
    return;
  }
  
  int numParams = 0;
  float param[maxParams];
  const char *paramText[maxParams];

  for(int i = 0; i < maxParams; i++)
    param[i] = 0.0;

  char *parsePtr = buf;
  
  while(*(parsePtr = parse(parsePtr))) {
    if(numParams < maxParams) {
      paramText[numParams] = parsePtr;
      param[numParams] = atof(paramText[numParams]);
      numParams++;
    }
  }
  
  int matches = 0, j = 0;
  struct Command command;
  
  while(1) {
    struct Command cache;
  
    memcpy_P(&cache, &commands[j++], sizeof(cache));

    if(cache.token == c_invalid)
      break;
    
    if(!strncmp(buf, cache.name, strlen(buf))) {
      command = cache;
      matches++;
    }
  }
  
  if(matches < 1) {
    consolePrint_P(PSTR("Command not recognized: \""));
    consolePrint(buf);
    consolePrintLn("\"");
    
  } else if(matches > 1) {
    consolePrint_P(PSTR("Ambiguos command: \""));
    consolePrint(buf);
    consolePrintLn("\"");
    
  } else if(command.var[0]) {
    //
    // Simple variable
    //
    
    for(int i = 0; i < numParams && command.var[i]; i++) {
      switch(command.varType) {
      case e_string:
	strncpy((char*) command.var[i], paramText[i], NAME_LEN-1);
	break;
      
      case e_uint16:
	*((uint16_t*) command.var[i]) = param[i];
	break;
      
      case e_int16:
	*((int16_t*) command.var[i]) = param[i];
	break;
      
      case e_int8:
	*((int8_t*) command.var[i]) = param[i];
	break;
      
      case e_float:
	*((float*) command.var[i]) = param[i];
	break;

      case e_percent:
	*((float*) command.var[i]) = param[i]/100;
	break;

      case e_angle:
	*((float*) command.var[i]) = param[i]/RADIAN;
	break;

      case e_angle90:
	*((float*) command.var[i]) = param[i]/90;
	break;
	
      case e_bool:
	*((bool*) command.var[i]) = param[i];
	break;

      case e_map:
	for(int k = 0; k < MAX_SERVO; k++)
	  ((uint8_t*) command.var[i])[k] = param[i+k];
	break;
      }
    }
  } else {
    //
    // Complex
    //

    int currentModel = nvState.model;
    float offset = 0.0;
    
    switch(command.token) {
    case c_atrim:
      vpParam.aileNeutral += vpParam.aileDefl*param[0];
      break;
      
    case c_etrim:
      vpParam.elevNeutral += vpParam.elevDefl*param[0];
      break;
      
    case c_rtrim:
      vpParam.rudderNeutral += vpParam.rudderDefl*param[0];
      break;
      
    case c_arm:
      vpStatus.armed = true;
      break;
    
    case c_disarm:
      vpStatus.armed = false;
      consoleNoteLn_P(PSTR("We're DISARMED"));
      break;
    
    case c_talk:
      vpStatus.silent = false;
      consoleNoteLn_P(PSTR("Hello world"));
      break;
    
    case c_test:
      if(numParams > 0)
	logTestSet(param[0]);

      consoleNote_P(PSTR("Current test channel = "));
      consolePrintLn(nvState.testNum);
      break;

    case c_gear:
      if(numParams > 0)
	gearSel = param[0];
      break;
      
    case c_calibrate:
      consoleNoteLn_P(PSTR("Receiver calibration STARTED"));
      calibStart();
      break;

    case c_rollrate:
      if(numParams > 0) {
	vpParam.roll_C
	  = param[0]/RADIAN/powf(vpDerived.minimumIAS, stabilityAileExp2_c);
	consoleNote_P(PSTR("Roll rate K = "));
	consolePrintLn(vpParam.roll_C);
	storeNVState();
      }
      break;
          
    case c_alpha:
      if(numParams > 0)
	offset = param[0];
      
      vpParam.alphaRef +=
	(int16_t) ((1L<<16) * (vpFlight.alpha - offset / RADIAN) / CIRCLE);
      consoleNoteLn_P(PSTR("Alpha calibrated"));
      break;

    case c_gauge:
      if(numParams < 1) {
	gaugeCount = 1;
	gaugeVariable[0] = 1;
      } else {
	gaugeCount = numParams;
	
	for(int i = 0; i < numParams; i++)
	  gaugeVariable[i] = param[i];
      }
      break;
	
    case c_store:
      consoleNoteLn_P(PSTR("Params & NV state stored"));
      storeNVState();
      storeParams();
      backupParams();
      break;

    case c_defaults:
      consoleNoteLn_P(PSTR("Default params restored"));
      defaultParams();
      break;

    case c_dump:
      logDumpBinary();
      break;

    case c_fault:
      if(numParams > 0)
	vpStatus.fault = param[0];
      else
	vpStatus.fault = 0;
      break;

    case c_scale:
      if(numParams > 0) {
	vpParam.i_Ku_C *= param[0];
	vpParam.i_Tu *= param[0];
	vpParam.s_Ku_C *= param[0];
	vpParam.s_Tu *= param[0];
	vpParam.cL_A *= param[0];
	vpParam.cL_B *= param[0];
	vpParam.cL_C *= param[0];
	vpParam.cL_D *= param[0];
	vpParam.cL_E *= param[0];
      }
      break;
    
    case c_update:
      if(!updateDescription) {
	for(int i = 0; i < maxModels(); i++) {
	  if(setModel(i, false)) {
	    updateDescription = applyParamUpdate();
	    storeParams();
	  }
	}
	
	setModel(currentModel, false);
	
	if(updateDescription) {
	  consoleNote_P(PSTR("Param update applied : "));
	  consolePrintLn_P(updateDescription);
	}
      }
      break;
      
    case c_stamp:
      if(numParams > 0) {
	nvState.logStamp = param[0];
	storeNVState();
      }
      consoleNote_P(PSTR("Current log stamp is "));
      consolePrintLn(nvState.logStamp);  
      break;

    case c_model:
      if(numParams > 0) {
	if(param[0] > maxModels()-1)
	  param[0] = maxModels()-1;
	setModel(param[0], true);
	storeNVState();
      } else { 
	consoleNote_P(PSTR("Current model is "));
	consolePrintLn(nvState.model); 
      }
      break;
    
    case c_trim:
      if(numParams > 0)
	vpControl.elevTrim = param[0]/100;
      consoleNote_P(PSTR("Current elev trim(%) = "));
      consolePrintLn(vpControl.elevTrim*100); 
      break;
      
    case c_params:
     consoleNote_P(PSTR("SETTINGS (MODEL "));
      consolePrint(nvState.model);
      consolePrintLn(")");
      printParams();
      break;

    case c_delete:
      if(numParams > 0) {
	if(param[0] > maxModels()-1)
	  param[0] = maxModels()-1;
	deleteModel(param[0]);
      }
      break;

    case c_curve:
      consoleNoteLn_P(PSTR("Feed-forward curve"));
  
      for(float aR = -1; aR <= 1; aR += 0.07)
	printCoeffElement(-1, 1, vpParam.alphaMax*aR*RADIAN, alphaPredictInverse(vpParam.alphaMax*aR));

      consoleNoteLn_P(PSTR("Inverse feed-forward curve"));
  
      for(float e = 1; e >= -1; e -= 0.07)
	printCoeffElement(-vpParam.alphaMax/2, vpParam.alphaMax, e, alphaPredict(e));

      consoleNoteLn_P(PSTR("Coeff of lift"));
  
      for(float aR = -0.3; aR <= 1.02; aR += 0.05)
	printCoeffElement(-0.2, 1, vpParam.alphaMax*aR*RADIAN,
			  coeffOfLift(vpParam.alphaMax*aR)/vpDerived.maxCoeffOfLift);
      break;
      
    case c_clear:
      logClear();
      break;

    case c_init:
      // logInit();
      break;

    case c_stop:
      logDisable();
      break;

    case c_start:
      logEnable();
      break;

    case c_log:
      vpMode.loggingSuppressed = false;
      break;

    case c_zl:
      if(numParams > 0) {
	vpParam.cL_B = vpDerived.maxCoeffOfLift/(vpParam.alphaMax - param[0]/RADIAN);
	vpParam.cL_A = -vpParam.cL_B*param[0]/RADIAN;
      }
      break;
      
    case c_peak:
      if(numParams > 0)
	vpParam.cL_B =
	  (1+param[0])*(vpDerived.maxCoeffOfLift - vpParam.cL_A)/vpParam.alphaMax;
      break;
      
    case c_stall:
      if(numParams > 0) {
	vpParam.cL_apex = G * vpDerived.totalMass / dynamicPressure(param[0]);
	if(numParams > 1)
	  vpParam.alphaMax = param[1]/RADIAN;
      }
      break;
      
    case c_max:
      if(numParams > 0)
	vpParam.alphaMax = param[0]/RADIAN;
      break;
      
    case c_report:
      consoleNote_P(PSTR("Idle avg = "));
      consolePrintLn(idleAvg*100,1);
      consoleNote_P(PSTR("PPM frequency = "));
      consolePrint(ppmFreq);
      consolePrint_P(PSTR(" channels = "));
      consolePrintLn(ppmNumChannels);
      consoleNote_P(PSTR("Sim link frequency = "));
      consolePrintLn(simInputFreq);
      consoleNote_P(PSTR("Alpha = "));
      consolePrint(vpFlight.alpha*RADIAN);
      consolePrint_P(PSTR(" (field = "));
      consolePrint(fieldStrength*100);
      consolePrint_P(PSTR("%)"));
      if(vpStatus.alphaFailed)
	consolePrintLn_P(PSTR(" FAIL"));
      else
	consolePrintLn_P(PSTR(" OK"));

      consoleNoteLn_P(PSTR("Sensor entropy"));
      consoleNote_P(PSTR("  Alpha = "));
      consolePrint(alphaEntropyAcc.output());
      consolePrint_P(PSTR("  IAS = "));
      consolePrintLn(iasEntropyAcc.output());

      consoleNote_P(PSTR("Warning flags :"));
      if(pciWarn)
	consolePrint_P(PSTR(" SPURIOUS_PCINT"));
      if(ppmWarnShort)
	consolePrint_P(PSTR(" PPM_SHORT"));
      if(ppmWarnSlow)
	consolePrint_P(PSTR(" PPM_SLOW"));
      if(eepromDevice.warning())
	consolePrint_P(PSTR(" EEPROM_FAILED"));
      if(vpStatus.pitotFailed)
	consolePrint_P(PSTR(" IAS_FAILED"));
      
      consolePrintLn("");

      consoleNote_P(PSTR("Log write bandwidth = "));
      consolePrint(logBandWidth);
      consolePrintLn_P(PSTR(" bytes/sec"));
      
      break;
      
    case c_reset:
      pciWarn = ppmWarnShort = ppmWarnSlow = false;
      consoleNoteLn_P(PSTR("Warning flags reset"));
      break;

    case c_function:
      if(numParams > 1 && param[0] >= 0 && param[0] < MAX_SERVO) {
	function_t fn = fn_invalid;
	
	switch(paramText[1][0]) {
	case 'L':
	  fn = fn_leftaileron;
	  break;
	case 'R':
	  fn = fn_rightaileron;
	  break;
	case 'c':
	  fn = fn_leftcanard;
	  break;
	case 'C':
	  fn = fn_rightcanard;
	  break;
	case 'v':
	  fn = fn_leftelevon;
	  break;
	case 'V':
	  fn = fn_rightelevon;
	  break;
	case 'f':
	  fn = fn_leftflap;
	  break;
	case 'F':
	  fn = fn_rightflap;
	  break;
	case 't':
	  fn = fn_lefttail;
	  break;
	case 'T':
	  fn = fn_righttail;
	  break;
	case 'y':
	  fn = fn_leftthrustvert;
	  break;
	case 'Y':
	  fn = fn_rightthrustvert;
	  break;
	case 'x':
	  fn = fn_thrusthoriz;
	  break;
	case 'a':
	  fn = fn_aileron;
	  break;
	case 'e':
	  fn = fn_elevator;
	  break;
	case 'r':
	  fn = fn_rudder;
	  break;
	case 'g':
	  fn = fn_gear;
	  break;
	case 'b':
	  fn = fn_brake;
	  break;
	case 's':
	  fn = fn_steering;
	  break;
	case 'p':
	  fn = fn_throttle;
	  break;
	case '-':
	  fn = fn_null;
	  break;
	}

	if(fn != fn_invalid)
	  vpParam.functionMap[(uint8_t) param[0]] = fn;
	else
	  consoleNoteLn_P(PSTR("Invalid function"));
      } else {
	consoleNoteLn_P(PSTR("SERVO  FUNCTION"));
	consoleNoteLn_P(PSTR("---------------------"));

	for(int i = 0; i < MAX_SERVO; i++) {
	  consoleNote_P(PSTR("  "));
	  consolePrint(i);
	  consoleTab(10);

	  switch(vpParam.functionMap[i]) {
	  case fn_leftaileron:
	    consolePrintLn_P(PSTR("aileron (left)"));
	    break;
	  case fn_rightaileron:
	    consolePrintLn_P(PSTR("aileron (right)"));
	    break;
	  case fn_leftflap:
	    consolePrintLn_P(PSTR("flap (left)"));
	    break;
	  case fn_rightflap:
	    consolePrintLn_P(PSTR("flap (right)"));
	    break;
	  case fn_leftcanard:
	    consolePrintLn_P(PSTR("canard (left)"));
	    break;
	  case fn_rightcanard:
	    consolePrintLn_P(PSTR("canard (right)"));
	    break;
	  case fn_lefttail:
	    consolePrintLn_P(PSTR("tail (left)"));
	    break;
	  case fn_righttail:
	    consolePrintLn_P(PSTR("tail (right)"));
	    break;
	  case fn_leftthrustvert:
	    consolePrintLn_P(PSTR("vertical thrust (left)"));
	    break;
	  case fn_rightthrustvert:
	    consolePrintLn_P(PSTR("vertical thrust (right)"));
	    break;
	  case fn_leftelevon:
	    consolePrintLn_P(PSTR("elevon (left)"));
	    break;
	  case fn_rightelevon:
	    consolePrintLn_P(PSTR("elevon (right)"));
	    break;
	  case fn_aileron:
	    consolePrintLn_P(PSTR("aileron"));
	    break;
	  case fn_elevator:
	    consolePrintLn_P(PSTR("elevator"));
	    break;
	  case fn_rudder:
	    consolePrintLn_P(PSTR("rudder"));
	    break;
	  case fn_throttle:
	    consolePrintLn_P(PSTR("throttle"));
	    break;
	  case fn_gear:
	    consolePrintLn_P(PSTR("landing gear"));
	    break;
	  case fn_steering:
	    consolePrintLn_P(PSTR("nose wheel"));
	    break;
	  case fn_brake:
	    consolePrintLn_P(PSTR("brake"));
	    break;
	  case fn_thrusthoriz:
	    consolePrintLn_P(PSTR("horizontal thrust"));
	    break;
	  case fn_null:
	    consolePrintLn_P(PSTR("---"));
	    break;
	  default:
	    consolePrintLn_P(PSTR("<invalid>"));
	    break;
	  }
	}
      }
      break;

    default:
      consolePrint_P(PSTR("Sorry, command not implemented: \""));
      consolePrint(buf);
      consolePrintLn("\"");
      break;
    }
  }
}

//
// Periodic tasks
//

void cacheTask()
{
  cacheFlush();
  
  if(paramsModified) {
    storeParams();
    paramsModified = false;
  }
}

void alphaTask()
{
  int16_t raw = 0;
  static int16_t prev = 0;
  
  if(alphaDevice.online() && alphaDevice.invoke(AS5048B_alpha(&raw))) {
    alphaFilter.input(CIRCLE*(float) raw / (1L<<(8*sizeof(raw))));
    alphaEntropy += ABS(raw - prev);
    sensorHash = crc16(sensorHash, (uint8_t*) &raw, sizeof(raw));
    prev = raw;
  }
}

void tocReportDisplay(bool result, int i, const char *s)
{
  obdMove((i % 3)*6, i/3 + 2);
  
  if(!result) {
    obdPrint(s, true);
    tocStatusFailed = true;
  } else
    obdPrint("     ");
}

void displayTask()
{
  static bool cleared = false;
  static int count = 0;
    
  count++;
  
  if(vpStatus.silent) {
    if(!cleared) {
      obdClear();
      cleared = true;
    }
    
    return;    
  } else
    cleared = false;

  // Model name
  
  obdMove(0, 0);
  obdPrint(vpParam.name);
  obdPrint("\n");

  // Status
  
  if(!vpStatus.armed) {
    obdMove(16-8, 0);
    obdPrint("DISARMED", true);
    return;
  } else if(vpMode.takeOff) {
    obdMove(16-7, 0);
    obdPrint("TAKEOFF", (count>>2) & 1);
  } else {
    char buffer[] =
      { (char) (nvState.testNum < 10 ? ' ' : ('0' + nvState.testNum / 10)),
	(char) ('0' + nvState.testNum % 10),
	' ',
	vpFlight.alpha > 0 ? '/' : '\\',
	'\0' };
    obdMove(16-strlen(buffer), 0);
    obdPrint(buffer);
  }

  // T/O/C test status

  tocTestStatus(tocReportDisplay);

  obdMove(0,7);
  obdPrint("T/O/C ");

  if(tocStatusFailed)
    obdPrint("WARNING", (count>>2) & 1);
  else
    obdPrint("GOOD   ");
}

void airspeedTask()
{
  int16_t raw = 0;
  static int16_t prev = 0;
  
  if(pitotDevice.online() && pitotDevice.invoke(MS4525DO_pressure(&raw))) {
    pressureBuffer.input((float) raw);
    iasEntropy += ABS(raw - prev);
    sensorHash = crc16(sensorHash, (uint8_t*) &raw, sizeof(raw));
    prev = raw;
  }
}

DelayLine elevDelay, aileDelay;
Derivator elevDeriv, aileDeriv;
const float maxSlope_c = 0.75*CONTROL_HZ, abruptDelay_c = 0.15;
uint32_t lastAbruptInput;
bool inputDelayed;
Derivator buttonSlope;
float lazyButtonValue;

void configurationTask();

#define NZ_BIG RATIO(10/100)
#define NZ_SMALL RATIO(5/100)

void receiverTask()
{
  if(inputValid(&aileInput))
    vpInput.aile = applyNullZone(inputValue(&aileInput), NZ_BIG, &vpInput.ailePilotInput);

#if RX_CHANNELS >= 8
  if(inputValid(&rudderInput))
    vpInput.rudder = applyNullZone(inputValue(&rudderInput), NZ_SMALL, &vpInput.rudderPilotInput);
#else
  vpInput.rudder = 0;
#endif
  
  if(inputValid(&elevInput))
    vpInput.elev = applyNullZone(inputValue(&elevInput), NZ_SMALL, &vpInput.elevPilotInput);

  vpInput.elevExpo = applyExpo(vpInput.elev);
  
  if(inputValid(&tuningKnobInput))
    vpInput.tuningKnob = inputValue(&tuningKnobInput)*1.05 - 0.05;
    
  if(inputValid(&throttleInput))
    vpInput.throttle = inputValue(&throttleInput);

  flightModeSelectorValue = readSwitch(&flightModeSelector);

#if RX_CHANNELS >= 8
  stabModeSelectorValue = readSwitch(&stabModeSelector);
#else
  stabModeSelectorValue = 1;
#endif

  // Button input

  float buttonValue = inputValue(&buttonInput);
  
  buttonSlope.input(buttonValue, 1);

  if(fabs(buttonSlope.output()) < 0.03)
    lazyButtonValue = buttonValue;
     
  LEVELBUTTON.input(lazyButtonValue);
  FLAPBUTTON.input(lazyButtonValue);
  TRIMBUTTON.input(lazyButtonValue);
  GEARBUTTON.input(lazyButtonValue);

  //
  // Receiver fail detection
  //
  
  if(LEVELBUTTON.state()
     && vpInput.throttle < 0.1 && vpInput.aile < -0.90 && vpInput.elev > 0.90
     && flightModeSelectorValue == -1) {
    if(!vpMode.radioFailSafe) {
      consoleNoteLn_P(PSTR("Radio failsafe mode ENABLED"));
      vpMode.radioFailSafe = true;
      vpMode.alphaFailSafe = vpMode.sensorFailSafe = vpMode.takeOff = false;
      // Allow the config task to react synchronously
      configurationTask();
    }
  } else if(vpMode.radioFailSafe) {
    consoleNoteLn_P(PSTR("Radio failsafe mode DISABLED"));
    vpMode.radioFailSafe = false;
  }

  // Delay the controls just to make sure we always detect the failsafe
  // mode before doing anything abrupt

  elevDeriv.input(vpInput.elev, controlCycle);
  aileDeriv.input(vpInput.aile, controlCycle);

  if(elevDeriv.output() > maxSlope_c || aileDeriv.output() > maxSlope_c) {
    // We're seeing an abrupt change, apply delay from now on
    
    if(!inputDelayed) {
      consoleNoteLn_P(PSTR("Seeing ABRUPT aile/elev input, delay applied"));
      elevDelay.setDelay(abruptDelay_c*CONTROL_HZ);
      aileDelay.setDelay(abruptDelay_c*CONTROL_HZ);
      inputDelayed = true;
    }
    
    lastAbruptInput = currentTime;

  } else if(inputDelayed && currentTime - lastAbruptInput > abruptDelay_c*1e6) {
    // No abrupt changes for a while, remove the delay
    
    consoleNoteLn_P(PSTR("Aile/elev input seems SMOOTH"));
    elevDelay.setDelay(0);
    aileDelay.setDelay(0);
    inputDelayed = false;
  }
  
  vpInput.elev = elevDelay.input(vpInput.elev);
  vpInput.aile = aileDelay.input(vpInput.aile);
}

void sensorTaskFast()
{
  // Alpha input
  
  vpFlight.alpha = alphaFilter.output();
  
  // Dynamic pressure, corrected for alpha
  
  const float pascalsPerPSI_c = 6894.7573, range_c = 2*1.1;
  const float factor_c = pascalsPerPSI_c * range_c / (1L<<(8*sizeof(uint16_t)));
    
  vpFlight.dynP = pressureBuffer.output() * factor_c
    / cos(clamp(vpFlight.relWind, vpDerived.zeroLiftAlpha, vpParam.alphaMax));
  
  // Attitude

  ins.wait_for_sample();
  
  ahrs.update();

  vpFlight.bank = ahrs.roll;
  vpFlight.pitch = ahrs.pitch;
  vpFlight.heading = (360 + (int) (ahrs.yaw*RADIAN)) % 360;
  
  // Angular velocities
  
  Vector3f gyro = ins.get_gyro();
  
  vpFlight.rollR = gyro.x;
  vpFlight.pitchR = gyro.y;
  vpFlight.yawR = gyro.z;

  // Acceleration
  
  Vector3f acc = ins.get_accel(0);

  vpFlight.accX = acc.x;
  vpFlight.accY = acc.y;
  vpFlight.accZ = -acc.z;

  ball.input(vpFlight.accY);
  
  // Altitude data acquisition

  barometer.update();
  barometer.accumulate();

  // Compass

#ifdef USE_COMPASS
  compass.accumulate();
#endif
  
  // Simulator link overrides
  
  if(vpStatus.simulatorLink) {
    vpFlight.alpha = sensorData.alpha/RADIAN;
    vpFlight.IAS = sensorData.ias*1852/60/60;
    vpFlight.rollR = sensorData.rrate;
    vpFlight.pitchR = sensorData.prate;
    vpFlight.yawR = sensorData.yrate;
    vpFlight.bank = sensorData.roll/RADIAN;
    vpFlight.pitch = sensorData.pitch/RADIAN;
    vpFlight.heading = (int) (sensorData.heading + 0.5);
    vpFlight.accX = sensorData.accx*FOOT;
    vpFlight.accY = sensorData.accy*FOOT;
    vpFlight.accZ = -sensorData.accz*FOOT;
    
    vpFlight.dynP = dynamicPressure(vpFlight.IAS);
    
  } else 
    vpFlight.IAS = dynamicPressureInverse(vpFlight.dynP);

  //
  // Derived values
  //
    
  iasFilter.input(vpFlight.IAS);
  iasFilterSlow.input(vpFlight.IAS);
  vpFlight.slope = vpFlight.alpha - vpParam.offset - vpFlight.pitch;
}

void sensorTaskSlow()
{
  // Altitude

  if(vpStatus.simulatorLink)
    vpFlight.alt = sensorData.alt*FOOT;
  else
    vpFlight.alt = (float) barometer.get_altitude();

  // Compass

#ifdef USE_COMPASS
  compass.read();
#endif

  // Alpha sensor field strength

  uint16_t raw = 0;
  
  if(alphaDevice.online() && alphaDevice.invoke(AS5048B_field(&raw))) {
    fieldStrength = (float) raw / (1L<<16);
  }
}

void fastLogTask()
{
  //  logAlpha();  
}

void slowLogTask()
{
  logAlpha();  
  logFlight();
  logInput();
  logActuator();
  logConfig();
  logPosition();
}

void measurementTask()
{
  static uint32_t prevMeasurement;
 
  // Idle measurement

  idleAvg = 7*idleAvg/8 + (float) idleMicros/1e6/8;
  idleMicros = 0;

  // PPM monitoring
  
  FORBID;
  ppmFreq = 1.0e6 * ppmFrames / (currentTime - prevMeasurement);
  ppmFrames = 0;
  PERMIT;

  // Sim link monitoring

  simInputFreq = 1.0e6 * simFrames / (currentTime - prevMeasurement);
  simFrames = 0;

  // Log bandwidth

  logBandWidth = 1.0e6 * writeBytesCum / (currentTime - prevMeasurement);
  writeBytesCum = 0;
  
  prevMeasurement = currentTime;

  // PPM monitoring

  if(ppmWarnSlow || ppmWarnShort) {
    lastPPMWarn = currentTime;
    ppmWarnSlow = ppmWarnShort = false;
  }
}

//
//
//

const int paramSteps = 20;
const float testRange_c = 5;

static float testGainExpoGeneric(float range, float param)
{
  static float state;
  return exp(log(testRange_c)*(1.3*quantize(param, &state, paramSteps)-0.3))*range;
}

float testGainExpo(float range)
{
  return testGainExpoGeneric(range, vpInput.tuningKnob);
}

float testGainExpoReversed(float range)
{
  return testGainExpoGeneric(range, 1 - vpInput.tuningKnob);
}

float testGainLinear(float start, float stop)
{
  static float state;
  float q = quantize(vpInput.tuningKnob, &state, paramSteps);
  return start + q*(stop - start);
}

float s_Ku_ref, i_Ku_ref;

static void failsafeDisable()
{
  if(vpMode.alphaFailSafe || vpMode.sensorFailSafe) {
    consoleNoteLn_P(PSTR("Alpha/Sensor failsafe DISABLED"));
    vpMode.alphaFailSafe = vpMode.sensorFailSafe = false;
  }
}

RunningAvgFilter liftFilter(CONFIG_HZ/4);

void statusTask()
{
  //
  // Entropy monitor
  //

  iasEntropyAcc.input(iasEntropy);
  alphaEntropyAcc.input(alphaEntropy);
  iasEntropy = alphaEntropy = 0;

  //
  // Random seed from hashed sensor data
  //
  
  srand(sensorHash);
  
  //
  // Alpha/IAS sensor status
  //

  vpStatus.pitotFailed = 
    vpStatus.fault == 1 || (!vpStatus.simulatorLink && !pitotDevice.online());
  vpStatus.alphaFailed = 
    vpStatus.fault == 2 || (!vpStatus.simulatorLink && !alphaDevice.online());

  //
  // Pitot block detection
  //
  
  static uint32_t iasLastAlive;

  if(vpFlight.IAS < vpDerived.minimumIAS/3 || fabsf(vpFlight.IAS - iasFilterSlow.output()) > 0.5) {
    if(vpStatus.pitotBlocked) {
      consoleNoteLn_P(PSTR("Pitot block CLEARED"));
      vpStatus.pitotBlocked = false;
    }
    
    iasLastAlive = currentTime;
  } else if(!vpStatus.simulatorLink
	    && currentTime - iasLastAlive > 10.0e6 && !vpStatus.pitotBlocked) {
    consoleNoteLn_P(PSTR("Pitot appears BLOCKED"));
    vpStatus.pitotBlocked = true;
  }
  
  //
  // Do we have positive airspeed?
  //

  static uint32_t lastIAS, lastStall, lastAlphaLocked;

  if(vpStatus.pitotFailed) {
    if(!vpStatus.positiveIAS) {
      consoleNoteLn_P(PSTR("Pitot failed, positive IAS ASSUMED"));
      vpStatus.positiveIAS = true;
    }
  } else if(iasFilter.output() < vpDerived.minimumIAS*RATIO(2/3)) {
    if(!vpStatus.positiveIAS)
      lastIAS = currentTime;
    else if(currentTime - lastIAS > 0.3e6) {
      consoleNoteLn_P(PSTR("Positive airspeed LOST"));
      vpStatus.positiveIAS = false;
    }
  } else {
    if(vpStatus.positiveIAS)
      lastIAS = currentTime;
    else if(currentTime - lastIAS > 0.3e6) {
      consoleNoteLn_P(PSTR("We have POSITIVE AIRSPEED"));
      vpStatus.positiveIAS = true;
    }
  }

  //
  // Movement detection
  //
  
  vpFlight.acc = sqrtf(square(vpFlight.accX) + square(vpFlight.accY) + square(vpFlight.accZ));
  
  accAvg.input(vpFlight.acc);

  float turnRate = sqrt(square(vpFlight.rollR) + square(vpFlight.pitchR) + square(vpFlight.yawR));
  
  bool motionDetected = (!vpStatus.pitotBlocked && vpStatus.positiveIAS)
    || turnRate > 10.0/RADIAN
    || fabsf(vpFlight.acc - accAvg.output()) > 0.5;
  
  static uint32_t lastMotion;

  if(motionDetected) {
    if(vpStatus.fullStop) {
      consoleNoteLn_P(PSTR("We appear to be MOVING"));
      vpStatus.fullStop = false;
    }
    
    lastMotion = currentTime;

  } else if(currentTime - lastMotion > 5.0e6 && !vpStatus.fullStop) {
    consoleNoteLn_P(PSTR("We have FULLY STOPPED"));
    vpStatus.fullStop = true;
    vpStatus.aloft = false;
  }

  //
  // Alpha/accel lockup detection (sensor blade detached?)
  //

  vpFlight.accDir = atan2(vpFlight.accZ, -vpFlight.accX);
  vpFlight.relWind = vpStatus.fault == 3 ? vpFlight.accDir : vpFlight.alpha - vpParam.offset;
  
  if(vpStatus.alphaFailed) {
      // Failed alpha is also unreliable
    
      vpStatus.alphaUnreliable = true;
      lastAlphaLocked = currentTime;
  } else {
    const float diff = fabsf(vpFlight.accDir - vpFlight.relWind),
      disagreement = MIN(diff, 2*PI - diff);

    if(vpMode.alphaFailSafe || vpMode.sensorFailSafe || vpMode.takeOff
       || (fabs(vpFlight.alpha) < 90/RADIAN && disagreement > 15/RADIAN)) {
      if(!vpStatus.alphaUnreliable)
	lastAlphaLocked = currentTime;
      else if(currentTime - lastAlphaLocked > 0.1e6) {
	consoleNoteLn_P(PSTR("Alpha sensor appears RELIABLE"));
	vpStatus.alphaUnreliable = false;
      }
    } else {
      if(vpStatus.alphaUnreliable)
	lastAlphaLocked = currentTime;
      else if(currentTime - lastAlphaLocked > 0.5e6) {
	consoleNoteLn_P(PSTR("Alpha sensor UNRELIABLE"));
	vpStatus.alphaUnreliable = true;
      }
    }
  }
  
  //
  // Stall detection
  //
  
  if(vpStatus.alphaUnreliable || vpMode.alphaFailSafe || vpMode.sensorFailSafe
     || vpMode.takeOff
     || vpFlight.alpha < vpParam.alphaMax/(1 + fminf(vpParam.stallMargin, 0))) {
    if(!vpStatus.stall)
      lastStall = currentTime;
    else if(currentTime - lastStall > 0.2e6) {
      consoleNoteLn_P(PSTR("Stall RECOVERED"));
      vpStatus.stall = false;
    }
  } else {
    if(vpStatus.stall)
      lastStall = currentTime;
    else if(currentTime - lastStall > 0.2e6) {
      consoleNoteLn_P(PSTR("We're STALLING"));
      vpStatus.stall = true;
    }
  }

  //
  // Below floor?
  //
  
  if(vpFlight.alt > vpParam.floor + 5 || vpParam.floor < 1) {
    if(vpStatus.belowFloor)
      consoleNoteLn_P(PSTR("We're ABOVE floor"));
    
    vpStatus.belowFloor = false;
  } else if(vpFlight.alt < vpParam.floor && !vpStatus.belowFloor) {
    vpStatus.belowFloor = true;
    consoleNoteLn_P(PSTR("We're BELOW floor altitude"));
  }

  //
  // Attitude is upright?
  //
  
  static uint32_t lastUpright;
  
  if(fabsf(vpFlight.bank) < 15/RADIAN && fabsf(vpFlight.pitch) < 15/RADIAN) {
    vpStatus.upright = true;
    lastUpright = currentTime;
  } else {
    if(!vpStatus.upright)
      lastUpright = currentTime;
    else if(currentTime - lastUpright > 0.5e6)
      vpStatus.upright = false;
  }

  //  
  // Weight on wheels?
  //

  const float
    weight = vpDerived.totalMass * G,
    lift = vpDerived.totalMass * vpFlight.accZ * cos(vpFlight.pitch),
    liftAvg = liftFilter.input(lift),
    liftExpected = coeffOfLift(vpFlight.alpha) * vpFlight.dynP,
    liftMax = vpDerived.maxCoeffOfLift * vpFlight.dynP;
      
  static uint32_t lastWoW;
  
  if(vpMode.alphaFailSafe || vpMode.sensorFailSafe || vpMode.radioFailSafe
     || vpStatus.alphaUnreliable || vpStatus.pitotFailed
     || !vpParam.haveGear || gearSel == 1 || !vpStatus.upright
     || vpFlight.IAS > vpDerived.minimumIAS*RATIO(3/2)) {
    if(vpStatus.weightOnWheels) {
      consoleNoteLn_P(PSTR("Weight assumed to be OFF THE WHEELS"));
      vpStatus.weightOnWheels = false;
    }
      
    lastWoW = currentTime;
  } else if(vpStatus.positiveIAS
	    && (liftAvg < weight/2 || liftAvg > 1.5*weight
		|| lift < liftExpected + liftMax/3)) {
    if(!vpStatus.weightOnWheels)
      lastWoW = currentTime;
    else if(currentTime - lastWoW > 0.3e6) {
      consoleNoteLn_P(PSTR("Weight is probably OFF THE WHEELS"));
      vpStatus.weightOnWheels = false;
    }
  } else {
    if(vpStatus.weightOnWheels)
      lastWoW = currentTime;
    else if(currentTime - lastWoW > 0.5e6) {
      consoleNoteLn_P(PSTR("We seem to have WEIGHT ON WHEELS"));
      vpStatus.weightOnWheels = true;
    }
  }
}
  
void configurationTask()
{
  //
  // Being armed?
  //
  
  if(leftUpButton.doublePulse() && !vpStatus.armed
     && vpInput.aile < -0.90 && vpInput.elev > 0.90) {
    consoleNoteLn_P(PSTR("We're now ARMED"));
    vpStatus.armed = true;
    leftDownButton.reset();
    rightUpButton.reset();
    rightDownButton.reset();
    tocTestReset();
  }

  // We skip the rest unless we're armed

  if(!vpStatus.armed)
    return;
  
  //
  // T/O config test
  //

  if(!vpStatus.silent)
    tocTestUpdate();

  //
  //   GEAR BUTTON
  //
  
  if(GEARBUTTON.doublePulse()) {
    //
    // DOUBLE PULSE: FAILSAFE MODE SELECT
    //
    
    if(!vpMode.alphaFailSafe) {
      consoleNoteLn_P(PSTR("Alpha FAILSAFE"));
      vpMode.alphaFailSafe = true;
      logMark();
      
    } else if(!vpMode.sensorFailSafe) {
      consoleNoteLn_P(PSTR("Total sensor FAILSAFE"));
      vpMode.sensorFailSafe = true;
      logMark();
      
    } else if(!vpStatus.positiveIAS)
      logDisable();
    
  } else if(GEARBUTTON.singlePulse()) {
    //
    // SINGLE PULSE: GEAR TOGGLE
    //

    if(vpDerived.haveRetracts) {
      gearSel = !gearSel;

      if(gearSel)
	consoleNoteLn_P(PSTR("Gear UP"));
      else
	consoleNoteLn_P(PSTR("Gear DOWN"));
    }
    
    vpMode.autoThrottle = false;

  } else if(GEARBUTTON.depressed() && !vpMode.autoThrottle) {
    //
    // CONTINUOUS: Autothrottle engage
    //
    
    if(vpMode.slowFlight && vpInput.throttle < RATIO(1/3)) {
      vpControl.minThrottle = 0;
      vpMode.autoThrottle = true;
      
    } else if(!vpMode.slowFlight && vpInput.throttle > RATIO(1/3)
	    && vpFlight.IAS > RATIO(3/2)*vpDerived.minimumIAS) {
      vpControl.targetPressure = dynamicPressure(vpFlight.IAS);
      vpControl.minThrottle = vpInput.throttle/8;
      vpMode.autoThrottle = true;
    }

    if(vpMode.autoThrottle)
      consoleNoteLn_P(PSTR("Autothrottle ENABLED"));
  }

  //
  // FLAP BUTTON
  //

  if(vpDerived.haveFlaps) {
    if(FLAPBUTTON.singlePulse() && flapSel > 0) {
      //
      // SINGLE PULSE: FLAPS UP one step
      //
    
      consoleNote_P(PSTR("Flaps RETRACTED to "));
      consolePrintLn(--flapSel);

    } else if(FLAPBUTTON.depressed() && flapSel < flapSteps_c) {
      //
      // CONTINUOUS: FLAPS DOWN one step
      //
    
      consoleNote_P(PSTR("Flaps EXTENDED to "));
      consolePrintLn(++flapSel);
    }
  }

  //
  // WING LEVELER BUTTON
  //

  if(LEVELBUTTON.singlePulse()) {
    //
    // PULSE : Takeoff mode enable
    //
  
    if(!vpStatus.positiveIAS) {
	    
      vpStatus.silent = false;

      bool prevMode = vpMode.takeOff;
      
      if(!vpMode.takeOff) {
	consoleNoteLn_P(PSTR("TakeOff mode ENABLED"));
	vpMode.takeOff = true;
      }
	
      if(tocTestStatus(tocReportConsole)) {
	consoleNoteLn_P(PSTR("T/o configuration is GOOD"));
	vpStatus.aloft = false;
      } else {
	consolePrintLn("");
	consoleNoteLn_P(PSTR("T/o configuration test FAILED"));
	vpMode.takeOff = prevMode;
      }
    }
  } else if(LEVELBUTTON.depressed()) {
    //
    // CONTINUOUS : LEVEL WINGS
    //
  
    failsafeDisable();
    
    if(!vpMode.wingLeveler && !vpInput.ailePilotInput) {
      consoleNoteLn_P(PSTR("Wing leveler ENABLED"));
      vpMode.wingLeveler = true;
    } 
  }
    
  //
  // Autothrottle disable
  //

  if(vpMode.autoThrottle && vpMode.slowFlight == (vpInput.throttle > RATIO(1/3))) {
    consoleNoteLn_P(PSTR("Autothrottle DISABLED"));
    vpMode.autoThrottle = false;
  }
  
  //
  // Logging control
  //
  
  if(vpMode.loggingSuppressed)
    logDisable();
  else if(vpMode.takeOff && vpInput.throttle > 0.90)
    logEnable();
  else if(vpStatus.aloft && !vpStatus.pitotBlocked && vpStatus.positiveIAS)
    logEnable();
  else if(vpStatus.fullStop)
    logDisable();
    
  //
  // Direct mode selector input
  //

  if(flightModeSelectorValue == -1 || vpStatus.belowFloor) {
    if(!vpMode.slowFlight)
      consoleNoteLn_P(PSTR("Slow flight mode ENABLED"));
    vpMode.slowFlight = vpMode.bankLimiter = true;
  } else {
    if(vpMode.slowFlight) {
      consoleNoteLn_P(PSTR("Slow flight mode DISABLED"));
      vpMode.slowFlight = false;
    }

    if(flightModeSelectorValue == 0) {
      if(vpMode.bankLimiter)
	consoleNoteLn_P(PSTR("Bank limiter DISABLED"));
    
      vpMode.bankLimiter = false;
    
    } else if(!vpMode.bankLimiter) {
      consoleNoteLn_P(PSTR("Bank limiter ENABLED"));
      vpMode.bankLimiter = true;
    }
  }

  //
  // Stabilizer selector input
  //

  if(stabModeSelectorValue == 1) {
    if(!vpMode.progressiveFlight)
      consoleNoteLn_P(PSTR("Progressive aileron ENABLED"));
    vpMode.progressiveFlight = true;
  } else if(vpMode.progressiveFlight) {
    consoleNoteLn_P(PSTR("Progressive aileron DISABLED"));
    vpMode.progressiveFlight = false;
  }
  
  if(stabModeSelectorValue == -1) {
    if(!vpMode.gusty)
      consoleNoteLn_P(PSTR("Gust mode ENABLED"));
    vpMode.gusty = true;
  } else if(vpMode.gusty) {
    consoleNoteLn_P(PSTR("Gust mode DISABLED"));
    vpMode.gusty = false;
  }
  
  //
  // Test mode control
  //

  if(!vpMode.test && vpInput.tuningKnob > 0.5) {
    vpMode.test = true;
    consoleNoteLn_P(PSTR("Test mode ENABLED"));

  } else if(vpMode.test && vpInput.tuningKnob < 0) {
    vpMode.test = false;
    consoleNoteLn_P(PSTR("Test mode DISABLED"));
  }

  // Wing leveler disable when stick input detected
  
  if(vpMode.wingLeveler && vpInput.ailePilotInput && fabsf(vpFlight.bank) > 15/RADIAN) {
    consoleNoteLn_P(PSTR("Wing leveler DISABLED"));
    vpMode.wingLeveler = false;
  }

  // TakeOff mode disabled when airspeed detected (or fails)

  if(vpMode.takeOff && vpStatus.positiveIAS
     && (vpFlight.IAS > vpDerived.minimumIAS || vpFlight.alpha > vpDerived.thresholdAlpha))  {
    consoleNoteLn_P(PSTR("TakeOff COMPLETED"));
    vpMode.takeOff = false;
    vpStatus.aloft = true;
    
    if(!vpStatus.consoleLink)
      vpStatus.silent = true;
  }

  //
  // Map mode to features : default
  //
  
  vpFeature.stabilizeBank = true;
  vpFeature.keepLevel = vpMode.wingLeveler;
  vpFeature.pusher = !vpMode.slowFlight;
  vpFeature.stabilizePitch = vpFeature.alphaHold =
    vpMode.slowFlight && fabs(vpFlight.bank) < 60/RADIAN;
  vpFeature.aileFeedforward = vpMode.progressiveFlight;
  vpFeature.ailePID = !vpMode.gusty;

  // Modify if taking off...
  
  if(vpMode.takeOff)
    vpFeature.pusher = vpFeature.stabilizePitch = vpFeature.alphaHold
      = vpFeature.stabilizeBank = false;

  // ... or weight is on wheels...
  
  if(vpParam.wowCalibrated && vpStatus.weightOnWheels)
    vpFeature.stabilizeBank = false;

  // ... or WoW not calibrated but wing leveling is enabled with wheels down
  
  else if(!vpParam.wowCalibrated && vpParam.haveGear
	  && gearSel == 0 && vpMode.wingLeveler)
    vpFeature.stabilizeBank = false;
  
  // ... or stalling...
  
  if(vpStatus.stall)
    vpFeature.stabilizeBank = vpFeature.keepLevel = false;

  // Disable alpha dependent stuff if the sensor fails
  
  if(vpStatus.alphaUnreliable)
    vpFeature.stabilizePitch = vpFeature.alphaHold = vpFeature.pusher = false;

  // Failsafe overrides

  if(vpMode.sensorFailSafe) {
    vpFeature.stabilizePitch = vpFeature.stabilizeBank
      = vpFeature.alphaHold = vpFeature.pusher
      = vpMode.bankLimiter = vpFeature.keepLevel = vpMode.takeOff = false;
    
    vpFeature.aileFeedforward = true;
    
  } else if(vpMode.alphaFailSafe)
    vpFeature.stabilizePitch = vpFeature.alphaHold
      = vpFeature.pusher = vpMode.takeOff = false;
  
  // Safety scaling (test mode 0)
  
  float scale = vpMode.gusty ? RATIO(2/3) : 1;
  
  if(vpMode.test && nvState.testNum == 0)
    scale = testGainLinear(RATIO(1/3), 1);
  
  // Default controller settings

  float s_Ku = scaleByIAS(vpParam.s_Ku_C, stabilityAileExp1_c);
  float i_Ku = scaleByIAS(vpParam.i_Ku_C, stabilityElevExp_c);

  if(vpFeature.ailePID)
    aileCtrl.setZieglerNicholsPID(s_Ku*scale, vpParam.s_Tu);
  else
    aileCtrl.setZieglerNicholsPI(s_Ku*scale, vpParam.s_Tu);
  
  elevCtrl.setZieglerNicholsPID(i_Ku*scale, vpParam.i_Tu);
  pushCtrl.setZieglerNicholsPID(i_Ku*scale, vpParam.i_Tu);

  if(vpMode.slowFlight)
    throttleCtrl.setZieglerNicholsPI(vpParam.at_Ku, vpParam.at_Tu);
  else
    throttleCtrl.setZieglerNicholsPI(vpParam.cc_Ku, vpParam.cc_Tu);

  outer_P = vpParam.o_P;
  rudderMix = vpParam.r_Mix;
  throttleMix = vpParam.t_Mix;
  
  aileRateLimiter.setRate(vpParam.servoRate/(90.0/2)/vpParam.aileDefl);

  //
  // Apply test mode
  //
  
  if(vpMode.test && !vpMode.takeOff) {
    switch(nvState.testNum) {
    case 1:
      // Wing stabilizer gain
         
      vpFeature.stabilizeBank = vpMode.bankLimiter = vpFeature.keepLevel = true;
      aileCtrl.setPID(vpControl.testGain = testGainExpo(s_Ku_ref), 0, 0);
      break;
            
    case 2:
      // Elevator stabilizer gain, outer loop disabled
         
      vpFeature.stabilizePitch = true;
      vpFeature.alphaHold = false;
      elevCtrl.setPID(vpControl.testGain = testGainExpo(i_Ku_ref), 0, 0);
      break;
         
    case 3:
      // Elevator stabilizer gain, outer loop enabled
         
      vpFeature.stabilizePitch = vpFeature.alphaHold = true;
      elevCtrl.setPID(vpControl.testGain = testGainExpo(i_Ku_ref), 0, 0);
      break;
         
    case 4:
      // Auto alpha outer loop gain
         
      vpFeature.stabilizePitch = vpFeature.alphaHold = true;
      outer_P = vpControl.testGain = testGainExpo(vpParam.o_P);
      break;
               
    case 8:
      // Stall behavior test
      
      vpFeature.pusher = false;
      break;
      
    case 10:
      // Aileron to rudder mix

      rudderMix = vpControl.testGain = testGainLinear(0.8, 0.0);
      break;

    case 11:
      // Autothrottle gain (Z-N)
      
      throttleCtrl.setPID(vpControl.testGain = testGainExpo(vpParam.at_Ku), 0, 0);
      break;
      
    case 12:
      // Autothrottle gain (empirical)
      
      throttleCtrl.setZieglerNicholsPI
	(vpControl.testGain = testGainExpo(vpParam.at_Ku), vpParam.at_Tu);
      break;

    case 13:
      // Disable stabilization for max roll rate test

      if(vpInput.ailePilotInput) {
	vpFeature.stabilizeBank = vpMode.bankLimiter
	  = vpFeature.keepLevel = false;
      } else {
	vpFeature.stabilizeBank = vpFeature.keepLevel = true;
	vpControl.aileNeutral = vpOutput.aile;
      }
      break;
      
    case 14:
      // Throttle to elev mix

      throttleMix = vpControl.testGain = testGainLinear(0, vpParam.t_Mix);
      break;

    }
  } else { 
    // Track s_Ku until a test is activated
    
    s_Ku_ref = s_Ku;
    i_Ku_ref = i_Ku;
  }
}

void trimTask()
{
  //
  // Trim rate
  //
    
  const float trimRateMin_c = 7.5/100, trimRateRange_c = 2*trimRateMin_c;
  const float elevTrimRate = trimRateMin_c + fabsf(vpInput.elev)*trimRateRange_c,
    steerTrimRate = trimRateMin_c + fabsf(vpInput.rudder)*trimRateRange_c;
    
  if(TRIMBUTTON.state() || vpMode.radioFailSafe) {
    //
    // Nose wheel
    //
    
    if(vpInput.rudderPilotInput && !gearSel && !vpStatus.positiveIAS) {
      vpParam.steerNeutral +=
	sign(vpParam.steerDefl)*sign(vpInput.rudder)*steerTrimRate/TRIM_HZ;
      vpParam.steerNeutral = clamp(vpParam.steerNeutral, -1, 1);
      paramsModified = true;
    }

    //
    // Elevator
    //
  
    if(vpInput.elevPilotInput)
      vpControl.elevTrim += sign(vpInput.elev) * elevTrimRate / TRIM_HZ;
  }

  //
  // Adjust for alpha-elev predictor error when moving in/out of slow flight
  //
  
  static bool prevMode;

  if(vpStatus.positiveIAS && !vpStatus.alphaUnreliable
     && prevMode != vpMode.slowFlight) {
    if(vpMode.slowFlight) {
      // Into slow flight: maintain alpha with current stick
      vpControl.elevTrim = alphaPredictInverse(vpFlight.alpha) - vpInput.elev;
    } else {
      // Maintain elevator position with current stick
      vpControl.elevTrim = vpOutput.elev - vpInput.elev;
    }
    
    consoleNote_P(PSTR("Elev trim adjusted to "));
    consolePrintLn(vpControl.elevTrim, 2);
  }

  prevMode = vpMode.slowFlight;

  //
  // Trim limits
  //
  
  if(vpMode.takeOff) {
    // Takeoff mode enabled, trim is fixed

    // const float pRatio
    //  = clamp(dynPressure / dynamicPressure(vpDerived.minimumIAS), 0, 1);
      
    vpControl.elevTrim = vpMode.slowFlight
      ? alphaPredictInverse(vpDerived.thresholdAlpha) : vpParam.takeoffTrim;
  } else
    vpControl.elevTrim =
      clamp(vpControl.elevTrim,
	    fminf(0, alphaPredictInverse(vpDerived.zeroLiftAlpha)),
	    alphaPredictInverse(vpDerived.thresholdAlpha));
}

void gaugeTask()
{
  if(gaugeCount > 0) {
    uint16_t tmp = 0;
	
    for(int g = 0; g < gaugeCount; g++) {
      switch(gaugeVariable[g]) {
      case 1:
	consolePrint_P(PSTR(" alpha = "));
	consolePrint(vpFlight.alpha*RADIAN, 1);
	consolePrint_P(PSTR(" ("));
	consolePrint(fieldStrength*100, 1);
	consolePrint_P(PSTR("%)"));
	consoleTab(25);
	consolePrint_P(PSTR(" IAS,K(m/s) = "));
	consolePrint((int) (vpFlight.IAS/KNOT));
	consolePrint_P(PSTR(" ("));
	consolePrint(vpFlight.IAS, 1);
	consolePrint_P(PSTR(")"));
	consoleTab(50);
	consolePrint_P(PSTR(" hdg = "));
	consolePrint(vpFlight.heading);
	consoleTab(65);
	consolePrint_P(PSTR(" alt = "));

	tmp = vpFlight.alt/FOOT;
	
	if(tmp < 100)
	  consolePrint(tmp);
	else
	  consolePrint(((tmp+5)/10)*10);
	
	break;

      case 2:
	consolePrint_P(PSTR(" alpha(target) = "));
	consolePrint(vpFlight.alpha*RADIAN);
	consolePrint_P(PSTR(" ("));
	consolePrint(vpControl.targetAlpha*RADIAN);
	consolePrint_P(PSTR(")"));
	consoleTab(25);
	consolePrint_P(PSTR(" vpFlight.pitchR(target) = "));
	consolePrint(vpFlight.pitchR*RADIAN, 1);
	consolePrint_P(PSTR(" ("));
	consolePrint(vpControl.targetPitchR*RADIAN);
	consolePrint_P(PSTR(")"));
	break;
	
      case 4:
	consolePrint_P(PSTR(" bank = "));
	consolePrint(vpFlight.bank*RADIAN, 2);
	consolePrint_P(PSTR(" pitch = "));
	consolePrint(vpFlight.pitch*RADIAN, 2);
	consolePrint_P(PSTR(" heading = "));
	consolePrint(vpFlight.heading);
	consolePrint_P(PSTR(" alt = "));
	consolePrint(vpFlight.alt);
	consolePrint_P(PSTR(" ball = "));
	consolePrint(ball.output(), 2);
	break;

      case 5:
	consolePrint_P(PSTR(" rollRate = "));
	consolePrint(vpFlight.rollR*RADIAN, 1);
	consolePrint_P(PSTR(" pitchRate = "));
	consolePrint(vpFlight.pitchR*RADIAN, 1);
	consolePrint_P(PSTR(" yawRate = "));
	consolePrint(vpFlight.yawR*RADIAN, 1);
	break;

      case 6:
        consolePrint_P(PSTR(" ppmFreq = "));
	consolePrint(ppmFreq);
	consolePrint_P(PSTR(" InputVec = ( "));
	for(uint8_t i = 0; i < sizeof(ppmInputs)/sizeof(void*); i++) {
	  consolePrint(inputValue(ppmInputs[i]), 2);
	  consolePrint(" ");
	}      
	consolePrint(")");
	break;

      case 7:
	consolePrint_P(PSTR(" aileStick = "));
	consolePrint(vpInput.aile);
	consolePrint_P(PSTR(" elevStick(expo) = "));
	consolePrint(vpInput.elev);
	consolePrint_P(PSTR("("));
	consolePrint(vpInput.elevExpo);
	consolePrint_P(PSTR(") thrStick = "));
	consolePrint(vpInput.throttle);
	consolePrint_P(PSTR(" rudderStick = "));
	consolePrint(vpInput.rudder);
	consolePrint_P(PSTR(" knob = "));
	consolePrint(vpInput.tuningKnob);
	break;

      case 8:
	consolePrint_P(PSTR(" aileOut(c) = "));
	consolePrint(vpOutput.aile);
	consolePrint_P(PSTR(" ("));
	consolePrint(aileCtrl.output());
	consolePrint_P(PSTR(") elevOut = "));
	consolePrint(vpOutput.elev);
	consolePrint_P(PSTR(" rudderOut = "));
	consolePrint(vpOutput.rudder);
	break;

      case 9:
	consolePrint_P(PSTR(" gain*IAS^("));
	for(float j = 0; j < 2; j += 0.5) {
	  if(j > 0)
	    consolePrint(", ");
	  consolePrint(j);
	}
	
	consolePrint_P(PSTR(") = "));
	
	for(float j = 0; j < 2; j += 0.5) {
	  if(j > 0)
	    consolePrint(", ");
	  consolePrint(vpControl.testGain*powf(vpFlight.IAS, j));
	}
	break;
	
      case 10:
	consolePrint_P(PSTR(" acc(avg) = "));
	consolePrint(vpFlight.acc);
	consolePrint_P(PSTR("("));
	consolePrint(accAvg.output());
	consolePrint_P(PSTR(") acc = ("));
	consolePrint(vpFlight.accX, 2);
	consolePrint_P(PSTR(", "));
	consolePrint(vpFlight.accY, 2);
	consolePrint_P(PSTR(", "));
	consolePrint(vpFlight.accZ, 2);
	consolePrint_P(PSTR(")"));
	break;

      case 11:
	consolePrint_P(PSTR(" alpha = "));
	consolePrint(vpFlight.alpha*RADIAN, 1);
	consoleTab(15);
	consolePrint_P(PSTR(" relWind = "));
	consolePrint(vpFlight.relWind*RADIAN, 1);
	consoleTab(30);
	consolePrint_P(PSTR(" accDir = "));
	consolePrint(vpFlight.accDir*RADIAN, 1);
	break;
	
      case 12:
	consoleNote_P(PSTR(" entropy(alpha,ias) = "));
	consolePrint(alphaEntropyAcc.output());
	consolePrint_P(PSTR(", "));
	consolePrint(iasEntropyAcc.output());
	consolePrint_P(PSTR(" hash = "));
	
	tmp = sensorHash;

	for(int i = 0; i < 16; i++) {
	  consolePrint((tmp & 1) ? "+" : " ");
	  tmp = tmp >> 1;
	}

	consolePrintLn("");
	break;
	
     case 13:
	consolePrint_P(PSTR(" alpha = "));
	consolePrint(vpFlight.alpha*RADIAN, 1);
	consoleTab(15);
	consolePrint_P(PSTR(" IAS,K(m/s) = "));
	consolePrint((int) (vpFlight.IAS/KNOT));
	consolePrint_P(PSTR(" ("));
	consolePrint(vpFlight.IAS, 1);
	consolePrint_P(PSTR(")"));
	consoleTab(40);
	consolePrint_P(PSTR(" slope = "));
	consolePrint(vpFlight.slope*RADIAN, 1);
	consoleTab(55);
	consolePrint_P(PSTR(" THR(auto) = "));
	consolePrint(throttleCtrl.output(), 2);
	break;
	
      case 14:
       consolePrint_P(PSTR(" roll_k = "));
       consolePrint(vpFlight.rollR/expo(vpOutput.aile-vpControl.aileNeutral, vpParam.expo)/vpFlight.IAS, 3);
       break;
       
      case 15:
	consolePrint_P(PSTR(" elevOutput(trim) = "));
	consolePrint(vpOutput.elev, 3);
	consolePrint_P(PSTR("("));
	consolePrint(vpControl.elevTrim, 3);
	consolePrint_P(PSTR(")"));
	break;

      }
    }

    consolePrint_P(PSTR("      "));
    consoleCR();
  }
}

void communicationTask()
{
  int len = 0;
       
  while((len = cliSerial->available()) > 0) {
    while(len-- > 0)
      datagramRxInputChar(cliSerial->read());
  }
}

/*
const int gpsBufLen = 1<<7, gpsMaxParam = 1<<5;
char gpsBuf[gpsBufLen], gpsMsg[gpsBufLen], gpsParam[gpsMaxParam+1];
int gpsBufIndex = 0, gpsMsgLen = 0;

const char *gpsParamIndex(int n)
{
  int start = 0, end = -1;
  int i = 0;

  do {
    if(end > -1)
      start = end+1;
      
    end = start;
    
    while(gpsMsg[end] != '\0' && gpsMsg[end] != ',')
      end++;
          
    if(i == n) {
      int len = min(end-start, gpsMaxParam);
      strncpy(gpsParam, &gpsMsg[start], len);
      gpsParam[len] = '\0';      
      break;
    }
      
    i++;
  } while(gpsMsg[end] != '\0');
  
  return gpsParam;
}

int hexDigit(char c)
{
  if(isdigit(c))
    return c - '0';
  else if(c >= 'A' && c <= 'F')
    return 10 + c - 'A';
  else if(c >= 'a' && c <= 'f')
    return 10 + c - 'a';
  else
    return -1;
}

bool gpsChecksum(void)
{
  if(gpsMsgLen < 3 || gpsMsg[gpsMsgLen-3] != '*')
    return false;
    
  uint8_t chkSum = hexDigit(gpsMsg[gpsMsgLen-2])*16+
    hexDigit(gpsMsg[gpsMsgLen-1]);
    
  uint8_t sum = 0;
  
  for(int i = 1; i < gpsMsgLen - 3; i++)
    sum ^= gpsMsg[i];
    
  return sum == chkSum;
}

void gpsSentence(const char *type)
{
  if(!strncmp("RMC", type, 3)) {
    if(!strcmp("A", gpsParamIndex(2))) {
      gpsFix.speed = atof(gpsParamIndex(7));
         gpsFix.track = atof(gpsParamIndex(8));
//      consoleNote("GPS speed = ");
//      consolePrintLn(gpsFix.speed);
    }    
  } else if(!strncmp("GGA", type, 3)) {
    if(atoi(gpsParamIndex(6)) > 0) {
      gpsFix.lat = atof(gpsParamIndex(2));
      gpsFix.lon = atof(gpsParamIndex(4));
      gpsFix.altitude = atof(gpsParamIndex(9));
//      consoleNote("GPS fix = ");
//      consolePrint(gpsFix.lat);
//      consolePrint(", ");
//      consolePrintLn(gpsFix.lon);
    }
  }
}

void gpsInput(const char *buf, int len)
{
  for(int i = 0; i < len; i++) {
    if(buf[i] == '\r') {
      if(!strncmp(gpsMsg, "$GP", 3) && gpsChecksum()) {
        gpsSentence(&gpsMsg[3]);
      } else
        consoleNoteLn("Corrupt GPS sentence");

      gpsMsgLen = 0;        
      gpsMsg[0] = '\0';
    } else if(buf[i] != '\n' && gpsMsgLen < gpsBufLen-1) {
      gpsMsg[gpsMsgLen++] = buf[i];
      gpsMsg[gpsMsgLen] = '\0';
    }
  }
}
*/

void gpsTask()
{
  /*
  int len = 0;
  bool dirty = false;
       
  while((len = Serial1.available()) > 0) {
    dirty = true;
    
    int spaceLeft = gpsBufLen - gpsBufIndex;
    
    if(len > spaceLeft) {
      for(int i = 0; i < len - spaceLeft; i++)
        Serial1.read();
      consoleNote("Lost ");
      consolePrintLn(len - spaceLeft);
    }
    
    len = min(len, spaceLeft);
    
    if(len > 0) {
      Serial1.readBytes(&gpsBuf[gpsBufIndex], len);
      gpsBufIndex += len;
    }
    
    if(gpsBufIndex > 0) {
//      consoleNote("Proc ");
//      consolePrintLn(gpsBufIndex);
      gpsInput(gpsBuf, gpsBufIndex);
      gpsBufIndex = 0;
    }        
  }
  */
}

//
// Control modules
//   Elevator
//

void elevatorModule()
{
  const float shakerLimit = RATIO(1/2);
  const float stickForce =
    vpMode.radioFailSafe ? 0 : fmaxf(vpInput.elev-shakerLimit, 0)/(1-shakerLimit);
  const float effMaxAlpha
    = mixValue(stickForce, vpDerived.shakerAlpha, vpDerived.pusherAlpha);
  
  vpOutput.elev =
    applyExpoTrim(vpInput.elev, vpMode.takeOff ? vpParam.takeoffTrim : vpControl.elevTrim);
  
  vpControl.targetAlpha = fminf(alphaPredict(vpOutput.elev), effMaxAlpha);

  if(vpMode.radioFailSafe)
    vpControl.targetAlpha = trimRateLimiter.input(vpControl.targetAlpha, controlCycle);
  else
    trimRateLimiter.reset(vpControl.targetAlpha);
    
  if(vpFeature.alphaHold)
    vpControl.targetPitchR = nominalPitchRateLevel(vpFlight.bank, vpControl.targetAlpha)
      + clamp(vpControl.targetAlpha - vpFlight.alpha,
	      -15/RADIAN - vpFlight.pitch,
	      clamp(vpParam.maxPitch, 30/RADIAN, 80/RADIAN) - vpFlight.pitch)
      *outer_P;

  else
    vpControl.targetPitchR = vpInput.elevExpo*PI/2;

  vpControl.elevPredict =
    mixValue(stickForce/2, alphaPredictInverse(vpControl.targetAlpha), vpOutput.elev);

  if(vpFeature.stabilizePitch) {
    elevCtrl.input(vpControl.targetPitchR - vpFlight.pitchR, controlCycle);
    
    vpOutput.elev = elevCtrl.output();

    if(vpFeature.alphaHold)
      vpOutput.elev += vpControl.elevPredict;
  } else {

    if(vpMode.radioFailSafe)
      vpOutput.elev = vpControl.elevPredict;
    
    elevCtrl.reset(vpOutput.elev - vpControl.elevPredict, 0.0);
      
    // Pusher

#ifdef HARD_PUSHER
    pushCtrl.limit(0, vpOutput.elev);
#else
    pushCtrl.limit(-vpOutput.elev, 0);
#endif

    vpControl.pusher = 0;
    
    if(vpFeature.pusher) {
      // Pusher active
        
      const float target = nominalPitchRate(vpFlight.bank, vpFlight.pitch, vpControl.targetAlpha)
	+ (effMaxAlpha - vpFlight.alpha)*outer_P;

      pushCtrl.input(target - vpFlight.pitchR, controlCycle);

#ifdef HARD_PUSHER
      vpControl.pusher = fminf(pushCtrl.output() - vpOutput.elev, 0);
#else
      vpControl.pusher = pushCtrl.output();
#endif

      vpOutput.elev += vpControl.pusher;
    } else
#ifdef HARD_PUSHER
      pushCtrl.reset(vpOutput.elev, 0.0);
#else
      pushCtrl.reset(0, 0.0);
#endif
  }
}

//
//   Aileron
//

void aileronModule()
{
  float maxBank = 45/RADIAN;

  if(vpMode.radioFailSafe) {
    maxBank = 15/RADIAN;
    if(vpStatus.stall)
      vpInput.aile = 0;
  } else if(vpFeature.alphaHold)
    maxBank /= 1 + alphaPredict(vpControl.elevTrim) / vpDerived.thresholdAlpha / 2;
  
  float targetRollR = rollRatePredict(vpInput.aile);
  
  // We accumulate individual contributions so start with 0

  vpOutput.aile = 0;
  
  if(vpFeature.stabilizeBank) {
    // Stabilization is enabled
    
    if(vpFeature.keepLevel)
      // Strong leveler enabled
      
      targetRollR = outer_P * (vpInput.aile*60/RADIAN - vpFlight.bank);

    else if(vpMode.bankLimiter) {

      if(vpMode.slowFlight)
	// Weak leveling
	targetRollR -= outer_P*clamp(vpFlight.bank, -2.0/RADIAN, 2.0/RADIAN);

      // Bank limiter
      
      targetRollR =
	clamp(targetRollR,
	      (-maxBank - vpFlight.bank)*outer_P, (maxBank - vpFlight.bank)*outer_P);
    }

    aileCtrl.input(targetRollR - vpFlight.rollR, controlCycle);
  } else {
    // Stabilization disabled
    
    aileCtrl.reset(0, 0);
    
    if(vpFeature.keepLevel)
      // Simple proportional wing leveler
      vpOutput.aile -= vpFlight.bank + vpFlight.rollR/32;
  }

  //   Apply controller output + feedforward
  
  vpControl.ailePredict =
    vpFeature.aileFeedforward ? rollRatePredictInverse(targetRollR) : 0;
  
  vpOutput.aile += vpControl.ailePredict + aileCtrl.output();

  //   Constrain & rate limit
  
  vpOutput.aile
    = aileRateLimiter.input(constrainServoOutput(vpOutput.aile), controlCycle);
}

//
//   Rudder & nose wheel
//

void rudderModule()
{
  vpOutput.rudder = vpInput.rudder;

  if(vpDerived.haveRetracts && gearSel)
    vpOutput.steer = 0;
  else
    vpOutput.steer = vpInput.rudder;
}

//
//   Autothrottle
//
  
void throttleModule()
{
  throttleCtrl.limit(vpControl.minThrottle, vpInput.throttle);
    
  if(vpMode.autoThrottle) {
    float thrError = 0;
    
    if(vpMode.slowFlight)
      thrError = vpFlight.slope - vpParam.glideSlope*(RATIO(3/2) - 3*vpInput.throttle);
    else
      thrError = 1 - vpFlight.dynP/vpControl.targetPressure;

    throttleCtrl.input(thrError, controlCycle);

  } else
    throttleCtrl.reset(vpInput.throttle, 0);
}

//
//   Thrust vectoring
//

void vectorModule()
{
  if(vpMode.slowFlight)
    vpOutput.thrustVert = vpOutput.thrustHoriz = 0;
  else {
    vpOutput.thrustVert = vpInput.elev + vpControl.pusher;
    vpOutput.thrustHoriz = vpInput.rudder;
  }
}

void ancillaryModule()
{
  //
  // Flaps
  //
  
  flapActuator.input((float) flapSel/flapSteps_c, controlCycle);

  //
  // Brake
  //
    
  if(gearSel == 1 || vpInput.elev > 0)
    vpOutput.brake = 0;
  else
    vpOutput.brake = -vpInput.elev;
}

//
// List of control modules in no particular order
//

void (*controlModules[])(void) = {
  elevatorModule,
  aileronModule,
  rudderModule,
  throttleModule,
  vectorModule,
  ancillaryModule };

//
// Final mixing
//

void mixingTask()
{
  // Throttle to elev mix
  
  vpOutput.elev =
    constrainServoOutput(vpOutput.elev +
			 throttleMix*powf(throttleCtrl.output(),
					  vpParam.t_Expo));

  // Aile to rudder mix
  
  vpOutput.rudder =
    constrainServoOutput(vpOutput.rudder + vpOutput.aile*rudderMix);  
  // constrainServoOutput(rudderOutput + aileOutput*rudderMix*coeffOfLift(alpha)/vpDerived.maxCoeffOfLift);  
}

void controlTask()
{
  //
  // Cycle time bookkeeping 
  //
  
  static uint32_t controlCycleEnded;
 
  if(controlCycleEnded > 0)
    controlCycle = (currentTime - controlCycleEnded)/1.0e6;
  
  controlCycleEnded = currentTime;

  //
  // Invoke individual control modules
  //

  for(uint8_t i = 0; i < sizeof(controlModules)/sizeof(void(*)()); i++)
    (*controlModules[i])();
  
  //
  // Final mixing
  //
  
  mixingTask();
}

//
// Actuator functions
//

float elevatorFn()
{
  return vpParam.elevDefl*vpOutput.elev + vpParam.elevNeutral;
}

float leftElevonFn()
{
    return vpParam.aileDefl*vpOutput.aile - vpParam.elevDefl*vpOutput.elev
      + vpParam.aileNeutral;
}

float rightElevonFn()
{
  return vpParam.aileDefl*vpOutput.aile + vpParam.elevDefl*vpOutput.elev
    + vpParam.elevNeutral;
}

float aileronFn()
{
    return vpParam.aileDefl*vpOutput.aile + vpParam.aileNeutral;
}

float flaperonFn()
{
  return vpParam.flaperon ? vpParam.flapDefl*flapActuator.output() : 0;
}

float leftAileronFn()
{
  return aileronFn() + flaperonFn();
}

float rightAileronFn()
{
  return aileronFn() - flaperonFn();
}

float rudderFn()
{
  return vpParam.rudderNeutral + vpParam.rudderDefl*vpOutput.rudder;
}

float leftTailFn()
{
  return vpParam.elevDefl*vpOutput.elev + vpParam.rudderDefl*vpOutput.rudder 
    + vpParam.elevNeutral;
}

float rightTailFn()
{
  return -vpParam.elevDefl*vpOutput.elev + vpParam.rudderDefl*vpOutput.rudder
    + vpParam.rudderNeutral;
}

float leftCanardFn()
{
  return vpParam.canardNeutral + vpParam.canardDefl*vpOutput.elev;
}

float rightCanardFn()
{
  return -leftCanardFn();
}

float leftThrustVertFn()
{
  return vpParam.vertNeutral + vpParam.vertDefl*vpOutput.thrustVert;
}

float rightThrustVertFn()
{
  return vpParam.vertNeutral - vpParam.vertDefl*vpOutput.thrustVert;
}

float thrustHorizFn()
{
  return vpParam.horizNeutral + vpParam.horizDefl*vpOutput.thrustHoriz;
}

float steeringFn()
{
  return vpParam.steerNeutral + vpParam.steerDefl*vpOutput.steer;
}

float leftFlapFn()
{
  return vpParam.flapNeutral + vpParam.flapDefl*flapActuator.output();
}

float rightFlapFn()
{
  return vpParam.flap2Neutral - vpParam.flapDefl*flapActuator.output();
}

float gearFn()
{
  return -RATIO(2/3)*(gearSel*2-1);
}

float brakeFn()
{
  return vpParam.brakeDefl*vpOutput.brake + vpParam.brakeNeutral;
}

float throttleFn()
{
  return THROTTLE_SIGN*RATIO(2/3)*(2*throttleCtrl.output() - 1);
}

float (*functionTable[])(void) = {
  [fn_null] = NULL,
  [fn_aileron] = aileronFn,
  [fn_elevator] = elevatorFn,
  [fn_rudder] = rudderFn,
  [fn_throttle] = throttleFn,
  [fn_gear] = gearFn,
  [fn_steering] = steeringFn,
  [fn_brake] = brakeFn,
  [fn_leftaileron] = leftAileronFn,
  [fn_rightaileron] = rightAileronFn,
  [fn_leftcanard] = leftCanardFn,
  [fn_rightcanard] = rightCanardFn,
  [fn_leftelevon] = leftElevonFn,
  [fn_rightelevon] = rightElevonFn,
  [fn_lefttail] = leftTailFn,
  [fn_righttail] = rightTailFn,
  [fn_leftflap] = leftFlapFn,
  [fn_rightflap] = rightFlapFn,
  [fn_leftthrustvert] = leftThrustVertFn,
  [fn_rightthrustvert] = rightThrustVertFn,
  [fn_thrusthoriz] = thrustHorizFn
};

void actuatorTask()
{
  if(!vpStatus.armed)
    return;

  for(unsigned int i = 0; i < MAX_SERVO; i++)
    if(functionTable[vpParam.functionMap[i]])
      pwmOutputWrite(i, clamp(functionTable[vpParam.functionMap[i]](), -1, 1));
}

void backgroundTask(uint32_t duration)
{
  uint32_t idleStart = currentMicros();
  
  if(!logReady(false))
    logInit(duration);
  else
    delayMicros(duration*1e3);

  idleMicros += currentMicros() - idleStart;
}

void heartBeatTask()
{
  if(!heartBeatCount && linkDownCount++ > 2)
    vpStatus.consoleLink = vpStatus.simulatorLink = false;

  if(vpStatus.simulatorLink && currentTime - simTimeStamp > 1.0e6) {
    consoleNoteLn_P(PSTR("Simulator link LOST"));
    vpStatus.simulatorLink = false;
  }    
  
  heartBeatCount = 0;
  
  if(vpStatus.consoleLink) {
    static uint32_t count = 0;

    datagramTxStart(DG_HEARTBEAT);
    datagramTxOut((uint8_t*) &count, sizeof(count));
    datagramTxEnd();
  
    count++;   
  }
}

void blinkTask()
{
  float ledRatio = vpMode.test ? 0.0 : (vpMode.sensorFailSafe || !vpStatus.armed) ? 0.5 : vpFlight.alpha > 0.0 ? 0.90 : 0.10;
  static int tick = 0;
  
  tick = (tick + 1) % (LED_TICK/LED_HZ);

  setPinState(&RED_LED, tick < ledRatio*LED_TICK/LED_HZ ? 0 : 1);
}

void simulatorLinkTask()
{
  if(vpStatus.simulatorLink && vpStatus.armed) {
    struct SimLinkControl control = { .aileron = vpOutput.aile,
				      .elevator = -vpOutput.elev,
				      .throttle = throttleCtrl.output(),
				      .rudder = vpOutput.rudder };

    datagramTxStart(DG_SIMLINK);
    datagramTxOut((const uint8_t*) &control, sizeof(control));
    datagramTxEnd();
  }
}

static void logStartCallback()
{
  fastLogTask();
  slowLogTask();
}

void logSaveTask()
{
  logSave(logStartCallback);
}

void controlTaskGroup()
{
  sensorTaskFast();
  receiverTask();
  controlTask();
  actuatorTask();
}

void configTaskGroup()
{
  statusTask();
  configurationTask();
  trimTask();
}

struct Task taskList[] = {
  { communicationTask,
    HZ_TO_PERIOD(100) },
  //  { gpsTask, HZ_TO_PERIOD(100) },
  { alphaTask,
    HZ_TO_PERIOD(ALPHA_HZ) },
  { airspeedTask,
    HZ_TO_PERIOD(AIRSPEED_HZ) },
  { blinkTask,
    HZ_TO_PERIOD(LED_TICK) },
  { obdRefresh,
    HZ_TO_PERIOD(20) },
  { displayTask,
    HZ_TO_PERIOD(8) },
  { controlTaskGroup,
    HZ_TO_PERIOD(CONTROL_HZ) },
  { simulatorLinkTask,
    HZ_TO_PERIOD(CONTROL_HZ) },
  { sensorTaskSlow,
    HZ_TO_PERIOD(CONTROL_HZ/5) },
  { configTaskGroup,
    HZ_TO_PERIOD(CONFIG_HZ) },
  { fastLogTask,
    HZ_TO_PERIOD(LOG_HZ_FAST) },
  { slowLogTask,
    HZ_TO_PERIOD(LOG_HZ_SLOW) },
  { logSaveTask,
    HZ_TO_PERIOD(LOG_HZ_COMMIT) },
  { cacheTask,
    HZ_TO_PERIOD(LOG_HZ_FLUSH) },
  { measurementTask,
    HZ_TO_PERIOD(1) },
  { heartBeatTask,
    HZ_TO_PERIOD(HEARTBEAT_HZ) },
  { gaugeTask,
    HZ_TO_PERIOD(10) },
  { NULL } };

bool scheduler()
{
  struct Task *task = taskList;
  
  while(task->code) {
    if(task->lastExecuted + task->period < currentTime
      || task->lastExecuted > currentTime) {
      task->code();
      task->lastExecuted = currentTime;
      
      if(task->period > 0)
        // Staggered execution for all but the critical tasks
        return true;
    }
    
    task++;
  }

  // Nothing to do right now
  
  return false;
}

void setup()
{
  // HAL

  hal.init(0, NULL);
  
  // initialise serial port
  
  cliSerial = hal.console;
  vpStatus.consoleLink = true;
  
  consoleNoteLn_P(PSTR("Project | Alpha"));   

  // PWM output

  consoleNoteLn_P(PSTR("Initializing PWM output"));
  pwmOutputInit();

  // I2C
  
  consoleNote_P(PSTR("Initializing I2C... "));
  
  I2c.begin();
  I2c.setSpeed(true);
  I2c.pullup(false);
  I2c.timeOut(2+EXT_EEPROM_LATENCY);

  consolePrintLn_P(PSTR("done. "));
  
  // Read the non-volatile state

  readNVState();
    
  consoleNote_P(PSTR("Current model is "));
  consolePrintLn(nvState.model);
  
  // Param record
  
  setModel(nvState.model, true);
                
  // RC input
  
  consoleNoteLn_P(PSTR("Initializing PPM receiver"));

  configureInput(&ppmInputPin, true);
  
  ppmInputInit(ppmInputs, sizeof(ppmInputs)/sizeof(struct RxInputRecord*),
	       nvState.rxMin, nvState.rxCenter, nvState.rxMax);

  // Misc sensors
  
  consoleNote_P(PSTR("Initializing barometer... "));
  consoleFlush();

  barometer.init();
  barometer.calibrate();
  
  consolePrintLn_P(PSTR("  done"));
  
  consoleNote_P(PSTR("Initializing INS/AHRS... "));
  consoleFlush();
  
  ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_50HZ);
  ahrs.init();

  consolePrintLn_P(PSTR("  done"));

#ifdef USE_COMPASS
  consoleNote_P(PSTR("Initializing compass... "));
  consoleFlush();

  if(compass.init()) {
    consolePrint_P(PSTR("  done, "));
    consolePrint(compass.get_count());
    consolePrintLn_P(PSTR(" sensor(s) detected."));
  
    ahrs.set_compass(&compass);
  
    compass.set_and_save_offsets(0,0,0,0);
    compass.set_declination(ToRad(0.0f));
    
  } else {
    consolePrintLn_P(PSTR("  FAILED."));
    consoleFlush();
    while (1) ;
  }
#endif

  // LED output

  configureOutput(&RED_LED);
  configureOutput(&GREEN_LED);
  configureOutput(&BLUE_LED);

  setPinState(&RED_LED, 1);
  setPinState(&GREEN_LED, 1);
  setPinState(&BLUE_LED, 1);

  // Static controller settings

  aileCtrl.limit(RATIO(2/3));
  flapActuator.setRate(0.5);
  
  // Misc filters

  accAvg.reset(G);
  trimRateLimiter.setRate(3/RADIAN);

  // Initial gear state is DOWN
  
  gearSel = 0;

  // Done
  
  consoleNote_P(PSTR("Initialized, "));
  consolePrint((unsigned long) hal.util->available_memory());
  consolePrintLn_P(PSTR(" bytes free."));
  
  datagramTxStart(DG_INITIALIZED);
  datagramTxEnd();
}

void loop() 
{
  // Invoke scheduler
  
  currentTime = currentMicros();

  if(!scheduler())
    // Idle
      
    backgroundTask(1);
}

AP_HAL_MAIN();
