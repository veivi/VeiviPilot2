#include <string.h>
#include <limits.h>
#include "AlphaPilot.h"
#include "StaP.h"
#include "Objects.h"
#include "RxInput.h"
#include "M24XX.h"
#include "Console.h"
#include "CRC16.h"
#include "DSP.h"
#include "Math.h"
#include "NVState.h"
#include "MS4525.h"
#include "SSD1306.h"
#include "AS5048B.h"
#include "Logging.h"
#include "Button.h"
#include "TOCTest.h"
#include "Function.h"

//
//
//

void annunciatorTalk(const char *text)
{
  datagramTxStart(DG_ANNUNCIATOR);
  datagramTxOut((const uint8_t*) text, strlen(text));
  datagramTxEnd();
}

void annunciatorTalkNumber(const char *text, int num)
{
  datagramTxStart(DG_ANNUNCIATOR);
  datagramTxOut((const uint8_t*) text, strlen(text));
  datagramTxOutByte(' ');
  
  if(num < 0) {
    datagramTxOutByte('-');
    num = -num;
  }

  int weight = 1;

  while(weight*10 <= num && weight < 10000)
    weight *= 10;

  do {
    uint8_t digit = num/weight;
    datagramTxOutByte('0' + digit);
    num -= digit*weight;
    weight /= 10;
  } while(weight > 0);
  
  datagramTxEnd();
}

//
// Periodic tasks
//

void alphaSampleTask()
{
  AS5048_alpha_t raw = 0;

  if(AS5048B_isOnline() && AS5048B_alpha(&raw)) {
    samplerInput(&alphaSampler, raw);
  }
}

void gyroTask()
{
#ifdef STAP_PERIOD_GYRO
  currentTask->period = STAP_PERIOD_GYRO;  // Might not be static
#endif
  
  if(!vpStatus.simulatorLink)
    stap_gyroUpdate();
}

void attiTask()
{
  if(!vpStatus.simulatorLink)
    stap_attiUpdate();
}

void accTask()
{
  if(!vpStatus.simulatorLink)
    stap_accUpdate();
}

void tocReportDisplay(bool result, int i, const char *s)
{
  obdMove((i % 3)*6, i/3 + 2);
  
  if(!result)
    obdPrintAttr(s, true);
  else
    obdPrint("     ");
}

void displayTask()
{
  static bool cleared = false;
  static int count = 0;
  int i = 0;
    
  count++;
  
  if(vpMode.silent) {
    if(!cleared) {
      obdClear();
      cleared = true;
    }
    
    return;    
  } else
    cleared = false;

  // Model name
  
  obdMove(0, 0);
  obdPrintAttr(vpParam.name, vpStatus.consoleLink);
  obdPrint("\n");

  // CPU load
  
  obdMove(0, 1);
  uint8_t load = MIN(99, (uint8_t) (vpStatus.load*100));
  char buffer0[] =
    { '0' + (load / 10), '0' + (load % 10), '%', '\0'};
  obdPrint(buffer0);
    
  // PPM freq
  
  obdMove(DISP_COLS-5, 1);
  uint8_t freq = (uint8_t) ppmFreq;
  char buffer1[] =
    { '0' + (freq / 10), '0' + (freq % 10), ' ', 'H', 'z', '\0'};
  obdPrint(buffer1);
    
  // Status
  
  if(!vpStatus.armed) {
    obdMove(DISP_COLS-8, 0);
    obdPrintAttr("DISARMED", true);

    for(i = 0; i < MAX_CH; i++) {
      obdMove((i%3)*6, 2+i/3);
      float value = inputValue(i);
      uint8_t valueInt = MIN((uint8_t) (fabsf(value)*100), 99);
      char buffer2[] =
	{ value < 0 ? '-' : ' ',
	  '.', '0' + (valueInt / 10), '0' + (valueInt % 10),
	  '\0'};
      obdPrint(buffer2);
    }
    
    return;
  } else if(vpMode.takeOff) {
    obdMove(DISP_COLS-7, 0);
    obdPrintAttr("TAKEOFF", (count>>2) & 1);
  } else {
    char buffer[] =
      { (char) (nvState.testNum[vpMode.testCount] < 10 ? ' ' : ('0' + nvState.testNum[vpMode.testCount] / 10)),
	(char) ('0' + nvState.testNum[vpMode.testCount] % 10),
	' ',
	vpFlight.alpha > 0 ? '/' : '\\',
	' ', '\0' };
    obdMove(DISP_COLS-strlen(buffer), 0);
    obdPrint(buffer);
  }

  if(!vpMode.silent) {
    // TOC status and bottom status line
    
    bool status = tocTestStatus(tocReportDisplay);

    if(vpMode.radioFailSafe) {
      obdMove(0, DISP_ROWS-1);
      obdPrint("   ");
      obdPrintAttr("RADIO FAIL", (count>>2) & 1);
      obdPrint("   ");
    } else {
      // T/O/C test status

      obdMove(0, DISP_ROWS-1);
      obdPrint("T/O/C ");

      if(!status)
	obdPrintAttr("WARNING", (count>>2) & 1);
      else
	obdPrint("GOOD");

      obdPrint("   ");
    }
  }  
}

void airspeedSampleTask()
{
  int16_t raw = 0;

  if(MS4525DO_isOnline() && MS4525DO_pressure(&raw)) {
    // We got a good value
    samplerInput(&iasSampler, raw);
  }
}

void configTaskGroup();

#define NZ_BIG RATIO(5/100)
#define NZ_SMALL RATIO(2.5/100)

void receiverTask()
{
  stap_rxInputPoll();

  controlInputTimeStamp = ppmInputTimeStamp;
  
  if(inputValid(CH_AILE))
    vpInput.aile = applyNullZone(inputValue(CH_AILE), NZ_BIG,
				 &vpInput.ailePilotInput);
  
  if(inputValid(CH_ELEV))
    vpInput.elev = applyNullZone(inputValue(CH_ELEV), NZ_SMALL,
				 &vpInput.elevPilotInput);

  if(inputValid(CH_TUNE))
    vpInput.tuningKnob = inputValue(CH_TUNE)*1.05f - 0.05f;
    
  if(inputValid(CH_THRO))
    vpInput.throttle = inputValue(CH_THRO);

  vpInput.modeSel = readSwitch(&flightModeSelector);

#ifdef CH_RUD 
  if(inputValid(CH_RUD))
    vpInput.rudder = applyNullZone(inputValue(CH_RUD), NZ_BIG,
				   &vpInput.rudderPilotInput);
#else
  vpInput.rudder = 0;
#endif

#ifdef CH_FLAP
  vpInput.flapSel = readSwitch(&flapSelector);
#else
  vpInput.flapSel = 1;
#endif

  // Button input

  float buttonValue = inputValue(CH_BUTTON);
  
  static Derivator_t buttonSlope;
  static float lazyButtonValue;

  derivatorInput(&buttonSlope, buttonValue, 1);

  if(fabsf(derivatorOutput(&buttonSlope)) < 0.03f)
    lazyButtonValue = buttonValue;
     
  buttonInput(&LEVELBUTTON, lazyButtonValue);
  buttonInput(&RATEBUTTON, lazyButtonValue);
  buttonInput(&TRIMBUTTON, lazyButtonValue);
  buttonInput(&GEARBUTTON, lazyButtonValue);

  //
  // PPM fail detection, simulate RX failsafe if PPM fails
  //

  if(ppmFreq < 15) {
    vpInput.tuningKnob = 1;
    vpInput.modeSel = -1;
    vpInput.throttle = 0;
    vpInput.aile = -1;
    vpInput.elev = 1;
    vpInput.elevPilotInput = vpInput.ailePilotInput = true;
  }

  //
  // RX failsafe detection
  //
  
  if( vpInput.tuningKnob > 0.75f && vpInput.modeSel == -1
      && vpInput.throttle < 0.25f
      && vpInput.aile < -0.75f && vpInput.elev > 0.75f ) {
    if(!vpMode.radioFailSafe) {
      consoleNoteLn_P(CS_STRING("Radio failsafe mode ENABLED"));
      vpMode.radioFailSafe = true;
      vpMode.alphaFailSafe = vpMode.sensorFailSafe = vpMode.takeOff = false;
      // Allow the config task to react synchronously
      configTaskGroup();
    }
  } else if(vpMode.radioFailSafe) {
    consoleNoteLn_P(CS_STRING("Radio failsafe mode DISABLED"));
    vpMode.radioFailSafe = false;
  }

  //
  // Expo
  //
  
  vpInput.aileExpo = applyExpo(vpInput.aile);
  vpInput.elevExpo = applyExpo(vpInput.elev);
}

void sensorTaskSync()
{
  // Attitude

#ifndef STAP_PERIOD_GYRO
  gyroTask();
#endif
#ifndef STAP_PERIOD_ATTI
  attiTask();
#endif
#ifndef STAP_PERIOD_ACC
  accTask();
#endif

  stap_Vector3f_t acc, atti, rate;
  
  stap_sensorRead(&acc, &atti, &rate);

  vpFlight.bank = atti.x;
  vpFlight.pitch = atti.y;
  vpFlight.heading = (360 + (int) (atti.z*RADIAN)) % 360;
  
  // Angular velocities

  vpFlight.rollR = rate.x;
  vpFlight.pitchR = rate.y;
  vpFlight.yawR = rate.z;

  // Acceleration

  vpFlight.accX = acc.x;
  vpFlight.accY = acc.y;
  vpFlight.accZ = acc.z;

  // Altitude data acquisition

  stap_baroUpdate();
  
  // Air data acquisition

#ifndef ASYNC_AIR_SENSORS
  int i = 0;

  for(i = 0; i < AIR_SENSOR_OVERSAMPLE; i++) {
    alphaSampleTask();
    airspeedSampleTask();
  }
  
#endif
  
  vpFlight.alpha =
    2*PI_F * samplerMean(&alphaSampler) / (1L<<(CHAR_BIT*sizeof(int16_t)));
  
  vpFlight.relWind = vpStatus.fault == 3
    ? vpFlight.accDir : vpFlight.alpha - vpParam.alphaOffset;
  
  // Dynamic pressure, corrected for alpha

  const float pascalsPerPSI_c = 6894.7573, range_c = 2*1.1f;
  const float factor_c
    = pascalsPerPSI_c * range_c / (1L<<(CHAR_BIT*sizeof(uint16_t)));

  bool primaryIASDataIsPressure = true;
  
  float primaryIASData
    = samplerMean(&iasSampler) * factor_c
    / cosf(clamp(vpFlight.relWind, -vpDerived.maxAlpha, vpDerived.maxAlpha));
  
  // Simulator link overrides
  
  if(vpStatus.simulatorLink) {
    primaryIASDataIsPressure = false;
    primaryIASData = sensorData.ias*1852/60/60;
    
    vpFlight.alpha = sensorData.alpha/RADIAN;
    vpFlight.rollR = sensorData.rrate;
    vpFlight.pitchR = sensorData.prate;
    vpFlight.yawR = sensorData.yrate;
    vpFlight.bank = sensorData.roll/RADIAN;
    vpFlight.pitch = sensorData.pitch/RADIAN;
    vpFlight.heading = (int) (sensorData.heading + 0.5f);
    vpFlight.accX = sensorData.accx*FOOT;
    vpFlight.accY = sensorData.accy*FOOT;
    vpFlight.accZ = -sensorData.accz*FOOT;
  }

  //
  // Primary IAS data filtering & interpretation
  //
  
  primaryIASData = swAvgInput(&primaryIASDataFilter, primaryIASData);

  if(primaryIASDataIsPressure)
    vpFlight.IAS = dynamicPressureInverse(vpFlight.dynP = primaryIASData);
  else
    vpFlight.dynP = dynamicPressure(vpFlight.IAS = primaryIASData);
  
  //
  // Derived values
  //
    
  derivedValidate();
  
  vpFlight.ball = atan2f(-vpFlight.accY, fabs(vpFlight.accZ));

  vpFlight.effIAS = fmaxf(vpFlight.IAS, vpDerived.minimumIAS);
  vpFlight.effDynP = fmaxf(vpFlight.dynP, vpDerived.minimumDynP);
  vpFlight.relativeIAS = vpFlight.IAS / vpDerived.minimumIAS;
  vpFlight.relativeEffIAS = fmaxf(vpFlight.relativeIAS, 1.0f);
}

void sensorTaskSlow()
{
  // Altitude

  if(vpStatus.simulatorLink)
    vpFlight.alt = sensorData.alt*FOOT;
  else
    vpFlight.alt = stap_baroRead();

  // Alpha sensor field strength

  AS5048_word_t raw = 0;
  
  if(AS5048B_isOnline() && AS5048B_field(&raw))
    fieldStrength = (float) raw / (1L<<(CHAR_BIT*sizeof(raw)));
}

void monitorTask()
{
  static VP_TIME_MILLIS_T prevMonitor;
 
  // Load measurement

  vpStatus.load =
    mixValue(RATIO(1/4), vpStatus.load,
	     1.0f - (float) idleMicros/1000/(vpTimeMillisApprox - prevMonitor));
  
  idleMicros = 0;

  // PPM monitoring

  ppmFreq = inputSourceRate();
  
  // Sim link monitoring

  simInputFreq = 1.0e3f * simFrames/(vpTimeMillisApprox - prevMonitor);
  simFrames = 0;

  // Log bandwidth

  logBandWidth = 1.0e3f * m24xxBytesWritten/(vpTimeMillisApprox - prevMonitor);
  m24xxBytesWritten = 0;
  
  // PPM monitoring

  if(!inputSourceGood())
    ppmGoodSeconds = 0;
  else if(ppmGoodSeconds < 0xFFFF)
    ppmGoodSeconds++;
  
  // I2C errors

  uint16_t num = stap_i2cErrorCount();

  if(num > 0) {
    consoleNote("I2C errors ");
    consolePrintUI(num);
    consolePrint(" (last code ");
    consolePrintUI(stap_i2cErrorCode());
    consolePrintLn(")");
  }

  // Control latency average

  if(controlLatencyCount > 0)
    controlLatencyAvg = (float) controlLatencyTotal / controlLatencyCount;
  else
    controlLatencyAvg = 0.0f;

  controlLatencyTotal = 0;
  controlLatencyCount = 0;

  prevMonitor = vpTimeMillisApprox;
}

//
//
//

const int paramSteps = 20;
const float testRange_c = 5;

static float testGainExpoGeneric(float range, float param)
{
  static float state;
  return expf(logf(testRange_c)*(1.3f*quantize(param, &state, paramSteps)-0.3f))*range;
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

static void failsafeDisable()
{
  if(vpMode.alphaFailSafe || vpMode.sensorFailSafe) {
    consoleNoteLn_P(CS_STRING("Alpha/Sensor failsafe DISABLED"));
    vpMode.alphaFailSafe = vpMode.sensorFailSafe = false;
  }
}

VPInertiaTimer_t
  iasAliveInertia = VP_INERTIA_TIMER_CONS(&vpStatus.positiveIAS, 0.3e3),
  stallInertia = VP_INERTIA_TIMER_CONS(&vpStatus.stall, 0.01e3),
  pitotBlockInertia = VP_INERTIA_TIMER_CONS(&vpStatus.pitotBlocked, 10e3),
  fullStopInertia = VP_INERTIA_TIMER_CONS(&vpStatus.fullStop, 5e3),
  alphaFailInertia = VP_INERTIA_TIMER_CONS(&vpStatus.alphaUnreliable, 0.3e3),
  flareInertia = VP_INERTIA_TIMER_CONS(&vpStatus.flare, 0.5e3),
  canopyInertia = VP_INERTIA_TIMER_CONS(&vpStatus.canopyClosed, 0.5e3),
  uprightInertia = VP_INERTIA_TIMER_CONS(&vpStatus.upright, 0.5e3),
  wowInertia = VP_INERTIA_TIMER_CONS(&vpStatus.weightOnWheels, 0.15e3);
  
void statusTask()
{
  // Cycle time
  
  static VP_TIME_MILLIS_T statusCycleEnded;
  float statusCycle = 0.0f;
  
  if(statusCycleEnded > 0)
    statusCycle = (vpTimeMillisApprox - statusCycleEnded)/1.0e3;
  
  statusCycleEnded = vpTimeMillisApprox;

  //
  // Canopy open/closed
  //

  if(STAP_CANOPY_CLOSED) {
    if(vpInertiaOn(&canopyInertia))
      consoleNoteLn_P(CS_STRING("Canopy is CLOSED"));

  } else if(vpInertiaOff(&canopyInertia))
    consoleNoteLn_P(CS_STRING("Canopy is OPEN"));
  
  //
  // Fuel quantity & total mass
  //

  if(vpStatus.canopyClosed || vpStatus.airborne) {
    // Assume we are disconnected from the overflow tank
    
    vpStatus.fuel -=
      polynomial(FuelFlow_degree, clamp(vpInput.throttle, 0, 1),
		 vpParam.coeff_Flow)
      * statusCycle / 60; // Convert 1/min to 1/s
    
    if(vpStatus.fuel < 0)
      vpStatus.fuel = 0;
    
  } else if(vpStatus.fuel != vpParam.fuel) {
    // Assume we are being topped up
    
    vpStatus.fuel = vpParam.fuel;
    consoleNote_P(CS_STRING("Fuel assumed TOPPED UP ("));
    consolePrintFP(vpStatus.fuel, 3);
    consolePrintLn_P(CS_STRING(" units)"));
  }
  
  vpStatus.mass =
    vpParam.weightDry + vpParam.battery + vpStatus.fuel*vpParam.fuelDensity;
  
  //
  // Alpha/IAS sensor status
  //

  vpStatus.pitotFailed = 
    vpStatus.fault == 1 || (!vpStatus.simulatorLink && !MS4525DO_isOnline());
  vpStatus.alphaFailed = 
    vpStatus.fault == 2 || (!vpStatus.simulatorLink && !AS5048B_isOnline());

  //
  // Pitot block detection
  //
  
  damperInput(&avgDynP, vpFlight.dynP);

  if(vpFlight.relativeIAS < RATIO(1/3)
     || fabsf(vpFlight.dynP - damperOutput(&avgDynP)) > dynamicPressure(2.0f)) {
    if(vpInertiaOffForce(&pitotBlockInertia))
      consoleNoteLn_P(CS_STRING("Pitot block CLEARED"));

  } else if(vpInertiaOn(&pitotBlockInertia))
    consoleNoteLn_P(CS_STRING("Pitot appears BLOCKED"));
  
  //
  // Do we have positive airspeed?
  //

  if(vpStatus.pitotFailed) {
    if(!vpStatus.positiveIAS) {
      consoleNoteLn_P(CS_STRING("Pitot failed, positive IAS ASSUMED"));
      vpStatus.positiveIAS = true;
    }
  } else if(vpFlight.relativeIAS < RATIO(4/5)) {
    if(vpInertiaOff(&iasAliveInertia))
      consoleNoteLn_P(CS_STRING("Positive airspeed LOST"));
  } else if(vpInertiaOn(&iasAliveInertia))
    consoleNoteLn_P(CS_STRING("We have POSITIVE AIRSPEED"));
  
  //
  // Movement detection
  //
  
  vpFlight.acc =
    sqrtf(sqrf(vpFlight.accX) + sqrf(vpFlight.accY) + sqrf(vpFlight.accZ));
  
  damperInput(&accAvg, vpFlight.acc);

  float turnRate =
    sqrtf(sqrf(vpFlight.rollR) + sqrf(vpFlight.pitchR) + sqrf(vpFlight.yawR));
  
  bool motionDetected = (!vpStatus.pitotBlocked && vpStatus.positiveIAS)
    || turnRate > 10.0f/RADIAN
    || fabsf(vpFlight.acc - damperOutput(&accAvg)) > 0.5f;
  
  if(motionDetected) {
    if(vpInertiaOffForce(&fullStopInertia))
      consoleNoteLn_P(CS_STRING("We appear to be MOVING"));
    
  } else if(vpInertiaOn(&fullStopInertia)) {
    consoleNoteLn_P(CS_STRING("We have FULLY STOPPED"));
    vpStatus.airborne = false;
  }

  //
  // Alpha/accel lockup detection (sensor vane detached?)
  //

  vpFlight.accDir = atan2(vpFlight.accZ, -vpFlight.accX);
  
  if(vpStatus.alphaFailed) {
      // Failed alpha is also unreliable
    
      vpStatus.alphaUnreliable = true;
  } else {
    const float diff = fabsf(vpFlight.accDir - vpFlight.relWind),
      disagreement = MIN(diff, 2*PI_F - diff);

    if(vpMode.alphaFailSafe || vpMode.sensorFailSafe || vpMode.takeOff
       || fabsf(vpFlight.alpha) < vpDerived.maxAlpha
       || disagreement > 15.0f/RADIAN) {
      if(vpInertiaOff(&alphaFailInertia))
	consoleNoteLn_P(CS_STRING("Alpha sensor appears RELIABLE"));

    } else if(vpInertiaOn(&alphaFailInertia))
      consoleNoteLn_P(CS_STRING("Alpha sensor UNRELIABLE"));
  }

  //
  // Flare detection
  //

  if(!(vpMode.test && nvState.testNum[vpMode.testCount] > 0)
     && vpMode.slowFlight
     && vpControl.gearSel == 0
     && vpInput.throttle < vpParam.idle
     && vpFlight.IAS < (1 + vpParam.thresholdMargin)*vpDerived.minimumIAS
     && fabsf(vpFlight.pitch) < vpDerived.maxAlpha
     && fabsf(vpFlight.bank) < 30.0f/RADIAN
     && vpParam.flare > 0.0f
     && vpInput.stickForce > 0.0f) {
    // We may be in a flare

    if(vpInertiaOnForce(&flareInertia)) {
      consoleNoteLn_P(CS_STRING("Assumed to be FLARING"));
      annunciatorTalk("Flaring");
    }
    
  } if(vpInertiaOff(&flareInertia))
      consoleNoteLn_P(CS_STRING("Flare ended"));
  
  //
  // Stall detection
  //
  
  if(vpStatus.alphaUnreliable || vpMode.alphaFailSafe || vpMode.sensorFailSafe
     || vpMode.takeOff || vpStatus.flare
     || vpFlight.alpha < fmaxf(vpDerived.stallAlpha, vpControl.targetAlpha)) {

    if(vpInertiaOff(&stallInertia))
      consoleNoteLn_P(CS_STRING("Stall RECOVERED"));

  } else if(vpInertiaOn(&stallInertia))
    consoleNoteLn_P(CS_STRING("We're STALLING"));

  //
  // Attitude is upright?
  //
  
  if(fabsf(vpFlight.bank) < 15.0f/RADIAN && fabsf(vpFlight.pitch) < 15.0f/RADIAN)
    vpInertiaOnForce(&uprightInertia);
  else
    vpInertiaOff(&uprightInertia);

  //  
  // Weight on wheels?
  //

  const float
    weight = vpStatus.mass * G,
    lift = vpStatus.mass * vpFlight.accZ * cosf(vpFlight.pitch),
    liftAvg = swAvgInput(&liftFilter, lift),
    liftExpected = coeffOfLift(vpFlight.alpha) * vpFlight.dynP,
    liftMax = vpDerived.maxCoeffOfLift * vpFlight.dynP;
      
  if(vpMode.alphaFailSafe || vpMode.sensorFailSafe || vpMode.radioFailSafe
     || vpStatus.alphaUnreliable || vpStatus.pitotFailed
     || !vpParam.haveGear || vpControl.gearSel == 1 || !vpStatus.upright
     || vpFlight.IAS > vpDerived.minimumIAS*RATIO(3/2)) {
    if(vpInertiaOffForce(&wowInertia))
      consoleNoteLn_P(CS_STRING("Weight assumed to be OFF THE WHEELS"));

  } else if(vpStatus.positiveIAS
	    && (liftAvg < weight/2 || liftAvg > 1.5f*weight
		|| lift < liftExpected + liftMax/4)) {
    if(vpInertiaOff(&wowInertia))
      consoleNoteLn_P(CS_STRING("Weight is probably OFF THE WHEELS"));

  } else if(vpInertiaOn(&wowInertia))
    consoleNoteLn_P(CS_STRING("We seem to have WEIGHT ON WHEELS"));
}

void configurationTask()
{
  //
  // Being armed?
  //
  
  if(!vpStatus.armed && buttonDoublePulse(&TRIMBUTTON) &&
     vpInput.throttle < 0.1f && vpInput.aile < -0.9f && vpInput.elev > 0.9f) {
    consoleNoteLn_P(CS_STRING("We're now ARMED"));
    annunciatorTalk("ARMED");
    vpStatus.armed = true;
    buttonReset(&GEARBUTTON);
    buttonReset(&LEVELBUTTON);
    buttonReset(&RATEBUTTON);

    tocTestReset();
  }
  
  // We skip the rest unless we're armed

  if(!vpStatus.armed)
    return;
  
  //
  // T/O config test
  //

  if(!vpMode.silent)
    tocTestUpdate();

  //
  //   GEAR BUTTON
  //
  
  if(buttonDoublePulse(&GEARBUTTON)) {
    //
    // DOUBLE PULSE: FAILSAFE MODE SELECT
    //
    
    if(!vpMode.alphaFailSafe) {
      annunciatorTalk("Alpha failsafe");
      consoleNoteLn_P(CS_STRING("Alpha FAILSAFE"));
      vpMode.alphaFailSafe = true;
      logMark();
      
    } else if(!vpMode.sensorFailSafe) {
      annunciatorTalk("Total failsafe");
      consoleNoteLn_P(CS_STRING("Total sensor FAILSAFE"));
      vpMode.sensorFailSafe = true;
      logMark();
      
    } else if(!vpStatus.positiveIAS && !vpStatus.airborne) {
      consoleNoteLn_P(CS_STRING("T/O/C test being RESET"));
      tocTestReset();
      logDisable();
    }
    
  } else if(buttonSinglePulse(&GEARBUTTON)) {
    //
    // SINGLE PULSE: GEAR TOGGLE
    //

    if(vpDerived.haveRetracts) {
      vpControl.gearSel = !vpControl.gearSel;
      vpMode.gearSelected = true;

      if(vpControl.gearSel) {
	consoleNoteLn_P(CS_STRING("Gear UP"));
	annunciatorTalk("Gear up");
      } else {
	consoleNoteLn_P(CS_STRING("Gear DOWN"));
	annunciatorTalk("Gear down");
      }
    }
  }

  //
  // RATE BUTTON
  //

  if(buttonDepressed(&RATEBUTTON) && !vpMode.halfRate) {
    // Continuous: half-rate enable
    
    consoleNoteLn_P(CS_STRING("Half-rate ENABLED"));
    annunciatorTalk("Half rate");
    vpMode.halfRate = true;
    
  } else if(buttonSinglePulse(&RATEBUTTON) && vpMode.halfRate) {
    // Single pulse: half-rate disable
    
    consoleNoteLn_P(CS_STRING("Half-rate DISABLED"));
    annunciatorTalk("Full rate");
    vpMode.halfRate = false;
  }

  //
  // WING LEVELER BUTTON
  //

  if(buttonSinglePulse(&LEVELBUTTON)) {
    //
    // PULSE : Takeoff mode enable / increment test
    //

    if(vpStatus.airborne) {
      //
      // Airborne already, increment the test count
      //

      if(vpMode.testCount < MAX_TESTS-1
	 && nvState.testNum[vpMode.testCount+1] >= 0)
	vpMode.testCount++;
      else
	vpMode.testCount = 0;

      consoleNote_P(CS_STRING("Test number "));
      consolePrintLnI(nvState.testNum[vpMode.testCount]);
      annunciatorTalkNumber("test", nvState.testNum[vpMode.testCount]);
      
    } else if(!vpStatus.positiveIAS || vpStatus.simulatorLink) {
	    
      vpMode.silent = false;

      bool prevMode = vpMode.takeOff;
      
      if(!vpMode.takeOff) {
	consoleNoteLn_P(CS_STRING("TakeOff mode ENABLED"));
	vpMode.takeOff = true;
      }

      if(tocTestStatus(tocReportConsole)) {
	consoleNoteLn_P(CS_STRING("T/o configuration is GOOD"));
	annunciatorTalk("Takeoff mode enabled");
	vpStatus.airborne = false;
      } else {
	consolePrintLn("");
	consoleNoteLn_P(CS_STRING("T/o configuration test FAILED"));
	annunciatorTalk("Takeoff test fail");
	vpMode.takeOff = prevMode;
      }
    }
  } else if(buttonDepressed(&LEVELBUTTON)) {
    //
    // CONTINUOUS : LEVEL WINGS
    //
  
    failsafeDisable();
    
    if(!vpMode.wingLeveler && !vpInput.ailePilotInput) {
      consoleNoteLn_P(CS_STRING("Wing leveler ENABLED"));
      annunciatorTalk("Wing leveler");
      vpMode.wingLeveler = true;
    } 
  }

  //
  // Logging control
  //
  
  if(vpMode.dontLog)
    logDisable();
  else if(vpMode.takeOff && vpInput.throttle > 0.90f) {
    logEnable();
  } else if(vpStatus.airborne && !vpStatus.pitotBlocked && vpStatus.positiveIAS)
    logEnable();
  else if(vpStatus.fullStop)
    logDisable();
    
  //
  // Direct mode selector input
  //

  if(vpInput.modeSel == -1) {
    if(!vpMode.slowFlight)
      consoleNoteLn_P(CS_STRING("Slow flight mode ENABLED"));
    vpMode.slowFlight = vpMode.bankLimiter = true;
  } else {
    if(vpMode.slowFlight) {
      consoleNoteLn_P(CS_STRING("Slow flight mode DISABLED"));
      vpMode.slowFlight = false;
    }

    if(vpInput.modeSel == 0) {
      if(vpMode.bankLimiter)
	consoleNoteLn_P(CS_STRING("Bank limiter DISABLED"));
    
      vpMode.bankLimiter = false;
    
    } else if(!vpMode.bankLimiter) {
      consoleNoteLn_P(CS_STRING("Bank limiter ENABLED"));
      vpMode.bankLimiter = true;
    }
  }

  //
  // Flap selector input
  //

  static int prevFlap;
  
  if(vpDerived.haveFlaps)
    vpControl.flapSel = FLAP_STEPS/2 - vpInput.flapSel;
  else
    vpControl.flapSel = 0;
  
  if(vpControl.flapSel != prevFlap) {
    annunciatorTalkNumber("flaps", vpControl.flapSel);
    prevFlap = vpControl.flapSel;
  }
  
  //
  // Test mode control
  //

  if(vpMode.radioFailSafe)
    vpMode.test = false;
  
  else if(!vpMode.test && vpInput.tuningKnob > 0.5f) {
    vpMode.test = true;
    consoleNoteLn_P(CS_STRING("Test mode ENABLED"));
    annunciatorTalk("Test start");

    /*
    annunciatorTalkNumber("test", -25);
    annunciatorTalkNumber("test", -1);
    annunciatorTalkNumber("test", 0);
    annunciatorTalkNumber("test", 9);
    annunciatorTalkNumber("test", 10);
    annunciatorTalkNumber("test", 39);
    annunciatorTalkNumber("test", 100);
    annunciatorTalkNumber("test", 1500);
    annunciatorTalkNumber("test", 10001);
    */
  } else if(vpMode.test && vpInput.tuningKnob < 0) {
    vpMode.test = false;
    consoleNoteLn_P(CS_STRING("Test mode DISABLED"));
    annunciatorTalk("End of test");
  }

  // Wing leveler disable when stick input detected
  
  if(vpMode.wingLeveler && vpInput.ailePilotInput
     && fabsf(vpFlight.bank) > 5.0f/RADIAN) {
    consoleNoteLn_P(CS_STRING("Wing leveler DISABLED"));
    vpMode.wingLeveler = false;
  }

  // TakeOff mode disabled when airspeed detected

  if(vpInput.throttle > 0.8 && vpMode.takeOff && vpStatus.positiveIAS
     && (vpFlight.IAS > vpDerived.minimumIAS
	 || vpFlight.alpha > vpDerived.thresholdAlpha))  {
    consoleNoteLn_P(CS_STRING("We seem to be AIRBORNE"));
    annunciatorTalk("Airborne");
    vpMode.takeOff = false;
    vpStatus.airborne = true;
    vpMode.silent = true;
  }

  //
  // Map mode to features : default
  //
  
  vpFeature.stabilizeBank = true;
  vpFeature.keepLevel = vpMode.wingLeveler;
  vpFeature.pusher = !vpMode.slowFlight;
  vpFeature.stabilizePitch = vpFeature.alphaHold =
    vpMode.slowFlight && fabsf(vpFlight.bank) < 60.0f/RADIAN;

  // Modify if taking off...
  
  if(vpMode.takeOff)
    vpFeature.pusher = vpFeature.stabilizePitch = vpFeature.alphaHold
      = vpFeature.stabilizeBank = false;

  // ... or weight is on wheels...
  
  if(vpParam.wowCalibrated && vpStatus.weightOnWheels)
    vpFeature.stabilizeBank = false;

  // ... or WoW not calibrated but wing leveling is enabled with wheels down
  
  else if(!vpParam.wowCalibrated && vpParam.haveGear
	  && vpControl.gearSel == 0 && vpMode.wingLeveler)
    vpFeature.stabilizeBank = false;
  
  // ... or stalling...
  
  if(vpStatus.stall)
    vpFeature.stabilizeBank = vpFeature.keepLevel = false;

  // Disable alpha dependent stuff if the sensor fails
  
  if(vpStatus.alphaUnreliable)
    vpFeature.stabilizePitch = vpFeature.alphaHold = vpFeature.pusher = false;

  // Failsafe overrides

  if(vpMode.sensorFailSafe)
    vpFeature.stabilizePitch = vpFeature.stabilizeBank
      = vpFeature.alphaHold = vpFeature.pusher
      = vpFeature.keepLevel = vpMode.takeOff = false;
    
  else if(vpMode.alphaFailSafe)
    vpFeature.stabilizePitch = vpFeature.alphaHold
      = vpFeature.pusher = vpMode.takeOff = false;
  
  // Safety scaling (test mode 0)
  
  float scale = 1;
  
  if(vpMode.test && nvState.testNum[vpMode.testCount] == 0)
    scale = testGainLinear(RATIO(1/5), 1);
  
  // Default controller settings

  float s_Ku = scaleByIAS_E(vpParam.s_Ku_C, stabGainExp_c);
  float i_Ku = scaleByIAS_E(vpParam.i_Ku_C, stabGainExp_c);
  float r_Ku = scaleByIAS_E(vpParam.r_Ku_C, stabGainExp_c);

  //   Aileron
  
  pidCtrlSetZNPID(&aileCtrl, s_Ku*scale, vpParam.s_Tu);

  //   Elevator inner loop and pusher
  
  pidCtrlSetZNPID(&elevCtrl, i_Ku*scale, vpParam.i_Tu);
  pidCtrlSetZNPID(&pushCtrl, i_Ku*scale, vpParam.i_Tu);

  //   Rudder
  
#if AUTO_RUDDER
  pidCtrlSetZNPI(&rudderCtrl, r_Ku*scale, vpParam.r_Tu);
#endif
  
  //   Yaw damper

#if YAW_DAMPER
  vpControl.yd_P = scaleByIAS_E(vpParam.yd_C, yawDamperExp_c) * scale;
#endif
  
  //   Turbine lag

  turbineSetTau(&engine, vpParam.lag*CONTROL_HZ);

  //   Canard neutral controller
  
  pidCtrlSetPID(&canardCtrl,
		vpParam.canardGain * scale, 0, vpParam.canardGainD * scale);

  //   Misc control params
  
  vpControl.o_P = vpParam.o_P;
  vpControl.r_Mix = vpParam.r_Mix;
  vpControl.t_Mix = vpParam.t_Mix;
  slopeSet(&aileActuator, vpParam.servoRate/(90.0f/2)/vpParam.aileDefl);
  
  //
  // Apply test mode
  //
  
  if(vpMode.test && !vpMode.takeOff) {
    switch(nvState.testNum[vpMode.testCount]) {
    case 1:
      // Wing stabilizer gain
      vpFeature.stabilizeBank = vpMode.bankLimiter = vpFeature.keepLevel = true;
      pidCtrlSetPID(&aileCtrl, vpControl.testGain
		    = testGainExpo(vpControl.s_Ku_ref), 0, 0);
      break;
            
    case 2:
      // Elevator stabilizer gain, outer loop disabled
      vpFeature.stabilizePitch = true;
      vpFeature.alphaHold = false;
      pidCtrlSetPID(&elevCtrl, vpControl.testGain
		    = testGainExpo(vpControl.i_Ku_ref), 0, 0);
      break;
         
    case 3:
      // Elevator stabilizer gain, outer loop enabled
      vpFeature.stabilizePitch = vpFeature.alphaHold = true;
      pidCtrlSetPID(&elevCtrl, vpControl.testGain
		    = testGainExpo(vpControl.i_Ku_ref), 0, 0);
      break;
         
    case 4:
      // Auto alpha outer loop gain
      vpFeature.stabilizePitch = vpFeature.alphaHold = true;
      vpControl.o_P = vpControl.testGain = testGainExpo(vpParam.o_P);
      break;

    case 5:
      // Max alpha tests, just to disable flare with fixed gear planes
      break;

    case 6:
      // Yaw damper gain
      vpControl.yd_P = vpControl.testGain = testGainExpo(vpControl.yd_P_ref);
      break;

    case 7:
      // Throttle-elev mix
      vpControl.t_Mix = vpControl.testGain
	= testGainLinear(0, vpParam.t_Mix*1.5f);
      break;
      
    case 8:
      // Stall behavior i.e. pusher disabled
      vpFeature.pusher = false;
      break;
      
    case 9:
      // Auto rudder gain
#if AUTO_RUDDER
      pidCtrlSetPID(&rudderCtrl, vpControl.testGain
		    = testGainExpo(vpControl.r_Ku_ref), 0, 0);
#endif
      break;
            
    case 10:
      // Aileron to rudder mix
      vpControl.r_Mix = vpControl.testGain = testGainLinear(1, 0);
      break;

    case 11:
      // Turbine lag
      turbineSetTau(&engine,
		    CONTROL_HZ*(vpControl.testGain
				= testGainLinear(vpParam.lag*1.5f, 0)));
      break;

    case 12:
      // Canard gain
      pidCtrlSetPID(&canardCtrl, vpParam.canardGain, 0, -(vpControl.testGain = testGainLinear(0.3, 0)));
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
      // Force telemetry link status
      
      if(vpInput.tuningKnob > 0.5) {
	vpStatus.telemetryLink = true;
	linkDownCount[1] = 0;
      } else
	vpStatus.telemetryLink = false;
      break;
    }
  } else { 
    // Track s_Ku until a test is activated
    
    vpControl.s_Ku_ref = s_Ku;
    vpControl.r_Ku_ref = r_Ku;
    vpControl.i_Ku_ref = i_Ku;
    vpControl.yd_P_ref = vpControl.yd_P;
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
    
  if(buttonState(&TRIMBUTTON) || vpMode.radioFailSafe) {
    //
    // Nose wheel
    //

    if(vpInput.rudderPilotInput && !vpControl.gearSel && !vpStatus.positiveIAS) {
      vpParam.steerTrim +=
	signf(vpParam.steerDefl)*signf(vpInput.rudder)*steerTrimRate/TRIM_HZ;
      vpParam.steerTrim = clamp(vpParam.steerTrim, -1, 1);
    }

    //
    // Elevator
    //
  
    if(vpInput.elevPilotInput)
      vpControl.elevTrim += signf(vpInput.elev) * elevTrimRate / TRIM_HZ;
  }

  //
  // Adjust for alpha-elev predictor error when moving in/out of slow flight
  //
  
  static bool prevMode;

  if(vpStatus.positiveIAS && !vpStatus.alphaUnreliable
     && prevMode != vpMode.slowFlight) {
    if(vpMode.slowFlight) {
      // Into slow flight
      //   Maintain alpha with current stick
      vpControl.elevTrim += alphaPredictInverse(vpFlight.alpha)
	- alphaPredictInverse(vpControl.targetAlpha);
    } else {
      // Out of slow flight
      //   Maintain elevator position with current stick
      vpControl.elevTrim += vpOutput.elev - vpControl.elevPredict;
    }
    
    consoleNote_P(CS_STRING("Elev trim adjusted to "));
    consolePrintLnFP(vpControl.elevTrim, 2);
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
    vpStatus.trimLimited = false;
  } else 
    vpControl.elevTrim =
      clampStatus(vpControl.elevTrim,
	    fminf(-0.15f, alphaPredictInverse(vpDerived.zeroLiftAlpha)),
	    alphaPredictInverse(vpDerived.thresholdAlpha),
	    &vpStatus.trimLimited);
}

void gaugeTask()
{  
  if(gaugeCount > 0) {
    uint16_t tmp = 0;
    uint8_t i = 0, g = 0;
    float j = 0;
    int p = 0;
	
    for(g = 0; g < gaugeCount; g++) {
      switch(gaugeVariable[g]) {
      case 1:
	consolePrint_P(CS_STRING(" alpha = "));
	consolePrintFP(vpFlight.alpha*RADIAN, 1);
	consolePrint_P(CS_STRING(" ("));
	consolePrintFP(fieldStrength*100, 1);
	consolePrint_P(CS_STRING("%)"));
	consoleTab(22);
	consolePrint_P(CS_STRING(" IAS,K(m/s) = "));
	consolePrintI((int) (vpFlight.IAS/KNOT));
	consolePrint_P(CS_STRING(" ("));
	consolePrintFP(vpFlight.IAS, 1);
	consolePrint_P(CS_STRING(")"));
	consoleTab(44);
	consolePrint_P(CS_STRING(" hdg = "));
	consolePrintI(vpFlight.heading);
	consoleTab(55);
	consolePrint_P(CS_STRING(" alt = "));

	tmp = vpFlight.alt/FOOT;
	
	if(tmp < 100)
	  consolePrintI(tmp);
	else
	  consolePrintI(((tmp+5)/10)*10);
	
	break;

      case 2:
	consolePrint_P(CS_STRING(" alpha(target) = "));
	consolePrintF(vpFlight.alpha*RADIAN);
	consolePrint_P(CS_STRING(" ("));
	consolePrintF(vpControl.targetAlpha*RADIAN);
	consolePrint_P(CS_STRING(")"));
	consoleTab(25);
	consolePrint_P(CS_STRING(" vpFlight.pitchR(target) = "));
	consolePrintFP(vpFlight.pitchR*RADIAN, 1);
	consolePrint_P(CS_STRING(" ("));
	consolePrintF(vpControl.targetPitchR*RADIAN);
	consolePrint_P(CS_STRING(")"));
	break;

      case 3:
	consolePrint_P(CS_STRING(" relative IAS(eff) = "));
	consolePrintF(vpFlight.relativeIAS);
	consoleTab(30);
	consolePrintF(vpFlight.relativeEffIAS);
	consoleTab(40);
	consolePrintF(vpFlight.dynP);
	consoleTab(50);
	consolePrintF(damperOutput(&avgDynP));
	break;
	
      case 4:
	consolePrint_P(CS_STRING(" bank = "));
	consolePrintFP(vpFlight.bank*RADIAN, 2);
	consolePrint_P(CS_STRING(" pitch = "));
	consolePrintFP(vpFlight.pitch*RADIAN, 2);
	consolePrint_P(CS_STRING(" heading = "));
	consolePrintF(vpFlight.heading);
	consolePrint_P(CS_STRING(" alt = "));
	consolePrintF(vpFlight.alt);
	consolePrint_P(CS_STRING(" ball = "));
	consolePrintFP(vpFlight.ball, 2);
	break;

      case 5:
	consolePrint_P(CS_STRING(" rollRate = "));
	consolePrintFP(vpFlight.rollR*RADIAN, 1);
	consolePrint_P(CS_STRING(" pitchRate = "));
	consolePrintFP(vpFlight.pitchR*RADIAN, 1);
	consolePrint_P(CS_STRING(" yawRate = "));
	consolePrintFP(vpFlight.yawR*RADIAN, 1);
	break;

      case 6:
        consolePrint_P(CS_STRING(" ppmFreq = "));
	consolePrintF(ppmFreq);
	consolePrint_P(CS_STRING(" InputVec = ( "));
	for(i = 0; i < MAX_CH; i++) {
	  consolePrintFP(inputValue(i), 2);
	  consolePrint(" ");
	}      
	consolePrint(")");
	break;

      case 7:
	consolePrint_P(CS_STRING(" aile(e) = "));
	consolePrintF(vpInput.aile);
	consolePrint_P(CS_STRING("("));
	consolePrintF(vpInput.aileExpo);
	consolePrint_P(CS_STRING(") elev(e) = "));
	consolePrintF(vpInput.elev);
	consolePrint_P(CS_STRING("("));
	consolePrintF(vpInput.elevExpo);
	consolePrint_P(CS_STRING(") thr = "));
	consolePrintF(vpInput.throttle);
	consolePrint_P(CS_STRING(" rud = "));
	consolePrintF(vpInput.rudder);
	break;

      case 8:
	consolePrint_P(CS_STRING(" aileOut(c) = "));
	consolePrintF(vpOutput.aile);
	consolePrint_P(CS_STRING(" ("));
	consolePrintF(pidCtrlOutput(&aileCtrl));
	consolePrint_P(CS_STRING(") elevOut = "));
	consolePrintF(vpOutput.elev);
	consolePrint_P(CS_STRING(" thrOut = "));
	consolePrintF(turbineOutput(&engine));
	consolePrint_P(CS_STRING(" rudderOut = "));
	consolePrintF(vpOutput.rudder);
	break;

      case 9:
	consolePrint_P(CS_STRING(" gain*IAS^("));
	for(j = 0; j < 2; j += 0.5f) {
	  if(j > 0)
	    consolePrint(", ");
	  consolePrintF(j);
	}
	
	consolePrint_P(CS_STRING(") = "));
	
	for(j = 0; j < 2; j += 0.5f) {
	  if(j > 0)
	    consolePrint(", ");
	  consolePrintF(vpControl.testGain*powf(vpFlight.IAS, j));
	}
	break;
	
      case 10:
	consolePrint_P(CS_STRING(" acc(avg) = "));
	consolePrintF(vpFlight.acc);
	consolePrint_P(CS_STRING("("));
	consolePrintF(damperOutput(&accAvg));
	consolePrint_P(CS_STRING(") acc = ("));
	consolePrintFP(vpFlight.accX, 2);
	consolePrint_P(CS_STRING(", "));
	consolePrintFP(vpFlight.accY, 2);
	consolePrint_P(CS_STRING(", "));
	consolePrintFP(vpFlight.accZ, 2);
	consolePrint_P(CS_STRING(")"));
	break;

      case 11:
	consolePrint_P(CS_STRING(" alpha = "));
	consolePrintFP(vpFlight.alpha*RADIAN, 1);
	consoleTab(15);
	consolePrint_P(CS_STRING(" relWind = "));
	consolePrintFP(vpFlight.relWind*RADIAN, 1);
	consoleTab(30);
	consolePrint_P(CS_STRING(" accDir = "));
	consolePrintFP(vpFlight.accDir*RADIAN, 1);
	break;
	
      case 12:
	consoleNote_P(CS_STRING(" entropy(alpha,ias) = "));
	consolePrintF(AS5048B_entropy());
	consolePrint_P(CS_STRING(", "));
	consolePrintLnF(MS4525DO_entropy());
	break;
	
     case 13:
	consolePrint_P(CS_STRING(" alpha = "));
	consolePrintFP(vpFlight.alpha*RADIAN, 1);
	consoleTab(15);
	consolePrint_P(CS_STRING(" IAS,K(m/s) = "));
	consolePrintI((int) (vpFlight.IAS/KNOT));
	consolePrint_P(CS_STRING(" ("));
	consolePrintFP(vpFlight.IAS, 1);
	consolePrint_P(CS_STRING(")"));
	break;
	
      case 14:
       consolePrint_P(CS_STRING(" roll_k = "));
       consolePrintFP(vpFlight.rollR/expo(vpOutput.aile-vpControl.aileNeutral, vpParam.roll_Expo)/vpFlight.IAS, 3);
       break;
       
      case 15:
	consolePrint_P(CS_STRING(" elevOutput(trim) = "));
	consolePrintFP(vpOutput.elev, 3);
	consolePrint_P(CS_STRING("("));
	consolePrintFP(vpControl.elevTrim, 3);
	consolePrint_P(CS_STRING(")"));
	break;

      case 16:
	p = 8*clamp(vpFlight.ball/(15.0/RADIAN), -1, 1);
	consolePrint("|");
	if(p < 0) {
	  consoleTab(8+p);
	  consolePrint("o");
	}
	consoleTab(8);
	if(p == 0)
	  consolePrint("0");
	else
	  consolePrint("|");
	if(p > 0) {
	  consoleTab(8+p);
	  consolePrint("o");
	}
	consoleTab(16);
	consolePrint("|");
	break;

      case 17:
	consoleTab(turbineOutput(&engine) * 20);
	consolePrint("*");
	consoleTab(20);
	break;

      case 18:
	if(buttonState(&TRIMBUTTON))
	  consolePrint_P(CS_STRING("[TRIM]"));

	consoleTab(10);
	
	if(buttonState(&LEVELBUTTON))
	  consolePrint_P(CS_STRING("[LEVEL]"));
	
	consoleTab(20);
	
	if(buttonState(&GEARBUTTON))
	  consolePrint_P(CS_STRING("[GEAR]"));
  
	consoleTab(30);
	
	if(buttonState(&RATEBUTTON))
	  consolePrint_P(CS_STRING("[RATE]"));

	consoleTab(40);
	break;
	
      case 19:
	p = 16*clamp(alphaPredictInverse(vpControl.targetAlpha), -1, 1);
	consolePrint("|");
	if(p < 0) {
	  consoleTab(16+p);
	  consolePrint("o");
	}
	consoleTab(16);
	if(p == 0)
	  consolePrint("0");
	else
	  consolePrint("|");
	if(p > 0) {
	  consoleTab(16+p);
	  consolePrint("o");
	}
	consoleTab(32);
	consolePrint("|");
	break;
	
      case 20:
	consolePrint_P(CS_STRING(" log bw = "));
	consolePrintI((int) logBandWidth);
	break;

      case 21:
	consolePrint_P(CS_STRING(" fuel qty (%, current mass) = "));
	consolePrintFP(vpStatus.fuel, 3);
	consolePrint_P(CS_STRING(" ("));
	consolePrintI((int) (vpStatus.fuel/vpParam.fuel*100));
	consolePrint_P(CS_STRING("%, "));
	consolePrintFP(vpStatus.mass, 3);
	consolePrint_P(CS_STRING(")"));
	break;
	
      case 22:
	p = 16*clamp(vpOutput.elev, -1, 1);
	consolePrint("|");
	if(p < 0) {
	  consoleTab(16+p);
	  consolePrint("o");
	}
	consoleTab(16);
	if(p == 0)
	  consolePrint("0");
	else
	  consolePrint("|");
	if(p > 0) {
	  consoleTab(16+p);
	  consolePrint("o");
	}
	consoleTab(32);
	consolePrint("|");
	break;
      }
    }

    consolePrint_P(CS_STRING("      "));
    consoleCR();
  }
}

void communicationTask()
{
  int len = 0;
  
  if((len = stap_hostReceiveState()) > 0) {
    while(len-- > 0)
      datagramRxInputChar(0, stap_hostReceiveChar());
  }

  if((len = stap_telemetryReceiveState()) > 0) {
    while(len-- > 0)
      datagramRxInputChar(1, stap_telemetryReceiveChar());
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

const float pusherBoost_c = 0.3f;
const float pusherBias_c = -2.25f/RADIAN;

void elevatorModule()
{
  const float shakerLimit = SHAKER_LIMIT; 

  if(vpParam.wowCalibrated && vpStatus.weightOnWheels)
    // Limit elevator nose-up wind up when weight is on wheels
    pidCtrlSetRangeAB(&elevCtrl, -RATIO(2/3), RATIO(2/3));
  else
    // Weight not on wheels, allow more authority
    pidCtrlSetRangeAB(&elevCtrl, -RATIO(2/3), RATIO(4/5));
  
  vpInput.stickForce =
    vpMode.radioFailSafe ? 0 : fmaxf(vpInput.elev-shakerLimit, 0)/(1-shakerLimit);

  float effMaxAlpha = mixValue(vpInput.stickForce,
			       vpDerived.shakerAlpha, vpDerived.pusherAlpha);
  
#if HARD_SHAKER
  if(vpStatus.telemetryLink && !vpMode.radioFailSafe)
    float effMaxAlpha = vpDerived.pusherAlpha;
#endif
  
  vpOutput.elev =
    applyExpoTrim(vpInput.elev,
		  vpMode.takeOff ? vpParam.takeoffTrim : vpControl.elevTrim);

  vpControl.targetAlpha = fminf(alphaPredict(vpOutput.elev), effMaxAlpha);
  vpControl.flaring = false;
  
  if(vpStatus.flare) {
    const float moderatedElev = alphaPredictInverse(vpControl.targetAlpha),
      // flareElev = mixValue(vpParam.flare, moderatedElev, vpInput.elev);
      flareElev = mixValue(vpInput.stickForce*vpParam.flare, moderatedElev, 1);

    if(flareElev > moderatedElev) {
      vpControl.targetAlpha = alphaPredict(flareElev);
      vpControl.flaring = true;
    }
  }
  
  if(vpMode.radioFailSafe)
    vpControl.targetAlpha = slopeInput(&trimRateLimiter, vpControl.targetAlpha, controlCycle);
  else
    slopeReset(&trimRateLimiter, vpControl.targetAlpha);
    
  if(vpFeature.alphaHold) {
    const float maxPitch =
      mixValue(1/sqrf(vpFlight.relativeEffIAS),
	       vpParam.maxPitch,
	       fminf(vpParam.maxPitch,
		     asin(turbineOutput(&engine)*vpParam.thrust/vpStatus.mass)
		     + 2.0f/RADIAN + vpControl.targetAlpha));
      
    vpControl.targetPitchR =
      nominalPitchRateLevel(vpFlight.bank, vpControl.targetAlpha)
      + clamp(vpControl.targetAlpha - vpFlight.alpha,
	      -30.0f/RADIAN - vpFlight.pitch, maxPitch - vpFlight.pitch)
      * vpControl.o_P * ( 1 + (vpStatus.stall ? pusherBoost_c : 0) );

    /*
    consolePrintF(vpFlight.bank*RADIAN);
    consolePrint(" ");
    consolePrintF(vpControl.targetAlpha*RADIAN);
    consolePrint(" ");
    consolePrintF(nominalPitchRateLevel(vpFlight.bank, vpControl.targetAlpha)*RADIAN);
    consolePrint(" ");
    consolePrintLnF(vpControl.targetPitchR*RADIAN);
    */
  } else
    vpControl.targetPitchR = vpInput.elevExpo*PI_F/2;

  vpControl.elevPredict =
    alphaPredictInverse(vpControl.targetAlpha);

  if(vpFeature.stabilizePitch) {
    if(vpStatus.stall)
      // Only apply target if it's less than the current rate + bias
      vpControl.targetPitchR =
	fminf(vpControl.targetPitchR, vpFlight.pitchR +  pusherBias_c);

    if(vpControl.flaring)
      pidCtrlSetGain(&elevCtrl, 1 - vpInput.stickForce);
    else
      pidCtrlSetGain(&elevCtrl, 1);
      
    pidCtrlInput(&elevCtrl, vpControl.targetPitchR - vpFlight.pitchR,
		 controlCycle);
    
    vpOutput.elev = pidCtrlOutput(&elevCtrl);

    if(vpFeature.alphaHold)
      vpOutput.elev += vpControl.elevPredict;
  } else {

    if(vpMode.radioFailSafe)
      vpOutput.elev = vpControl.elevPredict;
    
    pidCtrlReset(&elevCtrl, vpOutput.elev - vpControl.elevPredict, 0.0);
      
    // Pusher

#if HARD_PUSHER
    pidCtrlSetRangeAB(&pushCtrl, 0, vpOutput.elev);
#else
    pidCtrlSetRangeAB(&pushCtrl, -vpOutput.elev, 0);
#endif

    vpControl.pusher = 0;
    
    if(vpFeature.pusher) {
      // Pusher active
        
      float target = nominalPitchRate(vpFlight.bank, vpFlight.pitch, vpControl.targetAlpha)
	+ (effMaxAlpha - vpFlight.alpha) * vpControl.o_P
	* (1 + (vpStatus.stall ? pusherBoost_c : 0) );
      
      if(vpStatus.stall)
	target = fminf(target, vpFlight.pitchR + pusherBias_c);
      
      pidCtrlInput(&pushCtrl, target - vpFlight.pitchR, controlCycle);

#if HARD_PUSHER
      vpControl.pusher = fminf(pidCtrlOutput(&pushCtrl) - vpOutput.elev, 0);
#else
      vpControl.pusher = pidCtrlOutput(&pushCtrl);
#endif

      vpOutput.elev += vpControl.pusher;
    } else
#if HARD_PUSHER
      pidCtrlReset(&pushCtrl, vpOutput.elev, 0.0);
#else
    pidCtrlReset(&pushCtrl, 0, 0.0);
#endif
  }
}

//
//   Aileron
//

void aileronModule()
{
  float maxBank = 60/RADIAN;

  if(vpMode.radioFailSafe) {
    maxBank = 15.0f/RADIAN;
    if(vpStatus.stall)
      vpInput.aile = vpInput.aileExpo = 0;
  } else if(vpFeature.alphaHold)
    maxBank /= 1 + sqrf(1/vpFlight.relativeEffIAS);
  
  float targetRollR = rollRatePredict(vpInput.aileExpo);
  
  // We accumulate individual contributions so start with 0

  vpOutput.aile = 0;
  
  if(vpFeature.stabilizeBank) {
    // Stabilization is enabled
    
    if(vpFeature.keepLevel)
      // Strong leveler enabled
      
      targetRollR
	= vpControl.o_P * (vpInput.aile*80.0f/RADIAN - vpFlight.bank);

    else if(vpMode.bankLimiter) {

      if(vpMode.slowFlight)
	// Weak leveling
	targetRollR
	  -= vpControl.o_P*clamp(vpFlight.bank, -1.0f/RADIAN, 1.0f/RADIAN);

      // Bank limiter, disabled on extreme pitch up

      if(vpFlight.pitch < 70/RADIAN)
	targetRollR = clamp(targetRollR,
			    (-maxBank - vpFlight.bank) * vpControl.o_P,
			    (maxBank - vpFlight.bank) * vpControl.o_P);
    }

    pidCtrlInput(&aileCtrl, targetRollR - vpFlight.rollR, controlCycle);
  } else {
    // Stabilization disabled
    
    pidCtrlReset(&aileCtrl, 0, 0);
    
    if(vpFeature.keepLevel)
      // Simple proportional wing leveler
      vpOutput.aile -= vpFlight.bank + vpFlight.rollR/32;
  }

  //   Apply controller output + feedforward
  
  vpControl.ailePredict = rollRatePredictInverse(targetRollR);
  
  vpOutput.aile += vpControl.ailePredict + pidCtrlOutput(&aileCtrl);

  //   Constrain & rate limit
  
  vpOutput.aile
    = slopeInput(&aileActuator,
		 constrainServoOutput(vpOutput.aile), controlCycle);
}

//
// Canard neutral control (elevator is not included here)
//

void canardModule()
{
  if(!vpStatus.alphaUnreliable)
    pidCtrlInput(&canardCtrl, clamp(vpFlight.alpha, -vpDerived.maxAlpha, 1.2*vpDerived.maxAlpha), controlCycle);
  else
    pidCtrlReset(&canardCtrl, 0, 0);
  
  vpOutput.canard = -pidCtrlOutput(&canardCtrl);
}

//
//   Rudder & nose wheel
//

void rudderModule()
{
  // Apply stick input
  
  vpOutput.rudder = vpOutput.steer = vpInput.rudder;

#if AUTO_RUDDER
  if(vpInput.rudderPilotInput || vpMode.takeOff || vpStatus.weightOnWheels
     || vpMode.sensorFailSafe)
    // Pilot input is present/takeoff mode/WoW, keep auto-rudder reset
    pidCtrlReset(&rudderCtrl, 0, 0);
  else
    // No pilot input, auto-rudder is active
   pidCtrlInput(&rudderCtrl, vpFlight.ball, controlCycle);

  // Apply auto-rudder
  
  vpOutput.rudder += pidCtrlOutput(&rudderCtrl);
#endif
  
  // Apply yaw damper (unless there's pilot input on rudder)

#if YAW_DAMPER
  if(vpInput.rudderPilotInput)
    washoutReset(&yawDamper, 0);
  else
    vpOutput.rudder -= vpControl.yd_P * washoutInput(&yawDamper, vpFlight.yawR);
#endif
}

//
//   Throttle
//
  
void throttleModule()
{
  if(vpMode.takeOff)
    // Taking off, don't want lag
    turbineReset(&engine, vpInput.throttle);
  else if(!vpStatus.airborne || vpMode.radioFailSafe)
    // Not taking off or aloft, keep closed
    turbineReset(&engine, 0);
  else if(vpParam.wowCalibrated && vpStatus.weightOnWheels)
    // We've landed, direct control is nice so we can stop for full stop
    // landings and get back up quickly for touch and gos
    turbineReset(&engine, vpInput.throttle);
  else
    // We're flying, apply lag
    turbineInput(&engine, vpInput.throttle);
  // turbineInput(&engine, vpParam.idle + vpInput.throttle*(1 - vpParam.idle));
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
  
  vpOutput.flap =
    slopeInput(&flapActuator, (float) vpControl.flapSel/FLAP_STEPS, controlCycle);

  //
  // Brake
  //
    
  if(vpControl.gearSel == 1) {
    vpControl.parking = false;
    vpOutput.brake = 0;
  } else {
    float pedal = clamp(-vpInput.elev, 0, 1);
    static bool depressed;

    if(pedal > 0.8)
      depressed = true;
    else if(depressed && pedal < 0.5) {
      depressed = false;
      vpControl.parking = !vpStatus.positiveIAS && buttonState(&TRIMBUTTON);
    }
      
    vpOutput.brake = vpControl.parking ? 1.1f : pedal;
  }
}

//
// List of control modules in no particular order
//

void (*controlModules[])(void) = {
  elevatorModule,
  canardModule,
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
    constrainServoOutput(vpOutput.elev
			 + scaleByIAS_E(vpControl.t_Mix, -2)
			 * powf(turbineOutput(&engine), vpParam.t_Expo));

  // Aile to rudder mix

  const float liftRatio = (vpMode.alphaFailSafe | vpStatus.alphaUnreliable)
    ? 0.3f : coeffOfLiftClean(vpFlight.alpha)/vpDerived.maxCoeffOfLiftClean;

  vpOutput.rudder =
    constrainServoOutput(vpOutput.rudder
			 + cosf(vpFlight.alpha) * vpOutput.aile * vpControl.r_Mix * liftRatio);  
}

void controlTask()
{
  uint8_t i = 0;
  
  //
  // Cycle time bookkeeping 
  //
  
  static VP_TIME_MICROS_T controlCycleEnded;
 
  if(controlCycleEnded > 0) {
    controlCycle = (vpTimeMicrosApprox - controlCycleEnded)/1.0e6;
    controlFreq = (int16_t) (1.0f / controlCycle);
  }
  
  controlCycleEnded = vpTimeMicrosApprox;

  //
  // Invoke individual control modules
  //

  for(i = 0; i < sizeof(controlModules)/sizeof(void(*)()); i++)
    (*controlModules[i])();
  
  //
  // Final mixing
  //
  
  mixingTask();
}

//
// Actuator output
//

void actuatorTask()
{
  if(!vpStatus.armed || vpStatus.simulatorLink || vpParam.virtualOnly)
    return;

  int i = 0;
  
  for(i = 0; i < MAX_SERVO; i++) {
    if(ABS(vpParam.functionMap[i]) == fn_gear && !vpMode.gearSelected)
      // We haven't toggled the gear yet, stay inactive
      continue;

    float value = 0;

    if(functionInvoke(vpParam.functionMap[i], &value))
      stap_servoOutput(i, value + vpParam.neutral[i]);
  }

  controlLatencyTotal += vpTimeMicros() - controlInputTimeStamp;
  controlLatencyCount++;
  
  stap_servoOutputSync();
}

void heartBeatTask()
{
  if(!heartBeatCount[0] && linkDownCount[0]++ > 2) {
    if(vpStatus.consoleLink)
      consoleNoteLn_P(CS_STRING("Console DISCONNECTED"));
    
    vpStatus.consoleLink = vpStatus.simulatorLink = false;
  }
  
  if(!heartBeatCount[1] && linkDownCount[1]++ > 2) {
    if(vpStatus.telemetryLink)
      consoleNoteLn_P(CS_STRING("Telemetry DISCONNECTED"));
    
    vpStatus.telemetryLink = false;
  }
  
  if(vpStatus.simulatorLink && vpTimeMillisApprox - simTimeStamp > 1.0e3) {
    consoleNoteLn_P(CS_STRING("Simulator link LOST"));
    vpStatus.simulatorLink = false;
  }    
  
  heartBeatCount[0] = heartBeatCount[1] = 0;

  datagramHeartbeat(false);
  m24xxFlush();
}

void blinkTask()
{
  float ledRatio = vpMode.test ? 0.0f : (vpMode.sensorFailSafe || !vpStatus.armed) ? 0.5f : vpFlight.alpha > 0.0f ? 0.90f : 0.10f;
  static int tick = 0;
  
  tick = (tick + 1) % (LED_TICK/LED_HZ);

  if(tick < ledRatio*LED_TICK/LED_HZ)
    STAP_LED_OFF;
  else
    STAP_LED_ON;
}

VPPeriodicTimer_t downlinkTimer = VP_PERIODIC_TIMER_CONS(MAX_LATENCY_CONFIG);

void downlinkTask()
{
  uint16_t status =
    ((vpInput.throttle < vpParam.idle
      || turbineOutput(&engine) < vpParam.idle) ? (1<<8) : 0)
    | (!vpStatus.telemetryLink ? (1<<7) : 0)
    | ((vpStatus.trimLimited && !vpMode.radioFailSafe) ? (1<<6) : 0)
    | (logReady(false) ? (1<<5) : 0)
    | (vpMode.radioFailSafe ? (1<<4) : 0)
    | (vpStatus.alphaUnreliable ? (1<<3) : 0)
    | (vpStatus.airborne ? (1<<0) : 0);

  if(!vpStatus.alphaUnreliable) {
    status |= vpFlight.alpha > vpDerived.stallAlpha ? (1<<2) : 0;

    if((HARD_SHAKER && vpStatus.telemetryLink)
       || vpMode.alphaFailSafe || vpInput.stickForce > 0.0f)
       status |= vpFlight.alpha > vpDerived.shakerAlpha ? (1<<1) : 0;
  }
    
  //
  // Telemetry(Data)
  //
  
  struct TelemetryData data = { .status = status,
				.alpha = vpFlight.alpha,
				.IAS = vpFlight.IAS };

  datagramTxStart(DG_AIRDATA);
  datagramTxOut((const uint8_t*) &data, sizeof(data));
  datagramTxEnd();

  if(vpPeriodicEvent(&downlinkTimer)) {
    //
    // Telemetry(Configuration)
    //
  
    struct TelemetryConfig config = {
      .load = vpStatus.load,
      .latency = controlLatencyAvg,
      .trim = alphaPredict(vpControl.elevTrim),
      .maxAlpha = vpDerived.maxAlpha,
      .shakerAlpha = vpDerived.shakerAlpha,
      .threshAlpha = vpDerived.thresholdAlpha,
      .trimIAS = dynamicPressureInverse(vpStatus.mass*G/coeffOfLift(alphaPredict(vpControl.elevTrim))),
      .stallIAS = vpDerived.minimumIAS,
      .margin = vpParam.thresholdMargin,
      .fuel = vpStatus.fuel / vpParam.fuel
    };

    strncpy(config.name, vpParam.name, NAME_LEN);
    
    datagramTxStart(DG_CONFIG);
    datagramTxOut((const uint8_t*) &config, sizeof(config));
    datagramTxEnd();
  }

  //
  // Sim link if connected
  //
  
  if(vpStatus.simulatorLink && vpStatus.armed) {
    struct SimLinkControl control = { .aileron = vpOutput.aile,
				      .elevator = -vpOutput.elev,
				      .throttle = turbineOutput(&engine),
				      .rudder = vpOutput.rudder };

    datagramTxStartLocal(DG_SIMLINK);
    datagramTxOut((const uint8_t*) &control, sizeof(control));
    datagramTxEnd();
  }
}

void controlTaskGroup()
{
  //  derivedValidate();
  sensorTaskSync();
  receiverTask();
  controlTask();
  actuatorTask();
  downlinkTask();
}

void configTaskGroup()
{
  if(fabsf(vpOutput.flap - vpDerived.assumedFlap) > 0.015f
     || fabsf(vpStatus.mass - vpDerived.assumedMass) > 0.05f)
    // Flap setting or mass has changed
    derivedInvalidate();
  
  derivedValidate();
  
  statusTask();
  configurationTask();
  trimTask();
}

struct Task alphaPilotTasks[] = {
#ifdef STAP_PERIOD_GYRO
  { gyroTask, STAP_PERIOD_GYRO_STATIC, true },
#endif
#ifdef STAP_PERIOD_ATTI
  { attiTask, STAP_PERIOD_ATTI, true },
#endif
#ifdef STAP_PERIOD_ACC
  { accTask, STAP_PERIOD_ACC, true },
#endif
  { communicationTask, HZ_TO_PERIOD(60), true },
#ifdef ASYNC_AIR_SENSORS
  { alphaSampleTask, HZ_TO_PERIOD(AIR_SENSOR_OVERSAMPLE*CONTROL_HZ), true },
  { airspeedSampleTask, HZ_TO_PERIOD(AIR_SENSOR_OVERSAMPLE*CONTROL_HZ), true },
#endif
  { sensorTaskSlow, HZ_TO_PERIOD(CONTROL_HZ/5), true },
  { controlTaskGroup, HZ_TO_PERIOD(CONTROL_HZ), true, &ppmFrameReceived },
  { configTaskGroup, HZ_TO_PERIOD(CONFIG_HZ), true },
  // { gpsTask, HZ_TO_PERIOD(100) },
  { blinkTask, HZ_TO_PERIOD(LED_TICK), false },
  { obdRefresh, HZ_TO_PERIOD(30), false },
  { displayTask, HZ_TO_PERIOD(8), false },
  { logTask, HZ_TO_PERIOD(LOG_HZ), true },
  { logSave, HZ_TO_PERIOD(LOG_HZ_COMMIT), false },
  { consoleFlush, HZ_TO_PERIOD(CONSOLE_HZ), false },
  { monitorTask, HZ_TO_PERIOD(1), false },
  { heartBeatTask, HZ_TO_PERIOD(HEARTBEAT_HZ), false },
  { gaugeTask, HZ_TO_PERIOD(10), false },
  { NULL, 0, false } };


