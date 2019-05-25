#include <string.h>
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
// Periodic tasks
//

void cacheTask()
{
  m24xxFlush();
}

void alphaTask()
{
  int16_t raw = 0;

  if(AS5048B_isOnline() && AS5048B_alpha(&raw)) {
    samplerInput(&alphaSampler, raw);
    stap_entropyDigest((uint8_t*) &raw, sizeof(raw));
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
  obdPrint(vpParam.name);
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
      { (char) (nvState.testNum < 10 ? ' ' : ('0' + nvState.testNum / 10)),
	(char) ('0' + nvState.testNum % 10),
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

void airspeedTask()
{
  int16_t raw = 0;

  if(!MS4525DO_isOnline())
    // The sensor is offline, start calibrating when it comes back
    MS4525DO_calibrate();
  else if(MS4525DO_pressure(&raw)) {
    // We got a good value
    samplerInput(&iasSampler, raw);
    stap_entropyDigest((uint8_t*) &raw, sizeof(raw));
  }
}

void configTaskGroup();

#define NZ_BIG RATIO(5/100)
#define NZ_SMALL RATIO(2.5/100)

void receiverTask()
{
  stap_rxInputPoll();
 
  if(inputValid(CH_AILE))
    vpInput.aile = applyNullZone(inputValue(CH_AILE), NZ_BIG, &vpInput.ailePilotInput);
  
  if(inputValid(CH_ELEV))
    vpInput.elev = applyNullZone(inputValue(CH_ELEV), NZ_SMALL, &vpInput.elevPilotInput);

  if(inputValid(CH_TUNE))
    vpInput.tuningKnob = inputValue(CH_TUNE)*1.05f - 0.05f;
    
  if(inputValid(CH_THRO))
    vpInput.throttle = inputValue(CH_THRO);

  vpInput.modeSel = readSwitch(&flightModeSelector);

#ifdef CH_RUD 
  if(inputValid(CH_RUD))
    vpInput.rudder = applyNullZone(inputValue(CH_RUD), NZ_BIG, &vpInput.rudderPilotInput);
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
  // Alpha input

  vpFlight.alpha =
    2 * PI_F * samplerMean(&alphaSampler) / (1L<<(8*sizeof(int16_t)));
  
  // Dynamic pressure, corrected for alpha

  const float pascalsPerPSI_c = 6894.7573, range_c = 2*1.1f;
  const float factor_c = pascalsPerPSI_c * range_c / (1L<<(8*sizeof(uint16_t)));

  vpFlight.dynP = samplerMean(&iasSampler) * factor_c
    / cosf(clamp(vpFlight.relWind, -vpDerived.maxAlpha, vpDerived.maxAlpha));
  
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

  //  stap_baroUpdate();
  
  // Simulator link overrides
  
  if(vpStatus.simulatorLink) {
    vpFlight.alpha = sensorData.alpha/RADIAN;
    vpFlight.IAS = sensorData.ias*1852/60/60;
    vpFlight.rollR = sensorData.rrate;
    vpFlight.pitchR = sensorData.prate;
    vpFlight.yawR = sensorData.yrate;
    vpFlight.bank = sensorData.roll/RADIAN;
    vpFlight.pitch = sensorData.pitch/RADIAN;
    vpFlight.heading = (int) (sensorData.heading + 0.5f);
    vpFlight.accX = sensorData.accx*FOOT;
    vpFlight.accY = sensorData.accy*FOOT;
    vpFlight.accZ = -sensorData.accz*FOOT;
    
    vpFlight.dynP = dynamicPressure(vpFlight.IAS);
    
  } else 
    vpFlight.IAS = dynamicPressureInverse(vpFlight.dynP);

  //
  // Derived values
  //
    
  vpFlight.ball = atan2f(-vpFlight.accY, fabs(vpFlight.accZ));
  damperInput(&iasFilter, vpFlight.IAS);
  damperInput(&iasFilterSlow, vpFlight.IAS);
}

void sensorTaskSlow()
{
  // Altitude

  if(vpStatus.simulatorLink)
    vpFlight.alt = sensorData.alt*FOOT;
  else
    vpFlight.alt = stap_baroRead();

  // Alpha sensor field strength

  uint16_t raw = 0;
  
  if(AS5048B_isOnline() && AS5048B_field(&raw))
    fieldStrength = (float) raw / (1L<<16);
}

void monitorTask()
{
  static uint32_t prevMonitor;
 
  // Load measurement

  vpStatus.load = vpStatus.load/2 + (1.0f - (float) idleMicros/(stap_currentMicros - prevMonitor))/2;
  idleMicros = 0;

  // PPM monitoring

  ppmFreq = inputSourceRate();
  
  // Sim link monitoring

  simInputFreq = 1.0e6f * simFrames / (stap_currentMicros - prevMonitor);
  simFrames = 0;

  // Log bandwidth

  logBandWidth = 1.0e6f * m24xxBytesWritten / (stap_currentMicros - prevMonitor);
  m24xxBytesWritten = 0;
  
  // PPM monitoring

  if(!inputSourceGood())
    ; // lastPPMWarn = stap_currentMicros;
  
  // I2C errors

  uint16_t num = stap_i2cErrorCount();

  if(num > 0) {
    consoleNote("I2C errors ");
    consolePrintUI(num);
    consolePrint(" (last code ");
    consolePrintUI(stap_i2cErrorCode());
    consolePrintLn(")");
  }

  prevMonitor = stap_currentMicros;
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

void statusTask()
{
  // Cycle time
  
  static uint32_t statusCycleEnded;
  float statusCycle = 0.0f;
  
  if(statusCycleEnded > 0)
    statusCycle = (stap_currentMicros - statusCycleEnded)/1.0e6;
  
  statusCycleEnded = stap_currentMicros;

  //
  // Canopy open/closed
  //

  static uint32_t lastCanopy;
  
  if(STAP_CANOPY_CLOSED) {
    if(vpStatus.canopyClosed)
      lastCanopy = stap_currentMicros;
    else if(stap_currentMicros - lastCanopy > 0.5e6) {
      consoleNoteLn_P(CS_STRING("Canopy is CLOSED"));
      vpStatus.canopyClosed = true;
    }
  } else {
    if(!vpStatus.canopyClosed)
      lastCanopy = stap_currentMicros;
    else if(stap_currentMicros - lastCanopy > 0.5e6) {
      consoleNoteLn_P(CS_STRING("Canopy is OPEN"));
      vpStatus.canopyClosed = false;
    }
  }
  
  //
  // Fuel quantity
  //

  if(vpStatus.canopyClosed || vpStatus.aloft) {
    // Assume we are disconnected from the overflow tank
    
    vpStatus.fuel -=
      polynomial(FuelFlow_degree, clamp(vpInput.throttle, 0, 1),
		 vpParam.coeff_Flow)
      * statusCycle / 1000 / 60; // Convert g/min to kg/s
    
    if(vpStatus.fuel < 0)
      vpStatus.fuel = 0;
    
  } else if(vpStatus.fuel != vpParam.fuel) {
    // Assume we are being topped up
    
    vpStatus.fuel = vpParam.fuel;
    consoleNote_P(CS_STRING("Fuel assumed TOPPED UP ("));
    consolePrintFP(vpStatus.fuel, 3);
    consolePrintLn_P(CS_STRING(" kg)"));
  }
  
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
  
  static uint32_t iasLastAlive;

  if(vpFlight.IAS < vpDerived.minimumIAS/3 || fabsf(vpFlight.IAS - damperOutput(&iasFilterSlow)) > 0.5f) {
    if(vpStatus.pitotBlocked) {
      consoleNoteLn_P(CS_STRING("Pitot block CLEARED"));
      vpStatus.pitotBlocked = false;
    }
    
    iasLastAlive = stap_currentMicros;
  } else if(!vpStatus.simulatorLink
	    && stap_currentMicros - iasLastAlive > 10e6 && !vpStatus.pitotBlocked) {
    consoleNoteLn_P(CS_STRING("Pitot appears BLOCKED"));
    vpStatus.pitotBlocked = true;
  }
  
  //
  // Do we have positive airspeed?
  //

  static uint32_t lastIAS, lastStall, lastAlphaLocked;

  if(vpStatus.pitotFailed) {
    if(!vpStatus.positiveIAS) {
      consoleNoteLn_P(CS_STRING("Pitot failed, positive IAS ASSUMED"));
      vpStatus.positiveIAS = true;
    }
  } else if(damperOutput(&iasFilter) < vpDerived.minimumIAS*RATIO(4/5)) {
    if(!vpStatus.positiveIAS)
      lastIAS = stap_currentMicros;
    else if(stap_currentMicros - lastIAS > 0.3e6f) {
      consoleNoteLn_P(CS_STRING("Positive airspeed LOST"));
      vpStatus.positiveIAS = false;
    }
  } else {
    if(vpStatus.positiveIAS)
      lastIAS = stap_currentMicros;
    else if(stap_currentMicros - lastIAS > 0.3e6f) {
      consoleNoteLn_P(CS_STRING("We have POSITIVE AIRSPEED"));
      vpStatus.positiveIAS = true;
    }
  }

  //
  // Movement detection
  //
  
  vpFlight.acc = sqrtf(sq(vpFlight.accX) + sq(vpFlight.accY) + sq(vpFlight.accZ));
  
  damperInput(&accAvg, vpFlight.acc);

  float turnRate = sqrtf(sq(vpFlight.rollR) + sq(vpFlight.pitchR) + sq(vpFlight.yawR));
  
  bool motionDetected = (!vpStatus.pitotBlocked && vpStatus.positiveIAS)
    || turnRate > 10.0f/RADIAN
    || fabsf(vpFlight.acc - damperOutput(&accAvg)) > 0.5f;
  
  static uint32_t lastMotion;

  if(motionDetected) {
    if(vpStatus.fullStop) {
      consoleNoteLn_P(CS_STRING("We appear to be MOVING"));
      vpStatus.fullStop = false;
    }
    
    lastMotion = stap_currentMicros;

  } else if(stap_currentMicros - lastMotion > 5.0e6 && !vpStatus.fullStop) {
    consoleNoteLn_P(CS_STRING("We have FULLY STOPPED"));
    vpStatus.fullStop = true;
    vpStatus.aloft = false;
  }

  //
  // Alpha/accel lockup detection (sensor vane detached?)
  //

  vpFlight.accDir = atan2(vpFlight.accZ, -vpFlight.accX);
  vpFlight.relWind = vpStatus.fault == 3 ? vpFlight.accDir : vpFlight.alpha - vpParam.alphaOffset;
  
  if(vpStatus.alphaFailed) {
      // Failed alpha is also unreliable
    
      vpStatus.alphaUnreliable = true;
      lastAlphaLocked = stap_currentMicros;
  } else {
    const float diff = fabsf(vpFlight.accDir - vpFlight.relWind),
      disagreement = MIN(diff, 2*PI_F - diff);

    if(vpMode.alphaFailSafe || vpMode.sensorFailSafe || vpMode.takeOff
       || (fabsf(vpFlight.alpha) < 60.0f/RADIAN && disagreement > 15.0f/RADIAN)) {
      if(!vpStatus.alphaUnreliable)
	lastAlphaLocked = stap_currentMicros;
      else if(stap_currentMicros - lastAlphaLocked > 0.1e6) {
	consoleNoteLn_P(CS_STRING("Alpha sensor appears RELIABLE"));
	vpStatus.alphaUnreliable = false;
      }
    } else {
      if(vpStatus.alphaUnreliable)
	lastAlphaLocked = stap_currentMicros;
      else if(stap_currentMicros - lastAlphaLocked > 0.5e6) {
	consoleNoteLn_P(CS_STRING("Alpha sensor UNRELIABLE"));
	vpStatus.alphaUnreliable = true;
      }
    }
  }

  //
  // Flare detection
  //

  static uint32_t lastFlare;
  
  if(!(vpMode.test && nvState.testNum > 0) && vpMode.slowFlight
     && vpParam.haveGear
     && vpControl.gearSel == 0
     && (vpDerived.haveRetracts || vpStatus.simulatorLink || vpFlight.alt < 5 )
     && vpFlight.IAS < (1.1f + vpParam.thresholdMargin)*vpDerived.minimumIAS
     && fabsf(vpFlight.bank) < 30.0f/RADIAN
     && vpInput.throttle < 0.3f
     && vpInput.stickForce > RATIO(1/4)) {
    // We may be in a flare

    if(!vpStatus.flare)
      consoleNoteLn_P(CS_STRING("We seem to be FLARING"));
    
    vpStatus.flare = true;
    lastFlare = stap_currentMicros;
    
  } else if(vpStatus.flare && stap_currentMicros - lastFlare > 0.7e6) {
    consoleNoteLn_P(CS_STRING("Flare ended"));
    vpStatus.flare = false;
  }
  
  //
  // Stall detection
  //
  
  if(vpStatus.alphaUnreliable || vpMode.alphaFailSafe || vpMode.sensorFailSafe
     || vpMode.takeOff || vpStatus.flare
     || vpFlight.alpha < fmaxf(vpDerived.stallAlpha, vpControl.targetAlpha)) {
    if(!vpStatus.stall)
      lastStall = stap_currentMicros;
    else if(stap_currentMicros - lastStall > 0.05e6) {
      consoleNoteLn_P(CS_STRING("Stall RECOVERED"));
      vpStatus.stall = false;
    }
  } else {
    if(vpStatus.stall)
      lastStall = stap_currentMicros;
    else if(stap_currentMicros - lastStall > 0.05e6) {
      consoleNoteLn_P(CS_STRING("We're STALLING"));
      vpStatus.stall = true;
    }
  }

  //
  // Below floor?
  //
  
  if(vpFlight.alt > vpParam.floor + 5 || vpParam.floor < 1) {
    if(vpStatus.belowFloor)
      consoleNoteLn_P(CS_STRING("We're ABOVE floor"));
    
    vpStatus.belowFloor = false;
  } else if(vpFlight.alt < vpParam.floor && !vpStatus.belowFloor) {
    vpStatus.belowFloor = true;
    consoleNoteLn_P(CS_STRING("We're BELOW floor altitude"));
  }

  //
  // Attitude is upright?
  //
  
  static uint32_t lastUpright;
  
  if(fabsf(vpFlight.bank) < 15.0f/RADIAN && fabsf(vpFlight.pitch) < 15.0f/RADIAN) {
    vpStatus.upright = true;
    lastUpright = stap_currentMicros;
  } else {
    if(!vpStatus.upright)
      lastUpright = stap_currentMicros;
    else if(stap_currentMicros - lastUpright > 0.5e6)
      vpStatus.upright = false;
  }

  //  
  // Weight on wheels?
  //

  const float
    weight = totalMass() * G,
    lift = totalMass() * vpFlight.accZ * cosf(vpFlight.pitch),
    liftAvg = swAvgInput(&liftFilter, lift),
    liftExpected = coeffOfLift(vpFlight.alpha) * vpFlight.dynP,
    liftMax = vpDerived.maxCoeffOfLift * vpFlight.dynP;
      
  static uint32_t lastWoW;
  
  if(vpMode.alphaFailSafe || vpMode.sensorFailSafe || vpMode.radioFailSafe
     || vpStatus.alphaUnreliable || vpStatus.pitotFailed
     || !vpParam.haveGear || vpControl.gearSel == 1 || !vpStatus.upright
     || vpFlight.IAS > vpDerived.minimumIAS*RATIO(3/2)) {
    if(vpStatus.weightOnWheels) {
      consoleNoteLn_P(CS_STRING("Weight assumed to be OFF THE WHEELS"));
      vpStatus.weightOnWheels = false;
    }
      
    lastWoW = stap_currentMicros;
  } else if(vpStatus.positiveIAS
	    && (liftAvg < weight/2 || liftAvg > 1.5f*weight
		|| lift < liftExpected + liftMax/3)) {
    if(!vpStatus.weightOnWheels)
      lastWoW = stap_currentMicros;
    else if(stap_currentMicros - lastWoW > 0.3e6) {
      consoleNoteLn_P(CS_STRING("Weight is probably OFF THE WHEELS"));
      vpStatus.weightOnWheels = false;
    }
  } else {
    if(vpStatus.weightOnWheels)
      lastWoW = stap_currentMicros;
    else if(stap_currentMicros - lastWoW > 0.2e6) {
      consoleNoteLn_P(CS_STRING("We seem to have WEIGHT ON WHEELS"));
      vpStatus.weightOnWheels = true;
    }
  }
}
  
void configurationTask()
{
  //
  // Being armed?
  //
  
  if(buttonDoublePulse(&TRIMBUTTON) && !vpStatus.armed &&
     vpInput.throttle < 0.1f && vpInput.aile < -0.9f && vpInput.elev > 0.9f) {
    consoleNoteLn_P(CS_STRING("We're now ARMED"));
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
      consoleNoteLn_P(CS_STRING("Alpha FAILSAFE"));
      vpMode.alphaFailSafe = true;
      logMark();
      
    } else if(!vpMode.sensorFailSafe) {
      consoleNoteLn_P(CS_STRING("Total sensor FAILSAFE"));
      vpMode.sensorFailSafe = true;
      logMark();
      
    } else if(!vpStatus.positiveIAS)
      logDisable();
    
  } else if(buttonSinglePulse(&GEARBUTTON)) {
    //
    // SINGLE PULSE: GEAR TOGGLE
    //

    if(vpDerived.haveRetracts) {
      vpControl.gearSel = !vpControl.gearSel;
      vpMode.gearSelected = true;

      if(vpControl.gearSel)
	consoleNoteLn_P(CS_STRING("Gear UP"));
      else
	consoleNoteLn_P(CS_STRING("Gear DOWN"));
    }
  }

  //
  // RATE BUTTON
  //

  if(buttonDepressed(&RATEBUTTON) && !vpMode.halfRate) {
    // Continuous: half-rate enable
    
    consoleNoteLn_P(CS_STRING("Half-rate ENABLED"));
    vpMode.halfRate = true;
    
  } else if(buttonSinglePulse(&RATEBUTTON) && vpMode.halfRate) {
    // Single pulse: half-rate disable
    
    consoleNoteLn_P(CS_STRING("Half-rate DISABLED"));
    vpMode.halfRate = false;
  }

  //
  // WING LEVELER BUTTON
  //

  if(buttonSinglePulse(&LEVELBUTTON)) {
    //
    // PULSE : Takeoff mode enable
    //
  
    if(!vpStatus.positiveIAS || vpStatus.simulatorLink) {
	    
      vpMode.silent = false;

      bool prevMode = vpMode.takeOff;
      
      if(!vpMode.takeOff) {
	consoleNoteLn_P(CS_STRING("TakeOff mode ENABLED"));
	vpMode.takeOff = true;
      }

      if(tocTestStatus(tocReportConsole)) {
	consoleNoteLn_P(CS_STRING("T/o configuration is GOOD"));
	vpStatus.aloft = false;
      } else {
	consolePrintLn("");
	consoleNoteLn_P(CS_STRING("T/o configuration test FAILED"));
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
  } else if(vpStatus.aloft && !vpStatus.pitotBlocked && vpStatus.positiveIAS)
    logEnable();
  else if(vpStatus.fullStop)
    logDisable();
    
  //
  // Direct mode selector input
  //

  if(vpInput.modeSel == -1 || vpStatus.belowFloor) {
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

  vpControl.flapSel = FLAP_STEPS/2 - vpInput.flapSel;

  //
  // Test mode control
  //

  if(vpMode.radioFailSafe)
    vpMode.test = false;
  
  else if(!vpMode.test && vpInput.tuningKnob > 0.5f) {
    vpMode.test = true;
    consoleNoteLn_P(CS_STRING("Test mode ENABLED"));

  } else if(vpMode.test && vpInput.tuningKnob < 0) {
    vpMode.test = false;
    consoleNoteLn_P(CS_STRING("Test mode DISABLED"));
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
    consoleNoteLn_P(CS_STRING("We're assumed to have TAKEN OFF"));
    vpMode.takeOff = false;
    vpStatus.aloft = true;
    
    if(!vpStatus.consoleLink)
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
  
  if(vpMode.test && nvState.testNum == 0)
    scale = testGainLinear(RATIO(1/3), 1);
  
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
  
  pidCtrlSetZNPI(&rudderCtrl, r_Ku*scale, vpParam.r_Tu);

  //   Yaw damper
  
  vpControl.yd_P = scaleByIAS_E(vpParam.yd_C, yawDamperExp_c) * scale;

  //   Turbine lag

  turbineSetTau(&engine, vpParam.lag*CONTROL_HZ);
  
  vpControl.o_P = vpParam.o_P;
  vpControl.r_Mix = vpParam.r_Mix;
  vpControl.t_Mix = vpParam.t_Mix;
  
  slopeSet(&aileActuator, vpParam.servoRate/(90.0f/2)/vpParam.aileDefl);
  
  //
  // Apply test mode
  //
  
  if(vpMode.test && !vpMode.takeOff) {
    switch(nvState.testNum) {
    case 1:
      // Wing stabilizer gain
         
      vpFeature.stabilizeBank = vpMode.bankLimiter = vpFeature.keepLevel = true;
      pidCtrlSetPID(&aileCtrl, vpControl.testGain = testGainExpo(vpControl.s_Ku_ref), 0, 0);
      break;
            
    case 2:
      // Elevator stabilizer gain, outer loop disabled
         
      vpFeature.stabilizePitch = true;
      vpFeature.alphaHold = false;
      pidCtrlSetPID(&elevCtrl, vpControl.testGain = testGainExpo(vpControl.i_Ku_ref), 0, 0);
      break;
         
    case 3:
      // Elevator stabilizer gain, outer loop enabled
         
      vpFeature.stabilizePitch = vpFeature.alphaHold = true;
      pidCtrlSetPID(&elevCtrl, vpControl.testGain = testGainExpo(vpControl.i_Ku_ref), 0, 0);
      break;
         
    case 4:
      // Auto alpha outer loop gain
         
      vpFeature.stabilizePitch = vpFeature.alphaHold = true;
      vpControl.o_P = vpControl.testGain = testGainExpo(vpParam.o_P);
      break;

    case 5:
      // Max alpha tests, just flare disabled (because in test mode)
      break;

    case 6:
      // Yaw damper gain
      vpControl.yd_P = vpControl.testGain = testGainExpo(vpControl.yd_P_ref);
      break;

    case 7:
      // Throttle-elev mix
      
      vpControl.t_Mix = vpControl.testGain = testGainLinear(0, vpParam.t_Mix*1.2f);
      break;
      
    case 8:
      // Stall behavior test
      
      vpFeature.pusher = false;
      break;
      
    case 9:
      // Auto rudder gain
         
      pidCtrlSetPID(&rudderCtrl, vpControl.testGain = testGainExpo(vpControl.r_Ku_ref), 0, 0);
      break;
            
    case 10:
      // Aileron to rudder mix

      vpControl.r_Mix = vpControl.testGain = testGainLinear(0, vpParam.r_Mix*1.5f);
      break;

    case 11:
      // Turbine lag

      turbineSetTau(&engine, CONTROL_HZ*(vpControl.testGain = testGainLinear(vpParam.lag*1.5f, 0)));
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
  } else
    vpControl.elevTrim =
      clamp(vpControl.elevTrim,
	    // fmaxf(-0.10f, alphaPredictInverse(vpDerived.zeroLiftAlpha)),
	    -0.15f,
	    alphaPredictInverse(vpDerived.thresholdAlpha));
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
	
      case 20:
	consolePrint_P(CS_STRING(" log bw = "));
	consolePrintI((int) logBandWidth);
	break;

      case 21:
	consolePrint_P(CS_STRING(" fuel qty = "));
	consolePrintFP(vpStatus.fuel, 3);
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
       
  while((len = stap_hostReceiveState()) > 0) {
    while(len-- > 0)
      datagramRxInputChar(stap_hostReceiveChar());
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

const float pusherBoost_c = 0.2f;
const float pusherBias_c = -1.0f/RADIAN;

void elevatorModule()
{
  static MedianFilter_t elevFilter;
  const float shakerLimit = RATIO(1/3);
  const float effElev = medianInput(&elevFilter, vpInput.elev);

  if(vpParam.wowCalibrated && vpStatus.weightOnWheels)
    // Limit elevator nose-up wind up when weight is on wheels
    pidCtrlSetRangeAB(&elevCtrl, -RATIO(2/3), RATIO(1/8));
  else
    // Weight not on wheels, allow more authority
    pidCtrlSetRangeAB(&elevCtrl, -RATIO(2/3), RATIO(1/2));
  
  vpInput.stickForce =
    vpMode.radioFailSafe ? 0 : fmaxf(effElev-shakerLimit, 0)/(1-shakerLimit);
  const float effMaxAlpha
    = mixValue(vpInput.stickForce,
	       vpDerived.shakerAlpha, vpDerived.pusherAlpha);
  
  vpOutput.elev =
    applyExpoTrim(effElev,
		  vpMode.takeOff ? vpParam.takeoffTrim : vpControl.elevTrim);

  vpControl.targetAlpha = fminf(alphaPredict(vpOutput.elev), effMaxAlpha);
  
  if(vpStatus.flare) {
    const float flareAlpha = 
      mixValue(vpParam.flare * vpInput.stickForce,
	       vpControl.targetAlpha, alphaPredict(effElev));

    vpControl.targetAlpha = fmaxf(vpControl.targetAlpha, flareAlpha);
  }
  
  if(vpMode.radioFailSafe)
    vpControl.targetAlpha = slopeInput(&trimRateLimiter, vpControl.targetAlpha, controlCycle);
  else
    slopeReset(&trimRateLimiter, vpControl.targetAlpha);
    
  if(vpFeature.alphaHold) {
    const float maxPitch =
      mixValue(1 - vpDerived.minimumDynP/vpFlight.dynP,
	       asin(turbineOutput(&engine)*vpParam.thrust/totalMass())
	       + vpControl.targetAlpha,
	       vpParam.maxPitch);
      
    vpControl.targetPitchR =
      nominalPitchRateLevel(vpFlight.bank, vpControl.targetAlpha)
      + clamp(vpControl.targetAlpha - vpFlight.alpha,
	      -30.0f/RADIAN - vpFlight.pitch, maxPitch - vpFlight.pitch)
      * vpControl.o_P * ( 1 + (vpStatus.stall ? pusherBoost_c : 0) );

  } else
    vpControl.targetPitchR = vpInput.elevExpo*PI_F/2;

  vpControl.elevPredict =
    alphaPredictInverse(vpControl.targetAlpha);

  if(vpFeature.stabilizePitch) {
    if(vpStatus.stall) {
      // Apply a fixed pitch rate bias
      vpControl.targetPitchR += pusherBias_c;

      // Only apply target if it's less than the current rate
      vpControl.targetPitchR = fminf(vpControl.targetPitchR, vpFlight.pitchR);
    }
    
    pidCtrlInput(&elevCtrl, vpControl.targetPitchR - vpFlight.pitchR, controlCycle);
    
    vpOutput.elev = pidCtrlOutput(&elevCtrl);

    if(vpFeature.alphaHold)
      vpOutput.elev += vpControl.elevPredict;
  } else {

    if(vpMode.radioFailSafe)
      vpOutput.elev = vpControl.elevPredict;
    
    pidCtrlReset(&elevCtrl, vpOutput.elev - vpControl.elevPredict, 0.0);
      
    // Pusher

#ifdef HARD_PUSHER
    pidCtrlSetRangeAB(&pushCtrl, vpOutput.elev*RATIO(1/3), vpOutput.elev);
#else
    pidCtrlSetRangeAB(&pushCtrl, -vpOutput.elev*RATIO(2/3), 0);
#endif

    vpControl.pusher = 0;
    
    if(vpFeature.pusher) {
      // Pusher active
        
      const float target = nominalPitchRate(vpFlight.bank, vpFlight.pitch, vpControl.targetAlpha)
	+ (effMaxAlpha - vpFlight.alpha) * vpControl.o_P
	* (1 + (vpStatus.stall ? pusherBoost_c : 0) )
	+ (vpStatus.stall ? pusherBias_c : 0);

      pidCtrlInput(&pushCtrl, target - vpFlight.pitchR, controlCycle);

#ifdef HARD_PUSHER
      vpControl.pusher = fminf(pidCtrlOutput(&pushCtrl) - vpOutput.elev, 0);
#else
      vpControl.pusher = pidCtrlOutput(&pushCtrl);
#endif

      vpOutput.elev += vpControl.pusher;
    } else
#ifdef HARD_PUSHER
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
  float maxBank = 45.0f/RADIAN;

  if(vpMode.radioFailSafe) {
    maxBank = 15.0f/RADIAN;
    if(vpStatus.stall)
      vpInput.aile = vpInput.aileExpo = 0;
  } else if(vpFeature.alphaHold)
    maxBank /= 1 + alphaPredict(vpControl.elevTrim)/vpDerived.thresholdAlpha/2;
  
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

      // Bank limiter
      
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
    = slopeInput(&aileActuator, constrainServoOutput(vpOutput.aile), controlCycle);
}

//
//   Rudder & nose wheel
//

void rudderModule()
{
  // Apply stick input
  
  vpOutput.rudder = vpOutput.steer = vpInput.rudder;
    
  if(vpInput.rudderPilotInput || vpMode.takeOff || vpStatus.weightOnWheels
     || vpMode.sensorFailSafe)
    // Pilot input is present/takeoff mode/WoW, keep auto-rudder reset
    pidCtrlReset(&rudderCtrl, 0, 0);
  else
    // No pilot input, auto-rudder is active
   pidCtrlInput(&rudderCtrl, vpFlight.ball, controlCycle);

  // Apply auto-rudder
  
  vpOutput.rudder += pidCtrlOutput(&rudderCtrl);

  // Apply yaw damper
  
  vpOutput.rudder -= vpControl.yd_P * washoutInput(&yawDamper, vpFlight.yawR);
}

//
//   Throttle
//
  
void throttleModule()
{
  if(vpMode.takeOff)
    // Taking off, don't want lag
    turbineReset(&engine, vpInput.throttle);
  else if(!vpStatus.aloft || vpMode.radioFailSafe)
    // Not taking off or aloft, keep closed
    turbineReset(&engine, 0);
  else if(vpParam.wowCalibrated && vpStatus.weightOnWheels)
    // We've landed, direct control is nice so we can stop for full stop
    // landings and get back up quickly for touch and gos
    turbineReset(&engine, vpInput.throttle);
  else
    // We're flying, apply idle and lag settings
    turbineInput(&engine, vpParam.idle + vpInput.throttle*(1 - vpParam.idle));
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
      
    vpOutput.brake = (depressed || vpControl.parking) ? 1 : pedal;
  }
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
    constrainServoOutput(vpOutput.elev
			 + scaleByIAS_E(vpControl.t_Mix, -2)
			 * powf(turbineOutput(&engine), vpParam.t_Expo));

  // Aile to rudder mix

  const float liftRatio = (vpMode.alphaFailSafe | vpStatus.alphaUnreliable)
    ? 0.3f : coeffOfLiftClean(vpFlight.alpha)/vpDerived.maxCoeffOfLiftClean;

  vpOutput.rudder =
    constrainServoOutput(vpOutput.rudder
			 + vpOutput.aile * vpControl.r_Mix * liftRatio);  
}

void controlTask()
{
  uint8_t i = 0;
  
  //
  // Cycle time bookkeeping 
  //
  
  static uint32_t controlCycleEnded;
 
  if(controlCycleEnded > 0)
    controlCycle = (stap_currentMicros - controlCycleEnded)/1.0e6;
  
  controlCycleEnded = stap_currentMicros;

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

    if(functionInvoke(vpParam.functionMap[i], &value)) {
      value += vpParam.neutral[i];
      stap_servoOutput(i, clamp(value, -RATIO(5/4), RATIO(5/4)));
    }
  }
}

void heartBeatTask()
{
  if(!heartBeatCount && linkDownCount++ > 2)
    vpStatus.consoleLink = vpStatus.simulatorLink = false;

  if(vpStatus.simulatorLink && stap_currentMicros - simTimeStamp > 1.0e6) {
    consoleNoteLn_P(CS_STRING("Simulator link LOST"));
    vpStatus.simulatorLink = false;
  }    
  
  heartBeatCount = 0;
  consoleFlush();
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

void simulatorLinkTask()
{
  if(vpStatus.simulatorLink && vpStatus.armed) {
    struct SimLinkControl control = { .aileron = vpOutput.aile,
				      .elevator = -vpOutput.elev,
				      .throttle = turbineOutput(&engine),
				      .rudder = vpOutput.rudder };

    datagramTxStart(DG_SIMLINK);
    datagramTxOut((const uint8_t*) &control, sizeof(control));
    datagramTxEnd();
  }
}

void controlTaskGroup()
{
  deriveParams();
  sensorTaskSync();
  receiverTask();
  controlTask();
  actuatorTask();
  simulatorLinkTask();
}

void configTaskGroup()
{
  if(vpOutput.flap != vpDerived.assumedFlap)
    // Flap setting has changed
    derivedInvalidate();
  
  deriveParams();
  
  statusTask();
  configurationTask();
  trimTask();
}

struct Task alphaPilotTasks[] = {
#ifdef STAP_PERIOD_GYRO
  { gyroTask, STAP_PERIOD_GYRO_STATIC, true, 0 },
#endif
#ifdef STAP_PERIOD_ATTI
  { attiTask, STAP_PERIOD_ATTI, true, 0 },
#endif
#ifdef STAP_PERIOD_ACC
  { accTask, STAP_PERIOD_ACC, true, 0 },
#endif
  { alphaTask, HZ_TO_PERIOD(ALPHA_HZ), true, 0 },
  { airspeedTask, HZ_TO_PERIOD(AIRSPEED_HZ), true, 0 },
  { sensorTaskSlow, HZ_TO_PERIOD(CONTROL_HZ/5), true, 0 },
  { controlTaskGroup, HZ_TO_PERIOD(CONTROL_HZ), true, 0 },
  { configTaskGroup, HZ_TO_PERIOD(CONFIG_HZ), true, 0 },
  { communicationTask, HZ_TO_PERIOD(100), true, 0 },
  // { gpsTask, HZ_TO_PERIOD(100) },
  { blinkTask, HZ_TO_PERIOD(LED_TICK), false, 0 },
  { obdRefresh, HZ_TO_PERIOD(40), false, 0 },
  { displayTask, HZ_TO_PERIOD(8), false, 0 },
  { logTask, HZ_TO_PERIOD(LOG_HZ), true, 0 },
  { logSave, HZ_TO_PERIOD(LOG_HZ_COMMIT), false, 0 },
  { cacheTask, HZ_TO_PERIOD(LOG_HZ_FLUSH), false, 0 },
  { monitorTask, HZ_TO_PERIOD(1), false, 0 },
  { heartBeatTask, HZ_TO_PERIOD(HEARTBEAT_HZ), false, 0 },
  { gaugeTask, HZ_TO_PERIOD(10), false, 0 },
  { NULL, 0, false, 0 } };


