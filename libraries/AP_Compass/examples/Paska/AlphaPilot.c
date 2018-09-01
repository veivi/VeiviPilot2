#include <string.h>
#include "AlphaPilot.h"
#include "StaP.h"
#include "CoreObjects.h"
#include "RxInput.h"
#include "PPM.h"
#include "PWMOutput.h"
#include "Storage.h"
#include "Console.h"
#include "Datagram.h"
#include "CRC16.h"
#include "DSP.h"
#include "Math.h"
#include "NVState.h"
#include "MS4525.h"
#include "SSD1306.h"
#include "AS5048B.h"
#include "Logging.h"
#include "Command.h"
#include "Button.h"
#include "TOCTest.h"

//
// Misc local variables
//

static uint16_t sensorHash = 0xFFFF;
uint8_t datagramRxStore[MAX_DG_SIZE];

//
// Datagram protocol integration
//

void datagramRxError(const char *error)
{
  consoleNote_P(CS_STRING("DG "));
  consolePrintLn(error);
}
  
void datagramInterpreter(uint8_t t, uint8_t *data, int size)
{
  switch(t) {
  case DG_HEARTBEAT:
    if(!vpStatus.consoleLink) {
      consoleNoteLn_P(CS_STRING("Console CONNECTED"));
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
	consoleNoteLn_P(CS_STRING("Simulator CONNECTED"));
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
    consoleNote_P(CS_STRING("FUNNY DATAGRAM TYPE "));
    consolePrintLnI(t);
  }
}

void datagramSerialOut(uint8_t c)
{
  stap_hostTransmitChar(c);
}

void datagramSerialFlush()
{
  stap_hostFlush();
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
  
  if(AS5048B_isOnline() && AS5048B_alpha(&raw)) {
    swAvgInput(&alphaFilter, CIRCLE*(float) raw / (1L<<(8*sizeof(raw))));
    sensorHash = crc16(sensorHash, (uint8_t*) &raw, sizeof(raw));
  }
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

  // PPM freq
  
  obdMove(16-5, 1);
  uint8_t freq = (uint8_t) ppmFreq;
  char buffer1[] =
    { '0' + (freq / 10), '0' + (freq % 10), ' ', 'H', 'z', '\0'};
  obdPrint(buffer1);
    
  // Status
  
  if(!vpStatus.armed) {
    obdMove(16-8, 0);
    obdPrintAttr("DISARMED", true);

    for(i = 0; ppmInputs[i] != NULL; i++) {
      obdMove((i%3)*6, 2+i/3);
      float value = inputValue(ppmInputs[i]);
      uint8_t valueInt = MIN((uint8_t) (fabs(value)*100), 99);
      char buffer2[] =
	{ value < 0 ? '-' : ' ',
	  '.', '0' + (valueInt / 10), '0' + (valueInt % 10),
	  '\0'};
      obdPrint(buffer2);
    }
    
    return;
  } else if(vpMode.takeOff) {
    obdMove(16-7, 0);
    obdPrintAttr("TAKEOFF", (count>>2) & 1);
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

  // Bottom status line
    
  if(vpMode.radioFailSafe) {
    obdMove(0,7);
    obdPrint("   ");
    obdPrintAttr("RADIO FAIL", (count>>2) & 1);
    obdPrint("   ");
  } else {
      // T/O/C test status

    bool status = tocTestStatus(tocReportDisplay);

    obdMove(0,7);
    obdPrint("T/O/C ");

    if(!status)
      obdPrintAttr("WARNING", (count>>2) & 1);
    else
      obdPrint("GOOD");

    obdPrint("   ");
  }
}

void airspeedTask()
{
  int16_t raw = 0;
  
  if(MS4525DO_isOnline() && MS4525DO_pressure(&raw)) {
    samplerInput(&iasSampler, (float) raw);
    sensorHash = crc16(sensorHash, (uint8_t*) &raw, sizeof(raw));
  }
}

Derivator_t buttonSlope;
float lazyButtonValue;

void configurationTask();

#define NZ_BIG RATIO(10/100)
#define NZ_SMALL RATIO(5/100)

void receiverTask()
{
  if(inputValid(&aileInput))
    vpInput.aile = applyNullZone(inputValue(&aileInput), NZ_BIG, &vpInput.ailePilotInput);
  
  if(inputValid(&elevInput))
    vpInput.elev = applyNullZone(inputValue(&elevInput), NZ_SMALL, &vpInput.elevPilotInput);

  if(inputValid(&tuningKnobInput))
    vpInput.tuningKnob = inputValue(&tuningKnobInput)*1.05 - 0.05;
    
  if(inputValid(&throttleInput))
    vpInput.throttle = inputValue(&throttleInput);

  flightModeSelectorValue = readSwitch(&flightModeSelector);

#if RX_CHANNELS >= 8
  if(inputValid(&rudderInput))
    vpInput.rudder = applyNullZone(inputValue(&rudderInput), NZ_SMALL, &vpInput.rudderPilotInput);
  
  flapSelectorValue = readSwitch(&flapSelector);
#else
  vpInput.rudder = 0;
  flapSelectorValue = 1;
#endif

  // Button input

  float buttonValue = inputValue(&btnInput);
  
  derivatorInput(&buttonSlope, buttonValue, 1);

  if(fabs(derivatorOutput(&buttonSlope)) < 0.03)
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
    flightModeSelectorValue = -1;
    vpInput.throttle = 0;
    vpInput.aile = -1;
    vpInput.elev = 1;
  }

  //
  // RX failsafe detection
  //
  
  if( vpInput.tuningKnob > 0.75 && flightModeSelectorValue == -1
      && vpInput.throttle < 0.25
      && vpInput.aile < -0.75 && vpInput.elev > 0.75 ) {
    if(!vpMode.radioFailSafe) {
      consoleNoteLn_P(CS_STRING("Radio failsafe mode ENABLED"));
      vpMode.radioFailSafe = true;
      vpMode.alphaFailSafe = vpMode.sensorFailSafe = vpMode.takeOff = false;
      // Allow the config task to react synchronously
      configurationTask();
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
  
  vpFlight.alpha = swAvgOutput(&alphaFilter);
  
  // Dynamic pressure, corrected for alpha
  
  const float pascalsPerPSI_c = 6894.7573, range_c = 2*1.1;
  const float factor_c = pascalsPerPSI_c * range_c / (1L<<(8*sizeof(uint16_t)));
    
  vpFlight.dynP = samplerMean(&iasSampler) * factor_c
    / cos(clamp(vpFlight.relWind, -vpDerived.maxAlpha, vpDerived.maxAlpha));
  
  // Attitude

  stap_Vector3f_t acc, atti, rot;
  
  stap_gyroUpdate();
  stap_gyroRead(&acc, &atti, &rot);

  vpFlight.bank = atti.x;
  vpFlight.pitch = atti.y;
  vpFlight.heading = (360 + (int) (atti.z*RADIAN)) % 360;
  
  // Angular velocities

  vpFlight.rollR = rot.x;
  vpFlight.pitchR = rot.y;
  vpFlight.yawR = rot.z;

  // Acceleration

  vpFlight.accX = acc.x;
  vpFlight.accY = acc.y;
  vpFlight.accZ = acc.z;

  // Altitude data acquisition

  stap_baroUpdate();
  
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
    
  damperInput(&ball, vpFlight.accY);
  damperInput(&iasFilter, vpFlight.IAS);
  damperInput(&iasFilterSlow, vpFlight.IAS);
  vpFlight.slope = vpFlight.alpha - vpParam.offset - vpFlight.pitch;
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
 
  // Idle measurement

  idleAvg = 7*idleAvg/8 + (float) idleMicros/1e6/8;
  idleMicros = 0;

  // PPM monitoring

  ppmFreq = ppmFrameRate();
  
  // Sim link monitoring

  simInputFreq = 1.0e6 * simFrames / (currentTime - prevMonitor);
  simFrames = 0;

  // Log bandwidth

  logBandWidth = 1.0e6 * writeBytesCum / (currentTime - prevMonitor);
  writeBytesCum = 0;
  
  // PPM monitoring

  if(ppmWarnSlow || ppmWarnShort) {
    lastPPMWarn = currentTime;
    ppmWarnSlow = ppmWarnShort = false;
  }
  
  prevMonitor = currentTime;
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
    consoleNoteLn_P(CS_STRING("Alpha/Sensor failsafe DISABLED"));
    vpMode.alphaFailSafe = vpMode.sensorFailSafe = false;
  }
}

void statusTask()
{
  //
  // Random seed from hashed sensor data
  //

  stap_entropyDigest(sensorHash);
  
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

  if(vpFlight.IAS < vpDerived.minimumIAS/3 || fabsf(vpFlight.IAS - damperOutput(&iasFilterSlow)) > 0.5) {
    if(vpStatus.pitotBlocked) {
      consoleNoteLn_P(CS_STRING("Pitot block CLEARED"));
      vpStatus.pitotBlocked = false;
    }
    
    iasLastAlive = currentTime;
  } else if(!vpStatus.simulatorLink
	    && currentTime - iasLastAlive > 10.0e6 && !vpStatus.pitotBlocked) {
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
  } else if(damperOutput(&iasFilter) < vpDerived.minimumIAS*RATIO(2/3)) {
    if(!vpStatus.positiveIAS)
      lastIAS = currentTime;
    else if(currentTime - lastIAS > 0.3e6) {
      consoleNoteLn_P(CS_STRING("Positive airspeed LOST"));
      vpStatus.positiveIAS = false;
    }
  } else {
    if(vpStatus.positiveIAS)
      lastIAS = currentTime;
    else if(currentTime - lastIAS > 0.3e6) {
      consoleNoteLn_P(CS_STRING("We have POSITIVE AIRSPEED"));
      vpStatus.positiveIAS = true;
    }
  }

  //
  // Movement detection
  //
  
  vpFlight.acc = sqrtf(square(vpFlight.accX) + square(vpFlight.accY) + square(vpFlight.accZ));
  
  damperInput(&accAvg, vpFlight.acc);

  float turnRate = sqrt(square(vpFlight.rollR) + square(vpFlight.pitchR) + square(vpFlight.yawR));
  
  bool motionDetected = (!vpStatus.pitotBlocked && vpStatus.positiveIAS)
    || turnRate > 10.0/RADIAN
    || fabsf(vpFlight.acc - damperOutput(&accAvg)) > 0.5;
  
  static uint32_t lastMotion;

  if(motionDetected) {
    if(vpStatus.fullStop) {
      consoleNoteLn_P(CS_STRING("We appear to be MOVING"));
      vpStatus.fullStop = false;
    }
    
    lastMotion = currentTime;

  } else if(currentTime - lastMotion > 5.0e6 && !vpStatus.fullStop) {
    consoleNoteLn_P(CS_STRING("We have FULLY STOPPED"));
    vpStatus.fullStop = true;
    vpStatus.aloft = false;
  }

  //
  // Alpha/accel lockup detection (sensor vane detached?)
  //

  vpFlight.accDir = atan2(vpFlight.accZ, -vpFlight.accX);
  vpFlight.relWind = vpStatus.fault == 3 ? vpFlight.accDir : vpFlight.alpha - vpParam.offset;
  
  if(vpStatus.alphaFailed) {
      // Failed alpha is also unreliable
    
      vpStatus.alphaUnreliable = true;
      lastAlphaLocked = currentTime;
  } else {
    const float diff = fabsf(vpFlight.accDir - vpFlight.relWind),
      disagreement = MIN(diff, 2*PI_F - diff);

    if(vpMode.alphaFailSafe || vpMode.sensorFailSafe || vpMode.takeOff
       || (fabs(vpFlight.alpha) < 60/RADIAN && disagreement > 15/RADIAN)) {
      if(!vpStatus.alphaUnreliable)
	lastAlphaLocked = currentTime;
      else if(currentTime - lastAlphaLocked > 0.1e6) {
	consoleNoteLn_P(CS_STRING("Alpha sensor appears RELIABLE"));
	vpStatus.alphaUnreliable = false;
      }
    } else {
      if(vpStatus.alphaUnreliable)
	lastAlphaLocked = currentTime;
      else if(currentTime - lastAlphaLocked > 0.5e6) {
	consoleNoteLn_P(CS_STRING("Alpha sensor UNRELIABLE"));
	vpStatus.alphaUnreliable = true;
      }
    }
  }
  
  //
  // Stall detection
  //
  
  if(vpStatus.alphaUnreliable || vpMode.alphaFailSafe || vpMode.sensorFailSafe
     || vpMode.takeOff
     || vpFlight.alpha < fmaxf(vpDerived.stallAlpha, vpControl.targetAlpha)) {
    if(!vpStatus.stall)
      lastStall = currentTime;
    else if(currentTime - lastStall > 0.1e6) {
      consoleNoteLn_P(CS_STRING("Stall RECOVERED"));
      vpStatus.stall = false;
    }
  } else {
    if(vpStatus.stall)
      lastStall = currentTime;
    else if(currentTime - lastStall > 0.1e6) {
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
      
    lastWoW = currentTime;
  } else if(vpStatus.positiveIAS
	    && (liftAvg < weight/2 || liftAvg > 1.5*weight
		|| lift < liftExpected + liftMax/3)) {
    if(!vpStatus.weightOnWheels)
      lastWoW = currentTime;
    else if(currentTime - lastWoW > 0.3e6) {
      consoleNoteLn_P(CS_STRING("Weight is probably OFF THE WHEELS"));
      vpStatus.weightOnWheels = false;
    }
  } else {
    if(vpStatus.weightOnWheels)
      lastWoW = currentTime;
    else if(currentTime - lastWoW > 0.2e6) {
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
     vpInput.throttle < 0.10 && vpInput.aile < -0.90 && vpInput.elev > 0.90) {
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

  if(!vpStatus.silent)
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

      if(vpControl.gearSel)
	consoleNoteLn_P(CS_STRING("Gear UP"));
      else
	consoleNoteLn_P(CS_STRING("Gear DOWN"));
    }
    
    vpMode.autoThrottle = false;

  } else if(buttonDepressed(&GEARBUTTON) && !vpMode.autoThrottle) {
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
      consoleNoteLn_P(CS_STRING("Autothrottle ENABLED"));
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
	    
      vpStatus.silent = false;

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
  // Autothrottle disable
  //

  if(vpMode.autoThrottle && vpMode.slowFlight == (vpInput.throttle > RATIO(1/3))) {
    consoleNoteLn_P(CS_STRING("Autothrottle DISABLED"));
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
      consoleNoteLn_P(CS_STRING("Slow flight mode ENABLED"));
    vpMode.slowFlight = vpMode.bankLimiter = true;
  } else {
    if(vpMode.slowFlight) {
      consoleNoteLn_P(CS_STRING("Slow flight mode DISABLED"));
      vpMode.slowFlight = false;
    }

    if(flightModeSelectorValue == 0) {
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

  vpControl.flapSel = FLAP_STEPS/2 - flapSelectorValue;

  //
  // Test mode control
  //

  if(vpMode.radioFailSafe)
    vpMode.test = false;
  
  else if(!vpMode.test && vpInput.tuningKnob > 0.5) {
    vpMode.test = true;
    consoleNoteLn_P(CS_STRING("Test mode ENABLED"));

  } else if(vpMode.test && vpInput.tuningKnob < 0) {
    vpMode.test = false;
    consoleNoteLn_P(CS_STRING("Test mode DISABLED"));
  }

  // Wing leveler disable when stick input detected
  
  if(vpMode.wingLeveler && vpInput.ailePilotInput
     && fabsf(vpFlight.bank) > 7.5/RADIAN) {
    consoleNoteLn_P(CS_STRING("Wing leveler DISABLED"));
    vpMode.wingLeveler = false;
  }

  // TakeOff mode disabled when airspeed detected (or fails)

  if(vpMode.takeOff && vpStatus.positiveIAS
     && (vpFlight.IAS > vpDerived.minimumIAS || vpFlight.alpha > vpDerived.thresholdAlpha))  {
    consoleNoteLn_P(CS_STRING("TakeOff COMPLETED"));
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

  float s_Ku = scaleByIAS(vpParam.s_Ku_C, stabilityAileExp1_c);
  float i_Ku = scaleByIAS(vpParam.i_Ku_C, stabilityElevExp_c);

  pidCtrlSetZNPID(&aileCtrl, s_Ku*scale, vpParam.s_Tu);  
  pidCtrlSetZNPID(&elevCtrl, i_Ku*scale, vpParam.i_Tu);
  pidCtrlSetZNPID(&pushCtrl, i_Ku*scale, vpParam.i_Tu);

  if(vpMode.slowFlight)
    pidCtrlSetZNPI(&throttleCtrl, vpParam.at_Ku, vpParam.at_Tu);
  else
    pidCtrlSetZNPI(&throttleCtrl, vpParam.cc_Ku, vpParam.cc_Tu);

  outer_P = vpParam.o_P;
  rudderMix = vpParam.r_Mix;
  throttleMix = vpParam.t_Mix;
  
  slopeSet(&aileActuator, vpParam.servoRate/(90.0/2)/vpParam.aileDefl);
  slopeSet(&rollAccelLimiter, rollRatePredict(1) / 0.2);
  
  //
  // Apply test mode
  //
  
  if(vpMode.test && !vpMode.takeOff) {
    switch(nvState.testNum) {
    case 1:
      // Wing stabilizer gain
         
      vpFeature.stabilizeBank = vpMode.bankLimiter = vpFeature.keepLevel = true;
      pidCtrlSetPID(&aileCtrl, vpControl.testGain = testGainExpo(s_Ku_ref), 0, 0);
      break;
            
    case 2:
      // Elevator stabilizer gain, outer loop disabled
         
      vpFeature.stabilizePitch = true;
      vpFeature.alphaHold = false;
      pidCtrlSetPID(&elevCtrl, vpControl.testGain = testGainExpo(i_Ku_ref), 0, 0);
      break;
         
    case 3:
      // Elevator stabilizer gain, outer loop enabled
         
      vpFeature.stabilizePitch = vpFeature.alphaHold = true;
      pidCtrlSetPID(&elevCtrl, vpControl.testGain = testGainExpo(i_Ku_ref), 0, 0);
      break;
         
    case 4:
      // Auto alpha outer loop gain
         
      vpFeature.stabilizePitch = vpFeature.alphaHold = true;
      outer_P = vpControl.testGain = testGainExpo(vpParam.o_P);
      break;

    case 5:
      // Max alpha tests, just flare disabled (because in test mode)
      break;
      
    case 8:
      // Stall behavior test
      
      vpFeature.pusher = false;
      break;
      
    case 10:
      // Aileron to rudder mix

      rudderMix = vpControl.testGain = testGainLinear(vpParam.r_Mix/1.2, vpParam.r_Mix*1.2);
      break;

    case 11:
      // Autothrottle gain (Z-N)
      
      pidCtrlSetPID(&throttleCtrl, vpControl.testGain = testGainExpo(vpParam.at_Ku), 0, 0);
      break;
      
    case 12:
      // Autothrottle gain (empirical)
      
      pidCtrlSetZNPI(&throttleCtrl, vpControl.testGain = testGainExpo(vpParam.at_Ku), vpParam.at_Tu);
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

    case 15:
      // Throttle to elev mix fixed to zero

      throttleMix = 0;
      vpControl.testGain = 1;
      break;

    case 17:
      // Roll acceleration
      
      slopeSet(&rollAccelLimiter, rollRatePredict(1)/(vpControl.testGain = testGainLinear(0.01,2)));
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
    
  if(buttonState(&TRIMBUTTON) || vpMode.radioFailSafe) {
    //
    // Nose wheel
    //
    
    if(vpInput.rudderPilotInput && !vpControl.gearSel && !vpStatus.positiveIAS) {
      vpParam.steerNeutral +=
	sign(vpParam.steerDefl)*sign(vpInput.rudder)*steerTrimRate/TRIM_HZ;
      vpParam.steerNeutral = clamp(vpParam.steerNeutral, -1, 1);
      // paramsModified = true;
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
	    fminf(-0.15, alphaPredictInverse(vpDerived.zeroLiftAlpha)),
	    alphaPredictInverse(vpDerived.thresholdAlpha));
}

void gaugeTask()
{  
  if(gaugeCount > 0) {
    uint16_t tmp = 0;
    int i = 0, g = 0;
    float j = 0;
	
    for(g = 0; g < gaugeCount; g++) {
      switch(gaugeVariable[g]) {
      case 1:
	consolePrint_P(CS_STRING(" alpha = "));
	consolePrintFP(vpFlight.alpha*RADIAN, 1);
	consolePrint_P(CS_STRING(" ("));
	consolePrintFP(fieldStrength*100, 1);
	consolePrint_P(CS_STRING("%)"));
	consoleTab(25);
	consolePrint_P(CS_STRING(" IAS,K(m/s) = "));
	consolePrintI((int) (vpFlight.IAS/KNOT));
	consolePrint_P(CS_STRING(" ("));
	consolePrintFP(vpFlight.IAS, 1);
	consolePrint_P(CS_STRING(")"));
	consoleTab(50);
	consolePrint_P(CS_STRING(" hdg = "));
	consolePrintI(vpFlight.heading);
	consoleTab(65);
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
	consolePrintFP(damperOutput(&ball), 2);
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
	for(i = 0; ppmInputs[i] != NULL; i++) {
	  consolePrintFP(inputValue(ppmInputs[i]), 2);
	  consolePrint(" ");
	}      
	consolePrint(")");
	break;

      case 7:
	consolePrint_P(CS_STRING(" aile(exp) = "));
	consolePrintF(vpInput.aile);
	consolePrint_P(CS_STRING("("));
	consolePrintF(vpInput.aileExpo);
	consolePrint_P(CS_STRING(") elev(exp) = "));
	consolePrintF(vpInput.elev);
	consolePrint_P(CS_STRING("("));
	consolePrintF(vpInput.elevExpo);
	consolePrint_P(CS_STRING(") thr = "));
	consolePrintF(vpInput.throttle);
	consolePrint_P(CS_STRING(" rudder = "));
	consolePrintF(vpInput.rudder);
	consolePrint_P(CS_STRING(" knob = "));
	consolePrintF(vpInput.tuningKnob);
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
	for(j = 0; j < 2; j += 0.5) {
	  if(j > 0)
	    consolePrint(", ");
	  consolePrintF(j);
	}
	
	consolePrint_P(CS_STRING(") = "));
	
	for(j = 0; j < 2; j += 0.5) {
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
	consolePrintF(MS4525DO_entropy());
	consolePrint_P(CS_STRING(" hash = "));
	
	tmp = sensorHash;

	for(i = 0; i < 16; i++) {
	  consolePrint((tmp & 1) ? "+" : " ");
	  tmp = tmp >> 1;
	}

	consolePrintLn("");
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
	consoleTab(40);
	consolePrint_P(CS_STRING(" slope = "));
	consolePrintFP(vpFlight.slope*RADIAN, 1);
	consoleTab(55);
	consolePrint_P(CS_STRING(" THR(auto) = "));
	consolePrintFP(pidCtrlOutput(&throttleCtrl), 2);
	break;
	
      case 14:
       consolePrint_P(CS_STRING(" roll_k = "));
       consolePrintFP(vpFlight.rollR/expo(vpOutput.aile-vpControl.aileNeutral, vpParam.expo)/vpFlight.IAS, 3);
       break;
       
      case 15:
	consolePrint_P(CS_STRING(" elevOutput(trim) = "));
	consolePrintFP(vpOutput.elev, 3);
	consolePrint_P(CS_STRING("("));
	consolePrintFP(vpControl.elevTrim, 3);
	consolePrint_P(CS_STRING(")"));
	break;

      case 20:
	consolePrint_P(CS_STRING(" log bw = "));
	consolePrintI((int) logBandWidth);
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

const float pusherBoost_c = 0.2;
const float pusherBias_c = -2.5/RADIAN;

void elevatorModule()
{
  const float shakerLimit = RATIO(1/2);
  const float stickForce =
    vpMode.radioFailSafe ? 0 : fmaxf(vpInput.elev-shakerLimit, 0)/(1-shakerLimit);
  const float effMaxAlpha
    = mixValue(stickForce, vpDerived.shakerAlpha, vpDerived.pusherAlpha);
  
  vpOutput.elev =
    applyExpoTrim(vpInput.elev, vpMode.takeOff ? vpParam.takeoffTrim : vpControl.elevTrim);

  const bool flareAllowed = !vpMode.test && vpMode.slowFlight
    && vpControl.gearSel == 0 && (fabs(vpFlight.bank) < 30/RADIAN)
    && (vpDerived.haveRetracts || vpFlight.alt < 5)
    && vpInput.throttle < 0.3;

  vpControl.targetAlpha =
    mixValue(flareAllowed ? vpParam.flare * stickForce : 0,
	     fminf(alphaPredict(vpOutput.elev), effMaxAlpha),
	     alphaPredict(vpOutput.elev));

  if(vpMode.radioFailSafe)
    vpControl.targetAlpha = slopeInput(&trimRateLimiter, vpControl.targetAlpha, controlCycle);
  else
    slopeReset(&trimRateLimiter, vpControl.targetAlpha);
    
  if(vpFeature.alphaHold)
    vpControl.targetPitchR =
      nominalPitchRateLevel(vpFlight.bank, vpControl.targetAlpha)
      + clamp(vpControl.targetAlpha - vpFlight.alpha,
	      -30/RADIAN - vpFlight.pitch,
	      clamp(vpParam.maxPitch, 30/RADIAN, 80/RADIAN) - vpFlight.pitch)
      * outer_P * ( 1 + (vpStatus.stall ? pusherBoost_c : 0) );

  else
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
    pidCtrlSetRangeAB(&pushCtrl, vpOutput.elev*RATIO(1/4), vpOutput.elev);
#else
    pidCtrlSetRangeAB(&pushCtrl, -vpOutput.elev*RATIO(3/4), 0);
#endif

    vpControl.pusher = 0;
    
    if(vpFeature.pusher) {
      // Pusher active
        
      const float target = nominalPitchRate(vpFlight.bank, vpFlight.pitch, vpControl.targetAlpha)
	+ (effMaxAlpha - vpFlight.alpha) * outer_P
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
  float maxBank = 45/RADIAN;

  if(vpMode.radioFailSafe) {
    maxBank = 15/RADIAN;
    if(vpStatus.stall)
      vpInput.aile = vpInput.aileExpo = 0;
  } else if(vpFeature.alphaHold)
    maxBank /= 1 + alphaPredict(vpControl.elevTrim) / vpDerived.thresholdAlpha / 2;
  
  float targetRollR = rollRatePredict(vpInput.aileExpo);
  
  // We accumulate individual contributions so start with 0

  vpOutput.aile = 0;
  
  if(vpFeature.stabilizeBank) {
    // Stabilization is enabled
    
    if(vpFeature.keepLevel)
      // Strong leveler enabled
      
      targetRollR = outer_P * (vpInput.aileExpo*60/RADIAN - vpFlight.bank);

    else if(vpMode.bankLimiter) {

      if(vpMode.slowFlight)
	// Weak leveling
	targetRollR -= outer_P*clamp(vpFlight.bank, -2.0/RADIAN, 2.0/RADIAN);

      // Bank limiter
      
      targetRollR =
	clamp(targetRollR,
	      (-maxBank - vpFlight.bank)*outer_P, (maxBank - vpFlight.bank)*outer_P);
    }

    pidCtrlInput(&aileCtrl, targetRollR - vpFlight.rollR, controlCycle);
  } else {
    // Stabilization disabled
    
    pidCtrlReset(&aileCtrl, 0, 0);
    
    if(vpFeature.keepLevel)
      // Simple proportional wing leveler
      vpOutput.aile -= vpFlight.bank + vpFlight.rollR/32;
  }

  //   Apply angular accel limiter

  targetRollR = slopeInput(&rollAccelLimiter, targetRollR, controlCycle);
  
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
  vpOutput.rudder = vpOutput.steer = vpInput.rudder;
}

//
//   Autothrottle
//
  
void throttleModule()
{
  pidCtrlSetRangeAB(&throttleCtrl, vpControl.minThrottle, vpInput.throttle);
    
  if((!vpMode.takeOff && !vpStatus.aloft) || vpMode.radioFailSafe)
    pidCtrlReset(&throttleCtrl, 0, 0);
  
  else if(vpMode.autoThrottle) {
    float thrError = 0;
    
    if(vpMode.slowFlight)
      thrError = vpFlight.slope - vpParam.glideSlope*(RATIO(3/2) - 3*vpInput.throttle);
    else
      thrError = 1 - vpFlight.dynP/vpControl.targetPressure;

    pidCtrlInput(&throttleCtrl, thrError, controlCycle);

  } else 
    pidCtrlReset(&throttleCtrl, vpInput.throttle, 0);
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
    
  if(vpControl.gearSel == 1 || vpInput.elev > 0)
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
			 throttleMix*powf(pidCtrlOutput(&throttleCtrl),
					  vpParam.t_Expo));

  // Aile to rudder mix

  const float lift = (vpMode.alphaFailSafe | vpStatus.alphaUnreliable)
    ? 0.3 : coeffOfLiftClean(vpFlight.alpha)/vpDerived.maxCoeffOfLiftClean;

  vpOutput.rudder =
    constrainServoOutput(vpOutput.rudder + vpOutput.aile * rudderMix * lift);  
}

void controlTask()
{
  int i = 0;
  
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

  for(i = 0; i < sizeof(controlModules)/sizeof(void(*)()); i++)
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
  return vpParam.flaperon ? vpParam.flapDefl*vpOutput.flap : 0;
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
  if(vpDerived.haveRetracts && vpControl.gearSel)
    return vpParam.steerPark;
  else
    return vpParam.steerNeutral + vpParam.steerDefl*vpOutput.steer;
}

float leftFlapFn()
{
  return vpParam.flapNeutral + vpParam.flapDefl*vpOutput.flap;
}

float rightFlapFn()
{
  return vpParam.flap2Neutral - vpParam.flapDefl*vpOutput.flap;
}

float gearFn()
{
  return -RATIO(2/3)*(vpControl.gearSel*2-1);
}

float brakeFn()
{
  return vpParam.brakeDefl*vpOutput.brake + vpParam.brakeNeutral;
}

float throttleFn()
{
  return THROTTLE_SIGN*RATIO(2/3)*(2*pidCtrlOutput(&throttleCtrl) - 1);
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
  int i = 0;
  
  if(!vpStatus.armed)
    return;

  for(i = 0; i < MAX_SERVO; i++)
    if(functionTable[vpParam.functionMap[i]])
      pwmOutputWrite(i, clamp(functionTable[vpParam.functionMap[i]](), -1, 1));
}

void heartBeatTask()
{
  if(!heartBeatCount && linkDownCount++ > 2)
    vpStatus.consoleLink = vpStatus.simulatorLink = false;

  if(vpStatus.simulatorLink && currentTime - simTimeStamp > 1.0e6) {
    consoleNoteLn_P(CS_STRING("Simulator link LOST"));
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

  const struct PinDescriptor led = { PortA, 5 };

  configureOutput(&led);
  setPinState(&led, tick < ledRatio*LED_TICK/LED_HZ ? 0 : 1);
}

void simulatorLinkTask()
{
  if(vpStatus.simulatorLink && vpStatus.armed) {
    struct SimLinkControl control = { .aileron = vpOutput.aile,
				      .elevator = -vpOutput.elev,
				      .throttle = pidCtrlOutput(&throttleCtrl),
				      .rudder = vpOutput.rudder };

    datagramTxStart(DG_SIMLINK);
    datagramTxOut((const uint8_t*) &control, sizeof(control));
    datagramTxEnd();
  }
}

void controlTaskGroup()
{
  sensorTaskSync();
  receiverTask();
  controlTask();
  actuatorTask();
}

void configTaskGroup()
{
  statusTask();
  configurationTask();
  trimTask();

  if(vpOutput.flap != vpDerived.assumedFlap)
    // Update CoL curve
    deriveParams();
}

struct Task alphaPilotTasks[] = {
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
    HZ_TO_PERIOD(7) },
  { controlTaskGroup,
    HZ_TO_PERIOD(CONTROL_HZ) },
  { simulatorLinkTask,
    HZ_TO_PERIOD(CONTROL_HZ) },
  { sensorTaskSlow,
    HZ_TO_PERIOD(CONTROL_HZ/5) },
  { configTaskGroup,
    HZ_TO_PERIOD(CONFIG_HZ) },
  { logTask,
    HZ_TO_PERIOD(LOG_HZ) },
  { logSave,
    HZ_TO_PERIOD(LOG_HZ_COMMIT) },
  { cacheTask,
    HZ_TO_PERIOD(LOG_HZ_FLUSH) },
  { monitorTask,
    HZ_TO_PERIOD(1) },
  { heartBeatTask,
    HZ_TO_PERIOD(HEARTBEAT_HZ) },
  { gaugeTask,
    HZ_TO_PERIOD(10) },
  { NULL } };

