#include "Objects.h"
#include "AlphaPilot.h"

extern "C" {
#include "Logging.h"
#include "Math.h"
#include "Storage.h"
#include "Console.h"
#include "Time.h"
#include "CRC16.h"
#include "Serial.h"
#include "BaseI2C.h"
#include "NVState.h"
#include "MS4525.h"
#include "PWMOutput.h"
#include "InputOutput.h"
#include "PPM.h"
#include "Command.h"
}

//
// Datagram protocol integration
//

#define MAX_DG_SIZE  (1<<7)

extern "C" {

#include "Datagram.h"

int maxDatagramSize = MAX_DG_SIZE;
uint8_t datagramRxStore[MAX_DG_SIZE];

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
  serialOut(c);
}

void datagramSerialFlush()
{
  serialFlush();
}
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

bool scheduler()
{
  struct Task *task = alphaPilotTasks;
  
  while(task->code) {
    if(task->lastExecuted + task->period < currentTime
      || task->lastExecuted > currentTime) {
      task->code();
      task->lastExecuted = currentTime;
      
      return true; // We had something to do
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
  
  consoleNoteLn_P(CS_STRING("Project | Alpha"));   

  // PWM output

  consoleNoteLn_P(CS_STRING("Initializing PWM output"));
  pwmOutputInit();

  // I2C
  
  consoleNote_P(CS_STRING("Initializing I2C... "));

  basei2cInit();
  basei2cSetSpeed(true);
  basei2cSetPullup(true);
  basei2cSetTimeOut(2+EXT_EEPROM_LATENCY);

  consolePrintLn_P(CS_STRING("done. "));
  
  // Read the non-volatile state

  if(!readNVState())
    consolePanic_P(CS_STRING("NV State read failed."));
    
  consoleNote_P(CS_STRING("Current model is "));
  consolePrintLnI(nvState.model);
  
  // Param record
  
  setModel(nvState.model, true);
                
  // RC input
  
  consoleNoteLn_P(CS_STRING("Initializing PPM receiver"));

  ppmInputInit(nvState.rxMin, nvState.rxCenter, nvState.rxMax);

  // Misc sensors
  
  consoleNote_P(CS_STRING("Initializing barometer... "));
  consoleFlush();

  barometer.init();
  barometer.calibrate();
  
  consolePrintLn_P(CS_STRING("  done"));
  
  consoleNote_P(CS_STRING("Initializing INS/AHRS... "));
  consoleFlush();
  
  ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_50HZ);
  ahrs.init();

  consolePrintLn_P(CS_STRING("  done"));

#ifdef USE_COMPASS
  consoleNote_P(CS_STRING("Initializing compass... "));
  consoleFlush();

  if(compass.init()) {
    consolePrint_P(CS_STRING("  done, "));
    consolePrintI(compass.get_count());
    consolePrintLn_P(CS_STRING(" sensor(s) detected."));
  
    ahrs.set_compass(&compass);
  
    compass.set_and_save_offsets(0,0,0,0);
    compass.set_declination(ToRad(0.0f));
    
  } else {
    consolePanic_P(CS_STRING("  FAILED."));
  }
#endif

  // Static controller settings & filters

  aileCtrl.limit(RATIO(2/3));
  
  slopeInit(&flapActuator, 0.5);
  slopeInit(&trimRateLimiter, 3/RADIAN);
  
  damperInit(&ball, 1.5*CONTROL_HZ, 0);
  damperInit(&iasFilter, 2, 0);
  damperInit(&iasFilterSlow, 3*CONTROL_HZ, 0);
  damperInit(&accAvg, 2*CONTROL_HZ, G);
  damperInit(&iasEntropy, CONFIG_HZ, 0);
  damperInit(&alphaEntropy, CONFIG_HZ, 0);
  
  swAvgInit(&alphaFilter, ALPHAWINDOW*ALPHA_HZ);
  swAvgInit(&liftFilter, CONFIG_HZ/4);

  // Initial gear state is DOWN
  
  gearSel = 0;

  // Initiate IAS calibration

  MS4525DO_calibrate();

  // Done
  
  consoleNote_P(CS_STRING("Initialized, "));
  consolePrintUL((unsigned long) hal.util->available_memory());
  consolePrintLn_P(CS_STRING(" bytes free."));
  
  datagramTxStart(DG_INITIALIZED);
  datagramTxEnd();
}

void loop() 
{
  while(true) {
    // Invoke scheduler
  
    currentMicros();

    if(!scheduler())
      // Idle
      
      backgroundTask(1);
  }
}

AP_HAL_MAIN();
