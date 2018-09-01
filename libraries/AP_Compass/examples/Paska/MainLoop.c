#include "MainLoop.h"
#include "AlphaPilot.h"
#include "Logging.h"
#include "Storage.h"
#include "Console.h"
#include "BaseI2C.h"
#include "NVState.h"
#include "MS4525.h"
#include "PWMOutput.h"
#include "PPM.h"

static void backgroundTask(uint32_t duration)
{
  uint32_t idleStart = currentMicros();
  
  if(!logReady(false))
    logInit(duration);
  else
    delayMicros(duration*1e3);

  idleMicros += currentMicros() - idleStart;
}

static bool scheduler()
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

void mainLoopSetup()
{
  // Low level init & welcome
  
  stap_boot();  
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
  stap_baroInit();
  consolePrintLn_P(CS_STRING("  done"));
  
  consoleNote_P(CS_STRING("Initializing INS/AHRS... "));
  consoleFlush();
  stap_gyroInit();
  consolePrintLn_P(CS_STRING("  done"));

  // Static controller settings & filters

  pidCtrlInit(&elevCtrl);
  pidCtrlInit(&pushCtrl);
  pidCtrlInit(&throttleCtrl);
  pidCtrlInitUnwinding(&aileCtrl);
  
  pidCtrlSetRange(&aileCtrl, RATIO(2/3));
  
  slopeInit(&aileActuator, 0);
  slopeInit(&rollAccelLimiter, 0);
  slopeInit(&flapActuator, 0.5);
  slopeInit(&trimRateLimiter, 3/RADIAN);
  
  damperInit(&ball, 1.5*CONTROL_HZ, 0);
  damperInit(&iasFilter, 2, 0);
  damperInit(&iasFilterSlow, 3*CONTROL_HZ, 0);
  damperInit(&accAvg, 2*CONTROL_HZ, G);
  
  swAvgInit(&alphaFilter, ALPHAWINDOW*ALPHA_HZ);
  swAvgInit(&liftFilter, CONFIG_HZ/4);

  // Initial gear state is DOWN
  
  vpControl.gearSel = 0;

  // Initiate IAS calibration

  MS4525DO_calibrate();

  // Done
  
  consoleNote_P(CS_STRING("Initialized, "));
  consolePrintUL(stap_memoryFree());
  consolePrintLn_P(CS_STRING(" bytes free."));
  
  datagramTxStart(DG_INITIALIZED);
  datagramTxEnd();
}

void mainLoop() 
{
  while(true) {
    // Invoke scheduler
  
    currentMicros();

    if(!scheduler())
      // Idle
      
      backgroundTask(1);
  }
}