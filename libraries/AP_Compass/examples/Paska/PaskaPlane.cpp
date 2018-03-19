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
#include "AlphaPilot.h"
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
// Datagram protocol integration
//

#include "Serial.h"

#define MAX_DG_SIZE  (1<<7)

extern "C" {

#include "Datagram.h"

int maxDatagramSize = MAX_DG_SIZE;
uint8_t datagramRxStore[MAX_DG_SIZE];

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

  ppmInputInit(nvState.rxMin, nvState.rxCenter, nvState.rxMax);

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
  /*
  configureOutput(&RED_LED);
  configureOutput(&GREEN_LED);
  configureOutput(&BLUE_LED);

  setPinState(&RED_LED, 1);
  setPinState(&GREEN_LED, 1);
  setPinState(&BLUE_LED, 1);
  */
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
