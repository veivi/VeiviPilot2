#include "MainLoop.h"
#include "AlphaPilot.h"
#include "Logging.h"
#include "M24XX.h"
#include "Console.h"
#include "BaseI2C.h"
#include "NVState.h"
#include "MS4525.h"
#include "Command.h"
#include <string.h>
#include <ctype.h>

//
// Datagram protocol integration
//

#define MAX_DG_SIZE (1<<7)

uint16_t maxDatagramSize = MAX_DG_SIZE;
uint8_t datagramRxStore[MAX_DG_SIZE];
bool datagramLocalOnly;

void datagramRxError(const char *error, uint16_t code)
{
  consoleNote_P(CS_STRING("DG "));
  consolePrint(error);
  consolePrint(" (");
  consolePrintUI(code);
  consolePrintLn(")");
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
	vpStatus.simulatorLink = vpMode.dontLog = true;
      }

      memcpy(&sensorData, data, sizeof(sensorData));
      simTimeStamp = stap_currentMicros;
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
  if(vpStatus.consoleLink)
    stap_hostTransmitChar(c);

  if(!datagramLocalOnly)
    stap_telemetryTransmitChar(c);    
}

//
// Scheduler
//

struct Task *currentTask;

static bool scheduler()
{
  struct Task *task = alphaPilotTasks;
  bool status = false;
  
  //  consoleNote("S ");
      
  while(task->code) {
    stap_timeMicros();
    
    if(stap_currentMicros > task->nextInvocation ) {
      if(task->realTime && stap_currentMicros < task->nextInvocation + task->period/3)
	// A realtime task that has not slipped that much, try to catch up
	task->nextInvocation += task->period;
      else
 	// Either not a realtime task or we're slipping too much
	task->nextInvocation = stap_currentMicros + task->period;

      //      consoleNote("Invoking task ");
      // consolePrintLnUL(task->code);

      currentTask = task;
      task->code();
      status = true; // We had something to do
    }
    
    task++;
  }

  //  consoleNote("SX ");
  
  currentTask = NULL;
  return status;
}

//
// Comms test loop
//

void hostLoopback(void)
{
  while(1) {
    if(stap_hostReceiveState() > 0)
      stap_hostTransmitChar(toupper(stap_hostReceiveChar()));
  }
}

//
// Setup and main loop
//

void mainLoopSetup()
{  
  vpStatus.consoleLink = true; // Assume we have link until otherwise etc...

  stap_delayMillis(1000);
  
  consoleNoteLn_P(CS_STRING("Project | Alpha"));   

  stap_initialize();
  
  // Read the non-volatile state

  if(!readNVState())
    consoleNote_P(CS_STRING("NV State read failed."));
    
  consoleNote_P(CS_STRING("Current model is "));
  consolePrintLnI(nvState.model);
  
  // Param record
  
  setModel(nvState.model, false);
  consoleNote_P(CS_STRING("  Model name "));
  consolePrintLn(vpParam.name);
                
  // Initial gear state is DOWN
  
  vpControl.gearSel = 0;
  vpMode.gearSelected = false;

  // Initiate IAS calibration

  MS4525DO_calibrate();

  // Done
  
  consoleNote_P(CS_STRING("Initialized, "));
  consolePrintUL(stap_memoryFree());
  consolePrintLn_P(CS_STRING(" bytes free."));
}

void mainLoop() 
{
  bool idling = false;
  uint32_t idleStarted = 0, idleEnded = 0;
  
  while(true) {
    idleEnded = stap_timeMicros();
    
    if(scheduler()) {
      // Had something to do
      if(idling) {
	// Not idling anymore
	idleMicros += idleEnded - idleStarted;
	idling = false;
      }
    } else if(!idling) {
      // Just started idling
      idling = true;
      
      if(!logReady(false))
	logInit(10);

      idleStarted = stap_timeMicros();
    }
  }
}
