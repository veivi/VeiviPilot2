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
uint8_t datagramRxStore[MAX_DG_SIZE*2];
bool datagramLocalOnly;

void datagramRxError(const char *error, uint16_t code)
{
  consoleNote_P(CS_STRING("DG "));
  consolePrint(error);
  consolePrint(" (");
  consolePrintUI(code);
  consolePrintLn(")");
}
  
void datagramInterpreterKind(uint8_t port, uint8_t kind, const uint8_t *data, int size)
{
  switch(kind) {
  case DG_HEARTBEAT:
    //    consoleNoteLn_P(CS_STRING("HEARTBEAT"));

    if(port == 0 && !vpStatus.consoleLink) {
      consoleNoteLn_P(CS_STRING("Console CONNECTED"));
      vpStatus.consoleLink = true;
      
    } else if(port == 1 && !vpStatus.telemetryLink) {
      consoleNoteLn_P(CS_STRING("Telemetry CONNECTED"));
      vpStatus.telemetryLink = true;
    }
    break;
    
  case DG_CONSOLE:
    if(port == 0)
      executeCommand((char*) data);
    break;

  case DG_SIMLINK:
    if(port == 0 && vpStatus.consoleLink && size == sizeof(sensorData)) {
      if(!vpStatus.simulatorLink) {
	consoleNoteLn_P(CS_STRING("Simulator CONNECTED"));
	vpStatus.simulatorLink = vpMode.dontLog = true;
      }

      memcpy(&sensorData, data, sizeof(sensorData));
      simTimeStamp = vpTimeMillisApprox;
      simFrames++;    
    }
    break;

  case DG_PING:
    break;
    
  default:
    consoleNote_P(CS_STRING("FUNNY DATAGRAM KIND "));
    consolePrintI(kind);
    consolePrint_P(CS_STRING(" FROM PORT "));
    consolePrintLnI(port);
  }
}

void datagramInterpreter(uint8_t port, const uint8_t *data, int size)
{
  if(size < 1) {
    datagramRxError("EMPTY", 0);
    return;
  }

  if(port > 1) {
    datagramRxError("WEIRD PORT", 0);
    return;
  }

  heartBeatCount[port]++;
  linkDownCount[port] = 0;
    
  datagramInterpreterKind(port, data[0], &data[1], size - 1);
}

void datagramSerialOut(const uint8_t *b, int l)
{
  if(vpStatus.consoleLink)
    STAP_LinkPut(HOSTTX, b, l);

  if(!datagramLocalOnly)
    STAP_LinkPut(TELEMTX, b, l);
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
    vpTimeAcquire();
    
    if(VP_ELAPSED_MILLIS(task->lastInvoked, vpTimeMillisApprox) >= task->period
       || (task->signal && *task->signal)) {      
      if(task->realTime
	 && vpTimeMillisApprox - task->lastInvoked < 4*task->period/3)
	// A realtime task that has not slipped that much, try to catch up
	task->lastInvoked += task->period;
      else {
 	// Either not a realtime task or we're slipping too much
	task->lastInvoked = vpTimeMillisApprox;
	task->lagged++;
      }
      //      consoleNote("Invoking task ");
      // consolePrintLnUL(task->code);

      currentTask = task;
      
      VP_TIME_MICROS_T startTime = vpTimeMicrosApprox;

      task->code();

      if(task->signal && *task->signal) {
	task->triggered++;
	*task->signal = false;
      }
      
      if(vpStatus.consoleLink) 
	task->runTime += vpTimeMicros() - startTime;
      
      task->timesRun++;
      status = true; // We had something to do
    }
    
    task++;
  }

  //  consoleNote("SX ");
  
  currentTask = NULL;
  return status;
}

void schedulerReport(void)
{
  static VP_TIME_MICROS_T lastReport;
  VP_TIME_MICROS_T period = vpTimeMicros() - lastReport;
  float load = 0.0f, cum = 0.0f;
  int i = 0;  
      
  consoleNoteLn_P(CS_STRING("Task statistics"));

  for(i = 0; alphaPilotTasks[i].code != NULL; i++) {
    consoleNote("  ");
    consolePrintI(i);
    consoleTab(10);
    consolePrintF(alphaPilotTasks[i].timesRun / (period / 1.0e6));
    consolePrint(" Hz ");
    load = 100.0f * alphaPilotTasks[i].runTime / period;
    cum += load;
    consoleTab(20);
    consolePrintF(load);
    consolePrint(" %");
    consoleTab(30);
    if(alphaPilotTasks[i].signal) {
      consolePrintF((float) alphaPilotTasks[i].triggered / (period / 1.0e6));
      consolePrint(" Hz ");
    }
    consoleTab(40);
    if(alphaPilotTasks[i].realTime) {
      consolePrintF((float) alphaPilotTasks[i].lagged / (period / 1.0e6));
      consolePrint(" Hz ");
    }
    consoleNL();
    alphaPilotTasks[i].timesRun = 0;
    alphaPilotTasks[i].lagged = 0;
    alphaPilotTasks[i].triggered = 0;
    alphaPilotTasks[i].runTime = 0;
  }

  consoleTab(20);
  consolePrintF(cum);
  consolePrintLn(" %");

  lastReport = vpTimeMicros();

  consoleNote_P(CS_STRING("Uptime "));
  consolePrintUL(uptimeMinutes);
  consolePrintLn_P(CS_STRING(" minutes"));
}

//
// Comms test loop
//

void hostLoopback(void)
{
  while(1) {
    if(STAP_LinkStatus(HOSTRX) > 0)
      STAP_LinkPutChar(HOSTTX, toupper(STAP_LinkGetChar(HOSTRX)));
  }
}

//
// Setup and main loop
//

void mainLoopSetup()
{  
  vpStatus.consoleLink = true; // Assume we have link until otherwise etc...

  vpDelayMillis(1000);
  
  consoleNoteLn_P(CS_STRING("Project | Alpha"));   

  stap_initialize();
  
  // Initial gear state is DOWN
  
  vpControl.gearSel = 0;
  vpMode.gearSelected = false;

  // Initialization task state

  vpControl.initState = it_init;
  
  // Done
  
  consoleNote_P(CS_STRING("Initialized, "));
  consolePrintUL(stap_memoryFree());
  consolePrintLn_P(CS_STRING(" bytes free."));
}

VPPeriodicTimer_t minuteTimer = VP_PERIODIC_TIMER_CONS(60.0e3);

void mainLoop() 
{
  bool idling = false;
  VP_TIME_MICROS_T idleStarted = 0, idleEnded = 0;

  while(true) {
    idleEnded = vpTimeMicros();

    if(vpPeriodicEvent(&minuteTimer)) {
      uptimeMinutes++;

      if(uptimeMinutes % 5 == 0) {
	consoleNote_P(CS_STRING("Uptime "));
	consolePrintUL(uptimeMinutes);
	consolePrintLn_P(CS_STRING(" minutes"));
	consoleFlush();
      }
    }

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
      
      if(vpControl.initState == it_done && !logReady(false))
	logInit(20);

      idleStarted = vpTimeMicros();
    }
}
}
