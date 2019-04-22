#include <stdarg.h>
#include <string.h>
#include "AlphaPilot.h"
#include "StaP.h"
#include "Logging.h"
#include "M24XX.h"
#include "Console.h"
#include "Datagram.h"
#include "NVState.h"

typedef enum { invalid_c, find_stamp_c, find_start_c, ready_c, stop_c, run_c, failed_c } logState_t;

static logState_t logState;
static int32_t logPtr, logLen, logSize;
static uint16_t logEndStamp;
static bool logEnabled = false;

#define logOffset nvState.logPartition

static uint16_t logRead(int32_t index);

bool logReady(bool verbose)
{
  if(logState == stop_c || logState == run_c)
    return true;

  if(verbose)
    consoleNoteLn_P(CS_STRING("Log not ready"));
  
  return false;
}

bool logReadyVerbose(void)
{
  return logReady(true);
}

uint32_t logAddr(int32_t index)
{
  return logOffset + index*sizeof(uint16_t);
}

static int uncommitted = 0;
static uint16_t nextStamp;

static bool logWrite(int32_t index, const uint16_t *value, int count)
{
  bool status = false;
  
  if(logSize < 1)
    return false;

  if(index+count > logSize) {
    int32_t p = logSize - index;
    status = m24xxWrite(logAddr(index), (const uint8_t*) value, p*sizeof(uint16_t))
      && m24xxWrite(logAddr(0), (const uint8_t*) &value[p], (count-p)*sizeof(uint16_t));
  } else
    status = m24xxWrite(logAddr(index), (const uint8_t*) value, count*sizeof(uint16_t));

  uncommitted = count;
  return status;
}

uint16_t logRead(int32_t index)
{
  if(logSize < 1)
    return 0xFFFF;
    
  uint16_t entry = 0;
  
  m24xxRead(logAddr(index), (uint8_t*) &entry, sizeof(entry));
  
  return entry;
}

static void logCommit(void)
{
  if(!uncommitted)
    return;
    
  logPtr = logIndex(uncommitted);
  logLen += uncommitted;
  
  if(logLen > logSize)
    logLen = logSize;

  uncommitted = 0;
}

static bool logEnterGeneric(const uint16_t *value, int count)
{
  bool status = logWrite(logPtr, value, count);
  logCommit();

  nextStamp = ENTRY_VALUE(logEndStamp+1);
  uint16_t buffer[] = { ENTRY_TOKEN(t_stamp), nextStamp };

  if(!logWrite(logPtr, buffer, sizeof(buffer) / sizeof(uint16_t)))
    status = false;

  if(!status)
    logDisable();
}

static void logEnter(uint16_t value)
{
  logEnterGeneric(&value, 1);
}

void logClear(void)
{
  bool wasNotEmpty = logLen > 0;
  
  consoleNoteLn_P(CS_STRING("Log being CLEARED"));
  
  logEnter(ENTRY_TOKEN(t_start));
  
  logLen = 0;
    
  if(wasNotEmpty) {
    nvState.logStamp++;
    storeNVState();
    consoleNote_P(CS_STRING("Log STAMP incremented to "));
    consolePrintLnUI(nvState.logStamp);
  }    
}

static int prevCh = -1;

static void logWithCh(ChannelId_t ch, uint16_t value, bool force)
{
  if(logState != run_c)
    return;

  value = ENTRY_VALUE(value);    // Constrain to valid range
  
  if(ch != lc_alpha && value == logChannels[ch].value && !force)
    // Repeat, non-alpha and recent value, not stored
    
    return;
            
  if(prevCh > -1 && ch == prevCh + 1) {
    // Channel prediction good, just store value
    
    logEnter(value);
  } else {
    // Set channel first
    
    uint16_t buffer[] = { ENTRY_TOKEN(t_channel + ch), value };
    logEnterGeneric(buffer, sizeof(buffer)/sizeof(uint16_t));
  }
    
  logChannels[ch].value = value;
  prevCh = ch;
}

void logFloat(ChannelId_t ch, float value, bool force)
{
  float small = logChannels[ch].small, large = logChannels[ch].large;
  
  logWithCh(ch, (uint16_t)
	    clamp((float) VALUE_MASK*((value-small)/(large-small)),
		  0, VALUE_MASK), force);
}

void logInteger(ChannelId_t ch, uint16_t value, bool force)
{
  logWithCh(ch, value & VALUE_MASK, force);
}

void logMark(void)
{
  if(!logEnabled)
    return;
  
  logEnter(ENTRY_TOKEN(t_mark));
}

void logEnable()
{
  int i = 0;
  
  if(logEnabled)
    return;
    
  logEnabled = true;
  
  for(i = 0; i < lc_channels; i++)
    logChannels[i].value = TOKEN_MASK;
  
  prevCh = -1;  
  
  consoleNoteLn_P(CS_STRING("Logging ENABLED"));
}

void logDisable()
{
  if(!logEnabled)
    return;
    
  logEnabled = false;
  
  consoleNoteLn_P(CS_STRING("Logging DISABLED"));
}

void logDumpBinary(void)
{
  if(!logReadyVerbose())
    return;
  
  struct LogInfo info = {
    nvState.logStamp,
    nvState.testNum,
    logLen,
    LOG_HZ,
    vpParam.weightDry,
    "\0" };
  
  strncpy(info.name, vpParam.name, NAME_LEN);

  datagramTxStart(DG_LOGINFO);    
  datagramTxOut((const uint8_t*) &info, sizeof(info));
  datagramTxEnd();

  consoleNoteLn_P(CS_STRING("PARAMETER RECORD"));
  printParams();
  
  consolePrintLn("");
  consoleNote_P(CS_STRING("TEST CH = "));
  consolePrintLnUI(nvState.testNum);
  consolePrintLn("");

  int32_t total = 0, block = 0;

  datagramTxStart(DG_LOGDATA);

  while(total < logLen) {
    uint8_t *buffer = NULL;

    int good = m24xxReadIndirect(logAddr(logIndex(total-logLen)),
				 &buffer, sizeof(uint16_t)*(logLen-total));

    datagramTxOut(buffer, good);

    block += good;
    
    if(block > 1024-1) {
      datagramTxEnd();
      datagramTxStart(DG_LOGDATA);
      block = 0;
    }

    total += good/sizeof(uint16_t);
  }

  datagramTxEnd();

  // Finish with an empty block
  
  datagramTxStart(DG_LOGDATA);
  datagramTxEnd();
}

bool logInit(uint32_t maxDuration)
{
  uint32_t current = stap_timeMillis();
  static int32_t endPtr = -1, startPtr = -1, searchPtr = 0;
  static bool endFound = false;
  uint32_t eepromSize = 0;
  uint8_t dummy;
  
  switch(logState) {
  case invalid_c:
    logSize = 0;
    
    while(m24xxReadDirect(eepromSize+(1<<10)-1, &dummy, 1))
      eepromSize += 1<<10;
    
    if(m24xxReadDirect(eepromSize-(1<<10), &dummy, 1) || m24xxReadDirect(eepromSize-(1<<10), &dummy, 1)) {
      consoleNote_P(CS_STRING("EEPROM size = "));
      consolePrintUL(eepromSize/(1<<10));
      consolePrintLn("k bytes");
      
      logSize = (eepromSize - logOffset)/sizeof(uint16_t);

      consoleNote_P(CS_STRING("Inferred log size = "));
      consolePrintUL(logSize/(1<<10));
      consolePrint("k + ");
      consolePrintUL(logSize%(1<<10));
      consolePrintLn(" entries");
      
      logState = find_stamp_c;
      logPtr = 0;
      logLen = -1;

      return false;
    } else {
      consoleNoteLn_P(CS_STRING("Log EEPROM failed"));
      logState = failed_c;
    }
    break;

  case find_stamp_c:
    while(searchPtr < logSize) {
      if(searchPtr % (1<<12) == 0) {
	consoleNote_P(CS_STRING("  Searching for log STAMP at "));
	consolePrintUL(searchPtr);
	consolePrintLn("...");
      }

      uint16_t entry = logRead(searchPtr);
      
      if(entry >= ENTRY_TOKEN(t_start) && entry <= ENTRY_TOKEN(t_start+BYTE_MASK)) {
	startPtr = searchPtr;
      } else if(entry == ENTRY_TOKEN(t_stamp)) {
        uint16_t stamp = logRead(logIndex(searchPtr+1));

        if(endFound && stamp != ENTRY_VALUE(logEndStamp+1))
          break;
            
        endPtr = searchPtr;
        logEndStamp = stamp;
        endFound = true;

        searchPtr++;
      }
      
      searchPtr++;
      
      if(stap_timeMillis() - current > maxDuration)
        // Stop for now
        return false;
    }

    if(endFound) {
      logPtr = logIndex(endPtr+2);
      
      if(startPtr < 0) {
	// Don't know the log length yet
	consoleNoteLn_P(CS_STRING("Log STAMP found, looking for START"));
	logState = find_start_c;
      }	else {
	logState = ready_c;
       }
    } else {
      consoleNoteLn_P(CS_STRING("Log appears corrupted"));
      consoleNoteLn_P(CS_STRING("Log being INITIALIZED"));
  
      logEndStamp = 0;
      logPtr = logSize-1;
      logLen = 0;
      logClear();
      logState = ready_c;
    }
    break;

  case find_start_c:
    while(searchPtr < logSize) {
      if(searchPtr % (1<<12) == 0) {
       	consoleNote_P(CS_STRING("  Searching for log START at "));
	consolePrintUL(searchPtr);
	consolePrintLn("...");
      }

      uint16_t entry = logRead(searchPtr);
      
      if(entry >= ENTRY_TOKEN(t_start) && entry <= ENTRY_TOKEN(t_start+BYTE_MASK)) {
	startPtr = searchPtr;
      }
      
      searchPtr++;
      
      if(stap_timeMillis() - current > maxDuration)
        // Stop for now
        return false;
    }

    logState = ready_c;
    break;

  case ready_c:
    if(startPtr < 0)
      logLen = logSize;
    else
      logLen = (logSize + endPtr - startPtr - 1) % logSize;

    consoleNote_P(CS_STRING("LOG READY, PTR = "));
    consolePrintLnUL(logPtr);
    consoleNote_P(CS_STRING("  LENGTH = "));
    consolePrintLnUL(logLen);
    consoleNote_P(CS_STRING("  STAMP = "));
    consolePrintLnUI(logEndStamp);
    
    logState = stop_c;
    
    datagramTxStart(DG_READY);
    datagramTxEnd();
    return true;

  default:
    return true;
  }

  return false;
}

void logSave()
{
  if(logState == stop_c && logEnabled) {
    int i = 0;
    
    logState = run_c;
    
    for(i = 0; i < 4; i++)
      logMark();

    logTask();

    consoleNoteLn_P(CS_STRING("Logging STARTED"));
      
  } else if(logState == run_c) {

    if(uncommitted > 0) {
      logCommit();
      m24xxFlush();
      logEndStamp = nextStamp;
    }
    
    if(!logEnabled) {
      logState = stop_c;
      consoleNoteLn_P(CS_STRING("Logging STOPPED"));
    }
  }
}

uint32_t previousForced;

void logObjects()
{
  bool force = false;
  int i = 0;
  
  if(stap_timeMillis() > previousForced+10e3) {
    previousForced = stap_timeMillis();
    force = true;
  }
  
  for(i = 0; i < lc_channels; i++) {
    switch(logChannels[i].type) {
    case lt_real:
      logFloat((ChannelId_t) i, *((float*) logChannels[i].object), force);
      break;
      
    case lt_angle:
      logFloat((ChannelId_t) i, *((float*) logChannels[i].object)*RADIAN, force);
      break;
      
    case lt_percent:
      logFloat((ChannelId_t) i, *((float*) logChannels[i].object)*100, force);
      break;
      
    case lt_integer:
      logInteger((ChannelId_t) i, *((uint16_t*) logChannels[i].object), force);
      break;
    }
  }
}

//
// Log interface
//

static uint16_t encode(bool vec[], int num)
{
  uint16_t result = 0;

  while(num-- > 0)
    result = (result<<1) | (vec[num] ? 1 : 0);

  return result;
}

void logTask()
{
  bool mode[] = { vpMode.slowFlight,
		  vpMode.bankLimiter,
		  vpMode.wingLeveler,
		  vpMode.takeOff,
		  vpMode.autoThrottle,
		  vpMode.radioFailSafe,
		  vpMode.sensorFailSafe,
		  vpMode.alphaFailSafe };

  bool status[] = { vpStatus.weightOnWheels,
		    vpStatus.positiveIAS,
		    vpControl.gearSel == 1,
		    vpStatus.stall,
		    vpStatus.alphaFailed,
		    vpStatus.alphaUnreliable,
		    vpStatus.pitotFailed,
		    vpStatus.pitotBlocked };
  
  modeEncoded = encode(mode, sizeof(mode)/sizeof(bool));
  statusEncoded = encode(status, sizeof(status)/sizeof(bool));
  flapEncoded = slopeOutput(&flapActuator);
  testEncoded = vpMode.test ? nvState.testNum : 0;

  logObjects();
}

bool logTest(void)
{
  if(logSize < 1)
    return false;

  consoleNote_P(CS_STRING("Filling with RANDOM..."));

  uint16_t state = 0xFFFF;
  int i = 0;
  
  for(i = 0; i < logSize; i++) {
    if((i & 0xFFF) == 0) {
      consolePrint(".");
      consoleFlush();
    }
    
    uint16_t value = 0;
    pseudoRandom((uint8_t*) &value, sizeof(value), &state);
  
    if(!m24xxWrite(logAddr(i), (const uint8_t*) &value, sizeof(value)))
      return false;
  }
  
  consoleNL();
  consoleNote_P(CS_STRING("Checking..."));
  
  state = 0xFFFF;
  
  for(i = 0; i < logSize; i++) {
    if((i & 0xFFF) == 0) {
      consolePrint(".");
      consoleFlush();
    }
    
    uint16_t value = 0, valueR = 0;
    pseudoRandom((uint8_t*) &value, sizeof(value), &state);
    if(!m24xxRead(logAddr(i), (uint8_t*) &valueR, sizeof(valueR)) || value != valueR)
      return false;
  }

  consoleNL();
  consoleNote_P(CS_STRING("Filling with ZERO..."));
  
  for(i = 0; i < logSize; i++) {
    if((i & 0xFFF) == 0) {
      consolePrint(".");
      consoleFlush();
    }
    
    uint16_t value = 0;
    if(!m24xxWrite(logAddr(i), (const uint8_t*) &value, sizeof(value)))
      return false;
  }
  
  consoleNL();
  consoleNote_P(CS_STRING("Checking..."));
  
  for(i = 0; i < logSize; i++) {
    if((i & 0xFFF) == 0) {
      consolePrint(".");
      consoleFlush();
    }
    
    uint16_t value = 0xFFFF;
    if(!m24xxRead(logAddr(i), (uint8_t*) &value, sizeof(value)) || value != 0)
      return false;
  }

  consoleNL();
  return true;
}
