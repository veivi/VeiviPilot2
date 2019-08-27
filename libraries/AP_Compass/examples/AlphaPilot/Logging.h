#ifndef LOGGING_H
#define LOGGING_H

#include <stdint.h>
#include <stdbool.h>
#include "Objects.h"
#include "Log.h"
#include "StaP.h"

#define logIndex(i) ((logPtr + logSize + (i)) % logSize)

bool logReady(bool verbose);
bool logReadyVerbose(void);
bool logInit(STAP_MILLIS_T);
void logClear(void);
void logDumpBinary(void);
bool logTest(void);

void logGeneric(int ch, float value);
void logMark(void);

void logEnable(void);
void logDisable(void);

void logSave(void);  
void logTask(void);

#endif
