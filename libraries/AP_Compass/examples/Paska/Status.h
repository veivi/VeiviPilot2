//
// Global status record
//

#ifndef STATUS_H
#define STATUS_H

#include "NewI2C.h"

struct StatusRecord {
  bool armed;
  bool consoleLink;
  bool simulatorLink;
  bool silent;
  bool positiveIAS;
  bool fullStop;
  bool pitotFailed;
  bool pitotBlocked;
  bool stall;
  bool weightOnWheels;
  bool aloft;
  int fault;
  bool alphaFailed;
  bool alphaUnreliable;
};

extern struct StatusRecord vpStatus;

#endif
