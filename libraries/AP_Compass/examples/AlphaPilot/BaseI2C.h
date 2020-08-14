#ifndef BASEI2C_H
#define BASEI2C_H

#include <stdint.h>
#include <stdbool.h>
#include "Objects.h"
#include "Time.h"

typedef struct {
    const uint8_t *data;
    uint8_t size;
} I2CBuffer_t;

uint8_t basei2cWriteGeneric(uint8_t, const uint8_t*, uint8_t, const uint8_t*, uint8_t);
uint8_t basei2cWriteWithWord(uint8_t, uint16_t, const uint8_t*, uint8_t);
uint8_t basei2cWriteWithByte(uint8_t, uint8_t, const uint8_t*, uint8_t);
uint8_t basei2cWrite(uint8_t, const uint8_t*, uint8_t);
uint8_t basei2cWriteBuffers(uint8_t, const I2CBuffer_t*, int);
uint8_t basei2cReadWithWord(uint8_t, uint16_t, uint8_t*, uint8_t);
uint8_t basei2cReadWithByte(uint8_t, uint8_t, uint8_t*, uint8_t);
uint8_t basei2cRead(uint8_t, uint8_t*, uint8_t);

typedef struct BaseI2CTarget {
  const char *name;
  bool warn, failed;
  int failCount;
  VP_TIME_MILLIS_T failedAt, backoff;
  uint32_t entropyAcc;
  uint16_t entropyCount;
  float entropy;
  uint32_t lastEntropy;
  uint16_t prevValue;
} BaseI2CTarget_t;

void basei2cReset(BaseI2CTarget_t*);
bool basei2cInvoke(BaseI2CTarget_t*, uint8_t status);
bool basei2cIsOnline(BaseI2CTarget_t*);
bool basei2cWarning(BaseI2CTarget_t*);
void basei2cEntropySample(BaseI2CTarget_t*, uint16_t v);
float basei2cEntropy(BaseI2CTarget_t*);

#endif
