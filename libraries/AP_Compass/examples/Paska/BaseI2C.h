#ifndef BASEI2C_H
#define BASEI2C_H

#include <stdint.h>
#include <stdbool.h>
#include "CoreObjects.h"
#include "StaP.h"

typedef struct {
    const uint8_t *data;
    uint8_t size;
} I2CBuffer_t;

void basei2cInit(void);
void baeei2cShutdown(void);
void basei2cSetTimeOut(uint16_t);
void basei2cSetSpeed(uint8_t); 
void basei2cSetPullup(uint8_t);
uint8_t basei2cWait(uint8_t);
uint8_t basei2cWrite(uint8_t, const uint8_t*, uint8_t, const I2CBuffer_t*, int);
/*
uint8_t basei2cWrite(BaseI2CIF_t*, uint8_t, const uint8_t*, uint8_t);
uint8_t basei2cWrite(BaseI2CIF_t*, uint8_t, const I2CBuffer_t*, int);
uint8_t basei2cWrite(BaseI2CIF_t*, uint8_t, uint8_t, const uint8_t*, uint8_t);
uint8_t basei2cWrite(BaseI2CIF_t*, uint8_t, uint16_t, const uint8_t*, uint8_t);
uint8_t basei2cWrite(BaseI2CIF_t*, uint8_t, const uint8_t*, uint8_t, const uint8_t*, uint8_t);
*/
uint8_t basei2cRead(uint8_t, const uint8_t*, uint8_t, uint8_t*, uint8_t);
/*
uint8_t basei2cRead(BaseI2CIF_t*, uint8_t, uint8_t*, uint8_t);
uint8_t basei2cRead(BaseI2CIF_t*, uint8_t, uint8_t, uint8_t*, uint8_t);
uint8_t basei2cRead(BaseI2CIF_t*, uint8_t, uint16_t, uint8_t*, uint8_t);
*/
typedef struct BaseI2CTarget {
  const char *name;
  bool warn, failed;
  int failCount;
  uint32_t failedAt, backoff;
} BaseI2CTarget_t;

bool basei2cInvoke(BaseI2CTarget_t*, uint8_t status);
bool basei2cIsOnline(BaseI2CTarget_t*);
bool basei2cWarning(BaseI2CTarget_t*);

#endif
