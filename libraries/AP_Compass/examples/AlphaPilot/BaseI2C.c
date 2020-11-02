#include <stdlib.h>
#include <stdbool.h>
#include "BaseI2C.h"
#include "Console.h"

#define BACKOFF (0.1e3)
#define BACKOFF_FRACTION  3

void basei2cReset(BaseI2CTarget_t *target)
{
  target->failed = false;
  target->failCount = 0;
}

bool basei2cIsOnline(BaseI2CTarget_t *target)
{
  return !target->failed || VP_ELAPSED_MILLIS(target->failedAt, vpTimeMillisApprox) > target->backoff;
}

bool basei2cWarning(BaseI2CTarget_t *target)
{
  return target->warn || target->failed;
}

bool basei2cInvoke(BaseI2CTarget_t *target, uint8_t status)
{
  if(status) {
    target->warn = true;
    
    consoleNote_P(CS_STRING("I2C("));
    consolePrint(target->name);
    consolePrintLn_P(CS_STRING(") ERROR"));

    if(target->failed)
      target->backoff += target->backoff/BACKOFF_FRACTION;
    else if(target->failCount++ > 3) {
      consoleNote_P(CS_STRING("I2C("));
      consolePrint(target->name);
      consolePrintLn_P(CS_STRING(") is OFFLINE"));
      target->failed = true;
      target->backoff = BACKOFF;
    }
    
    target->failedAt = vpTimeMillis();
  } else {    
    if(target->failCount > 0) {
      consoleNote_P(CS_STRING("I2C("));
      consolePrint(target->name);
      consolePrintLn_P(CS_STRING(") RECOVERED"));
      target->failCount = 0;
      target->failed = target->warn = false;
      target->backoff = BACKOFF;
    }
  }
  
  return status == 0;
}

void basei2cEntropySample(BaseI2CTarget_t *target, uint16_t v)
{
  int16_t diff = (int16_t) (v - target->prevValue);

  target->prevValue = v;
  target->entropyAcc += ABS(diff);
  target->entropyCount++;

  if(vpTimeMillisApprox - target->lastEntropy > 0.3e3) {
    target->lastEntropy = vpTimeMillisApprox;
    target->entropy = (float) target->entropyAcc / target->entropyCount;
    target->entropyAcc = target->entropyCount = 0;
  }
}

float basei2cEntropy(BaseI2CTarget_t *target)
{
  return target->entropy;
}

uint8_t basei2cWriteGeneric(uint8_t address, const uint8_t *addrArray, uint8_t addrSize, const uint8_t *data, uint8_t numberBytes)
{
  STAP_I2CBuffer_t buffer = { data, numberBytes };
  return STAP_I2CWrite(address, addrArray, addrSize, &buffer, 1);
}

uint8_t basei2cWriteBuffers(uint8_t address, const STAP_I2CBuffer_t *buffers, int numberBuffers)
{
  return STAP_I2CWrite(address, NULL, 0, buffers, numberBuffers);
}

uint8_t basei2cWrite(uint8_t address, const uint8_t *data, uint8_t numberBytes)
{
  STAP_I2CBuffer_t buffer = { data, numberBytes };
  return STAP_I2CWrite(address, NULL, 0, &buffer, 1);
}

uint8_t basei2cWriteWithByte(uint8_t address, uint8_t registerAddress, const uint8_t *data, uint8_t numberBytes)
{
  return basei2cWriteGeneric(address, &registerAddress, sizeof(registerAddress), data, numberBytes);
}

static void addrFromUINT16(uint8_t *buffer, uint16_t addr)
{
  int i = 0;
  
  for(i = 0; i < sizeof(addr); i++)
    buffer[i] = (addr >> 8*(sizeof(addr) - i - 1)) & 0xFF;
}

uint8_t basei2cWriteWithWord(uint8_t address, uint16_t memAddress, const uint8_t *data, uint8_t numberBytes)
{
  uint8_t addrArray[sizeof(memAddress)];
  addrFromUINT16(addrArray, memAddress);    
  return basei2cWriteGeneric(address, addrArray, sizeof(addrArray), data, numberBytes);
}

uint8_t basei2cRead(uint8_t address, uint8_t *dataBuffer, uint8_t numberBytes)
{
  return STAP_I2CRead(address, NULL, 0, dataBuffer, numberBytes);
}

uint8_t basei2cReadWithByte(uint8_t address, uint8_t registerAddress, uint8_t *dataBuffer, uint8_t numberBytes)
{
  return STAP_I2CRead(address, &registerAddress, sizeof(registerAddress), dataBuffer, numberBytes);
}

uint8_t basei2cReadWithWord(uint8_t address, uint16_t memAddress, uint8_t *dataBuffer, uint8_t numberBytes)
{
  uint8_t addrArray[sizeof(memAddress)];
  addrFromUINT16(addrArray, memAddress);        
  return STAP_I2CRead(address, addrArray, sizeof(addrArray), dataBuffer, numberBytes);
}


