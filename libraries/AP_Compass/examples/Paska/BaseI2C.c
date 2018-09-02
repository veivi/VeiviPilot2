#include <stdlib.h>
#include <stdbool.h>
#include "BaseI2C.h"
#include "Console.h"
#include "StaP.h"

#define BACKOFF (0.5e3)

bool basei2cIsOnline(BaseI2CTarget_t *target)
{
  return !target->failed || stap_timeMillis() > target->failedAt+target->backoff;
}

bool basei2cWarning(BaseI2CTarget_t *target)
{
  return target->warn || target->failed;
}

bool basei2cInvoke(BaseI2CTarget_t *target, uint8_t status)
{
  if(status) {
    target->warn = true;
    
    consoleNote_P(PSTR("Bad "));
    consolePrintLn(target->name);

    if(target->failed)
      target->backoff += target->backoff/2;
    else if(++target->failCount > 3) {
      consoleNote("");
      consolePrint(target->name);
      consolePrintLn_P(PSTR(" failed"));
      target->failed = true;
    }
    
    target->failedAt = stap_timeMillis();
  } else {    
    if(target->failCount > 0) {
      consoleNote("");
      consolePrint(target->name);
      consolePrintLn_P(PSTR(" recovered"));
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

  if(stap_currentMicros - target->lastEntropy > 0.3e6) {
    target->lastEntropy = stap_currentMicros;
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
  I2CBuffer_t buffer = { data, numberBytes };
  return basei2cWriteGenericBuffers(address, addrArray, addrSize, &buffer, 1);
}

uint8_t basei2cWriteBuffers(uint8_t address, const I2CBuffer_t *buffers, int numberBuffers)
{
  return basei2cWriteGenericBuffers(address, NULL, 0, buffers, numberBuffers);
}

uint8_t basei2cWrite(uint8_t address, const uint8_t *data, uint8_t numberBytes)
{
  I2CBuffer_t buffer = { data, numberBytes };
  return basei2cWriteGenericBuffers(address, NULL, 0, &buffer, 1);
}

uint8_t basei2cWriteWithByte(uint8_t address, uint8_t registerAddress, const uint8_t *data, uint8_t numberBytes)
{
  return basei2cWriteGeneric(address, &registerAddress, sizeof(registerAddress), data, numberBytes);
}

uint8_t basei2cWriteWithWord(uint8_t address, uint16_t memAddress, const uint8_t *data, uint8_t numberBytes)
{
  uint8_t addrArray[sizeof(memAddress)], i = 0;
  
  for(i = 0; i < sizeof(memAddress); i++)
    addrArray[i] = (memAddress >> 8*(sizeof(memAddress) - i - 1)) & 0xFF;
    
  return basei2cWriteGeneric(address, addrArray, sizeof(addrArray), data, numberBytes);
}

uint8_t basei2cRead(uint8_t address, uint8_t *dataBuffer, uint8_t numberBytes)
{
  return basei2cReadGeneric(address, NULL, 0, dataBuffer, numberBytes);
}

uint8_t basei2cReadWithByte(uint8_t address, uint8_t registerAddress, uint8_t *dataBuffer, uint8_t numberBytes)
{
  return basei2cReadGeneric(address, &registerAddress, sizeof(registerAddress), dataBuffer, numberBytes);
}

uint8_t basei2cReadWithWord(uint8_t address, uint16_t memAddress, uint8_t *dataBuffer, uint8_t numberBytes)
{
  uint8_t addrArray[sizeof(memAddress)], i = 0;
  
  for(i = 0; i < sizeof(memAddress); i++)
    addrArray[i] = (memAddress >> 8*(sizeof(memAddress) - i - 1)) & 0xFF;
    
  return basei2cReadGeneric(address, addrArray, sizeof(addrArray), dataBuffer, numberBytes);
}


