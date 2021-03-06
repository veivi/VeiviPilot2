#include "AS5048B.h"
#include "NVState.h"
#include "BaseI2C.h"

static BaseI2CTarget_t target = { "alpha" };

bool AS5048B_isOnline(void)
{
  return basei2cIsOnline(&target);
}

bool AS5048B_maybeOnline(void)
{
  return basei2cMaybeOnline(&target);
}

bool AS5048B_read(uint8_t addr, uint8_t *storage, uint8_t bytes) 
{
  return basei2cInvoke(&target, basei2cReadWithByte(AS5048_ADDRESS, addr, storage, bytes));
}

bool AS5048B_readWord(uint8_t addr, AS5048_word_t *result)
{
  uint8_t buf[sizeof(AS5048_word_t)];
  uint8_t status = AS5048B_read(addr, buf, sizeof(buf));
  
  if(status && result)
    *result = ((((AS5048_word_t) buf[0]) << 6) + (buf[1] & 0x3F))<<2;

  return status;
}

bool AS5048B_alpha(AS5048_alpha_t *result)
{
  AS5048_word_t raw = 0;
  uint8_t status = AS5048B_readWord(AS5048B_ANGLMSB_REG, &raw);

  basei2cEntropySample(&target, raw);
  
  if(vpParam.sensorOrient)
    raw = ~raw;

  if(status && result)
    *result = (AS5048_alpha_t) (raw - vpParam.alphaRef);
  
  return status;
}

bool AS5048B_field(AS5048_word_t *result)
{
  AS5048_word_t raw = 0;
  uint8_t status = AS5048B_readWord(AS5048B_MAGNMSB_REG, &raw);
  
  if(status && result)
    *result = raw;
  
  return status;
}

float AS5048B_entropy(void)
{
  return basei2cEntropy(&target);
}


