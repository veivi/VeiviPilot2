#include "AS5048B.h"
#include "NVState.h"
#include "BaseI2C.h"

static BaseI2CTarget_t target = { "alpha" };

bool AS5048B_isOnline(void)
{
  return basei2cIsOnline(&target);
}

uint8_t AS5048B_read(uint8_t addr, uint8_t *storage, uint8_t bytes) 
{
  return basei2cInvoke(&target, basei2cReadWithByte(AS5048_ADDRESS, addr, storage, bytes));
}

uint8_t AS5048B_readWord(uint8_t addr, uint16_t *result)
{
  uint8_t buf[sizeof(uint16_t)];
  uint8_t status = AS5048B_read(addr, buf, sizeof(buf));
  
  if(!status && result)
    *result = ((((uint16_t) buf[0]) << 6) + (buf[1] & 0x3F))<<2;

  return status;
}

uint8_t AS5048B_alpha(int16_t *result)
{
  uint16_t raw = 0;
  uint8_t status = AS5048B_readWord(AS5048B_ANGLMSB_REG, &raw);

  if(vpParam.sensorOrient)
    raw = ~raw;
  
  if(!status && result)
    *result = (int16_t) (raw - vpParam.alphaRef);
  
  return status;
}

uint8_t AS5048B_field(uint16_t *result)
{
  uint16_t raw = 0;
  uint8_t status = AS5048B_readWord(AS5048B_MAGNMSB_REG, &raw);
  
  if(!status && result)
    *result = raw;
  
  return status;
}

