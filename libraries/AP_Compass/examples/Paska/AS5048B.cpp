#include "AS5048B.h"
#include "NVState.h"
#include "NewI2C.h"

#define AS5048_ADDRESS 0x40 
#define AS5048B_PROG_REG 0x03
#define AS5048B_ADDR_REG 0x15
#define AS5048B_ZEROMSB_REG 0x16 //bits 0..7
#define AS5048B_ZEROLSB_REG 0x17 //bits 0..5
#define AS5048B_GAIN_REG 0xFA
#define AS5048B_DIAG_REG 0xFB
#define AS5048B_MAGNMSB_REG 0xFC //bits 0..7
#define AS5048B_MAGNLSB_REG 0xFD //bits 0..5
#define AS5048B_ANGLMSB_REG 0xFE //bits 0..7
#define AS5048B_ANGLLSB_REG 0xFF //bits 0..5

uint8_t AS5048B_read(uint8_t addr, uint8_t *storage, uint8_t bytes) 
{
  return I2c.read(AS5048_ADDRESS, addr, storage, bytes);
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

