#include <stdlib.h>
#include "MS4525.h"
#include "NewI2C.h"
#include "Console.h"

uint8_t MS4525DO_read(uint8_t *storage, uint8_t bytes) 
{
  const uint8_t addr_c = 0x28;
 
  return I2c.read(addr_c, NULL, 0, storage, bytes);
}

uint8_t MS4525DO_read(uint16_t *result)
{
  uint8_t buf[sizeof(uint16_t)];
  uint8_t status = MS4525DO_read(buf, sizeof(buf));
  
  if(!status)
    *result = (((uint16_t) (buf[0] & 0x3F)) << 8) + buf[1];

  return status | (buf[0] & (1<<6));
}

uint8_t MS4525DO_pressure(int16_t *result) 
{
  static uint32_t acc;
  static int accCount;
  static bool done = false;
  const int log2CalibWindow = 9;
  uint16_t raw = 0;
  uint8_t status = MS4525DO_read(&raw);

  if(status)
    return status;

  if(accCount < 1<<log2CalibWindow) {
    acc += raw;
    accCount++;
  } else {
    if(!done)
      consoleNoteLn_P(PSTR("Airspeed calibration DONE"));
    
    done = true;
    
    if(result)
      *result = (raw<<2) - (acc>>(log2CalibWindow - 2));
  }
  
  return status;
}

