#ifndef NEWI2C_H
#define NEWI2C_H

#include <stdint.h>
#include <stdbool.h>

extern "C" {
#include "BaseI2C.h"
}

class NewI2C
{
  public:
    void begin();
    void end();
    void timeOut(uint16_t);
    void setSpeed(uint8_t); 
    void pullup(uint8_t);
    uint8_t wait(uint8_t);
    uint8_t write(uint8_t, const uint8_t*, uint8_t);
    uint8_t write(uint8_t, const I2CBuffer_t*, int);
    uint8_t write(uint8_t, uint8_t, const uint8_t*, uint8_t);
    uint8_t write(uint8_t, uint16_t, const uint8_t*, uint8_t);
    uint8_t write(uint8_t, const uint8_t*, uint8_t, const uint8_t*, uint8_t);
    uint8_t write(uint8_t, const uint8_t*, uint8_t, const I2CBuffer_t*, int);
    uint8_t read(uint8_t, uint8_t*, uint8_t);
    uint8_t read(uint8_t, uint8_t, uint8_t*, uint8_t);
    uint8_t read(uint8_t, uint16_t, uint8_t*, uint8_t);
    uint8_t read(uint8_t, const uint8_t*, uint8_t, uint8_t*, uint8_t);


  private:
    uint8_t start();
    uint8_t transmitByte(uint8_t);
    uint8_t receiveByte(bool);
    uint8_t stop();
    void lockUp();
    uint8_t returnStatus;
    uint8_t nack;
    static uint16_t timeOutDelay;
};

extern NewI2C I2c;

#endif


