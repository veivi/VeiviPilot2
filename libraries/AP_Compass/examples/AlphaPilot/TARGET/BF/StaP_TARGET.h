#ifndef STAP_TARGET_H
#define STAP_TARGET_H

#include "platform.h"
#include "sensors/gyro.h"
#include "drivers/system.h"
#include "drivers/light_led.h"

#define STAP_LinkStatus(port) BFSTAP_LinkStatus(STAP_LINKID(port))
#define STAP_LinkGetChar(port) BFSTAP_LinkGetChar(STAP_LINKID(port))
#define STAP_LinkPutChar(port, c) BFSTAP_LinkPutChar(STAP_LINKID(port), c)
#define STAP_LinkPut(port, b, s) BFSTAP_LinkPut(STAP_LINKID(port), b, s)
#define STAP_LinkDrain(port) BFSTAP_LinkDrain(STAP_LINKID(port))

int BFSTAP_LinkStatus(int);
uint8_t BFSTAP_LinkGetChar(int);
void BFSTAP_LinkPutChar(int, uint8_t);
void BFSTAP_LinkPut(int, const uint8_t *, int);
void BFSTAP_LinkDrain(int);

#define STAP_I2CWrite(dev, a, as, b, bn) BFSTAP_I2CWrite(dev, a, as, b, bn)
#define STAP_I2CRead(dev, a, as, d, ds) BFSTAP_I2CRead(dev, a, as, d, ds)
#define STAP_I2CWait(dev) BFSTAP_I2CWait(dev)
#define STAP_I2CErrorCount BFSTAP_I2CErrorCount()
#define STAP_I2CErrorCode  BFSTAP_I2CErrorCode()

uint8_t BFSTAP_I2CWrite(uint8_t, const uint8_t*, uint8_t, const STAP_I2CBuffer_t*, int);
uint8_t BFSTAP_I2CRead(uint8_t, const uint8_t*, uint8_t, uint8_t*, uint8_t);
uint8_t BFSTAP_I2CWait(uint8_t);
uint16_t BFSTAP_I2CErrorCount(void);
uint16_t BFSTAP_I2CErrorCode(void);

#define STAP_PERIOD_GYRO         gyro.targetLooptime
#define STAP_PERIOD_GYRO_STATIC  HZ_TO_PERIOD(100)
#define STAP_PERIOD_ATTI         HZ_TO_PERIOD(100)
#define STAP_PERIOD_ACC          HZ_TO_PERIOD(100)

#define STAP_FORBID      { __disable_irq(); nestCount++; }
#define STAP_PERMIT      if(!--nestCount) __enable_irq()
#define STAP_LED_ON      LED0_ON
#define STAP_LED_OFF     LED0_OFF

extern volatile uint8_t nestCount;

#define STAP_CANOPY_CLOSED  0

//
// Constant storage access (alias nasty AVR hack called "progmem")
//

#define CS_QUALIFIER  
#define CS_MEMCPY memcpy
#define CS_READCHAR(s) (*((const char *)s))
#define CS_STRNCPY(dst, src, s) strncpy(dst, src, s)
#define CS_STRING(s) s

#endif

