#ifndef STAP_TARGET_H
#define STAP_TARGET_H

#include "platform.h"
#include "sensors/gyro.h"
#include "drivers/system.h"
#include "drivers/light_led.h"

#define STAP_rxStatus(port) BFSTAP_rxStatus(STAP_PORTID(port))
#define STAP_rxGetChar(port) BFSTAP_rxGetChar(STAP_PORTID(port))
#define STAP_txStatus(port) BFSTAP_txStatus(STAP_PORTID(port))
#define STAP_txPutChar(port, c) BFSTAP_txPutChar(STAP_PORTID(port), c)
#define STAP_txPut(port, b, s) BFSTAP_txPut(STAP_PORTID(port), b, s)
#define STAP_txDrain(port) BFSTAP_txDrain(STAP_PORTID(port))

int BFSTAP_rxStatus(int);
uint8_t BFSTAP_rxGetChar(int);
int BFSTAP_txStatus(int);
void BFSTAP_txPutChar(int, uint8_t);
void BFSTAP_txPut(int, const uint8_t *, int);
void BFSTAP_txDrain(int);

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

