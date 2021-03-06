#ifndef STAP_TARGET_H
#define STAP_TARGET_H

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

//
// Serial interface: disable unsupported links
//

#undef STAP_LINK_SRXLINB
#undef STAP_LINK_SRXLOUTA
#undef STAP_LINK_SRXLOUTB

#define STAP_LinkStatus(port) APM2STAP_LinkStatus(port)
#define STAP_LinkGetChar(port) APM2STAP_LinkGetChar(port)
#define STAP_LinkPutChar(port, c) APM2STAP_LinkPutChar(port, c)
#define STAP_LinkPut(port, b, s) APM2STAP_LinkPut(port, b, s)

uint8_t APM2STAP_LinkStatus(uint8_t);
uint8_t APM2STAP_LinkGetChar(uint8_t);
void APM2STAP_LinkPutChar(uint8_t, uint8_t);
void APM2STAP_LinkPut(uint8_t, const uint8_t *, int);

#define STAP_I2CWrite(dev, a, as, b, bn) APM2STAP_I2CWrite(dev, a, as, b, bn)
#define STAP_I2CRead(dev, a, as, d, ds) APM2STAP_I2CRead(dev, a, as, d, ds)
#define STAP_I2CWait(dev) APM2STAP_I2CWait(dev)
#define STAP_I2CErrorCount APM2STAP_I2CErrorCount()
#define STAP_I2CErrorCode  APM2STAP_I2CErrorCode()

uint8_t APM2STAP_I2CWrite(uint8_t, const uint8_t*, uint8_t, const STAP_I2CBuffer_t*, int);
uint8_t APM2STAP_I2CRead(uint8_t, const uint8_t*, uint8_t, uint8_t*, uint8_t);
uint8_t APM2STAP_I2CWait(uint8_t);
uint16_t APM2STAP_I2CErrorCount(void);
uint16_t APM2STAP_I2CErrorCode(void);

//
// RX and Servo (PWM) interface
//

#define STAP_pwmOutput(num, pulse)    APM2stap_pwmOutput(num, pulse)

void APM2stap_pwmOutput(uint8_t num, const uint16_t value[]);

#define CS_QUALIFIER  PROGMEM
#define CS_MEMCPY memcpy_P
#define CS_READCHAR(s) pgm_read_byte(s)
#define CS_STRNCPY(dst, src, s) strncpy_P(dst, src, s)
#define CS_STRING(s) (__extension__({static const char __c[] PROGMEM = (s); &__c[0]; }))

typedef enum { PortA, PortB, PortC, PortD, PortE, PortF, PortG, PortH, PortK, PortL } portName_t;

struct PortDescriptor {
  volatile uint8_t *pin, *port, *ddr, *mask;
  uint8_t pci;
};

struct PinDescriptor {
  portName_t port;
  uint8_t index;
};

void pinOutputEnable(const struct PinDescriptor *pin, bool output);
void setPinState(const struct PinDescriptor *pin, uint8_t state);
uint8_t getPinState(const struct PinDescriptor *pin);  
void configureInput(const struct PinDescriptor *pin, bool pullup);
void configureOutput(const struct PinDescriptor *pin);

#define STAP_FORBID { cli(); nestCount++; }
#define STAP_PERMIT if(!--nestCount) sei()

extern volatile uint8_t nestCount;

extern const struct PinDescriptor led;

#define STAP_LED_OFF     setPinState(&led, false)
#define STAP_LED_ON     setPinState(&led, true)

extern const struct PinDescriptor latch;

#define STAP_CANOPY_CLOSED  (getPinState(&latch) == 1)

#endif
