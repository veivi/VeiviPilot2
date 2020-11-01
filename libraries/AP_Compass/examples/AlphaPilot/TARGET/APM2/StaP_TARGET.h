#ifndef STAP_TARGET_H
#define STAP_TARGET_H

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#define STAP_rxStatus(port) APM2_stap_rxStatus(STAP_PORTID(port))
#define STAP_rxGetChar(port) APM2_stap_rxGetChar(STAP_PORTID(port))
#define STAP_txStatus(port) APM2_stap_txStatus(STAP_PORTID(port))
#define STAP_txPutChar(port, c) APM2_stap_txPutChar(STAP_PORTID(port), c)
#define STAP_txPut(port, b, s) APM2_stap_txPut(STAP_PORTID(port), b, s)

int APM2_stap_rxStatus(int);
uint8_t APM2_stap_rxGetChar(int);
int APM2_stap_txStatus(int);
void APM2_stap_txPutChar(int, uint8_t);
void APM2_stap_txPut(int, const uint8_t *, int);

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
