#ifndef STAP_H
#define STAP_H

#include <stdint.h>
#include <stdbool.h>

//
// Constant storage access (alias nasty AVR hack called "progmem")
//

#include <avr/pgmspace.h>

#define CS_QUALIFIER  PROGMEM
#define CS_MEMCPY memcpy_P
#define CS_STRING(s) (__extension__({static const char __c[] PROGMEM = (s); &__c[0]; }))

//
// Interrupt control
//

#include <avr/interrupt.h>

#define STAP_FORBID if(!nestCount++) cli()
#define STAP_PERMIT if(!--nestCount) sei()

extern uint8_t nestCount;

//
// RNG interface
//

void stap_entropyDigest(uint16_t value);

//
// Boot interface
//

bool stap_boot(void);

//
// Host (serial) interface
//

bool stap_hostInit(void);
int stap_hostReceiveState(void);   // How many chars in the buffer
int stap_hostReceive(uint8_t *buffer, int size);
uint8_t stap_hostReceiveChar(void);
int stap_hostTransmitState(void);  // How many chars will fit
int stap_hostTransmit(const uint8_t *buffer, int size);

bool stap_initialize(void);
void stap_mainLoop(void);

//
// System interface
//

extern uint32_t currentTime;  // Updated on every call to currentMicros()

uint32_t currentMicros(void);
uint32_t currentMillis(void);
void delayMicros(uint32_t x);
uint32_t stap_memoryFree(void);

//
// Gyro interface
//

typedef struct stap_Vector3f {
  float x, y, z;
} stap_Vector3f_t;

bool stap_gyroInit(void);
bool stap_gyroUpdate(void);
bool stap_gyroRead(stap_Vector3f_t *acc, stap_Vector3f_t *atti, stap_Vector3f_t *rot);

//
// Altimeter interface
//

bool stap_altiInit(void);
bool stap_altiUpdate(void);
float stap_altiRead(void);

#endif

