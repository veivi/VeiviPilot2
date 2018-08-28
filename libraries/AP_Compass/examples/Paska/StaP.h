#ifndef STAP_H
#define STAP_H

#include <stdint.h>
#include <avr/interrupt.h>

//
// Constant storage access (alias nasty AVR hack called "progmem")
//

#include <avr/pgmspace.h>

#define CS_QUALIFIER  PROGMEM
#define CS_MEMCPY memcpy_P
#define CS_STRING(s) (__extension__({static const char __c[] PROGMEM = (s); &__c[0]; }))

#define STAP_FORBID if(!nestCount++) cli()
#define STAP_PERMIT if(!--nestCount) sei()

extern uint8_t nestCount;

#endif

