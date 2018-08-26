#ifndef STAP_H
#define STAP_H

//
// Constant storage access (alias nasty AVR hack called "progmem")
//

#include <avr/pgmspace.h>
// #include <AP_ProgMem/AP_ProgMem.h>

typedef char prog_char;

#define CS_QUALIFIER  PROGMEM
#define CS_CHAR_T prog_char
#define CS_MEMCPY memcpy_P
#define CS_STRING(s) (__extension__({static const prog_char __c[] PROGMEM = (s); &__c[0]; }))

#endif

