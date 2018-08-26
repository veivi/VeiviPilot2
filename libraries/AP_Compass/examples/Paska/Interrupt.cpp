#include "Interrupt.h"
#include "RxInput.h"
#include <stdlib.h>

extern "C" {
#include "Console.h"
}
uint8_t nestCount = 0;

extern "C" ISR(BADISR_vect)
{
   sei();
   consoleNoteLn("PASKA KESKEYTYS.");
   abort();
}


extern "C" ISR(PCINT0_vect) { rxInterrupt_callback(0); }
extern "C" ISR(PCINT1_vect) { rxInterrupt_callback(1); }
extern "C" ISR(PCINT2_vect) { rxInterrupt_callback(2); }

