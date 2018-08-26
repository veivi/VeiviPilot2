#include <stdint.h>

extern uint32_t currentTime;  // Updated on every call to currentMicros()

uint32_t currentMicros(void);
uint32_t currentMillis(void);
void delayMicros(uint32_t x);
