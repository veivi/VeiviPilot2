#include <stdint.h>

//
// MS4525DO (dynamic pressure) sensor interface
//

bool MS4525DO_isOnline();
void MS4525DO_calibrate();
uint8_t MS4525DO_pressure(int16_t *result);
 
