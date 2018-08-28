#include <stdint.h>
#include <stdbool.h>

//
// MS4525DO (dynamic pressure) sensor interface
//

#define MS4525DO_DEVICE  0x28

bool MS4525DO_isOnline();
void MS4525DO_calibrate();
bool MS4525DO_pressure(int16_t *result);
 
