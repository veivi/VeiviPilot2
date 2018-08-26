#include <stdint.h>

//
// AS5048B (alpha) sensor interface
//

bool AS5048B_isOnline(void);
uint8_t AS5048B_alpha(int16_t *result);
uint8_t AS5048B_field(uint16_t *result);
