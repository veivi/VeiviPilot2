#include <stdint.h>
#include <stdbool.h>

//
// AS5048B (alpha) sensor interface
//

typedef uint16_t AS5048_word_t;
typedef int16_t AS5048_alpha_t;

bool AS5048B_isOnline(void);
bool AS5048B_maybeOnline(void);
bool AS5048B_alpha(AS5048_alpha_t *result);
float AS5048B_entropy(void);
bool AS5048B_field(AS5048_word_t *result);

//
// Reg map
//

#define AS5048_ADDRESS 0x40 
#define AS5048B_PROG_REG 0x03
#define AS5048B_ADDR_REG 0x15
#define AS5048B_ZEROMSB_REG 0x16 //bits 0..7
#define AS5048B_ZEROLSB_REG 0x17 //bits 0..5
#define AS5048B_GAIN_REG 0xFA
#define AS5048B_DIAG_REG 0xFB
#define AS5048B_MAGNMSB_REG 0xFC //bits 0..7
#define AS5048B_MAGNLSB_REG 0xFD //bits 0..5
#define AS5048B_ANGLMSB_REG 0xFE //bits 0..7
#define AS5048B_ANGLLSB_REG 0xFF //bits 0..5

