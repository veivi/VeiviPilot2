#include <string.h>
#include "SSD1306.h"
#include "StaP.h"

const uint8_t fontWidth = 8;
const uint8_t fontData[] CS_QUALIFIER = {
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,  // Char ' '
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,  // Char ' '
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x0, 0xC6, 0x66, 0x30, 0x18, 0xCC, 0xC6,  // Char '%'
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x80, 0xC0, 0x60, 0x20, 0x0, 0x0, 0x0,  // Char ','
0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0,  // Char '-'
0x0, 0x0, 0x0, 0x60, 0x60, 0x0, 0x0, 0x0,  // Char '.'
0x0, 0x40, 0x60, 0x30, 0x18, 0xC, 0x6, 0x2,  // Char '/'
0x0, 0x7C, 0xFE, 0x92, 0x8A, 0xFE, 0x7C, 0x0,  // Char '0'
0x0, 0x80, 0x88, 0xFE, 0xFE, 0x80, 0x80, 0x0,  // Char '1'
0x0, 0xC4, 0xE6, 0xA2, 0x92, 0x9E, 0x8C, 0x0,  // Char '2'
0x0, 0x44, 0xC6, 0x92, 0x92, 0xFE, 0x6C, 0x0,  // Char '3'
0x0, 0x30, 0x30, 0x28, 0x2C, 0xFE, 0xFE, 0x0,  // Char '4'
0x0, 0x4E, 0xCE, 0x8A, 0x8A, 0xFA, 0x72, 0x0,  // Char '5'
0x0, 0x7C, 0xFE, 0x92, 0x92, 0xF6, 0x64, 0x0,  // Char '6'
0x0, 0x6, 0x6, 0xF2, 0xFA, 0xE, 0x6, 0x0,  // Char '7'
0x0, 0x0, 0x6C, 0xFE, 0x92, 0x92, 0xFE, 0x6C,  // Char '8'
0x0, 0x4C, 0xDE, 0x92, 0x92, 0xFE, 0x7C, 0x0,  // Char '9'
0x0, 0x0, 0xCC, 0xCC, 0x0, 0x0, 0x0, 0x0,  // Char ':'
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x28, 0x28, 0x28, 0x28, 0x28, 0x0, 0x0,  // Char '='
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0xF8, 0xFC, 0x16, 0x16, 0xFC, 0xF8, 0x0,  // Char 'A'
0x0, 0xFE, 0xFE, 0x92, 0x92, 0xFE, 0x6C, 0x0,  // Char 'B'
0x0, 0x7C, 0xFE, 0x82, 0x82, 0xC6, 0x44, 0x0,  // Char 'C'
0x0, 0xFE, 0xFE, 0x82, 0xC6, 0x7C, 0x38, 0x0,  // Char 'D'
0x0, 0xFE, 0xFE, 0x92, 0x92, 0x82, 0x82, 0x0,  // Char 'E'
0x0, 0xFE, 0xFE, 0x12, 0x12, 0x2, 0x2, 0x0,  // Char 'F'
0x0, 0x7C, 0xFE, 0x82, 0x92, 0xF6, 0x74, 0x0,  // Char 'G'
0x0, 0xFE, 0xFE, 0x10, 0x10, 0xFE, 0xFE, 0x0,  // Char 'H'
0x0, 0x0, 0x82, 0xFE, 0xFE, 0x82, 0x0, 0x0,  // Char 'I'
0x0, 0x40, 0xC0, 0x82, 0xFE, 0x7E, 0x2, 0x0,  // Char 'J'
0x0, 0xFE, 0xFE, 0x38, 0x6C, 0xC6, 0x82, 0x0,  // Char 'K'
0x0, 0xFE, 0xFE, 0x80, 0x80, 0x80, 0x80, 0x0,  // Char 'L'
0x0, 0xFE, 0xFE, 0xC, 0x18, 0xC, 0xFE, 0xFE,  // Char 'M'
0x0, 0xFE, 0xFE, 0x1C, 0x38, 0xFE, 0xFE, 0x0,  // Char 'N'
0x0, 0x7C, 0xFE, 0x82, 0x82, 0xFE, 0x7C, 0x0,  // Char 'O'
0x0, 0xFE, 0xFE, 0x12, 0x12, 0x1E, 0xC, 0x0,  // Char 'P'
0x0, 0x3C, 0x7E, 0x42, 0xC2, 0xFE, 0xBC, 0x0,  // Char 'Q'
0x0, 0xFE, 0xFE, 0x32, 0x72, 0xDE, 0x8C, 0x0,  // Char 'R'
0x0, 0x4C, 0xDE, 0x92, 0x92, 0xF6, 0x64, 0x0,  // Char 'S'
0x0, 0x2, 0x2, 0xFE, 0xFE, 0x2, 0x2, 0x0,  // Char 'T'
0x0, 0x7E, 0xFE, 0x80, 0x80, 0xFE, 0x7E, 0x0,  // Char 'U'
0x0, 0x3E, 0x7E, 0xC0, 0xC0, 0x7E, 0x3E, 0x0,  // Char 'V'
0x0, 0xFE, 0xFE, 0x60, 0x30, 0x60, 0xFE, 0xFE,  // Char 'W'
0x0, 0xC6, 0xEE, 0x38, 0x38, 0xEE, 0xC6, 0x0,  // Char 'X'
0x0, 0xE, 0x1E, 0xF0, 0xF0, 0x1E, 0xE, 0x0,  // Char 'Y'
0x0, 0xC2, 0xE2, 0xB2, 0x9A, 0x8E, 0x86, 0x0,  // Char 'Z'
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x2, 0x6, 0xC, 0x18, 0x30, 0x60, 0x40,  // Char '\'
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x0,  // Char '_'
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0xF8, 0xFC, 0x16, 0x16, 0xFC, 0xF8, 0x0,  // Char 'a'
0x0, 0xFE, 0xFE, 0x92, 0x92, 0xFE, 0x6C, 0x0,  // Char 'b'
0x0, 0x7C, 0xFE, 0x82, 0x82, 0xC6, 0x44, 0x0,  // Char 'c'
0x0, 0xFE, 0xFE, 0x82, 0xC6, 0x7C, 0x38, 0x0,  // Char 'd'
0x0, 0xFE, 0xFE, 0x92, 0x92, 0x82, 0x82, 0x0,  // Char 'e'
0x0, 0xFE, 0xFE, 0x12, 0x12, 0x2, 0x2, 0x0,  // Char 'f'
0x0, 0x7C, 0xFE, 0x82, 0x92, 0xF6, 0x74, 0x0,  // Char 'g'
0x0, 0xFE, 0xFE, 0x10, 0x10, 0xFE, 0xFE, 0x0,  // Char 'h'
0x0, 0x0, 0x82, 0xFE, 0xFE, 0x82, 0x0, 0x0,  // Char 'i'
0x0, 0x40, 0xC0, 0x82, 0xFE, 0x7E, 0x2, 0x0,  // Char 'j'
0x0, 0xFE, 0xFE, 0x38, 0x6C, 0xC6, 0x82, 0x0,  // Char 'k'
0x0, 0xFE, 0xFE, 0x80, 0x80, 0x80, 0x80, 0x0,  // Char 'l'
0x0, 0xFE, 0xFE, 0xC, 0x18, 0xC, 0xFE, 0xFE,  // Char 'm'
0x0, 0xFE, 0xFE, 0x1C, 0x38, 0xFE, 0xFE, 0x0,  // Char 'n'
0x0, 0x7C, 0xFE, 0x82, 0x82, 0xFE, 0x7C, 0x0,  // Char 'o'
0x0, 0xFE, 0xFE, 0x12, 0x12, 0x1E, 0xC, 0x0,  // Char 'p'
0x0, 0x3C, 0x7E, 0x42, 0xC2, 0xFE, 0xBC, 0x0,  // Char 'q'
0x0, 0xFE, 0xFE, 0x32, 0x72, 0xDE, 0x8C, 0x0,  // Char 'r'
0x0, 0x4C, 0xDE, 0x92, 0x92, 0xF6, 0x64, 0x0,  // Char 's'
0x0, 0x2, 0x2, 0xFE, 0xFE, 0x2, 0x2, 0x0,  // Char 't'
0x0, 0x7E, 0xFE, 0x80, 0x80, 0xFE, 0x7E, 0x0,  // Char 'u'
0x0, 0x3E, 0x7E, 0xC0, 0xC0, 0x7E, 0x3E, 0x0,  // Char 'v'
0x0, 0xFE, 0xFE, 0x60, 0x30, 0x60, 0xFE, 0xFE,  // Char 'w'
0x0, 0xC6, 0xEE, 0x38, 0x38, 0xEE, 0xC6, 0x0,  // Char 'x'
0x0, 0xE, 0x1E, 0xF0, 0xF0, 0x1E, 0xE, 0x0,  // Char 'y'
0x0, 0xC2, 0xE2, 0xB2, 0x9A, 0x8E, 0x86, 0x0,  // Char 'z'
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0x0, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x0, 
0 };
