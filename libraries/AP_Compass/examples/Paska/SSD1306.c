#include <string.h>
#include "SSD1306.h"

// extern "C" {
#include "DSP.h"
#include "StaP.h"
#include "BaseI2C.h"
// }

//
// OLED display interface
//

#define SSD1306_128_64 1

#if defined SSD1306_128_64
  #define SSD1306_LCDWIDTH                  128
  #define SSD1306_LCDHEIGHT                 64
#endif
#if defined SSD1306_128_32
  #define SSD1306_LCDWIDTH                  128
  #define SSD1306_LCDHEIGHT                 32
#endif
#if defined SSD1306_96_16
  #define SSD1306_LCDWIDTH                  96
  #define SSD1306_LCDHEIGHT                 16
#endif

#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF

#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA

#define SSD1306_SETVCOMDETECT 0xDB

#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9

#define SSD1306_SETMULTIPLEX 0xA8

#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10

#define SSD1306_SETSTARTLINE 0x40

#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR   0x22

#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8

#define SSD1306_SEGREMAP 0xA0

#define SSD1306_CHARGEPUMP 0x8D

#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2

// Scrolling #defines
#define SSD1306_ACTIVATE_SCROLL 0x2F
#define SSD1306_DEACTIVATE_SCROLL 0x2E
#define SSD1306_SET_VERTICAL_SCROLL_AREA 0xA3
#define SSD1306_RIGHT_HORIZONTAL_SCROLL 0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL 0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 0x2A

#define SSD1306_ADDR (0x78>>1)
#define SSD1306_TOKEN_DATA     (1<<6)
#define SSD1306_TOKEN_COMMAND  0

static uint8_t displayBuffer[16*8];
static uint8_t cursorCol, cursorRow;
static int8_t modifiedLeft[8], modifiedRight[8];
static bool inverseVideo = false;
static BaseI2CTarget_t target = { "display" };

static bool SSD1306_transmitBuffers(const I2CBuffer_t *buffers, int numBuffers) 
{
  if(!basei2cIsOnline(&target))
    return false;
  
  return basei2cInvoke(&target, basei2cWriteBuffers(SSD1306_ADDR, buffers, numBuffers));
}

static bool SSD1306_transmit(uint8_t token, const uint8_t *data, uint8_t bytes) 
{
  I2CBuffer_t buffers[] = { { &token, 1 }, { data, bytes } };
    
  return SSD1306_transmitBuffers(buffers, sizeof(buffers)/sizeof(I2CBuffer_t));
}

static bool SSD1306_data(const uint8_t *storage, uint8_t bytes) 
{
  return SSD1306_transmit(SSD1306_TOKEN_DATA, storage, bytes);
}

static bool SSD1306_command(const uint8_t value)
{
  return SSD1306_transmit(SSD1306_TOKEN_COMMAND, &value, 1);
}

void obdMove(uint8_t col, uint8_t row)
{
  cursorCol = col;
  cursorRow = row;
}

static void markModified(uint8_t col)
{
  if(col > 15)
    col = 15;
  
  if(modifiedLeft[cursorRow] < 0)
    modifiedLeft[cursorRow] = modifiedRight[cursorRow] = col;
  else {
    modifiedLeft[cursorRow] = MIN(modifiedLeft[cursorRow], col);
    modifiedRight[cursorRow] = MAX(modifiedRight[cursorRow], col);
  }
}

static void nprint(const char *s, uint8_t l)
{
  int i = 0;
  
  for(i = 0; i < l; i++) {
    uint8_t c = s ? s[i] : '\0';

    if(c == '\n') {
      nprint(NULL, 16-cursorCol);
      continue;
    }

    if(inverseVideo)
      c |= 0x80;
    
    if(displayBuffer[cursorRow*16 + cursorCol] != c) {
      displayBuffer[cursorRow*16 + cursorCol] = c;
      markModified(cursorCol);
    }

    if(cursorCol < 16-1)
      cursorCol++;
    else {
      cursorCol = 0;

      if(cursorRow < 8-1)
	cursorRow++;
      else
	cursorRow = 0;
    }
  }
}

void obdPrint(const char *s)
{
  inverseVideo = false;
  nprint(s, strlen(s));
}

void obdPrintAttr(const char *s, bool inv)
{
  inverseVideo = inv;
  nprint(s, strlen(s));
}

void obdClear()
{
  int i = 0;
  
  for(i = 0; i < 8; i++) {
    obdMove(0, i);
    obdPrint("\n");
  }
}

const uint8_t fontData[] CS_QUALIFIER = {
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0x0, 0x0, 0x80, 0x60, 0x60, 0x0, 0x0, 0x0,  // Char ','
0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0,  // Char '-'
0x0, 0x0, 0x0, 0xC0, 0xC0, 0x0, 0x0, 0x0,  // Char '.'
0x80, 0x40, 0x20, 0x10, 0x8, 0x4, 0x2, 0x0,  // Char '/'
0x7C, 0xC2, 0xA2, 0x92, 0x8A, 0x86, 0x7C, 0x0,  // Char '0'
0x0, 0x0, 0x84, 0xFE, 0x80, 0x0, 0x0, 0x0,  // Char '1'
0x84, 0xC2, 0xA2, 0xA2, 0x92, 0x92, 0x8C, 0x0,  // Char '2'
0x44, 0x82, 0x82, 0x82, 0x92, 0x92, 0x6C, 0x0,  // Char '3'
0x0, 0x1E, 0x10, 0x10, 0x10, 0x10, 0xFC, 0x0,  // Char '4'
0x9E, 0x92, 0x92, 0x92, 0x92, 0x92, 0x62, 0x0,  // Char '5'
0x7C, 0x92, 0x92, 0x92, 0x92, 0x92, 0x60, 0x0,  // Char '6'
0x82, 0x42, 0x22, 0x12, 0xA, 0x6, 0x2, 0x0,  // Char '7'
0x6C, 0x92, 0x92, 0x92, 0x92, 0x92, 0x6C, 0x0,  // Char '8'
0xC, 0x92, 0x92, 0x92, 0x92, 0x92, 0x7C, 0x0,  // Char '9'
0x0, 0x0, 0xCC, 0xCC, 0x0, 0x0, 0x0, 0x0,  // Char ':'
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0x0, 0x28, 0x28, 0x28, 0x28, 0x28, 0x0, 0x0,  // Char '='
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0xC0, 0x30, 0x2C, 0x22, 0x2C, 0x30, 0xC0, 0x0,  // Char 'A'
0xFE, 0x92, 0x92, 0x92, 0x92, 0x92, 0x6C, 0x0,  // Char 'B'
0x38, 0x44, 0x82, 0x82, 0x82, 0x82, 0x44, 0x0,  // Char 'C'
0xFE, 0x82, 0x82, 0x82, 0x82, 0x44, 0x38, 0x0,  // Char 'D'
0xFE, 0x92, 0x92, 0x92, 0x92, 0x82, 0x82, 0x0,  // Char 'E'
0xFE, 0x12, 0x12, 0x12, 0x12, 0x2, 0x2, 0x0,  // Char 'F'
0x7C, 0x82, 0x82, 0x82, 0x92, 0x92, 0x64, 0x0,  // Char 'G'
0xFE, 0x10, 0x10, 0x10, 0x10, 0x10, 0xFE, 0x0,  // Char 'H'
0x0, 0x0, 0x82, 0xFE, 0x82, 0x0, 0x0, 0x0,  // Char 'I'
0x60, 0x80, 0x80, 0x80, 0x82, 0x7E, 0x2, 0x0,  // Char 'J'
0xFE, 0x20, 0x10, 0x28, 0x44, 0x82, 0x0, 0x0,  // Char 'K'
0xFE, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x0,  // Char 'L'
0xFE, 0x4, 0x8, 0x10, 0x8, 0x4, 0xFE, 0x0,  // Char 'M'
0xFE, 0x4, 0x8, 0x10, 0x20, 0x40, 0xFE, 0x0,  // Char 'N'
0x7C, 0x82, 0x82, 0x82, 0x82, 0x82, 0x7C, 0x0,  // Char 'O'
0xFE, 0x12, 0x12, 0x12, 0x12, 0x12, 0xC, 0x0,  // Char 'P'
0xFC, 0x42, 0xA2, 0x92, 0x82, 0x82, 0x7C, 0x0,  // Char 'Q'
0xFE, 0x12, 0x12, 0x32, 0x52, 0x92, 0xC, 0x0,  // Char 'R'
0x4C, 0x92, 0x92, 0x92, 0x92, 0x92, 0x64, 0x0,  // Char 'S'
0x2, 0x2, 0x2, 0xFE, 0x2, 0x2, 0x2, 0x0,  // Char 'T'
0x7E, 0x80, 0x80, 0x80, 0x80, 0x80, 0x7E, 0x0,  // Char 'U'
0x6, 0x18, 0x60, 0x80, 0x60, 0x18, 0x6, 0x0,  // Char 'V'
0x7E, 0x80, 0x80, 0x70, 0x80, 0x80, 0x7E, 0x0,  // Char 'W'
0x82, 0x44, 0x28, 0x10, 0x28, 0x44, 0x82, 0x0,  // Char 'X'
0x2, 0x4, 0x8, 0xF0, 0x8, 0x4, 0x2, 0x0,  // Char 'Y'
0x82, 0xC2, 0xA2, 0x92, 0x8A, 0x86, 0x82, 0x20,  // Char 'Z'
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80, 0x0,  // Char '\'
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x0,  // Char '_'
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0xC0, 0x30, 0x2C, 0x22, 0x2C, 0x30, 0xC0, 0x0,  // Char 'a'
0xFE, 0x92, 0x92, 0x92, 0x92, 0x92, 0x6C, 0x0,  // Char 'b'
0x38, 0x44, 0x82, 0x82, 0x82, 0x82, 0x44, 0x0,  // Char 'c'
0xFE, 0x82, 0x82, 0x82, 0x82, 0x44, 0x38, 0x0,  // Char 'd'
0xFE, 0x92, 0x92, 0x92, 0x92, 0x82, 0x82, 0x0,  // Char 'e'
0xFE, 0x12, 0x12, 0x12, 0x12, 0x2, 0x2, 0x0,  // Char 'f'
0x7C, 0x82, 0x82, 0x82, 0x92, 0x92, 0x64, 0x0,  // Char 'g'
0xFE, 0x10, 0x10, 0x10, 0x10, 0x10, 0xFE, 0x0,  // Char 'h'
0x0, 0x0, 0x82, 0xFE, 0x82, 0x0, 0x0, 0x0,  // Char 'i'
0x60, 0x80, 0x80, 0x80, 0x82, 0x7E, 0x2, 0x0,  // Char 'j'
0xFE, 0x20, 0x10, 0x28, 0x44, 0x82, 0x0, 0x0,  // Char 'k'
0xFE, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x0,  // Char 'l'
0xFE, 0x4, 0x8, 0x10, 0x8, 0x4, 0xFE, 0x0,  // Char 'm'
0xFE, 0x4, 0x8, 0x10, 0x20, 0x40, 0xFE, 0x0,  // Char 'n'
0x7C, 0x82, 0x82, 0x82, 0x82, 0x82, 0x7C, 0x0,  // Char 'o'
0xFE, 0x12, 0x12, 0x12, 0x12, 0x12, 0xC, 0x0,  // Char 'p'
0xFC, 0x42, 0xA2, 0x92, 0x82, 0x82, 0x7C, 0x0,  // Char 'q'
0xFE, 0x12, 0x12, 0x32, 0x52, 0x92, 0xC, 0x0,  // Char 'r'
0x4C, 0x92, 0x92, 0x92, 0x92, 0x92, 0x64, 0x0,  // Char 's'
0x2, 0x2, 0x2, 0xFE, 0x2, 0x2, 0x2, 0x0,  // Char 't'
0x7E, 0x80, 0x80, 0x80, 0x80, 0x80, 0x7E, 0x0,  // Char 'u'
0x6, 0x18, 0x60, 0x80, 0x60, 0x18, 0x6, 0x0,  // Char 'v'
0x7E, 0x80, 0x80, 0x70, 0x80, 0x80, 0x7E, 0x0,  // Char 'w'
0x82, 0x44, 0x28, 0x10, 0x28, 0x44, 0x82, 0x0,  // Char 'x'
0x2, 0x4, 0x8, 0xF0, 0x8, 0x4, 0x2, 0x0,  // Char 'y'
0x82, 0xC2, 0xA2, 0x92, 0x8A, 0x86, 0x82, 0x20,  // Char 'z'
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
};

void obdRefresh()
{
  static bool initialized = false;
  static uint8_t row;
  int i = 0;

  if(!basei2cIsOnline(&target)) {
    initialized = false;
    return;
  }
  
  if(!initialized) {
    SSD1306_command(SSD1306_DISPLAYOFF);          // 0xAE
    SSD1306_command(SSD1306_SETDISPLAYCLOCKDIV);  // 0xD5
    SSD1306_command(0x80);                        // the suggested ratio 0x80
    SSD1306_command(SSD1306_SETMULTIPLEX);        // 0xA8
    SSD1306_command(SSD1306_LCDHEIGHT - 1);
    SSD1306_command(SSD1306_SETDISPLAYOFFSET);    // 0xD3
    SSD1306_command(0x0);                         // no offset
    SSD1306_command(SSD1306_SETSTARTLINE | 0x0);  // line #0
    SSD1306_command(SSD1306_CHARGEPUMP);          // 0x8D
    SSD1306_command(0x14);
    SSD1306_command(SSD1306_MEMORYMODE);          // 0x20
    SSD1306_command(0x00);                        // 0x0 act like ks0108
    SSD1306_command(SSD1306_SEGREMAP | 0x1);
    SSD1306_command(SSD1306_COMSCANDEC);
    SSD1306_command(SSD1306_SETCOMPINS);          // 0xDA
    SSD1306_command(0x12);
    SSD1306_command(SSD1306_SETCONTRAST);         // 0x81
    SSD1306_command(0xCF);
    SSD1306_command(SSD1306_SETPRECHARGE);        // 0xd9
    SSD1306_command(0xF1);
    SSD1306_command(SSD1306_SETVCOMDETECT);       // 0xDB
    SSD1306_command(0x40);
    SSD1306_command(SSD1306_DISPLAYALLON_RESUME); // 0xA4
    SSD1306_command(SSD1306_NORMALDISPLAY);       // 0xA6
    SSD1306_command(SSD1306_DEACTIVATE_SCROLL);
    
    SSD1306_command(SSD1306_DISPLAYON);
    
    for(i = 0; i < 8; i++) {
      modifiedLeft[i] = 0;
      modifiedRight[i] = 15;
    }

    initialized = true;
  }

  while(row < 8) {    
    if(modifiedLeft[row] > -1) {
      int col = 0;
      
      SSD1306_command(SSD1306_PAGEADDR);
      SSD1306_command(row);
      SSD1306_command(row);
      
      SSD1306_command(SSD1306_COLUMNADDR);
      SSD1306_command(modifiedLeft[row]*8);
      SSD1306_command((uint8_t) ~0U);

      for(col = modifiedLeft[row]; col < modifiedRight[row]+1; col++) {
	uint8_t buffer[8], chr = displayBuffer[row*16+col];

	CS_MEMCPY(buffer, &fontData[(chr & 0x7F)*8], sizeof(buffer));

	if(chr & 0x80) {
	  for(i = 0; i < sizeof(buffer); i++)
	    buffer[i] = ~buffer[i];
	}

	SSD1306_data(buffer, sizeof(buffer));
      }

      modifiedLeft[row++] = -1;
      
      return; // We refresh no more than one row at a time
    }

    row++;
  }

  row = 0;
}


