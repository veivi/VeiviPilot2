#include <string.h>
#include <stdlib.h>
#include "SSD1306.h"
#include "DSP.h"
#include "BaseI2C.h"

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

static uint8_t *displayBuffer;
static uint8_t cursorCol, cursorRow;
static int8_t modifiedLeft[DISP_ROWS], modifiedRight[DISP_ROWS];
static bool inverseVideo = false;
static BaseI2CTarget_t target = { "display" };
static uint8_t scanRow, scanCol;
static bool scanning = false, initialized = false;

static bool SSD1306_transmitBuffers(const STAP_I2CBuffer_t *buffers, int numBuffers) 
{
  if(!basei2cMaybeOnline(&target))
    return false;
  
  return basei2cInvoke(&target, basei2cWriteBuffers(SSD1306_ADDR, buffers, numBuffers));
}

static bool SSD1306_transmit(uint8_t token, const uint8_t *data, uint8_t bytes) 
{
  STAP_I2CBuffer_t buffers[] = { { &token, 1 }, { data, bytes } };
    
  return SSD1306_transmitBuffers(buffers, sizeof(buffers)/sizeof(STAP_I2CBuffer_t));
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
  if(col > DISP_COLS-1)
    col = DISP_COLS-1;
  
  if(modifiedLeft[cursorRow] < 0)
    modifiedLeft[cursorRow] = modifiedRight[cursorRow] = col;
  else {
    modifiedLeft[cursorRow] = MIN(modifiedLeft[cursorRow], col);
    modifiedRight[cursorRow] = MAX(modifiedRight[cursorRow], col);
  }

  if(scanning && scanRow == cursorRow && scanCol > cursorCol) {
    // We hit a line on the left of the scan, abort
    scanning = false;
    scanRow++;
  }  
}

static void nprint(const char *s, uint8_t l)
{
  int i = 0;

  if(!displayBuffer)
    // Failed to allocate
    return;
  
  for(i = 0; i < l; i++) {
    uint8_t c = s ? s[i] : '\0';

    if(c == '\n') {
      nprint(NULL, DISP_COLS-cursorCol);
      continue;
    }

    if(inverseVideo)
      c |= 0x80;
    
    if(displayBuffer[cursorRow*DISP_COLS + cursorCol] != c) {
      displayBuffer[cursorRow*DISP_COLS + cursorCol] = c;
      markModified(cursorCol);
    }

    if(cursorCol < DISP_COLS-1)
      cursorCol++;
    else {
      cursorCol = 0;

      if(++cursorRow > DISP_ROWS-1)
	cursorRow = DISP_ROWS-1;
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
  
  for(i = 0; i < DISP_ROWS; i++) {
    obdMove(0, i);
    obdPrint("\n");
  }
}

void obdRefresh()
{
  int i = 0;

  if(!basei2cMaybeOnline(&target)) {
    initialized = false;
    return;
  }
  
  if(!displayBuffer) {
    displayBuffer = malloc(DISP_ROWS*DISP_COLS);
    if(!displayBuffer)
      return;
    memset(displayBuffer, '\0', DISP_ROWS*DISP_COLS);
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
    
    for(i = 0; i < DISP_ROWS; i++) {
      modifiedLeft[i] = 0;
      modifiedRight[i] = DISP_COLS-1;
    }

    scanRow = 0;
    initialized = true;
    scanning = false;
  }

  while(scanRow < DISP_ROWS) {    
    if(!scanning && modifiedLeft[scanRow] > -1) {
      // Initialize the scan of a (partial) row

      scanning = true;
      scanCol = modifiedLeft[scanRow];
      
      SSD1306_command(SSD1306_PAGEADDR);
      SSD1306_command(scanRow);
      SSD1306_command(scanRow);
      
      SSD1306_command(SSD1306_COLUMNADDR);
      SSD1306_command(scanCol*FONT_WIDTH);
      SSD1306_command((uint8_t) ~0U);
    }

    while(scanning && scanCol < modifiedRight[scanRow]+1) {
      // We're scanning a row and not done yet
      
      uint8_t buffer[FONT_WIDTH], chr = displayBuffer[scanRow*DISP_COLS+scanCol];

      CS_MEMCPY(buffer, &fontData[(chr & 0x7F)*FONT_WIDTH], sizeof(buffer));

      if(chr & 0x80) {
	for(i = 0; i < sizeof(buffer); i++)
	  buffer[i] = ~buffer[i];
      }

      const int size = MIN(sizeof(buffer), DISP_WIDTH - scanCol*FONT_WIDTH);
      SSD1306_data(buffer, size);
      scanCol++;

      if(vpMode.silent)
	// Supposed to be silent, only one column at a time to save bandwidth
	return;
    }

    if(scanning && scanCol > modifiedRight[scanRow]) {
      // Done with the scan line
      modifiedLeft[scanRow++] = scanCol = -1;
      scanning = false;
      return; // We refresh no more than one row at a time
    }

    scanRow++;
  }

  scanRow = 0;
}


