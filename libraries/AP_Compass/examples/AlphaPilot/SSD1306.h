#include <stdint.h>
#include <stdbool.h>

//
// Font width
//

#define DISP_WIDTH 128
#define DISP_HEIGHT 64
#define FONT_WIDTH fontWidth
#define FONT_HEIGHT 8
#define DISP_ROWS (DISP_HEIGHT/FONT_HEIGHT)
#define DISP_COLS (DISP_WIDTH/FONT_WIDTH+(DISP_WIDTH%FONT_WIDTH ? 1 : 0))

//
// On-Board Display (SSD1306 OLED) interface
//

void obdRefresh();
void obdClear();
void obdMove(uint8_t col, uint8_t row);
void obdPrint(const char *s);
void obdPrintAttr(const char *s, bool inv);
  
extern const uint8_t fontWidth;
extern const uint8_t fontData[];
