#include <stdint.h>

//
// On-Board Display (SSD1306 OLED) interface
//

void obdRefresh();
void obdClear();
void obdMove(uint8_t col, uint8_t row);
void obdPrint(const char *s);
void obdPrint(const char *s, bool inv);
  
