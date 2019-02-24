#include <stdarg.h>
#include <math.h>
#include "Datagram.h"
#include "Console.h"
#include "Objects.h"

void consolevNotef(const char *s, va_list argp);
void consoleNotef(const char *s, ...);
void consoleNotefLn(const char *s, ...);
void consolevPrintf(const char *s, va_list argp);
void consolePrintf(const char *s, ...);
void consolePrintfLn(const char *s, ...);

#define BUF_SIZE (1<<6)

static uint8_t outputBuf[BUF_SIZE];
static uint8_t bufPtr;
static int column;

void consoleHeartbeat()
{
  static uint32_t last;
  
  if(vpStatus.consoleLink && stap_timeMillis() - last > 1e3) {
    datagramTxStart(DG_HEARTBEAT);
    datagramTxEnd();
    last = stap_timeMillis();
  }
}

void consoleFlush()
{
  consoleHeartbeat();
  
  if(bufPtr > 0) {
    datagramTxStart(DG_CONSOLE);
    datagramTxOut(outputBuf, bufPtr);
    datagramTxEnd();
  }
  
  bufPtr = 0;
}

void consoleOut(const uint8_t c)
{
  if(!vpStatus.consoleLink || vpMode.silent)
    return;
  
  if(bufPtr > BUF_SIZE-1)
    consoleFlush();

  outputBuf[bufPtr++] = c;
  column++;
}

void consoleNL(void)
{
  consoleOut('\n');
  consoleFlush();
  column = 0;
}

void consoleCR(void)
{
  consoleOut('\r');
  consoleFlush();
  column = 0;
}

void consoleTab(int i)
{
  while(column < i)
    consoleOut(' ');
}

void consoleNote(const char *s)
{
  consolePrint("// ");
  consolePrint(s);
}

void consoleNoteLn(const char *s)
{
  consoleNote(s);
  consoleNL();
}

void consoleNote_P(const char *s)
{
  consolePrint("// ");
  consolePrint_P(s);
}

void consoleNoteLn_P(const char *s)
{
  consoleNote_P(s);
  consoleNL();
}

void consolePanic_P(const char *s)
{
  consoleNote("// PANIC: ");
  consolePrintLn_P(s);
  consoleNoteLn("// HALT/REBOOT");
  consoleFlush();
  delay(5000);
  stap_reboot(false);
}

void consolevNotef(const char *s, va_list argp)
{
  consolePrint("// ");
  consolevPrintf(s, argp);
}

void consoleNotef(const char *s, ...)
{
  va_list argp;

  va_start(argp, s);
  consolevNotef(s, argp);
  va_end(argp);
}

void consoleNotefLn(const char *s, ...)
{
  va_list argp;

  va_start(argp, s);
  consolevNotef(s, argp);
  va_end(argp);
  
  consoleNL();
}

void consolePrintf(const char *s, ...)
{
  va_list argp;

  va_start(argp, s);
  consolevPrintf(s, argp);
  va_end(argp);
}

void consolePrintfLn(const char *s, ...)
{
  va_list argp;

  va_start(argp, s);
  consolevPrintf(s, argp);
  va_end(argp);

  consoleNL();
}

void consolevPrintf(const char *s, va_list argp)
{
  consolePrint(s);
}

void consolePrint(const char *s)
{
  while(*s)
    consoleOut(*s++);
}

void consolePrintN(const char *s, int l)
{
  while(*s && l-- > 0)
    consoleOut(*s++);
}

void consolePrint_P(const char *s)
{
  char c = 0;

  while((c = CS_READCHAR(s++)))
    consoleOut(c);
}

#define printDigit(d) consoleOut('0'+d)

void consolePrintFP(float v, int p)
{
  if(v < 0.0) {
    consoleOut('-');
    v = -v;
  }

  v += 0.5f*powf(0.1f, p);
  
  uint32_t i = (uint32_t) v;

  consolePrintUL(i);

  if(p > 0) {
    float f = v - (float) i;
  
    consolePrintC('.');

    while(p > 0) {
      f *= 10.0f;
      printDigit(((uint32_t) f) % 10);
      p--;
    }
  }
}

void consolePrintC(const char c)
{
  consoleOut(c);
}  

void consolePrintF(float v)
{
  consolePrintFP(v, 2);
}

void consolePrintDP(double v, int p)
{
  consolePrintFP((float) v, p);
}

void consolePrintD(double v)
{
  consolePrintDP(v, 2);
}

void consolePrintI(int v)
{
  consolePrintL((long) v);
}

void consolePrintUI(unsigned int v)
{
  consolePrintUL((unsigned long) v);
}

void consolePrintL(long v)
{
  if(v < 0) {
    v = -v;
    consoleOut('-');
  }

  consolePrintUL((unsigned long) v);
}

void consolePrintUL(unsigned long v)
{
  uint8_t buf[20];
  int l = 0;

  while(v > 0 && l < 20) {
    buf[l++] = v % 10;
    v /= 10;
  }

  if(l > 0) {
    while(l > 0)
      printDigit(buf[--l]);
  } else
    printDigit(0);
}

void consolePrintUI8(uint8_t v)
{
  consolePrintUI((unsigned int) v);
}

void consolePrintLn(const char *s)
{
  consolePrint(s);
  consoleNL();
}

void consolePrintLn_P(const char *s)
{
  consolePrint_P(s);
  consoleNL();
}

void consolePrintLnF(float v)
{
  consolePrintF(v);
  consoleNL();
}

void consolePrintLnFP(float v, int p)
{
  consolePrintFP(v, p);
  consoleNL();
}

void consolePrintLnD(double v)
{
  consolePrintD(v);
  consoleNL();
}

void consolePrintLnDP(double v, int p)
{
  consolePrintDP(v, p);
  consoleNL();
}

void consolePrintLnI(int v)
{
  consolePrintI(v);
  consoleNL();
}

void consolePrintLnUI(unsigned int v)
{
  consolePrintUI(v);
  consoleNL();
}

void consolePrintLnUI8(uint8_t v)
{
  consolePrintUI8(v);
  consoleNL();
}

void consolePrintLnL(long v)
{
  consolePrintL(v);
  consoleNL();
}

void consolePrintLnUL(unsigned long v)
{
  consolePrintUL(v);
  consoleNL();
}

