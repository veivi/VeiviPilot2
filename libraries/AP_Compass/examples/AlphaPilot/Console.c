#include <stdarg.h>
#include <math.h>
#include "Datagram.h"
#include "Console.h"
#include "Objects.h"
#include "Time.h"
#include "Buffer.h"

void consolevNotef(const char *s, va_list argp);
void consoleNotef(const char *s, ...);
void consoleNotefLn(const char *s, ...);
void consolevPrintf(const char *s, va_list argp);
void consolePrintf(const char *s, ...);
void consolePrintfLn(const char *s, ...);

static VPBuffer_t consoleBuffer;
static int column;

void consoleFlush()
{
  char buffer[VPBUFFER_SIZE];
  int8_t s = vpMode.silent ? 16 : VPBUFFER_SIZE;

  s = vpbuffer_extract(&consoleBuffer, buffer, s);

  if(s > 0) {
    datagramTxStart(DG_CONSOLE);
    datagramTxOut((const uint8_t*) buffer, s);
    datagramTxEnd();
  }
}

void consoleOut(const char *b, int8_t s)
{
  int8_t space = vpbuffer_space(&consoleBuffer);
  
  if(space >= s || vpMode.silent) {
    // There's room in the buffer or we must overwrite anyway

    vpbuffer_insert(&consoleBuffer, b, s, true);
    column += s;

  } else {
    // No room, block and transmit everything in pieces

    do {
      consoleFlush();

      int8_t w = vpbuffer_insert(&consoleBuffer, b, s, false);
      b += w;
      s -= w;
      column += w;
    } while(s > 0);
  }
}

void consoleOutChar(const char c)
{
  consoleOut(&c, 1);
}

void consoleNL(void)
{
  consoleOutChar('\n');
  if(!vpMode.silent)
    consoleFlush();
  column = 0;
}

void consoleCR(void)
{
  consoleOutChar('\r');
  if(!vpMode.silent)
    consoleFlush();
  column = 0;
}

void consoleTab(int i)
{
  int n = i - column;
  
  while(n-- > 0)
    consoleOutChar(' ');
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
  vpDelayMillis(5000);
  stap_reboot(false);
}

/*
void consoleAssert(bool value, const char *msg)
{
  if(vpMode.silent || value)
    return;

  consoleNote_P(CS_STRING("ASSERTION FAILURE : "));
  consolePrintLn_P(msg);
  consolePanic_P(CS_STRING(""));
}
*/

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
  consolePrintN(s, VPBUFFER_SIZE);
}

void consolePrintN(const char *s, int8_t l)
{
  char buffer[VPBUFFER_SIZE];
  strncpy(buffer, s, VPBUFFER_SIZE-1);
  consoleOut(buffer, strlen(buffer));
}

void consolePrint_P(const char *s)
{
  char buffer[VPBUFFER_SIZE];
  CS_STRNCPY(buffer, s, VPBUFFER_SIZE-1);
  consoleOut(buffer, strlen(buffer));  
}

#define printDigit(d) consoleOutChar('0'+d)

void consolePrintFP(float v, int p)
{
  if(isinf(v))
    consolePrint("inf");
  else if(isnan(v))
    consolePrint("nan");
  else {
  if(v < 0.0) {
    consoleOutChar('-');
    v = -v;
  }

  v += 0.5f*powf(0.1f, p);
  
  uint32_t i = (uint32_t) v;

  consolePrintUL(i);

  if(p > 0) {
    float f = v - (float) i;
  
    consolePrintC('.');

    while(p-- > 0) {
      f *= 10.0f;
      printDigit(((uint32_t) f) % 10);
    }
  }
  }
}

void consolePrintC(const char c)
{
  consoleOutChar(c);
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
    consoleOutChar('-');
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

void consolePrintPoly(int degree, float *coeff, int p)
{
  int i = 0;
  
  for(i = 0; i < degree+1; i++) {
    if(i > 0)
      consolePrint_P(CS_STRING(" + "));
    consolePrintFP(coeff[i], p);
    if(i > 0) {
      consolePrint_P(CS_STRING(" x"));
      if(i > 1) {
	consolePrint("^");
	consolePrintI(i);
      }
    }
  }
}
