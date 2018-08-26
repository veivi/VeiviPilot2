#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdint.h>
#include "StaP.h"

void consoleFlush();
void consoleCR();
void consoleNL();
void consolePrintC(const char c);
void consoleTab(int i);
void consoleNote_P(const CS_CHAR_T *s);
void consoleNote(const char *s);
void consolePanic_P(const CS_CHAR_T *s);
void consolePanic(const char *s);
void consoleNoteLn_P(const CS_CHAR_T *s);
void consoleNoteLn(const char *s);
void consolePrint_P(const CS_CHAR_T *s);
void consolePrint(const char *s);
void consolePrintN(const char *s, int);
void consolePrintFP(float v, int p);
void consolePrintF(float v);
void consolePrintDP(double v, int p);
void consolePrintD(double v);
void consolePrintI(int v);
void consolePrintUI(unsigned int v);
void consolePrintUI8(uint8_t v);
void consolePrintL(long v);
void consolePrintUL(unsigned long v);
void consolePrintLn_P(const CS_CHAR_T *s);
void consolePrintLn(const char *s);
void consolePrintLnF(float v);
void consolePrintLnFP(float v, int p);
void consolePrintLnD(double v);
void consolePrintLnDP(double v, int p);
void consolePrintLnI(int v);
void consolePrintLnUI(unsigned int v);
void consolePrintLnL(long v);
void consolePrintLnUL(unsigned long v);

#endif
