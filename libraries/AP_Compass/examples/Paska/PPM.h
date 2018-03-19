#ifndef PPM_H
#define PPM_H

#include "RxInput.h"

//

extern uint8_t ppmNumChannels;
extern bool ppmWarnSlow, ppmWarnShort;
extern struct RxInputRecord *ppmInputs[];

void ppmInputInit(const int32_t *min, const int32_t *center, const int32_t *max);
float ppmFrameRate();

void calibStart();
void calibStop(int32_t *min, int32_t *center, int32_t *max);
  
#endif

