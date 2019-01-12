#ifndef RXINPUT_H
#define RXINPUT_H

#include <stdint.h>
#include <stdbool.h>

//
// Receiver channel mapping
//

#define RX_CHANNELS          8

#if RX_CHANNELS < 8
#define CH_AILE       0
#define CH_ELEV       1
#define CH_THRO       2
#define CH_BUTTON     3
#define CH_TUNE       4
#define CH_MODE       5
#else
#define CH_AILE       0
#define CH_ELEV       1
#define CH_THRO       2
#define CH_RUD        3
#define CH_BUTTON     4
#define CH_TUNE       5
#define CH_MODE       6
#define CH_FLAP       7
#endif

struct RxInputRecord {
  bool alive;
  uint16_t pulseWidth;
};
  
struct SwitchRecord {
  uint8_t ch;
  int8_t state;
  float prevValue;
};

extern struct RxInputRecord rxInput[RX_CHANNELS];
extern struct SwitchRecord flightModeSelector, flapSelector;

void inputSource(const uint16_t *pulse, int numCh);
bool inputSourceGood(void);
float inputSourceRate(void);
void inputCalibStart(void);
void inputCalibStop(void);
bool inputValid(uint8_t);
float inputValue(uint8_t);
int8_t readSwitch(struct SwitchRecord *record);

float applyNullZone(float value, float nz, bool *pilotInput);
float applyNullZoneBlind(float value, float nz);
float applyExpo(float value);
float applyExpoTrim(float value, float trim);

#endif

