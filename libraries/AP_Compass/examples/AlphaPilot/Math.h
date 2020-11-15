#ifndef VPMATH_H
#define VPMATH_H

#include <stdint.h>
#include <math.h>

//
// MIN/MAX macros
//

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define ABS(a) ((a) < 0 ? -(a) : (a))

extern const float stabGainExp_c, yawDamperExp_c, airDensity_c;

int sign(float x);
float signf(float x);
float sqrf(float x);
float nominalPitchRate(float bank, float pitch, float target);
float nominalPitchRateLevel(float bank, float target);
float constrainServoOutput(float value);
float coeffOfLift(float aoa);
float coeffOfLiftClean(float aoa);
float coeffOfLiftLand(float aoa);
float coeffOfLiftInverse(float col);
float dynamicPressure(float ias);
float dynamicPressureInverse(float pressure);
float alphaPredict(float x);
float alphaPredictInverse(float x);
float rollRatePredict(float r);
float rollRatePredictInverse(float x);
float scaleByIAS_E(float k, float p);
float scaleByIAS(float k);
void pseudoRandom(uint8_t *value, uint8_t size, uint16_t *state);
float randomNum(float small, float large);
uint16_t randomUINT16(void);
uint32_t randomUINT32(void);

#endif

