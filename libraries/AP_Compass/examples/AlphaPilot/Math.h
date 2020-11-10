#ifndef MATH_H
#define MATH_H

#include <stdint.h>
#include <math.h>

//
// MIN/MAX macros
//

#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef ABS
#define ABS(a) ((a) < 0 ? -(a) : (a))
#endif

//
// Various constants
//

#define RATIO(v) (v ## .0f)
#define PI_F    3.141592654f
#define CIRCLE  (2*PI_F)
#define RADIAN  (180/PI_F)
#define G       9.81f
#define FOOT    (12*25.4f/1000)
#define KNOT    (1852.0f/60/60)
#define PSF     47.880259f

extern const float stabGainExp_c, yawDamperExp_c, airDensity_c;

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
uint16_t randomUINT16(void);

#endif

