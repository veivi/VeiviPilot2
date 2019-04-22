#ifndef MATH_H
#define MATH_H

#include <stdint.h>
#include <math.h>

//
//
//

#define RATIO(v) (v ## .0f)
#define PI_F 3.141592654f
#define CIRCLE (2*PI_F)
#define RADIAN (180/PI_F)

extern const float stabGainExp_c, yawDamperExp_c, airDensity_c, G, FOOT, KNOT, PSF, servoOutputRange_c;

float signf(float x);
float sq(float x);
float nominalPitchRate(float bank, float pitch, float target);
float nominalPitchRateLevel(float bank, float target);
float constrainServoOutput(float value);
float coeffOfLift(float aoa);
float coeffOfLiftClean(float aoa);
float coeffOfLiftInverse(float col);
float dynamicPressure(float ias);
float dynamicPressureInverse(float pressure);
float alphaPredict(float x);
float alphaPredictInverse(float x);
float rollRatePredict(float r);
float rollRatePredictInverse(float x);
float scaleByIAS_E(float k, float p);
float scaleByIAS(float k);
float effIAS();
float effDP();
float totalMass(void);
void pseudoRandom(uint8_t *value, uint8_t size, uint16_t *state);

#endif

