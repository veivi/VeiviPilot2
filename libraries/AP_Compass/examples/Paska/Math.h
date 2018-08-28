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

extern const float stabilityElevExp_c, stabilityAileExp1_c, stabilityAileExp2_c , stabilityPusherExp_c, airDensity_c, G, FOOT, KNOT, PSF, servoOutputRange_c;

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
float scaleByIAS(float k, float p);
float effIAS();
float effDP();

#endif

