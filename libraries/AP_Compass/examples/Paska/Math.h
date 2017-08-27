#ifndef MATH_H
#define MATH_H

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <AP_Math/AP_Math.h>
#include "Filter.h"

//
//
//

const float stabilityElevExp_c = -1.5;
const float stabilityAileExp1_c = -1.5;
const float stabilityAileExp2_c = 1.0;
const float stabilityPusherExp_c = -0.5;

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define ABS(a) ((a) < 0 ? -(a) : (a))
#define RATIO(v) (v ## .0f)

extern Damper iasFilter;

const float airDensity_c = 1.225;

const float G = 9.81, CIRCLE = 2*PI, RADIAN = 360/CIRCLE, FOOT = 12*25.4/1000, KNOT = 1852.0/60/60, PSF = 47.880259;

const float servoOutputRange_c = RATIO(6/5);

float sign(float x);
float clamp(float value, float a, float b);
float nominalPitchRate(float bank, float target);
float constrainServoOutput(float value);
float expo(float a, float b);
float mixValue(float mixRatio, float a, float b);
uint8_t population(uint16_t a);
float randomNum(float small, float large);
uint32_t randomUInt32();
float quantize(float value, float *state, int numSteps);
float coeffOfLift(float aoa);
float coeffOfLiftInverse(float col);
float dynamicPressure(float ias);
float dynamicPressureInverse(float pressure);
float alphaPredict(float x);
float alphaPredictInverse(float x);
float rollRatePredict(float r);
float rollRatePredictInverse(float x);
float scaleByIAS(float k, float p);

#endif

