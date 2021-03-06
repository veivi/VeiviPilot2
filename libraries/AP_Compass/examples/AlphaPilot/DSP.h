#ifndef DSP_H
#define DSP_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "Math.h"

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

float clamp(float value, float a, float b);
float clampStatus(float value, float a, float b, bool *status);
float expo(float a, float b);
float mixValue(float mixRatio, float a, float b);
float quantize(float value, float *state, int numSteps);
float polynomial(int deg, float x, const float c[]);

typedef struct SWAvg {
  int window;
  float *memory, sum;
  int ptr;
} SWAvg_t;

#define SWAVG_CONS(w) { w, NULL, 0.0f, 0 };

float swAvgInput(SWAvg_t *, float v);
float swAvgOutput(SWAvg_t *);
    
typedef struct Damper_s {
  float tau, state;
} Damper_t;

#define DAMPER_TAU(t) (1.0 / (1.0 + (t)))
#define DAMPER_CONS(t, s) { DAMPER_TAU(t), s }

void damperReset(Damper_t*, float v);
void damperSetTau(Damper_t*, float tau);
float damperInput(Damper_t*, float v);
float damperOutput(Damper_t*);
    
typedef struct Washout_s {
  float state;
  struct Damper_s dc;
} Washout_t;

#define WASHOUT_CONS(t, s) (Washout_t) { s, { DAMPER_TAU(t), s } }

void washoutReset(Washout_t*, float v);
void washoutSetTau(Washout_t*, float tau);
float washoutInput(Washout_t*, float v);
float washoutOutput(Washout_t*);

typedef struct Derivator_s {
  float value, prev, delta;
} Derivator_t;

float derivatorInput(Derivator_t*, float v, float dt);
float derivatorOutput(Derivator_t*);

typedef struct SlopeLimiter {
  float maxRate;
  float state;
} SlopeLimiter_t;

#define SLOPE_CONS(r) (SlopeLimiter_t) { r, 0 }

float slopeInput(SlopeLimiter_t *, float v, float dt);
float slopeOutput(SlopeLimiter_t *);
void slopeSet(SlopeLimiter_t *, float v);
void slopeReset(SlopeLimiter_t *, float v);

typedef struct Sampler {
  int32_t acc;
  int16_t value;
  int count;
  bool warn;
} Sampler_t;

#define SAMPLER_CONS { 0UL, 0.0f, 0 }

void samplerInput(Sampler_t*, int16_t v);
int16_t samplerMean(Sampler_t*);

typedef struct Turbine_s {
  float tau, state;
} Turbine_t;

void turbineReset(Turbine_t*, float v);
void turbineSetTau(Turbine_t*, float tau);
float turbineInput(Turbine_t*, float v);
float turbineOutput(Turbine_t*);

#endif
