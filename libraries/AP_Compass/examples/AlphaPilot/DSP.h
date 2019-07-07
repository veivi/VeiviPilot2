#ifndef DSP_H
#define DSP_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define ABS(a) ((a) < 0 ? -(a) : (a))

float sign(float x);
float clamp(float value, float a, float b);
float expo(float a, float b);
float mixValue(float mixRatio, float a, float b);
float randomNum(float small, float large);
uint32_t randomUInt32(void);
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
  float value;
  int count;
  bool warn;
} Sampler_t;

#define SAMPLER_CONS { 0UL, 0.0f, 0 }

void samplerInput(Sampler_t*, int16_t v);
float samplerMean(Sampler_t*);

typedef struct Turbine_s {
  float tau, state;
} Turbine_t;

void turbineReset(Turbine_t*, float v);
void turbineSetTau(Turbine_t*, float tau);
float turbineInput(Turbine_t*, float v);
float turbineOutput(Turbine_t*);

#endif
