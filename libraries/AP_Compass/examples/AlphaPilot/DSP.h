#ifndef DSP_H
#define DSP_H

#include <stdint.h>
#include <stdbool.h>

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define ABS(a) ((a) < 0 ? -(a) : (a))

float sign(float x);
float clamp(float value, float a, float b);
float expo(float a, float b);
float mixValue(float mixRatio, float a, float b);
float randomNum(float small, float large);
uint32_t randomUInt32();
float quantize(float value, float *state, int numSteps);
float polynomial(int deg, float x, const float c[]);

typedef struct SWAvg {
  float *memory, sum;
  int windowLen;
  int ptr;
} SWAvg_t;

bool swAvgInit(SWAvg_t *, int w);
void swAvgFinalize(SWAvg_t *);
float swAvgInput(SWAvg_t *, float v);
float swAvgOutput(SWAvg_t *);
    
typedef struct Delay {
  float *memory;
  int delay, delayMax;
  int ptr;
} Delay_t;

bool delayInit(Delay_t *, int max);
void delayFinalize(Delay_t *);
void delaySet(Delay_t *, int l);
float delayInput(Delay_t *, float v);
float delayOutput(Delay_t *);

typedef struct Damper_s {
  float tau, state;
} Damper_t;

bool damperInit(Damper_t*, float tau, float state);
void damperFinalize(Damper_t*);
void damperReset(Damper_t*, float v);
void damperSetTau(Damper_t*, float tau);
float damperInput(Damper_t*, float v);
float damperOutput(Damper_t*);
    
typedef struct Washout_s {
  float state;
  struct Damper_s dc;
} Washout_t;

bool washoutInit(Washout_t*, float tau, float state);
void washoutFinalize(Washout_t*);
void washoutReset(Washout_t*, float v);
void washoutSetTau(Washout_t*, float tau);
float washoutInput(Washout_t*, float v);
float washoutOutput(Washout_t*);
    
typedef struct Derivator_s {
  float value, prev, delta;
} Derivator_t;

bool derivatorInit(Derivator_t*);
void derivatorFinalize(Derivator_t*);
float derivatorInput(Derivator_t*, float v, float dt);
float derivatorOutput(Derivator_t*);
    
typedef struct SlopeLimiter {
  float maxRate;
  float state;
} SlopeLimiter_t;

bool slopeInit(SlopeLimiter_t *, float r);
void slopeFinalize(SlopeLimiter_t *);
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

bool samplerInit(Sampler_t*);
void samplerFinalize(Sampler_t*);
void samplerInput(Sampler_t*, int16_t v);
float samplerMean(Sampler_t*);

#define MEDIANWINDOW  3
  
typedef struct MedianFilter {  
  float memory[MEDIANWINDOW];
  float state;
} MedianFilter_t;

float medianInput(MedianFilter_t *i, float v);
float medianOutput(MedianFilter_t *i);

typedef struct Turbine_s {
  float tau, state;
} Turbine_t;

bool turbineInit(Turbine_t*, float tau, float state);
void turbineFinalize(Turbine_t*);
void turbineReset(Turbine_t*, float v);
void turbineSetTau(Turbine_t*, float tau);
float turbineInput(Turbine_t*, float v);
float turbineOutput(Turbine_t*);

#endif
