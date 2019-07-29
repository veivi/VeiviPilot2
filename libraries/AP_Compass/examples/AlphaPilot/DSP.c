#include "DSP.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "Console.h"
#include "Math.h"

float expo(float a, float b)
{
  return sign(a)*powf(fabsf(a), b);
}    

float polynomial(int deg, float x, const float c[])
{
  float acc = 0, p = 1;
  int i = 0;

  for(i = 0; i < deg+1; i++) {
    acc += c[i] * p;
    p *= x;
  }

  return acc;
}

float sign(float x)
{
  return x < 0.0 ? -1.0 : 1.0;
}

float clamp(float value, float a, float b)
{
  if(a > b) {
    // Swap limits
    float t = a;
    a = b;
    b = t;
  }

  if(value > a && value < b)
    return value;  
  else if(value <= a)
    return a;
  else if(value >= b)
    return b;

  // All comparisons failed, must be NaN or some such
  
  return 0.0;
}

float mixValue(float mixRatio, float a, float b)
{
  if(mixRatio < 0)
    return a;
  else if(mixRatio > 1)
    return b;
  else
    return (1.0 - mixRatio)*a + mixRatio*b;
}

float randomNum(float small, float large)
{
  return small + (large-small)*(float) ((rand()>>3) & 0xFFF) / 0x1000;
}

uint32_t randomUInt32()
{
  uint32_t buffer = 0;
  unsigned int i = 0;
  for(i = 0; i < sizeof(buffer); i++)
    buffer = (buffer<<8) | (uint32_t) randomNum(0, 1<<8);
  return buffer;
}

float quantize(float value, float *state, int numSteps)
{
  if((int) ((value-1.0/numSteps/2)*numSteps) > *state)
    *state = (value-1.0/numSteps/2)*numSteps;
  else if((int) ((value+1.0/numSteps/2)*numSteps) < *state)
    *state = (value+1.0/numSteps/2)*numSteps;

  return (float) *state / numSteps;
}

float damperInput(Damper_t *damper, float v)
{
  damper->state = mixValue(damper->tau, damper->state, v);
  return damperOutput(damper);
}

void damperReset(Damper_t *damper, float v)
{
  damper->state = v;
}

float damperOutput(Damper_t *damper)
{
  return damper->state;
}

void damperSetTau(Damper_t *damper, float t)
{
  damper->tau = DAMPER_TAU(t);
}

void washoutReset(Washout_t *i, float v)
{
  i->state = v;
  damperReset(&i->dc, v);
}

void washoutSetTau(Washout_t *i, float tau)
{
  damperSetTau(&i->dc, tau);
}

float washoutInput(Washout_t *i, float v)
{
  i->state = v;
  damperInput(&i->dc, v);
  return washoutOutput(i);
}

float washoutOutput(Washout_t *i)
{
  return i->state - damperOutput(&i->dc);
}
    
static bool swAvgAllocate(SWAvg_t *f)
{
  if(f->memory)
    return true;

  f->sum = 0.0;
  f->memory = malloc(sizeof(float)*f->window);

  if(f->memory)
    memset(f->memory, '\0', sizeof(float)*f->window);

  return f->memory != NULL;
}

float swAvgInput(SWAvg_t *f, float v)
{
  if(swAvgAllocate(f)) {
    f->ptr = (f->ptr + 1) % f->window;
    f->sum -= f->memory[f->ptr];
    f->sum += f->memory[f->ptr] = v;
  }
  
  return swAvgOutput(f);
}

float swAvgOutput(SWAvg_t *f)
{
  if(f->memory)
    return f->sum / f->window;
  else
    return NAN;
}

float derivatorInput(Derivator_t *d, float v, float dt)
{
  d->prev = d->value;
  d->value = v;
  d->delta = dt;
  
  return derivatorOutput(d);
}

float derivatorOutput(Derivator_t *d)
{
  return (d->value - d->prev) / d->delta;
}

float slopeInput(SlopeLimiter_t *f, float v, float dt)
{
  f->state += clamp(v - f->state, -f->maxRate*dt, f->maxRate*dt);
  return slopeOutput(f);
}

float slopeOutput(SlopeLimiter_t *f)
{
  return f->state;
}

void slopeSet(SlopeLimiter_t *f, float v)
{
  f->maxRate = v;
}

void slopeReset(SlopeLimiter_t *f, float v)
{
  f->state = v;
}

void samplerInput(Sampler_t *f, int16_t v)
{
  f->acc += (int32_t) v;
  f->count++;
}

float samplerMean(Sampler_t *f)
{
  if(f->count > 0) {
    f->value = (float) f->acc / f->count;
    f->acc = 0;
    f->count = 0;
  } else if(!f->warn) {
    consoleNoteLn_P(CS_STRING("Sample buffer starved"));
    f->warn = true;
  }  
  return f->value;
}

void turbineReset(Turbine_t *i, float v)
{
  i->state = v;
}

void turbineSetTau(Turbine_t *i, float tau)
{
  i->tau = 1.0f / (0.1 + tau);
}

float turbineInput(Turbine_t *i, float v)
{
  const float idle_c = 0.3f, boost_c = 5;

  if(v > i->state) {
    i->state += boost_c*(v - i->state)*i->tau
      *sqrf((i->state + idle_c)/(1 + idle_c));
    i->state = clamp(i->state, 0, v);
  } else {
    i->state -= MIN(boost_c * (i->state - v) * i->tau, 0.8*i->tau);
    i->state = clamp(i->state, v, 1);
  }
  
  return turbineOutput(i);
}

float turbineOutput(Turbine_t *i)
{
  return i->state;
}
