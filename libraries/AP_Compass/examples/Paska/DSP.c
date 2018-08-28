#include "DSP.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "Console.h"

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

/*
void Median3Filter::input(float v)
{
  memory[ptr++] = v;
  if(ptr > MedianWindow_c-1) ptr = 0;
}

float Median3Filter::output(void)
{
  return max(min(memory[0],memory[1]), min(max(memory[0],memory[1]),memory[2]));
}
*/

bool damperInit(Damper_t *damper, float tau, float state)
{
  damper->state = state;
  damperSetTau(damper, tau);
  return true;
}

float damperInput(Damper_t *damper, float v)
{
  damper->state = mixValue(damper->tau, damper->state, v);
  return damper->state;
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
  damper->tau = 1.0 / (1 + t);
}

bool swAvgInit(SWAvg_t *f, int w)
{
  f->sum = 0.0;
  f->windowLen = w;
  f->memory = malloc(sizeof(float)*w);

  if(f->memory)
    memset(f->memory, '\0', sizeof(float)*w);
  
  return f->memory != NULL;
}

void swAvgFinalize(SWAvg_t *f)
{
  if(f->memory)
    free(f->memory);

  f->memory = NULL;
  f->windowLen = 0;
}

float swAvgInput(SWAvg_t *f, float v)
{
  if(!f->memory)
    return 0;
  
  f->ptr = (f->ptr + 1) % f->windowLen;

  f->sum -= f->memory[f->ptr];
  f->sum += f->memory[f->ptr] = v;
    
  return swAvgOutput(f);
}

float swAvgOutput(SWAvg_t *f)
{
  return (float) f->sum / f->windowLen;
}
    
bool delayInit(Delay_t *f, int m)
{
  f->ptr = 0;
  f->delay = f->delayMax = m;
  f->memory = malloc(sizeof(float)*m);

  if(f->memory)
    memset(f->memory, '\0', sizeof(float)*m);

  return f->memory != NULL;
}

void delayFinalize(Delay_t *d)
{
  if(d->memory)
    free(d->memory);
  
  d->memory = NULL;
  d->delay = d->delayMax = 0;
}

void delaySet(Delay_t *d, int a)
{
  if(a < 0 || a > d->delayMax-1)
     a = d->delayMax-1;

  d->delay = a;
}

float delayInput(Delay_t *d, float v)
{
  d->ptr = (d->ptr + 1) % d->delayMax;
  d->memory[d->ptr] = v;
    
  return delayOutput(d);
}
  
float delayOutput(Delay_t *d)
{
  if(!d->memory)
    return 0;
  
  return d->memory[(d->ptr+d->delayMax-d->delay)%d->delayMax];
}

bool derivatorInit(Derivator_t *d)
{
  return true;
}

void derivatorFinalize(Derivator_t *d)
{
}

float derivatorinput(Derivator_t *d, float v, float dt)
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

bool slopeInit(SlopeLimiter_t *f)
{
  return true;
}

void slopeFinalize(SlopeLimiter_t *f)
{
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

void slopeSetRate(SlopeLimiter_t *f, float v)
{
  f->maxRate = v;
}

void slopeReset(SlopeLimiter_t *f, float v)
{
  f->state = v;
}

bool samplerInit(Sampler_t *f)
{
  return true;
}

void samplerFinalize(Sampler_t *f)
{
}

void samplerInput(Sampler_t *f, float v)
{
  f->count++;
  f->sum += v;
}

float samplerMean(Sampler_t *f)
{
  if(f->count > 0) {
    f->value = f->sum / f->count;
    f->sum = 0.0;
    f->count = 0;
  } else if(!f->warn) {
    consoleNoteLn_P(CS_STRING("Sample buffer starved"));
    f->warn = true;
  }  
  return f->value;
}
