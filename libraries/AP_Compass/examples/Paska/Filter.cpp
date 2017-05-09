#include "Filter.h"
#include "Console.h"
#include "NVState.h"
#include "Math.h"

void Median3Filter::input(float v)
{
  memory[ptr++] = v;
  if(ptr > MedianWindow_c-1) ptr = 0;
}

float Median3Filter::output(void)
{
  return max(min(memory[0],memory[1]), min(max(memory[0],memory[1]),memory[2]));
}

Damper::Damper(void)
{
  avg = 0;
  setTau(1);
}

Damper::Damper(float t)
{
  avg = 0;
  setTau(t);
}

Damper::Damper(float t, float i)
{
  avg = i;
  setTau(t);
}

float Damper::input(float v)
{
  avg = mixValue(tau, avg, v);
  return output();
}

void Damper::reset(float v)
{
  avg = v;
}

float Damper::output(void)
{
  return avg;
}

void Damper::setTau(float tauValue)
{
  tau = 1.0/(1+tauValue);
}

void Derivator::input(float v, float dt)
{
  prev = value;
  value = v;
  delta = dt;
}

float Derivator::output(void)
{
  return (value - prev) / delta;
}

void RateLimiter::setRate(float r)
{
  maxRate = r;
}  

void RateLimiter::reset(float v)
{
  state = v;
}  

float RateLimiter::input(float v, float dt)
{
  state += clamp(v - state, -maxRate*dt, maxRate*dt);
  return output();
}

float RateLimiter::output(void)
{
  return state;
}

RunningAvgFilter::RunningAvgFilter(int w)
{
  sum = 0.0;
  windowLen = w;
  memory = new float[w];
  memset(memory, '\0', sizeof(float)*w);
}

RunningAvgFilter::~RunningAvgFilter(int w)
{
  delete[] memory;
  memory = NULL;
  windowLen = 0;
}

float RunningAvgFilter::output() 
{ 
  return (float) sum / windowLen;
}

float RunningAvgFilter::input(float v) 
{ 
    ptr = (ptr + 1) % windowLen;

    sum -= memory[ptr];
    sum += memory[ptr] = v;
    
    return output();
}
  
void DelayLine::setDelay(int a) 
{
  if(a < 0 || a > DelayMax-1)
     a = DelayMax-1;

   delay = a;

   for(int i = 0; i < DelayMax; i++)
     memory[i] = 0.0;
}

float DelayLine::output() 
{ 
  return memory[(ptr+DelayMax-delay)%DelayMax];
}

float DelayLine::input(float v) 
{
  ptr = (ptr + 1) % DelayMax;
  memory[ptr] = v;
    
  return output();
}
  
float AlphaBuffer::output(void) { 
  if(length > 0) {
    value = sum / length;
    sum = 0.0;
    length = 0;
  } else if(!warn) {
    consoleNoteLn_P(PSTR("Alpha/IAS buffer starved"));
    warn = true;
  }  
  return value;
}

void AlphaBuffer::input(float v) { 
  length++;
  sum += v;
}
