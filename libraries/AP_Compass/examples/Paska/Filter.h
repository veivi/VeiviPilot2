#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>
#include <stdlib.h>

class RunningAvgFilter {
 public:
  RunningAvgFilter(int w);
  ~RunningAvgFilter();
  float input(float v);
  float output();
    
 private:
  float *memory, sum;
  int windowLen;
  int ptr;
};

const int DelayMax = 10;

class DelayLine {
  public:
    void setDelay(int l);
    float input(float v);
    float output();
    
  private:
    float memory[DelayMax];
    int delay;
    int ptr;
};

class Derivator {
  public:
  void input(float v, float dt);
  float output();
    
  private:
  float value, prev, delta;
};

class RateLimiter {
  public:
  RateLimiter(void);
  RateLimiter(float);
  float input(float v, float dt);
  float output();
  void setRate(float v);
  void reset(float v);
    
  private:
  float maxRate;
  float state;
};

const int MedianWindow_c = 3;
  
class Median3Filter {  
  public:
    void input(float v);
    float output();
    
  private:
    float memory[MedianWindow_c];
    int ptr;
};

class AlphaBuffer {
public:
  float output(void);
  void input(float v);
  bool warn;
  
private:
  float sum, value;
  int length;
};

#endif
