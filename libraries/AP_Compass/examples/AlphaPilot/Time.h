#ifndef VP_TIME_H
#define VP_TIME_H

#include <stdint.h>
#include <stdbool.h>

typedef uint64_t VP_TIME_MICROS_T;
typedef uint32_t VP_TIME_MILLIS_T;

typedef struct VPInertiaTimer {
  VP_TIME_MILLIS_T inertia;
  VP_TIME_MILLIS_T startTime;
  bool state;
} VPInertiaTimer_t;

bool vpInertiaOn(VPInertiaTimer_t*);  // Is it okay to turn on?
bool vpInertiaOff(VPInertiaTimer_t*); // Okay to turn off?

typedef struct VPPeriodicTimer {
  VP_TIME_MILLIS_T period;
  VP_TIME_MILLIS_T previousEvent;
} VPPeriodicTimer_t;

#define VP_PERIODIC_TIMER_CONS(p) { p, 0 }

bool vpPeriodicEvent(VPPeriodicTimer_t*);

extern VP_TIME_MICROS_T vpTimeMicrosApprox;
extern VP_TIME_MILLIS_T vpTimeMillisApprox;

void vpTimeAcquire(void);
VP_TIME_MICROS_T vpTimeMicros(void);
VP_TIME_MILLIS_T vpTimeMillis(void);
void vpDelayMillis(VP_TIME_MILLIS_T);

#endif


