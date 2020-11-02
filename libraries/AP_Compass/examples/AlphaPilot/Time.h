#ifndef VP_TIME_H
#define VP_TIME_H

#include <stdint.h>
#include <stdbool.h>

typedef uint32_t VP_TIME_MICROS_T;
typedef uint16_t VP_TIME_MILLIS_T;

#define VP_TIME_MICROS_MAX 0xFFFFFFFFUL
#define VP_TIME_MILLIS_MAX 0xFFFFU

#define VP_ELAPSED_MILLIS(start, stop) (VP_TIME_MILLIS_T) ((stop - start) & 0xFFFF)
#define VP_ELAPSED_MICROS(start, stop) (VP_TIME_MICROS_T) ((stop - start) & 0xFFFFFFFFUL)

typedef struct VPInertiaTimer {
  bool *state;
  VP_TIME_MILLIS_T inertia;
  VP_TIME_MILLIS_T startTime;
} VPInertiaTimer_t;

#define VP_INERTIA_TIMER_CONS(s, i) { s, i, 0 }

bool vpInertiaOn(VPInertiaTimer_t*);  // Is it okay to turn on?
bool vpInertiaOff(VPInertiaTimer_t*); // Okay to turn off?
bool vpInertiaOnForce(VPInertiaTimer_t*);
bool vpInertiaOffForce(VPInertiaTimer_t*);

typedef struct VPPeriodicTimer {
  VP_TIME_MILLIS_T period;
  VP_TIME_MILLIS_T previousEvent;
} VPPeriodicTimer_t;

#define VP_PERIODIC_TIMER_CONS(p) { p, 0 }

bool vpPeriodicEvent(VPPeriodicTimer_t*);

typedef struct VPEventTimer {
  const uint16_t *delay;
  VP_TIME_MILLIS_T startTime;
} VPEventTimer_t;

#define VP_EVENT_TIMER_CONS(d) { d, 0 }

void vpEventTimerReset(VPEventTimer_t*);
bool vpEventTimerElapsed(VPEventTimer_t*);

extern VP_TIME_MICROS_T vpTimeMicrosApprox;
extern VP_TIME_MILLIS_T vpTimeMillisApprox;

void vpTimeAcquire(void);
VP_TIME_MICROS_T vpTimeMicros(void);
VP_TIME_MILLIS_T vpTimeMillis(void);
void vpDelayMillis(VP_TIME_MILLIS_T);

#endif


