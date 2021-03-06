#include "Time.h"
#include "StaP.h"

VP_TIME_MICROS_T vpTimeMicrosApprox;
VP_TIME_MILLIS_T vpTimeMillisApprox;

void vpTimeAcquire(void)
{
  vpTimeMicrosApprox = (VP_TIME_MICROS_T) stap_timeMicros();
  vpTimeMillisApprox = (VP_TIME_MILLIS_T) (vpTimeMicrosApprox>>10);
}

VP_TIME_MICROS_T vpTimeMicros(void)
{
  vpTimeAcquire();
  return vpTimeMicrosApprox;
}

VP_TIME_MILLIS_T vpTimeMillis(void)
{
  vpTimeAcquire();
  return vpTimeMillisApprox;  
}

void vpDelayMillis(VP_TIME_MILLIS_T x)
{
  VP_TIME_MILLIS_T start = vpTimeMillis();
  while(VP_ELAPSED_MILLIS(start, vpTimeMillis()) < x);
}

bool vpPeriodicEvent(VPPeriodicTimer_t *timer)
{
  if(VP_ELAPSED_MILLIS(timer->previousEvent, vpTimeMillisApprox) >= timer->period) {
    timer->previousEvent = vpTimeMillisApprox;
    return true;
  } else
    return false;
}

void vpEventTimerReset(VPEventTimer_t *timer)
{
  timer->startTime = vpTimeMillisApprox;
}

bool vpEventTimerElapsed(VPEventTimer_t *timer)
{
  return VP_ELAPSED_MILLIS(timer->startTime, vpTimeMillisApprox) >= *(timer->delay);
}

static bool vpInertiaOnOff(VPInertiaTimer_t *timer, bool state, bool force)
{
  bool status = false;

  if(*timer->state == state) {
    timer->startTime = vpTimeMillisApprox;
    timer->elapsed = 0;
  } else if(timer->elapsed < timer->inertia) {
    timer->elapsed = VP_ELAPSED_MILLIS(timer->startTime, vpTimeMillisApprox);

    if(timer->elapsed >= timer->inertia || force) {
      status = true;
      *timer->state = state;
    }
  }
  
  return status;
}

bool vpInertiaOn(VPInertiaTimer_t *timer)
{
  return vpInertiaOnOff(timer, true, false);
}

bool vpInertiaOff(VPInertiaTimer_t *timer)
{
  return vpInertiaOnOff(timer, false, false);
}

bool vpInertiaOnForce(VPInertiaTimer_t *timer)
{
  return vpInertiaOnOff(timer, true, true);
}

bool vpInertiaOffForce(VPInertiaTimer_t *timer)
{
  return vpInertiaOnOff(timer, false, true);
}

