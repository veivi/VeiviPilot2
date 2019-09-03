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
  while(vpTimeMillis() - start < x);
}

bool vpPeriodicEvent(VPPeriodicTimer_t *timer)
{
  if(vpTimeMillisApprox - timer->previousEvent >= timer->period) {
    timer->previousEvent = vpTimeMillisApprox;
    return true;
  } else
    return false;
}

static bool vpInertiaOnOff(VPInertiaTimer_t *timer, bool state)
{
  if(timer->state == state) {
    timer->startTime = vpTimeMillisApprox;
    return false;
  } else if(vpTimeMillisApprox - timer->startTime > timer->inertia) {
    timer->state = state;
    return true;
  } else
    return false;
}

bool vpInertiaOn(VPInertiaTimer_t *timer)
{
  return vpInertiaOnOff(timer, true);
}

bool vpInertiaOff(VPInertiaTimer_t *timer)
{
  return vpInertiaOnOff(timer, false);
}


