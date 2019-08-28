#include "Time.h"
#include "StaP.h"

VP_TIME_MICROS_T vpTimeMicrosApprox;
VP_TIME_MILLIS_T vpTimeMillisApprox;

void vpTimeAcquire(void)
{
  vpTimeMicrosApprox = (VP_TIME_MICROS_T) stap_timeMicros();
  vpTimeMillisApprox = (VP_TIME_MILLIS_T) (vpTimeMicrosApprox>>10);
}

VP_TIME_MICROS_T vpTimeMicrosLive(void)
{
  vpTimeAcquire();
  return vpTimeMicrosApprox;
}

VP_TIME_MILLIS_T vpTimeMillisLive(void)
{
  vpTimeAcquire();
  return vpTimeMillisApprox;  
}

void vpDelayMillis(VP_TIME_MILLIS_T x)
{
  VP_TIME_MILLIS_T current = vpTimeMillisLive();
  while(vpTimeMillisLive() < current+x);
}
  
