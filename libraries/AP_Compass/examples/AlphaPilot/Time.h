#ifndef VP_TIME_H
#define VP_TIME_H

#include <stdint.h>

typedef uint64_t VP_TIME_MICROS_T;
typedef uint32_t VP_TIME_MILLIS_T;

extern VP_TIME_MICROS_T vpTimeMicrosApprox;
extern VP_TIME_MILLIS_T vpTimeMillisApprox;

void vpTimeAcquire(void);
VP_TIME_MICROS_T vpTimeMicrosLive(void);
VP_TIME_MILLIS_T vpTimeMillisLive(void);
void vpDelayMillis(VP_TIME_MILLIS_T);

#endif


