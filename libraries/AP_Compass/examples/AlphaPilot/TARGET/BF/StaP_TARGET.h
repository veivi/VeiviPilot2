#ifndef STAP_TARGET_H
#define STAP_TARGET_H

#include "platform.h"
#include "sensors/gyro.h"
#include "drivers/system.h"
#include "drivers/light_led.h"

#define STAP_PERIOD_GYRO         gyro.targetLooptime
#define STAP_PERIOD_GYRO_STATIC  HZ_TO_PERIOD(100)
#define STAP_PERIOD_ATTI         HZ_TO_PERIOD(100)
#define STAP_PERIOD_ACC          HZ_TO_PERIOD(100)

#define STAP_FORBID      __disable_irq(); nestCount++
#define STAP_PERMIT      if(!--nestCount) __enable_irq()
#define STAP_LED_ON      LED0_ON
#define STAP_LED_OFF     LED0_OFF

extern volatile uint8_t nestCount;

#define STAP_CANOPY_CLOSED  0

//
// Constant storage access (alias nasty AVR hack called "progmem")
//

#define CS_QUALIFIER  
#define CS_MEMCPY memcpy
#define CS_READCHAR(s) (*((const char *)s))
#define CS_STRNCPY(dst, src, s) strncpy(dst, src, s)
#define CS_STRING(s) s

#endif

