#ifndef ALPHAPILOT_H
#define ALPHAPILOT_H

#include "Math.h"
#include <stdbool.h>

//
// Configuration
//

#define THROTTLE_SIGN        1
// #define HARD_PUSHER 1     // Uncomment to select "hard" pusher
#define FLAP_STEPS           2

//
// Periodic task timing
//

#define CONTROL_HZ 50
#define CONFIG_HZ (CONTROL_HZ/3.0f)
#define ALPHA_HZ (CONTROL_HZ*4)
#define AIRSPEED_HZ (CONTROL_HZ*2)
#define TRIM_HZ CONFIG_HZ
#define LED_HZ 2
#define LED_TICK 30
#define LOG_HZ (CONTROL_HZ/4.0f)
#define LOG_HZ_COMMIT 3
#define LOG_HZ_FLUSH 5
#define HEARTBEAT_HZ 1
  
struct Task {
  void (*code)(void);
  uint32_t period;
  bool realTime;
  uint32_t nextInvocation;
};

#define HZ_TO_PERIOD(f) ((uint32_t) (1.0e6f/(f)))

extern struct Task alphaPilotTasks[], *currentTask;

#endif
