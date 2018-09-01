#ifndef ALPHAPILOT_H
#define ALPHAPILOT_H

#include "Math.h"

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
#define CONFIG_HZ (CONTROL_HZ/3.0)
#define ALPHA_HZ (CONTROL_HZ*8)
#define AIRSPEED_HZ (ALPHA_HZ/2)
#define TRIM_HZ CONFIG_HZ
#define LED_HZ 3
#define LED_TICK 100
#define LOG_HZ (CONTROL_HZ/4.0)
#define LOG_HZ_COMMIT 3
#define LOG_HZ_FLUSH 5
#define HEARTBEAT_HZ 1
  
struct Task {
  void (*code)(void);
  uint32_t period, lastExecuted;
};

#define HZ_TO_PERIOD(f) ((uint32_t) (1.0e6/(f)))

extern struct Task alphaPilotTasks[];

#endif
