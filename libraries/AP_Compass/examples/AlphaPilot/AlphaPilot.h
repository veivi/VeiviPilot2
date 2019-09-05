#ifndef ALPHAPILOT_H
#define ALPHAPILOT_H

#include "Math.h"
#include "Time.h"
#include <stdbool.h>

//
// Configuration
//

// #define HARD_PUSHER 1     // Uncomment to select "hard" pusher
#define FLAP_STEPS           2

//
// Periodic task timing
//

#define AIR_SENSOR_OVERSAMPLE 2
// #define ASYNC_AIR_SENSORS 1

#define CONTROL_HZ 50.0f
#define CONFIG_HZ (CONTROL_HZ/3)
#define TRIM_HZ CONFIG_HZ
#define LED_HZ 2
#define LED_TICK 30
#define LOG_HZ (CONTROL_HZ/3)
#define LOG_HZ_COMMIT 2
#define LOG_HZ_FLUSH 2
#define HEARTBEAT_HZ 1
  
//
// Downlink (telemetry) max latencies
//

#define MAX_LATENCY_CONFIG    0.7e3

struct Task {
  void (*code)(void);
  VP_TIME_MILLIS_T period;
  bool realTime;
  VP_TIME_MILLIS_T nextInvocation;
  VP_TIME_MILLIS_T cumTime;
  int16_t cumCount, slipCount;
};

#define HZ_TO_PERIOD(f) ((VP_TIME_MILLIS_T) (1.0e3f/(f)))

extern struct Task alphaPilotTasks[], *currentTask;

#endif
