#ifndef ALPHAPILOT_H
#define ALPHAPILOT_H

#include "Math.h"
#include "Time.h"
#include <stdbool.h>

//
// Configuration
//

#define RX_CHANNELS          8
#define SYNC_PWM_OUTPUT      1
#define PWM_HZ               50
#define HARD_PUSHER          0
#define FLAP_STEPS           2
#define SHAKER_LIMIT         RATIO(1/3)
#define HARD_SHAKER          0
#define YAW_DAMPER           0
#define AUTO_RUDDER          0
#define STICK_PULL_BRAKE_THR RATIO(1/3)
#define PWM_PERIOD           8

//
// PWM/RX in/out constants
//

#define RX_CHANNELS_MAX      16
#define SERVO_NEUTRAL        1500U

//
// Periodic task timing
//

#define AIR_SENSOR_OVERSAMPLE 2
// #define ASYNC_AIR_SENSORS 1

#define CONTROL_HZ 40.0f
#define CONFIG_HZ (CONTROL_HZ/3)
#define TRIM_HZ CONFIG_HZ
#define LED_HZ 2
#define LED_TICK 30
#define LOG_HZ (CONTROL_HZ/4)
#define LOG_HZ_COMMIT 2.5f
#define CONSOLE_HZ 4.1111f
#define HEARTBEAT_HZ 1
  
//
// Downlink (telemetry) max latencies
//

#define MAX_LATENCY_CONFIG    0.7e3

struct Task {
  void (*code)(void);
  VP_TIME_MICROS_T period;
  bool realTime;
  bool *signal;
  VP_TIME_MICROS_T lastInvoked;
  VP_TIME_MICROS_T runTime;
  uint16_t timesRun, lagged, triggered;
};

#define HZ_TO_PERIOD(f) ((VP_TIME_MICROS_T) (1.0e6f/(f)))

extern struct Task alphaPilotTasks[], *currentTask;

#endif
