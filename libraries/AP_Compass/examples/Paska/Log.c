#include "stdlib.h"
#include "Log.h"

#ifdef CONFIG_HAL_BOARD
#include "CoreObjects.h"

#define ALPHA &vpFlight.alpha
#define DYNP &vpFlight.dynP
#define ACCX   &vpFlight.accX
#define ACCY    &vpFlight.accY,
#define ACCZ    &vpFlight.accZ
#define BANK    &vpFlight.bank
#define ROLLR    &vpFlight.rollR
#define PITCH    &vpFlight.pitch
#define PITCHR    &vpFlight.pitchR
#define HEADING    &vpFlight.heading
#define YAWR    &vpFlight.yawR
#define AILESTK    &vpInput.aile
#define ELEVSTK    &vpInput.elev
#define THROSTK    &vpInput.throttle
#define RUDSTK    &vpInput.rudder
#define AILE    &vpOutput.aile
#define ELEV    &vpOutput.elev
#define RUDDER    &vpOutput.rudder
#define FLAP    &flapEncoded
#define MODE    &modeEncoded
#define STATUS    &statusEncoded
#define TRIM    &vpControl.elevTrim
#define GAIN    &vpControl.testGain
#define TEST    &testEncoded
#define ALT    &vpFlight.alt
#else
#define ALPHA NULL
#define DYNP NULL
#define ACCX NULL
#define ACCY NULL
#define ACCZ NULL
#define BANK NULL
#define ROLLR NULL
#define PITCH NULL
#define PITCHR NULL
#define HEADING NULL
#define YAWR NULL
#define AILESTK NULL
#define ELEVSTK NULL
#define THROSTK NULL
#define RUDSTK NULL
#define AILE NULL
#define ELEV NULL
#define RUDDER NULL
#define FLAP NULL
#define MODE NULL
#define STATUS NULL
#define TRIM NULL
#define GAIN NULL
#define TEST NULL
#define ALT NULL
#endif

uint8_t logTest;
float flapEncoded;
uint16_t modeEncoded, statusEncoded, testEncoded;

struct LogChannel logChannels[] = {
  [lc_alpha] = { lc_alpha, "ALPH", lt_angle, -180, 180, ALPHA },
  [lc_dynpressure] = { lc_dynpressure, "PRES", lt_real, -100, 10000, DYNP },
  [lc_accx] = { lc_accx, "ACCX", lt_real, -150, 150, ACCX },
  [lc_accy] = { lc_accy, "ACCY", lt_real, -150, 150, ACCY },
  [lc_accz] = { lc_accz, "ACCZ", lt_real, -150, 150, ACCZ },
  [lc_roll] = { lc_roll, "ROLL", lt_angle, -180, 180, BANK },
  [lc_rollrate] = { lc_rollrate, "RRTE", lt_angle, -1440, 1440, ROLLR },
  [lc_pitch] = { lc_pitch, "PTCH", lt_angle, -90, 90, PITCH },
  [lc_pitchrate] = { lc_pitchrate, "PRTE", lt_angle, -720, 720, PITCHR },
  [lc_heading] = { lc_heading, "HEAD", lt_integer, 0, 359, HEADING },
  [lc_yawrate] = { lc_yawrate, "YRTE", lt_angle, -720, 720, YAWR },
  [lc_ailestick] = { lc_ailestick, "ASTK", lt_real, -1, 1, AILESTK },
  [lc_elevstick] = { lc_elevstick, "ESTK", lt_real, -1, 1, ELEVSTK },
  [lc_thrstick] = { lc_thrstick, "THRO", lt_real, 0, 1, THROSTK },
  [lc_rudstick] = { lc_rudstick, "RSTK", lt_real, -1, 1, RUDSTK },
  [lc_aileron] = { lc_aileron, "AILE", lt_real, -1.5, 1.5, AILE },
  [lc_elevator] = { lc_elevator, "ELEV", lt_real, -1.5, 1.5, ELEV },
  [lc_rudder] = { lc_rudder, "RUDR", lt_real, -1.5, 1.5, RUDDER },
  [lc_flap] = { lc_flap, "FLAP", lt_percent, 0, 100, FLAP },
  [lc_mode] = { lc_mode, "MODE", lt_integer, 0, 1, MODE },
  [lc_status] = { lc_status, "STAT", lt_integer, 0, 1, STATUS },
  [lc_trim] = { lc_trim, "TRIM", lt_percent, -100, 100, TRIM },
  [lc_gain] = { lc_gain, "GAIN", lt_real, 0, 50, GAIN },
  [lc_test] = { lc_test, "TEST", lt_integer, 0, 1, TEST },
  [lc_alt] = { lc_alt, "ALTI", lt_real, -10, 500, ALT } };

