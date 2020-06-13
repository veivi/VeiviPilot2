#include "stdlib.h"
#include "Log.h"

#ifndef __APPLE__
#include "Objects.h"

float flapEncoded;
uint16_t modeEncoded, statusEncoded, testEncoded;

#define ALPHA    &vpFlight.alpha
#define DYNP     &vpFlight.dynP
#define ACCX     &vpFlight.accX
#define ACCY     &vpFlight.accY,
#define ACCZ     &vpFlight.accZ
#define BANK     &vpFlight.bank
#define ROLLR    &vpFlight.rollR
#define PITCH    &vpFlight.pitch
#define PITCHR   &vpFlight.pitchR
#define HEADING  &vpFlight.heading
#define YAWR     &vpFlight.yawR
#define BALL     &vpFlight.ball
#define AILESTK  &vpInput.aile
#define ELEVSTK  &vpInput.elev
#define THROSTK  &vpInput.throttle
#define RUDSTK   &vpInput.rudder
#define AILE     &vpOutput.aile
#define ELEV     &vpOutput.elev
#define ELEV_P   &vpControl.elevPredict
#define RUDDER   &vpOutput.rudder
#define FLAP     &flapEncoded
#define CANARD   &vpOutput.canard
#define MODE     &modeEncoded
#define STATUS   &statusEncoded
#define TRIM     &vpControl.elevTrim
#define GAIN     &vpControl.testGain
#define TEST     &testEncoded
#define ALTI     &vpFlight.alt
#define FUEL     &vpStatus.fuel
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
#define BALL NULL
#define AILESTK NULL
#define ELEVSTK NULL
#define THROSTK NULL
#define RUDSTK NULL
#define AILE NULL
#define ELEV NULL
#define CANARD NULL
#define ELEV_P NULL
#define RUDDER NULL
#define FLAP NULL
#define MODE NULL
#define STATUS NULL
#define TRIM NULL
#define GAIN NULL
#define TEST NULL
#define ALTI NULL
#define FUEL NULL
#endif

struct LogChannel logChannels[] = {
  [lc_alpha] =   { "ALPH",   lt_angle,    -180, 180,    ALPHA },
  [lc_dynp] =    { "PRES",   lt_real,     -100, 10000,  DYNP },
  [lc_alti] =    { "ALTI",   lt_real,     -10, 500,     ALTI },
  [lc_accx] =    { "ACCX",   lt_real,     -150, 150,    ACCX },
  [lc_accy] =    { "ACCY",   lt_real,     -150, 150,    ACCY },
  [lc_accz] =    { "ACCZ",   lt_real,     -150, 150,    ACCZ },
  [lc_roll] =    { "ROLL",   lt_angle,    -180, 180,    BANK },
  [lc_rollr] =   { "RRTE",   lt_angle,    -1440, 1440,  ROLLR },
  [lc_pitch] =   { "PTCH",   lt_angle,    -90, 90,      PITCH },
  [lc_pitchr] =  { "PRTE",   lt_angle,    -720, 720,    PITCHR },
  [lc_head] =    { "HEAD",   lt_integer,   0, 359,      HEADING },
  [lc_yawr] =    { "YRTE",   lt_angle,    -720, 720,    YAWR },
  [lc_ball] =    { "BALL",   lt_angle,    -90, 90,      BALL },
  [lc_aile] =    { "AILE",   lt_real,     -1.5, 1.5,    AILE },
  [lc_elev] =    { "ELEV",   lt_real,     -1.5, 1.5,    ELEV },
  [lc_elev_p] =  { "ELEP",   lt_real,     -1.5, 1.5,    ELEV_P },
  [lc_rudder] =  { "RUDR",   lt_real,     -1.5, 1.5,    RUDDER },
  [lc_flap] =    { "FLAP",   lt_percent,   0, 100,      FLAP },
  [lc_canard] =  { "CNRD",   lt_real,     -1.5, 1.5,    CANARD },
  [lc_ailestk] = { "ASTK",   lt_real,     -1, 1,        AILESTK },
  [lc_elevstk] = { "ESTK",   lt_real,     -1, 1,        ELEVSTK },
  [lc_thrstk] =  { "THRO",   lt_real,      0, 1,        THROSTK },
  [lc_rudstk] =  { "RSTK",   lt_real,     -1, 1,        RUDSTK },
  [lc_mode] =    { "MODE",   lt_integer,   0, 1,        MODE },
  [lc_status] =  { "STAT",   lt_integer,   0, 1,        STATUS },
  [lc_trim] =    { "TRIM",   lt_percent,   -100, 100,   TRIM },
  [lc_test] =    { "TEST",   lt_integer,   0, 1,        TEST },
  [lc_gain] =    { "GAIN",   lt_real,      0, 50,       GAIN },
  [lc_fuel] =    { "FUEL",   lt_real,      0, 30000,    FUEL }
};

