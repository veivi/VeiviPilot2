#ifndef FUNCTION_H
#define FUNCTION_H

#include <stdint.h>
#include <stdbool.h>

#define THROTTLE_SIGN        1

typedef enum {
  fn_null,
  fn_aileron,
  fn_elevator,
  fn_rudder,
  fn_flap,
  fn_throttle,
  fn_gear,
  fn_steering,
  fn_brake,
  fn_flaperon1, fn_flaperon2,
  fn_canard1, fn_canard2,
  fn_elevon1, fn_elevon2,
  fn_tail1, fn_tail2,
  fn_thrustvert,
  fn_thrusthoriz,
  fn_invalid
} function_t;

void functionSet(uint8_t ch, const char *name);
bool functionInvoke(int8_t fn, float *result);

#endif
