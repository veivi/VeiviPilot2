#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct PIDCtrl {
  bool autoUnwind; // Unwind when I is very small
  float rangeMin, rangeMax;
  float I, errD, prevErrD, delta, prevErr, Kp, Ki, Kd;
  float gain;
} PIDCtrl_t;

#define PIDCTRL_CONS(r) { false, -(r), r }
#define PIDCTRL_U_CONS(r) { true, (-r), r }

void pidCtrlSetPID(PIDCtrl_t *ctrl, float P, float I, float D);
void pidCtrlSetZNPID(PIDCtrl_t *ctrl, float Ku, float Tu);
void pidCtrlSetZNPI(PIDCtrl_t *ctrl, float Ku, float Tu);
void pidCtrlReset(PIDCtrl_t *ctrl, float value, float err);
void pidCtrlInput(PIDCtrl_t *ctrl, float err, float dt);
void pidCtrlSetGain(PIDCtrl_t *ctrl, float c);
void pidCtrlSetRangeAB(PIDCtrl_t *ctrl, float, float);
void pidCtrlSetRange(PIDCtrl_t *ctrl, float);
float pidCtrlOutput(PIDCtrl_t *ctrl);

extern const float gainTweak_c;

#endif
