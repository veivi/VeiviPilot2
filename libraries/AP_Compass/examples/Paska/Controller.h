#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct PIDCtrl {
  bool autoUnwind; // Unwind when I is very small
  float I, D, prevD, delta, prevErr, Kp, Ki, Kd;
  float rangeMin, rangeMax;
} PIDCtrl_t;

bool pidCtrlInit(PIDCtrl_t *ctrl);
bool pidCtrlInitUnwinding(PIDCtrl_t *ctrl);
void pidCtrlFinalize(PIDCtrl_t *ctrl);
void pidCtrlSetPID(PIDCtrl_t *ctrl, float P, float I, float D);
void pidCtrlSetZNPID(PIDCtrl_t *ctrl, float Ku, float Tu);
void pidCtrlSetZNPI(PIDCtrl_t *ctrl, float Ku, float Tu);
void pidCtrlReset(PIDCtrl_t *ctrl, float value, float err);
void pidCtrlInput(PIDCtrl_t *ctrl, float err, float dt);
void pidCtrlSetRangeAB(PIDCtrl_t *ctrl, float, float);
void pidCtrlSetRange(PIDCtrl_t *ctrl, float);
float pidCtrlOutput(PIDCtrl_t *ctrl);

extern const float gainTweak_c;

#endif
