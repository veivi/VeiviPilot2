#include <math.h>
#include "Controller.h"
#include "DSP.h"

const float gainTweak_c = 0.80;

bool pidCtrlInit(PIDCtrl_t *ctrl)
{
  ctrl->autoUnwind = false;
  ctrl->rangeMin = -1;
  ctrl->rangeMax = 1;
  return true;
}

bool pidCtrlInitUnwinding(PIDCtrl_t *ctrl)
{
  bool status = pidCtrlInit(ctrl);
  if(status)
    ctrl->autoUnwind = true;
  return status;
}

void PIDCtrlFinalize(PIDCtrl_t *ctrl)
{
}

void pidCtrlSetPID(PIDCtrl_t *ctrl, float kP, float kI, float kD)
{
  ctrl->Kp = kP; 
  ctrl->Ki = kI;
  ctrl->Kd = kD;
}

void pidCtrlSetZNPID(PIDCtrl_t *ctrl, float Ku, float Tu)
{
  Ku *= gainTweak_c;
  ctrl->Kp = 0.6*Ku; 
  ctrl->Ki = 2*ctrl->Kp/Tu; 
  ctrl->Kd = ctrl->Kp*Tu/8;
}

void pidCtrlSetZNPI(PIDCtrl_t *ctrl, float Ku, float Tu)
{
  Ku *= gainTweak_c;
  ctrl->Kp = 0.45*Ku; 
  ctrl->Ki = 1.2*ctrl->Kp/Tu; 
  ctrl->Kd = 0;
}

void pidCtrlReset(PIDCtrl_t *ctrl, float value, float err)
{
  ctrl->prevErr = err;
  ctrl->I = value - ctrl->Kp*err;
}

void pidCtrlSetRangeAB(PIDCtrl_t *ctrl, float a, float b)
{
  ctrl->rangeMin = a;
  ctrl->rangeMax = b;
  if(ctrl->rangeMin > ctrl->rangeMax)
    ctrl->rangeMin = ctrl->rangeMax;
}

void pidCtrlSetRange(PIDCtrl_t *ctrl, float r)
{
  r = fabs(r);
  pidCtrlSetRangeAB(ctrl, -r, r);
}

void pidCtrlInput(PIDCtrl_t *ctrl, float err, float d)
{
  ctrl->prevD = d;
  ctrl->D = err - ctrl->prevErr;
  ctrl->prevErr = err;
  
  ctrl->delta = d;

  ctrl->I = clamp(ctrl->I + ctrl->Ki*err*ctrl->delta,
		  ctrl->rangeMin - ctrl->Kp*err,
		  ctrl->rangeMax - ctrl->Kp*err);

  if(ctrl->autoUnwind && ctrl->Ki < 1.0E-5)
    ctrl->I = 0.0;
}

float pidCtrlOutput(PIDCtrl_t *ctrl) {
  const float diffLimit = (ctrl->rangeMax-ctrl->rangeMin)/6,
    diffTerm = clamp(ctrl->Kd*(ctrl->D+ctrl->prevD)/2/ctrl->delta,
		     -diffLimit, diffLimit);
  return clamp(ctrl->Kp*ctrl->prevErr + ctrl->I + diffTerm,
	       ctrl->rangeMin, ctrl->rangeMax);
}
