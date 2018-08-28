#include <math.h>
#include "Math.h"
#include "NVState.h"
#include "Console.h"
#include "CoreObjects.h"
#include "DSP.h"

const float stabilityElevExp_c = -1.5;
const float stabilityAileExp1_c = -1.5;
const float stabilityAileExp2_c = 1.0;
const float stabilityPusherExp_c = -0.5;

const float airDensity_c = 1.225;

const float G = 9.81, FOOT = 12*25.4/1000, KNOT = 1852.0/60/60, PSF = 47.880259;

const float servoOutputRange_c = RATIO(6/5);


float effIAS()
{
  return fmaxf(vpFlight.IAS, vpDerived.minimumIAS);
}

float effDP()
{
  return fmaxf(vpFlight.dynP, vpDerived.minimumDynP);
}

float nominalPitchRateLevel(float bank, float target)
{
  const float CoL = coeffOfLift(target), m = vpDerived.totalMass;
  
  return 1/effIAS() * effDP() * CoL * square(sin(bank)) / m;
}

float nominalPitchRate(float bank, float pitch, float target)
{
  const float CoL = coeffOfLift(target), m = vpDerived.totalMass;

  return
    1/effIAS() * (effDP() * CoL / m - G * cos(bank) * cos(pitch-target)); 
}

float constrainServoOutput(float value)
{
  return clamp(value, -servoOutputRange_c, servoOutputRange_c);
}

float alphaPredictInverse(float x)
{
  if(vpDerived.coeff_FF[2] != 0.0 && x > vpDerived.apexAlpha)
    return vpDerived.apexElev;
  else
    return clamp(polynomial(FF_degree, x, vpDerived.coeff_FF), -1, 1);
}

float alphaPredict(float y)
{
  const float a = vpDerived.coeff_FF[2], b = vpDerived.coeff_FF[1], c = vpDerived.coeff_FF[0];
  
  if(a == 0.0)
    return (y - c)/b;
  else if(y > vpDerived.apexElev)
    return vpDerived.apexAlpha;
  else
    return (-b+sqrt(square(b)-4*a*(c - y)))/(2*a);
}

float rollRatePredict(float pos)
{
  return expo(pos, vpParam.expo)
    * scaleByIAS(vpParam.roll_C, stabilityAileExp2_c);
}

float rollRatePredictInverse(float rate)
{
  return clamp(expo(rate/scaleByIAS(vpParam.roll_C, stabilityAileExp2_c),
		    1/vpParam.expo), -1, 1);
}

float scaleByIAS(float k, float expo)
{
  float ias = effIAS();
  
  if(vpStatus.pitotFailed || vpStatus.pitotBlocked)
    // Failsafe value chosen to be ... on the safe side
    ias = expo > 0 ? vpDerived.minimumIAS : vpDerived.minimumIAS * 3/2;
  
  return k * powf(ias, expo);
}

float dynamicPressure(float ias)
{
    return airDensity_c * square(ias) / 2;
}

float dynamicPressureInverse(float pressure)
{
  return sign(pressure)*sqrtf(fabsf(2 * pressure / airDensity_c));
}

float coeffOfLiftGeneric(float aoa, const float coeff[])
{
  aoa = clamp(aoa, -vpDerived.maxAlpha, vpDerived.maxAlpha);
  return polynomial(CoL_degree, aoa, coeff);
}

float coeffOfLift(float aoa)
{
  return coeffOfLiftGeneric(aoa, vpDerived.coeff_CoL);
}

float coeffOfLiftClean(float aoa)
{
  return coeffOfLiftGeneric(aoa, vpParam.coeff_CoL[0]);
}

float coeffOfLiftInverse(float target)
{
  float left = -vpDerived.maxAlpha, right = vpDerived.maxAlpha;
  float center = 0, approx = 0;

  int i = 0;
  
  do {
    center = (left+right)/2;
    approx = coeffOfLift(center);
    
    if(approx > target)
      right = center;
    else
      left = center;

    if(i++ > 1<<8) {
      consoleNote_P(CS_STRING("Inverse cL not defined for "));
      consolePrintLnF(target);
      return -1e6;
    }
  } while(fabs(approx - target) > 0.001);

  return center;
}

