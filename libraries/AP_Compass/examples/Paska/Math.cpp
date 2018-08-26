#include "Math.h"
#include "NVState.h"
#include "Status.h"
#include "Objects.h"
#include <math.h>

extern "C" {
#include "Console.h"
}

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

float expo(float a, float b)
{
  return sign(a)*powf(fabsf(a), b);
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
  float effIAS = fmaxf(iasFilter.output(), vpDerived.minimumIAS);
  
  if(vpStatus.pitotFailed || vpStatus.pitotBlocked)
    // Failsafe value chosen to be ... on the safe side
    effIAS = expo > 0 ? vpDerived.minimumIAS : vpDerived.minimumIAS * 3/2;
  
  return k * powf(effIAS, expo);
}

float dynamicPressure(float ias)
{
    return airDensity_c * square(ias) / 2;
}

float dynamicPressureInverse(float pressure)
{
  return sign(pressure)*sqrtf(fabsf(2 * pressure / airDensity_c));
}

float polynomial(int deg, float x, const float c[])
{
  float acc = 0, p = 1;

  for(int i = 0; i < deg+1; i++) {
    acc += c[i] * p;
    p *= x;
  }

  return acc;
}

float coeffOfLiftGeneric(float aoa, float coeff[])
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

float sign(float x)
{
  return x < 0.0 ? -1.0 : 1.0;
}

float clamp(float value, float a, float b)
{
  if(a > b) {
    // Swap limits
    float t = a;
    a = b;
    b = t;
  }

  if(value > a && value < b)
    return value;  
  else if(value <= a)
    return a;
  else if(value >= b)
    return b;

  // All comparisons failed, must be NaN or some such
  
  return 0.0;
}

float mixValue(float mixRatio, float a, float b)
{
  if(mixRatio < 0)
    return a;
  else if(mixRatio > 1)
    return b;
  else
    return (1.0 - mixRatio)*a + mixRatio*b;
}

float randomNum(float small, float large)
{
  return small + (large-small)*(float) ((rand()>>3) & 0xFFF) / 0x1000;
}

uint32_t randomUInt32()
{
  uint32_t buffer = 0;
  for(unsigned int i = 0; i < sizeof(buffer); i++)
    buffer = (buffer<<8) | (uint32_t) randomNum(0, 1<<8);
  return buffer;
}

float quantize(float value, float *state, int numSteps)
{
  if((int) ((value-1.0/numSteps/2)*numSteps) > *state)
    *state = (value-1.0/numSteps/2)*numSteps;
  else if((int) ((value+1.0/numSteps/2)*numSteps) < *state)
    *state = (value+1.0/numSteps/2)*numSteps;

  return (float) *state / numSteps;
}

