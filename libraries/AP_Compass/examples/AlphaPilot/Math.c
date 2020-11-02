#include <math.h>
#include "Math.h"
#include "NVState.h"
#include "Console.h"
#include "Objects.h"
#include "DSP.h"
#include "CRC16.h"

const float stabGainExp_c = -1.5;
const float yawDamperExp_c = -2.0;
const float airDensity_c = 1.225;
const float servoOutputRange_c = RATIO(5/4);

float sqrf(float x)
{
  return x * x;
}

float signf(float x)
{
  if(x < 0.0f)
    return -1.0f;
  else if(x > 0.0f)
    return 1.0f;
  else
    return 0.0;
}

float nominalPitchRateLevel(float bank, float target)
{
  const float CoL = coeffOfLift(target), m = vpStatus.mass;

  /*
    static VPPeriodicTimer_t debug = VP_PERIODIC_TIMER_CONS(1e3);

  if(vpPeriodicEvent(&debug)) {
    consolePrintF(vpFlight.effIAS);
    consolePrint(" ");
    consolePrintF(CoL);
    consolePrint(" ");
    consolePrintLnF(m);
  }
  */
  return 1/vpFlight.effIAS * vpFlight.effDynP * CoL * sqrf(sinf(bank)) / m;
}

float nominalPitchRate(float bank, float pitch, float target)
{
  const float CoL = coeffOfLift(target), m = vpStatus.mass;

  return
    1/vpFlight.effIAS * (vpFlight.effDynP * CoL / m - G * cosf(bank) * cosf(pitch-target)); 
}

float constrainServoOutput(float value)
{
  return clamp(value, -servoOutputRange_c, servoOutputRange_c);
}

float alphaPredictInverse(float x)
{
  derivedValidate();
  
  if(vpDerived.coeff_FF[2] != 0.0f && x > vpDerived.apexAlpha)
    return vpDerived.apexElev;
  else
    return clamp(polynomial(FF_degree, x, vpDerived.coeff_FF), -1, 1);
}

float alphaPredict(float y)
{
  derivedValidate();
  
  const float
    a = vpDerived.coeff_FF[2], b = vpDerived.coeff_FF[1],
    c = vpDerived.coeff_FF[0];
  
  if(a == 0.0f)
    return (y - c)/b;
  else if(y > vpDerived.apexElev)
    return vpDerived.apexAlpha;
  else
    return (-b+sqrtf(sqrf(b)-4*a*(c - y)))/(2*a);
}

float rollRatePredict(float pos)
{
  return expo(pos, vpParam.roll_Expo) * scaleByIAS(vpParam.roll_C);
}

float rollRatePredictInverse(float rate)
{
  return clamp(expo(rate/scaleByIAS(vpParam.roll_C), 1/vpParam.roll_Expo), -1, 1);
}

float scaleByIAS_E(float k, float expo)
{
  float ias = vpFlight.effIAS;
  
  if(vpStatus.pitotFailed || vpStatus.pitotBlocked)
    // Failsafe value chosen to be ... on the safe side
    ias = expo > 0 ? vpDerived.minimumIAS : vpDerived.minimumIAS * 3/2;
  
  return k * powf(ias, expo);
}

float scaleByIAS(float k)
{
  return scaleByIAS_E(k, 1);
}

float dynamicPressure(float ias)
{
    return airDensity_c * sqrf(ias) / 2;
}

float dynamicPressureInverse(float pressure)
{
  return signf(pressure)*sqrtf(fabsf(2 * pressure / airDensity_c));
}

float coeffOfLiftGeneric(float aoa, const float coeff[])
{
  derivedValidate();
  
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

float coeffOfLiftLand(float aoa)
{
  return coeffOfLiftGeneric(aoa,
			    vpParam.coeff_CoL[vpDerived.haveFlaps ? 1 : 0]);
}

float coeffOfLiftInverse(float target)
{
  derivedValidate();
  
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
  } while(fabsf(approx - target) > 0.001f);

  return center;
}

void pseudoRandom(uint8_t *value, uint8_t size, uint16_t *state)
{
  uint8_t i = 0;
  
  for(i = 0; i < size; i++) {
    *state = crc16_update(*state, 0xFF);
    value[i] = (*state) & 0xFF;
  }
}

uint16_t randomUINT16()
{
  static uint16_t state = 0xFFFF;
  uint16_t value = 0;
  pseudoRandom((uint8_t*) &value, sizeof(value), &state);
  return value;
}


