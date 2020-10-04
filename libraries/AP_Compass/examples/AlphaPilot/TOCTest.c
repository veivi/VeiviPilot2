#include <string.h>
#include "StaP.h"
#include "TOCTest.h"
#include "Logging.h"
#include "RxInput.h"
#include "M24XX.h"
#include "Console.h"
#include "NVState.h"
#include "DSP.h"
#include "Math.h"
#include "Button.h"
#include "AS5048B.h"
#include "MS4525.h"
#include "Time.h"

const float toc_margin_c = RATIO(3/100);

bool toc_test_mode(bool reset)
{
  return !vpMode.test
    && vpMode.wingLeveler && vpMode.bankLimiter && vpMode.takeOff;
}

bool toc_test_gear(bool reset)
{
  return vpMode.gearSelected || !vpDerived.haveRetracts;
}

bool toc_test_link(bool reset)
{
  return ppmGoodSeconds > 10 && ppmFreq > 30
    && (!vpParam.virtualOnly || vpStatus.simulatorLink);
}

bool toc_test_ram(bool reset)
{
  return stap_memoryFree() > (1<<9);
}

bool toc_test_load(bool reset)
{
  return vpStatus.simulatorLink || vpStatus.load < 0.95;
}

bool toc_test_fdr(bool reset)
{
  return m24xxIsOnline() && logReady(false);
}

bool toc_test_alpha_sensor(bool reset)
{
  return !vpStatus.alphaFailed && AS5048B_entropy() > MIN_ENTROPY
    && fieldStrength > 0.15 && fieldStrength < 0.80;
}

bool toc_test_alpha_range(bool reset)
{
  static bool bigAlpha, zeroAlpha;
  static VP_TIME_MILLIS_T lastNonZeroAlpha, lastSmallAlpha;

  if(reset) {
    zeroAlpha = bigAlpha = false;
    lastNonZeroAlpha = lastSmallAlpha = vpTimeMillisApprox;
    
  } else if(!zeroAlpha) {
    if(fabs(vpFlight.alpha) > 1.5/RADIAN) {
      lastNonZeroAlpha = vpTimeMillisApprox;
    } else if(vpTimeMillisApprox > lastNonZeroAlpha + 1.0e3) {
      consoleNoteLn_P(CS_STRING("Stable ZERO ALPHA"));
      zeroAlpha = true;
    }
  } else if(!bigAlpha) {
    if(fabsf(vpFlight.alpha - 90/RADIAN) > 30/RADIAN) {
      lastSmallAlpha = vpTimeMillisApprox;
    } else if(vpTimeMillisApprox > lastSmallAlpha + 1.0e3) {
      consoleNoteLn_P(CS_STRING("Stable BIG ALPHA"));
      bigAlpha = true;
    }
  }
  
  return (bigAlpha && zeroAlpha);
}

bool toc_test_alpha(bool reset)
{
  return (toc_test_alpha_sensor(reset) && toc_test_alpha_range(reset))
     || vpStatus.simulatorLink;
}

bool toc_test_pitot(bool reset)
{
  static bool positiveIAS;
  
  if(reset)
    positiveIAS = false;
  else if(vpStatus.positiveIAS)
    positiveIAS = true;
  
  return (!vpStatus.pitotFailed && MS4525DO_entropy() > MIN_ENTROPY
	  && !vpStatus.pitotBlocked && positiveIAS && vpFlight.IAS < 5)
    || vpStatus.simulatorLink;
}

bool toc_test_attitude(bool reset)
{
  return fabsf(vpFlight.pitch) < 15/RADIAN && fabsf(vpFlight.bank) < 5/RADIAN;
}

bool toc_test_gyro(bool reset)
{
  return vpStatus.simulatorLink
    || (fabsf(vpFlight.pitchR) < 1.0/RADIAN
	&& fabsf(vpFlight.rollR) < 1.0/RADIAN
	&& fabsf(vpFlight.yawR) < 2.0/RADIAN);
}

struct TOCRangeTestState {
  float valueMin, valueMax;
};

bool toc_test_range_generic(struct TOCRangeTestState *state, bool reset, uint8_t ch, float expectedMin, float expectedMax)
{
  const float value = inputValue(ch);
  
  if(reset) {
    state->valueMin = state->valueMax = value;
    return true;
  } else {
    state->valueMin = fminf(state->valueMin, value);
    state->valueMax = fmaxf(state->valueMax, value);
  }
  
  return (fabsf(state->valueMin - expectedMin) < toc_margin_c
	  && fabsf(state->valueMax - expectedMax) < toc_margin_c);
}

bool toc_test_rstick_range(bool reset)
{
  static struct TOCRangeTestState stateElev, stateAile;
  return toc_test_range_generic(&stateElev, reset, CH_ELEV, -1, 1)
    && toc_test_range_generic(&stateAile, reset, CH_AILE, -1, 1);
}

bool toc_test_rstick_neutral(bool reset)
{
  return ( fabsf(inputValue(CH_AILE)) < toc_margin_c )
    && ( fabsf(inputValue(CH_ELEV)) < toc_margin_c );
}

bool toc_test_rstick(bool reset)
{
  return toc_test_rstick_range(reset) && toc_test_rstick_neutral(reset);
}

bool toc_test_lstick_range(bool reset)
{
  static struct TOCRangeTestState stateThr;
  bool status = toc_test_range_generic(&stateThr, reset, CH_THRO,
				       vpParam.fuelDensity > 0 ? -1 : 0, 1);

#if RX_CHANNELS >= 8
  static struct TOCRangeTestState stateRudder;
  bool status2 = toc_test_range_generic(&stateRudder, reset, CH_RUD, -1, 1);
  status = status && status2;
#endif

  return status;
}

bool toc_test_lstick_neutral(bool reset)
{
  bool status = fabsf(inputValue(CH_THRO)) < toc_margin_c;
    
#if RX_CHANNELS >= 8
  status = status && fabsf(inputValue(CH_RUD)) < toc_margin_c;
#endif
  
  return status;
}

bool toc_test_lstick(bool reset)
{
  return toc_test_lstick_range(reset) && toc_test_lstick_neutral(reset);
}

bool toc_test_tuning_range(bool reset)
{
  static struct TOCRangeTestState state;
  bool result = toc_test_range_generic(&state, reset, CH_TUNE, 0, 1);
  return result;
}

bool toc_test_tuning_zero(bool reset)
{
  bool result = fabsf(inputValue(CH_TUNE)) < toc_margin_c;
  return result;
}

bool toc_test_tuning(bool reset)
{
  return toc_test_tuning_range(reset) && toc_test_tuning_zero(reset);
}

bool toc_test_button_range(bool reset)
{
#ifdef CH_BUTTON
  static struct TOCRangeTestState state;
  return toc_test_range_generic(&state, reset, CH_BUTTON, -1, 1);
#else
  static struct TOCRangeTestState state1, state2;
  return toc_test_range_generic(&state1, reset, CH_TRIM, 0, 1) &&
    toc_test_range_generic(&state2, reset, CH_LEVEL, 0, 1);
#endif
}

bool toc_test_button_neutral(bool reset)
{
#ifdef GEARBUTTON
  bool status = !buttonState(&GEARBUTTON) && !buttonState(&RATEBUTTON);
#else
  bool status = true;
#endif
  
  return status && !buttonState(&TRIMBUTTON);
}

bool toc_test_button(bool reset)
{
  return toc_test_button_range(reset) && toc_test_button_neutral(reset);
}

const struct TakeoffTest tocTest[] CS_QUALIFIER =
  {
    [toc_alpha] = { "ALPHA", toc_test_alpha },
    [toc_pitot] = { "PITOT", toc_test_pitot },
    [toc_link] = { "LINK", toc_test_link },
    [toc_lstick] = { "LSTIK", toc_test_lstick },
    [toc_rstick] = { "RSTIK", toc_test_rstick },
    [toc_tuning] = { "TUNE", toc_test_tuning },
    [toc_button] = { "BUTTN", toc_test_button },
    [toc_attitude] = { "ATTI", toc_test_attitude },
    [toc_gyro] = { "GYRO", toc_test_gyro },
    [toc_mode] = { "MODE", toc_test_mode },
    [toc_fdr] = { "DATA", toc_test_fdr },
    [toc_ram] = { "RAM", toc_test_ram },
    [toc_load] = { "LOAD", toc_test_load },
    [toc_gear] = { "GEAR", toc_test_gear }
  };

const int tocNumOfTests = sizeof(tocTest)/sizeof(struct TakeoffTest);

static bool tocStatusFailed;

bool tocTestInvoke(bool reset, bool challenge, void (*report)(bool, int, const char *))
{
  struct TakeoffTest cache;
  int i = 0;

  tocStatusFailed = false;
  
  for(i = 0; i < tocNumOfTests; i++) {
    CS_MEMCPY(&cache, &tocTest[i], sizeof(cache));

    bool result = (*cache.function)(reset);
    
    if(challenge) {
      (*report)(result, i, cache.description); 

      if(!result)
	tocStatusFailed = true;
    }
  }

  return !tocStatusFailed;
}

bool tocTestReset()
{
  return tocTestInvoke(true, false, NULL);
}

void tocTestUpdate()
{
  tocTestInvoke(false, false, NULL);
}

void tocReportConsole(bool result, int i, const char *s)
{  
  if(!result) {
    if(!tocStatusFailed)
      consoleNote_P(CS_STRING("T/O/C FAIL :"));

    consolePrint(" ");
    consolePrint(s);
  }
}

bool tocTestStatus(void (*reportFn)(bool, int, const char*))
{
  return tocTestInvoke(false, true, reportFn);
}

